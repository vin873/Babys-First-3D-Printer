#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations
from obstacle_detector.msg import Obstacles
from eurobot2023_main_small.srv import *

# subscribe each enemy's pos
enemies = [[-1, -1], [-1, -1]]
# subscribe our robots pos
startPos = [[1500, 1000], [1500, 1000]]
absAng = [0, 0]

picked = []
used = Pose()
tempFull = [[0, 0, 0, 0], [0, 0, 0, 0]]
plates = [[1125, 225], [1125, 1775], [1875, 1775], [1875, 225], [2775, 225], [2775, 725], [2775, 1275], [2775, 1775]]
tempPlates = []
camCakes = PoseArray()
plumpCakes = [-1, -1,"", [-1, -1]]
minAngle = 360
headAng = -45
camAng = 45
stealDis = 240

robotNum = 0
side = 0
run_mode = ''
robotPose = PoseStamped()

def handle_steal(req):
    # print('in')
    publisher()
    return stealResponse(robotPose)

def startPos1_callback(msg):
    global startPos, absAng, headAng
    startPos[0][0] = msg.pose.pose.position.x * 1000
    startPos[0][1] = msg.pose.pose.position.y * 1000
    absAng[0] = quaternion2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) - headAng

def startPos2_callback(msg):
    global startPos, absAng, headAng
    startPos[1][0] = msg.pose.pose.position.x * 1000
    startPos[1][1] = msg.pose.pose.position.y * 1000
    absAng[1] = quaternion2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) - headAng

def enemiesPos1_callback(msg):
    global enemies
    enemies[0][0] = msg.pose.pose.position.x * 1000
    enemies[0][1] = msg.pose.pose.position.y * 1000

def enemiesPos2_callback(msg):
    global enemies
    enemies[1][0] = msg.pose.pose.position.x * 1000
    enemies[1][1] = msg.pose.pose.position.y * 1000

def enemies_callback(msg):
    global enemies
    if len(msg.circles) >= 1:
        enemies[0][0] = msg.circles[0].center.x * 1000
        enemies[0][1] = msg.circles[0].center.y * 1000
    else:
        enemies[0][0] = -1
        enemies[0][1] = -1
    if len(msg.circles) >= 2:
        enemies[1][0] = msg.circles[1].center.x * 1000
        enemies[1][1] = msg.circles[1].center.y * 1000
    else:
        enemies[1][0] = -1
        enemies[1][1] = -1

def closerEnemy(target):
    global enemies
    speed = 0.9  # enemy/our
    min = 99999
    for enemy in range(2):
        if enemies[enemy] != [-1, -1]:
            if min > euclidean(enemies[enemy], target) / speed:
                min = euclidean(enemies[enemy], target) / speed
    return min

def tPoint(pos, target):
    # print(pos ,target)
    if target == [-0.001, -0.001]:
        return [-1, -1]
    else:
        dis = stealDis
        point = [-1, -1]
        if target[0] == pos[0]:
            if target[1] > pos[1]:
                point = [target[0], target[1]-dis]
            elif target[1] < pos[1]:
                point = [target[0], target[1]+dis]
        elif target[1] == pos[1]:
            if target[0] > pos[0]:
                point = [target[0]-dis, target[1]]
            elif target[0] < pos[0]:
                point = [target[0]+dis, target[1]]
        else:
            if target[0] > pos[0]:
                x = target[0] - pos[0]
            else:
                x = pos[0] - target[0]
            if target[1] > pos[1]:
                y = target[1]-pos[1]
            else:
                y = pos[1]-target[1]
            if target[1] > pos[1]:
                point[1] = target[1] - dis*math.sin(math.atan(y/x))
            else:
                point[1] = target[1] + dis*math.sin(math.atan(y/x))
            if target[0] > pos[0]:
                point[0] = target[0] - dis*math.cos(math.atan(y/x))
            else:
                point[0] = target[0] + dis*math.cos(math.atan(y/x))
        return point

def robotPublish(num, target):
    global robotPose, startPos, robotNum

    robotPose.header.stamp = rospy.Time.now()

    if target == []:
        robotPose.header.frame_id = '?'
        robotPose.pose.position.x = -1
        robotPose.pose.position.y = -1
    else:
        robotPose.header.frame_id = str(target[2])
        pt = tPoint(startPos[robotNum], [target[0], target[1]])
        # print(pt)
        robotPose.pose.position.x = pt[0] * 0.001
        robotPose.pose.position.y = pt[1] * 0.001
        # print(robotPose)

def euler2quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return Quaternion(qx, qy, qz, qw)

def quaternion2euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z / math.pi * 180 # in radians

def listener():
    global side, robotNum, run_mode
    rospy.init_node("steal")
    side = rospy.get_param('side')
    robotNum = rospy.get_param('robot')
    run_mode = rospy.get_param('run_mode')
    rospy.Service('steal'+str(robotNum), steal, handle_steal)
    rospy.Subscriber("cake_node/obstacle_position_array", PoseArray, plumpCakes_callback)
    rospy.Subscriber("using_steal"+str(not bool(robotNum)), Pose, used_callback)
    publisher()
    if run_mode == 'run':
        rospy.Subscriber("/robot1/ekf_pose", PoseWithCovarianceStamped, startPos1_callback)
        rospy.Subscriber("/robot2/ekf_pose", PoseWithCovarianceStamped, startPos2_callback)
        rospy.Subscriber("/RivalObstacle", Obstacles, enemies_callback)
    elif run_mode == 'sim':
        rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
        rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
        rospy.Subscriber("/rival1/odom", Odometry, enemiesPos1_callback)
        rospy.Subscriber("/rival2/odom", Odometry, enemiesPos2_callback)
    rospy.spin()

def plumpCakes_callback(msg):
    global camCakes
    camCakes = msg
    # print(camCakes)

def used_callback(msg):
    global used
    used = msg

def publisher():
    global enemies, startPos, picked, used, robotNum, robotPose, tempFull, minAngle, plumpCakes, tempPlates
    tempMinDist = 99999
    tempPlatePicked = 0
    picked = []
    num = robotNum
    pos = list(startPos[robotNum])

    tempPlates = []
    plumpCakes = []
    tempPlump=[]
    for i in range(len(camCakes.poses)):
        tempPlump=[]
        tempPlump.append(camCakes.poses[i].position.x * 1000)
        tempPlump.append(camCakes.poses[i].position.y * 1000)
        tempPlump.append(int(camCakes.poses[i].position.z))
        plumpCakes.append(tempPlump)
    for i in range(len(plates)):
        if i % 2 == side:
            tempPlates.append(list(plates[i]))
    purgeList=[]
    for cake in plumpCakes:
        flag=0
        for plate in tempPlates:
            if abs(cake[0] - plate[0]) < 225 and abs(cake[1] - plate[1]) < 225:
                cake.append(plates.index(plate)+1)
                flag=1
                break
        if flag==0:
            purgeList.append(cake)
    for cake in purgeList:
        plumpCakes.remove(cake)

    for target in plumpCakes:
        if target[0] != used.position.x and target[1] != used.position.y and target[0] != -1  and target[1] != -1:
            if tempMinDist > euclidean(pos, (target[0], target[1])) - closerEnemy((target[0], target[1])):
                tempMinDist = euclidean(pos, (target[0], target[1])) - closerEnemy((target[0], target[1]))
                tempPlatePicked = target[3]
    tempMinDist = 99999
    for target in plumpCakes:
        if target[3] == tempPlatePicked and target[0] != -1 and target[1] != -1:
             if tempMinDist > euclidean(pos, (target[0], target[1])):
                tempMinDist = euclidean(pos, (target[0], target[1]))
                picked = list(target)

    if picked != []:
        pub = rospy.Publisher('using_steal'+str(robotNum), Pose, queue_size=1000)
        used_pose = Pose()
        used_pose.position.x = picked[0]
        used_pose.position.y = picked[1]
        used_pose.position.z = picked[2]
        pub.publish(used_pose)

        robotPublish(robotNum, picked)
        camtAng = (np.rad2deg(np.arctan2(picked[1] - pos[1], picked[0] - pos[0])) + 360 - camAng) % 360
        quat = euler2quaternion(0, 0, camtAng*math.pi/180)
        robotPose.pose.orientation.x = quat.x
        robotPose.pose.orientation.y = quat.y
        robotPose.pose.orientation.z = quat.z
        robotPose.pose.orientation.w = quat.w
    else:
        robotPublish(robotNum, picked)

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass