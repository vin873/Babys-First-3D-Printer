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
from eurobot2023_main_small.srv import *

# subscribe each enemy's pos
enemies = [[-1, -1], [-1, -1]]
# subscribe our robots pos
startPos = [[1500, 1000], [1500, 1000]]
absAng = [0, 0]

picked = [[[-1, -1],  [-1, -1],  [-1, -1]], [[-1, -1],  [-1, -1],  [-1, -1]]]
used = []
tempFull = [[0, 0, 0, 0], [0, 0, 0, 0]]
tempPicked=[]
plates = [[225, 225], [225, 1775], [1125, 225], [1125, 1775], [1875, 225], [1875, 1775], [2775, 225], [2775, 725], [2775, 1275], [2775, 1775]]
plumpCakes=[[-1, -1,"", [-1, -1]], [125, 125, 10, [225, 225]]]
minAngle = 360
headAng = -45
outAngle = [0, 0]
stealDis = 240

robotNum = 0
side = 0
run_mode = ''
quaternion = Quaternion()
robotPose = PoseArray()

def handle_steal(req):
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
    global robotPose, tempFull, startPos, robotNum

    robotPose.header.frame_id = target[2]
    robotPose.header.stamp = rospy.Time.now()

    pose = Pose()
    pt = tPoint(startPos[robotNum], [target[0], target[1]])
    # print(pt)
    pose.position.x = pt[0]
    pose.position.y = pt[1]
    pose.orientation.x = quaternion.x
    pose.orientation.y = quaternion.y
    pose.orientation.z = quaternion.z
    pose.orientation.w = quaternion.w
    robotPose.poses=pose
    print(robotPose)

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
    # run_mode = rospy.get_param('run_mode')
    rospy.Service('steal'+str(robotNum), steal, handle_steal)
    publisher()
    if run_mode == 'run':
        rospy.Subscriber("/robot1/ekf_pose", PoseWithCovarianceStamped, startPos1_callback)
        rospy.Subscriber("/robot2/ekf_pose", PoseWithCovarianceStamped, startPos2_callback)
        rospy.Subscriber("/rival1/ekf_pose", PoseWithCovarianceStamped, enemiesPos1_callback)
        rospy.Subscriber("/rival2/ekf_pose", PoseWithCovarianceStamped, enemiesPos2_callback)
    elif run_mode == 'sim':
        rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
        rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
        rospy.Subscriber("/rival1/odom", Odometry, enemiesPos1_callback)
        rospy.Subscriber("/rival2/odom", Odometry, enemiesPos2_callback)
    # rospy.Subscriber("/plumpCakes", PoseArray, plumpCakes_callback)
    rospy.spin()

def plumpCakes_callback(msg):
    global plumpCakes, plates
    for i in range(len(msg.poses)):
        plumpCakes[i][0] = msg.poses[i].position.x * 1000
        plumpCakes[i][1] = msg.poses[i].position.y * 1000
        plumpCakes[i][2] = int(msg.poses[i].position.z)
    for i in range(len(plates)):
        if i %2 != side:
            plates.pop(i)
    for cake in plumpCakes:
        for plate in plates:
            if abs(cake[0] - plate[0]) < 225 and abs(cake[1] - plate[1]) < 225:
                cake.append(plates.index(plate))
            else:
                plumpCakes.remove(cake)

def publisher():
    global quaternion, enemies, startPos, picked, used, robotNum, robotPose, tempFull, minAngle, tempPicked, outAngle, quaternion
    tempMinDist = 99999
    tempPlatePicked = [-1, -1]
    tempPicked = [-1, -1]
    pos = startPos[robotNum]
    for target in plumpCakes:
        if target not in used and target[0] != -1 and target[1] != -1:
            if tempMinDist > euclidean(pos, (target[0], target[1])) - closerEnemy((target[0], target[1])):
                tempMinDist = euclidean(pos, (target[0], target[1])) - closerEnemy((target[0], target[1]))
                tempPlatePicked = target[3]
    print(tempPlatePicked)
    tempMinDist = 99999
    for target in plumpCakes:
        if target[3]==tempPlatePicked and target[0] != -1 and target[1] != -1:
             if tempMinDist > euclidean(pos, (target[0], target[1])):
                tempMinDist = euclidean(pos, (target[0], target[1]))
                tempPicked = target
    print(picked)

    anglePos = startPos[robotNum]
    tempAng = list(absAng)
    minAngle = 360
    num = robotNum
    tAngle = (np.rad2deg(np.arctan2(picked[1] - anglePos[1], picked[0] - anglePos[0])) - tempAng[robotNum] + 360) % 360
    tAngles = [360, 360, 360, 360]
    for i in range(4):
        if tempFull[num][i] == 0:
            tAngles[i] = (tAngle - i * 90 + 360) % 360
            if tAngles[i] > 180:
                tAngles[i] -= 360
    for i in tAngles:
        if abs(i) < abs(minAngle):
            minAngle = i
    outAngle[num] = minAngle + tempAng[num] + headAng

    quaternion = euler2quaternion(0, 0, outAngle[robotNum] * math.pi / 180)
    robotPublish(robotNum, tempPicked)

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass