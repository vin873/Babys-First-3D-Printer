#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations
from Eurobot2023_main_test.srv import *

# subscribe our robots pos
startPos = [[-1, -1], [-1, -1]]
absAng = [0, 0]

enemies = [[-1, -1], [-1, -1]]

picked = [[[-1, -1],  [-1, -1],  [-1, -1]], [[-1, -1],  [-1, -1],  [-1, -1]]]
used = []
got = [[0, 0, 0], [0, 0, 0]]  # brown, yellow, pink
tempGot = [[0, 0, 0], [0, 0, 0]]
fullness = [[0, 0, 0, 0], [0, 0, 0, 0]]
tempFull = [[0, 0, 0, 0], [0, 0, 0, 0]]
plates = [[225,225],[225,1775],[1125,225],[1125,1775],[1875,225],[1875,1775],[2775,225],[2775,725],[2775,1275],[2775,1775]]
plumpCakes=[]
currMin = [99999, 99999]
minAngle = 360
headAng = -45
outAngle = [[0, 0, 0], [0, 0, 0]]
dockDis = 0.240
cakeDis = 0.075

robotNum = 0
side = 0
position = [-1, -1]
preposition = [-1, -1]
quaternion = Quaternion()
robotPose = PoseArray()

def plumpCakes_callback(msg):
    global plumpCakes, plates
    for i in range(len(msg.poses)):
        plumpCakes[i][0] = msg.poses[i].position.x * 1000
        plumpCakes[i][1] = msg.poses[i].position.y * 1000
        plumpCakes[i][2] = int(msg.poses[i].position.z)
    for i in range(len(plates)):
        if i%2!=side:
            plates.pop(i)
    for cake in plumpCakes:
        for plate in plates:
            if abs(cake[0]-plate[0])<225 and abs(cake[1]-plate[1])<225:
                cake.extend(plates.index(plate))
            else:
                plumpCakes.remove(cake)

def handle_cake(req):
    publisher()
    return cakeResponse(robotPose)

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

def tPoint(mode, pos, target):
    # print(pos ,target)
    if target == [-0.001, -0.001]:
        return [-1, -1]
    else:
        if mode == 'd':
            dis = dockDis
        elif mode == 'c':
            dis = cakeDis
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

def robotPublish(num, color):
    global robotPose, tempFull

    c = '?'
    if color == 0:
        c = 'b'
    elif color == 1:
        c = 'y'
    elif color == 2:
        c = 'p'

    robotPose.header.frame_id += c
    for full in tempFull[num]:
        if full == color+1:
            robotPose.header.frame_id += str(tempFull[num].index(full))
    robotPose.header.stamp = rospy.Time.now()

    pose = Pose()
    pt = tPoint('d', preposition, position)
    # print(pt)
    pose.position.x = pt[0]
    pose.position.y = pt[1]
    pose.orientation.x = quaternion.x
    pose.orientation.y = quaternion.y
    pose.orientation.z = quaternion.z
    pose.orientation.w = quaternion.w
    robotPose.poses.append(pose)
    
    pose2 = Pose()
    pt = tPoint('c', preposition, position)
    pose2.position.x = pt[0]
    pose2.position.y = pt[1]
    pose2.orientation.x = quaternion.x
    pose2.orientation.y = quaternion.y
    pose2.orientation.z = quaternion.z
    pose2.orientation.w = quaternion.w
    robotPose.poses.append(pose2)

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
    global side, robotNum
    rospy.init_node("better_cake")
    side = rospy.get_param('side')
    robotNum = rospy.get_param('robot')
    rospy.Service('cake'+str(robotNum), cake, handle_cake)
    # rospy.Subscriber("/robot1/ekf_pose", PoseWithCovarianceStamped, startPos1_callback)
    # rospy.Subscriber("/robot2/ekf_pose", PoseWithCovarianceStamped, startPos2_callback)
    rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
    rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
    rospy.Subscriber("/rival1/odom", Odometry, enemiesPos1_callback)
    rospy.Subscriber("/rival2/odom", Odometry, enemiesPos2_callback)
    # rospy.Subscriber("/plumpCakes", PoseArray, plumpCakes_callback)
    rospy.spin()

def publisher():
    global quaternion, enemies, startPos, currMin, picked, used, robotNum, robotPose, tempFull, minAngle, tempAllCakes, outAngle, position, quaternion, tempGot, preposition
    tempColorMin = 99999
    tempPlatePicked =  [-1, -1]
    for i in range(len(plumpCakes)):
        for target in plumpCakes[i]:
            if target not in used and target[0] != -1 and target[1] != -1:
                if tempColorMin > euclidean(pos, target) - closerEnemy(target):
                    tempColorMin = euclidean(pos, target) - closerEnemy(target)
                    tempPlatePicked = target[3]
    

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass