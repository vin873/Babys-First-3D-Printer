#! /usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.distance import euclidean
from obstacle_detector.msg import Obstacles
from eurobot2023_main_small.srv import *

cherries = [[[160, 820], [440, 820]], [[1370, 1815], [1800, 1815]], [[2530, 835], [2830, 835]], [[1150, 181], [1600, 181]], [[172, 1160], [480, 1160]], [[2600, 1170], [2840, 1170]]]

# subscribe cherries’ existence
cherryE = [1, 1, 1, 1]

# subscribe each enemy's pos
enemies = [[-1, -1],[-1, -1]]
# subscribe our robots pos
startPos = [[-1, -1],[-1, -1]]

tempMin = [99999, 99999]
tempSide = [[[-1, -1], [-1, -1]], [[-1, -1], [-1, -1]]]
used = []

robotNum = 0
side = 0
run_mode = ''
robotPose = PoseArray()
pickedSide = [[[-1, -1], [-1, -1]], [[-1, -1], [-1, -1]]]

def handle_cherry(req):
    publisher()
    return cherryResponse(robotPose)

def startPos1_callback(msg):
    global startPos
    startPos[0][0] = msg.pose.pose.position.x * 1000
    startPos[0][1] = msg.pose.pose.position.y * 1000

def startPos2_callback(msg):
    global startPos
    startPos[1][0] = msg.pose.pose.position.x * 1000
    startPos[1][1] = msg.pose.pose.position.y * 1000

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

def cherryPublish():
    global robotPose, pickedSide
    cside = pickedSide[robotNum]
    # if side != [[-1, -1], [-1, -1]]:
    num = '-'
    for i in cherries:
        if cside[0] in i:
            if cherries.index(i) < 4:
                num = str(cherries.index(i))
            elif cherries.index(i) == 4:
                num = '4'
            elif cherries.index(i) == 5:
                num = '5'
    robotPose.header.frame_id = num
    robotPose.header.stamp = rospy.Time.now()

    if num == '0' or num == '1':
        cAng = 274
    elif num == '2':
        cAng = 180
    elif num == '3' or num == '5':
        cAng = 90
    elif num == '4':
        cAng = 0
    else:
        cAng = 0

    quat = Quaternion()
    quat = euler2quaternion(0, 0, cAng*math.pi/180)

    for i in cside:
        pose = Pose()
        pose.position.x = i[0] * 0.001
        pose.position.y = i[1] * 0.001
        pose.orientation.x = quat.x
        pose.orientation.y = quat.y
        pose.orientation.z = quat.z
        pose.orientation.w = quat.w
        robotPose.poses.append(pose)

def cherryE_callback(msg):
    global cherryE
    # print(cherry)
    for i in range(4):
        cherryE[i] = msg.data[i]

def euler2quaternion(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return Quaternion(qx, qy, qz, qw)

def closerEnemy(target):
    global enemies
    speed = 0.9  # enemy/our
    min = 99999
    for enemy in enemies:
        if enemy != [-1, -1]:
            for i in target:
                if min > euclidean(enemy, i) / speed:
                    min = euclidean(enemy, i) / speed
    return min

def where2suck(pos, num):
    global tempMin, tempSide, used, cherryE
    tempMin[num] = 99999
    tempChoice = [-1, -1]
    tempSide[num] = [[-1, -1], [-1, -1]]

    for sides in range(6):
        if (sides < 4 and cherryE[sides] != 0) or (sides == 4 and cherryE[0] != 0) or (sides == 5 and cherryE[2] != 0):
            # print(sides)
            for cherrySide in cherries[sides]:
                if sides == 0 or sides == 4:
                    temp = list(cherries[0]) + list(cherries[4])
                elif sides == 2 or sides == 5:
                    temp = list(cherries[2]) + list(cherries[5])
                else:
                    temp = list(cherries[sides])
                if (sides < 4 and used != sides) or (sides == 4 and used != 0) or (sides == 5 and used != 2):
                    dis2cherry = euclidean(pos, cherrySide) - closerEnemy(temp)
                    if dis2cherry > 0 and closerEnemy(temp) <= 300:
                        continue
                    if tempMin[num] > dis2cherry:
                        tempMin[num] = dis2cherry
                        tempSide[num] = list(cherries[sides])
                        tempChoice = list(cherrySide)
    for i in cherries:
        if tempChoice in i:
            if euclidean(pos, tempChoice) > euclidean(pos, i[int(not bool(i.index(tempChoice)))]):
                tempChoice = list(i[int(not bool(i.index(tempChoice)))])
            if tempChoice == tempSide[num][1]:
                tempSide[num][0], tempSide[num][1] = tempSide[num][1], tempSide[num][0]
            break

    return tempSide[num]

def listener():
    global side, robotNum, run_mode
    rospy.init_node("first_cherry")
    side = rospy.get_param('side')
    robotNum = rospy.get_param('robot')
    run_mode = rospy.get_param('run_mode')
    rospy.Service('cherry'+str(robotNum), cherry, handle_cherry)
    rospy.Subscriber('cherryExistence', Int32MultiArray, cherryE_callback)
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

def publisher():
    # print(cherryE)
    global robotPose, used, pickedSide, tempMin

    pickedSideNum = -1

    for robot in startPos:
        if robot != [-1, -1]:
            pickedSide[startPos.index(robot)] = where2suck(robot, startPos.index(robot))

    if tempMin[0] < tempMin[1]:
        for i in cherries:
            if pickedSide[0][0] in i:
                pickedSideNum = cherries.index(i)
                break
        if pickedSideNum < 4:
            used = pickedSideNum
        elif pickedSideNum == 4:
            used = 0
        elif pickedSideNum == 5:
            used = 2
        if startPos[1] != [-1, -1]:
            pickedSide[1] = [[-1, -1], [-1, -1]]
            tempMin[1] = 99999
            pickedSide[1] = where2suck(startPos[1], 1)
    else:
        for i in cherries:
            if pickedSide[1][0] in i:
                pickedSideNum = cherries.index(i)
                break
        if pickedSideNum < 4:
            used = pickedSideNum
        elif pickedSideNum == 4:
            used = 0
        elif pickedSideNum == 5:
            used = 2
        if startPos[0] != [-1, -1]:
            pickedSide[0] = [[-1, -1], [-1, -1]]
            tempMin[0] = 99999
            pickedSide[0] = where2suck(startPos[0], 0)
    
    robotPose.poses = []
    print(pickedSide, cherryE)
    cherryPublish()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
