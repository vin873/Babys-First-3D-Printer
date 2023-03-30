#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.distance import euclidean
from Eurobot2023_main_test.srv import *

cherries = [[[100, 900], [300, 900]], [[1350, 1885], [1650, 1885]], [[2700, 900], [2900, 900]], [[1350, 115], [1650, 115]], [[100, 1100], [300, 1100]], [[2700, 1100], [2900, 1100]]]

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
robotPose = PoseArray()
pickedSide = [[[-1, -1], [-1, -1]], [[-1, -1], [-1, -1]]]

def handle_cherry(req):
    publisher()
    return cherryResponse(robotPose)

def startPos1_callback(msg):
    global startPos, absAng
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

def cherryPublish():
    global robotPose, pickedSide
    side = pickedSide[robotNum]
    if side != [[-1, -1], [-1, -1]]:
        num = '-'
        for i in cherries:
            if side[0] in i:
                if cherries.index(i) < 4:
                    num = str(cherries.index(i))
                elif cherries.index(i) == 4:
                    num = '0'
                elif cherries.index(i) == 5:
                    num = '2'
        robotPose.header.frame_id = num
        robotPose.header.stamp = rospy.Time.now()

        for i in side:
            pose = Pose()
            pose.position.x = i[0] * 0.001
            pose.position.y = i[1] * 0.001
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            robotPose.poses.append(pose)

def cherry_callback(msg):
    if msg.data:
        publisher()

def cherryE_callback(msg):
    # print(cherry)
    for i in range(4):
        cherryE[i] = msg.data[i]

def closerEnemy(target):
    global enemies
    speed = 0.9  # enemy/our
    min = 99999
    side = [-1, -1]
    for enemy in enemies:
        if enemy != [-1, -1]:
            for i in target:
                if min > euclidean(enemy, i) / speed:
                    min = euclidean(enemy, i) / speed
                    side = list(i)
    return min, side

def where2suck(pos, num):
    global tempMin, tempSide, used
    tempMin[num] = 99999
    tempChoice = [-1, -1]
    enemyChoice = [-1, -1]
    tempSide[num] = [[-1, -1], [-1, -1]]

    for sides in range(4):
        if cherryE[sides] != 0 and cherries[sides][0] not in used:
            if sides == 0 or sides == 4:
                temp = list(cherries[0]) + list(cherries[4])
                enenmyDis, enemyChoice = closerEnemy(temp)
            elif side == 2 or side == 5:
                temp = list(cherries[2]) + list(cherries[5])
                enenmyDis, enemyChoice = closerEnemy(temp)
            else:
                enenmyDis, enemyChoice = closerEnemy(cherries[sides])
            if euclidean(pos, enemyChoice) - enenmyDis < tempMin[num]:
                    tempMin[num] = euclidean(pos, enemyChoice) - enenmyDis
                    tempChoice = list(enemyChoice)
                    tempSide[num] = list(cherries[sides])
    # print(num, tempChoice, tempSide[num])
    tM = 99999
    for sides in range(6):
        if tempChoice in cherries[sides]:
            if sides == 0:
                for j in range(2):
                    if tM > euclidean(pos, cherries[4][j]):
                        tM = euclidean(pos, cherries[4][j])
                        tempChoice = list(cherries[4][j])
                        tempSide[num] = list(cherries[4])
            elif sides == 2:
                for j in range(2):
                    if tM > euclidean(pos, cherries[5][j]):
                        tM = euclidean(pos, cherries[5][j])
                        tempChoice = list(cherries[5][j])
                        tempSide[num] = list(cherries[5])
            elif sides == 4:
                for j in range(2):
                    if tM > euclidean(pos, cherries[0][j]):
                        tM = euclidean(pos, cherries[0][j])
                        tempChoice = list(cherries[0][j])
                        tempSide[num] = list(cherries[0])
            elif sides == 5:
                for j in range(2):
                    if tM > euclidean(pos, cherries[2][j]):
                        tM = euclidean(pos, cherries[2][j])
                        tempChoice = list(cherries[2][j])
                        tempSide[num] = list(cherries[2])
            for i in cherries[sides]:
                if tM > euclidean(pos, i):
                    tM = euclidean(pos, i)
                    tempChoice = list(i)
                    tempSide[num] = list(cherries[sides])
    # print(tempChoice, tempSide)
    if tempChoice == tempSide[num][1]:
            tempSide[num][0], tempSide[num][1] = tempSide[num][1], tempSide[num][0]

    return tempSide[num]

def listener():
    global side, robotNum
    rospy.init_node("first_cherry")
    side = rospy.get_param('side')
    robotNum = rospy.get_param('robot')
    rospy.Service('cherry'+str(robotNum), cherry, handle_cherry)
    rospy.Subscriber("/cherryExistence", Int32MultiArray, cherryE_callback)
    rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
    rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
    rospy.Subscriber("/rival1/odom", Odometry, enemiesPos1_callback)
    rospy.Subscriber("/rival2/odom", Odometry, enemiesPos2_callback)
    rospy.spin()

def publisher():
    # print(cherryE)
    global robotPose, used, pickedSide
    for robot in startPos:
        if robot != [-1, -1]:
            pickedSide[startPos.index(robot)] = where2suck(robot, startPos.index(robot))

    if tempMin[0] < tempMin[1]:
        used = pickedSide[0]
        if startPos[1] != [-1, -1]:
            pickedSide[1] = [[-1, -1], [-1, -1]]
            tempMin[1] = 99999
            pickedSide[1] = where2suck(startPos[1], 1)
    else:
        used = pickedSide[1]
        if startPos[0] != [-1, -1]:
            pickedSide[0] = [[-1, -1], [-1, -1]]
            tempMin[0] = 99999
            pickedSide[0] = where2suck(startPos[0], 0)
    
    robotPose.poses = []
    cherryPublish()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
