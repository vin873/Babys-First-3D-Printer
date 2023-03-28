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
from Eurobot2023_main_small_test.srv import *

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
    for enemy in enemies:
        if enemy != [-1, -1]:
            if min > euclidean(enemy, target) / speed:
                min = euclidean(enemy, target) / speed
    return min

def where2suck(pos, num):
    global tempMin, tempSide, used
    tempMin[num] = 99999
    tempSide[num] = [[-1, -1], [-1, -1]]

    for sides in cherries:
        if (cherries.index(sides) < 4 and cherryE[cherries.index(sides)] != 0) or (cherries.index(sides) == 4 and cherryE[0] != 0) or (cherries.index(sides) == 5 and cherryE[2] != 0):
            for cherrySide in sides:
                if cherrySide not in used:
                    dis2cherry = euclidean(pos, cherrySide) - closerEnemy(cherrySide)
                    if dis2cherry < tempMin[num]:
                        tempMin[num] = dis2cherry
                        tempSide[num] = sides
                        if euclidean(pos, cherrySide) > euclidean(pos, sides[not bool(sides.index(cherrySide))]):
                            cherrySide = sides[not bool(sides.index(cherrySide))]
                        if cherrySide == tempSide[num][1]:
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
    global robotPose, used
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
