#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.distance import euclidean

cherries = [[[100, 900], [300, 900]], [[1350, 1885], [1650, 1885]], [[2700, 900], [2900, 900]], [[1350, 115], [1650, 115]]]

# subscribe cherries’ existence
cherry = [1, 1, 1, 1]

# subscribe each enemy's pos
enemies = [[1125, 1775], [1875, 225]]
# subscribe our robots pos
startPos = [[-1, -1],[-1, -1]]

tempMin = [99999, 99999]
tempSide = [[[-1, -1], [-1, -1]], [[-1, -1], [-1, -1]]]
used = []

robotNum = 0
robotPose = PoseArray()
pickedSide = [[[-1, -1], [-1, -1]], [[-1, -1], [-1, -1]]]

def startPos1_callback(msg):
    global startPos, absAng
    startPos[0][0] = msg.pose.pose.position.x * 1000
    startPos[0][1] = msg.pose.pose.position.y * 1000

def startPos2_callback(msg):
    global startPos
    startPos[1][0] = msg.pose.pose.position.x * 1000
    startPos[1][1] = msg.pose.pose.position.y * 1000

def cherryPublish():
    global robotPose, pickedSide
    pub = rospy.Publisher('/cherry_picked' + str(robotNum), PoseArray, queue_size=1000)
    side = pickedSide[robotNum]
    if side != [[-1, -1], [-1, -1]]:
        robotPose.header.frame_id = "/robot" + str(pickedSide.index(side)+1) + "/map"
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

        rospy.sleep(0.3)
        pub.publish(robotPose)

def cherry_callback(msg):
    if msg.data:
        publisher()

def where2suck(pos, num):
    global tempMin, tempSide, used
    tempMin[num] = 99999
    tempSide[num] = [[-1, -1], [-1, -1]]

    for sides in cherries:
        for cherrySide in sides:
            if cherrySide not in used:
                dis2cherry = euclidean(pos, cherrySide)
                if dis2cherry < tempMin[num]:
                    tempMin[num] = dis2cherry
                    tempSide[num] = sides
                    if cherrySide == tempSide[num][1]:
                        tempSide[num][0], tempSide[num][1] = tempSide[num][1], tempSide[num][0]
    return tempSide[num]

def listener():
    rospy.init_node("first_cherry")
    rospy.Subscriber("/cherry" + str(robotNum), Bool, cherry_callback)
    rospy.Subscriber("/cherryExistence", Bool, cherry_callback)
    rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
    rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
    rospy.spin()

def publisher():
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
        
    cherryPublish()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
