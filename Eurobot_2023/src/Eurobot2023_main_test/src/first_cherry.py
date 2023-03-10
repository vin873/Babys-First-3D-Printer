#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.distance import euclidean

cherries = [[(0, 1000), (300, 1000)], [(1350, 1985), (1650, 1985)], [(2700, 1000), (3000, 1000)], [(1350, 15), (1650, 15)]]

# subscribe cherries’ existence
cherry = [1, 1, 1, 1]

# subscribe each enemy's pos
enemies = [(1125, 1775), (1875, 225)]
# subscribe our robots pos
startPos = [(1125, 225), (-1, -1)]

robotPose = PoseStamped()
picked = [(-1, -1), (-1, -1)]

def cherryPublish():
    global robotPose, picked
    pub = rospy.Publisher('/cherry_picked', PoseStamped, queue_size=100)
    for i in picked:
        if i != (-1, -1):
            robotPose.header.frame_id = "/robot" + str(picked.index(i)+1) + "/map"
            robotPose.header.stamp = rospy.Time.now()
            robotPose.pose.position.x = i[0]
            robotPose.pose.position.y = i[1]
            robotPose.pose.orientation.x = 0
            robotPose.pose.orientation.y = 0
            robotPose.pose.orientation.z = 0
            robotPose.pose.orientation.w = 0
            rospy.sleep(0.3)
            pub.publish(robotPose)

def cherry_callback(msg):
    if msg.data:
        publisher()

def where2suck(pos):
    tempCherry = (-1, -1)
    tempMin = 99999
    for sides in cherries:
        for cherrySide in sides:
            dis2cherry = euclidean(pos, cherrySide)
            if dis2cherry < tempMin:
                tempMin = dis2cherry
                tempCherry = cherrySide
    return tempCherry

def listener():
    rospy.init_node("first_cherry")
    rospy.Subscriber("/cherry", Bool, cherry_callback)
    rospy.spin()

def publisher():
    global robotPose
    for robot in startPos:
        if robot != (-1, -1):
            picked[startPos.index(robot)] = where2suck(robot)
    cherryPublish()

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass




