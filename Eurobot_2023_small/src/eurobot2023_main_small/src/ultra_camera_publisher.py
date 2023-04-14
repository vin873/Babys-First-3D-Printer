#! /usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

browns = [[1.125, 0.725], [1.125, 1.275], [1.875, 0.725], [1.875, 1.275]]
yellows = [[0.775, 0.225], [0.775, 1.775], [2.225, 0.225], [2.225, 1.775]]
pinks = [[0.575, 0.225], [0.575, 1.775], [2.425, 0.225], [2.425, 1.775]]
allCakes = [browns, yellows, pinks]  

def cam_callback(msg):
    pub2 = rospy.Publisher('/onRobot/relative_where', Point, queue_size=1000)
    caker = Point()
    changedCake = [0.5, 0]
    caker.x, caker.y = changedCake[0], changedCake[1]
    caker.z = -1
    # print(caker)
    rospy.sleep(0.5)
    pub2.publish(caker)

    rospy.sleep(2)
    caker.z = -2
    rospy.sleep(0.5)
    pub2.publish(caker)

    rospy.sleep(0.5)
    caker.z = 3
    # print(caker)
    rospy.sleep(0.3)
    pub2.publish(caker)
    rospy.sleep(10)

def listener():
    rospy.Subscriber('cam_which_color', Int32, cam_callback)
    rospy.spin()

def publisher():
    rospy.init_node("ultra_camera_publisher")
    pub1 = rospy.Publisher('/adjustCake', Point, queue_size=1000)

    caker = Point()
    changeNum = 2
    # changedCake = [-0.001, -0.001]
    changedCake = [1.875, 0.925]
    caker.x, caker.y = changedCake[0], changedCake[1]
    caker.z = changeNum
    rospy.sleep(0.3)
    pub1.publish(caker)

if __name__=="__main__":
    try:
        publisher()
        listener()

    except rospy.ROSInterruptException:
        pass