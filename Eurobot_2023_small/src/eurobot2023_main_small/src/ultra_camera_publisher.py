#! /usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

browns = [[1.125, 0.725], [1.125, 1.275], [1.875, 0.725], [1.875, 1.275]]
yellows = [[0.775, 0.225], [0.775, 1.775], [2.225, 0.225], [2.225, 1.775]]
pinks = [[0.575, 0.225], [0.575, 1.775], [2.425, 0.225], [2.425, 1.775]]
allCakes = [browns, yellows, pinks]  

def publisher():
    rospy.init_node("ultra_camera_publisher")
    pub1 = rospy.Publisher('/adjustCake', Pose, queue_size=1000)
    pub2 = rospy.Publisher('/onRobot/relative_where', Pose, queue_size=1000)

    caker = Pose()
    changeNum = 1
    changedCake = [1.125, 1.375]
    caker.position.x, caker.position.y = changedCake[0], changedCake[1]
    caker.position.z = changeNum
    rospy.sleep(0.3)
    pub1.publish(caker)

    caker = Pose()
    changedCake = [0, -0.3]
    caker.position.x, caker.position.y = changedCake[0], changedCake[1]
    caker.position.z = 3
    rospy.sleep(0.5)
    pub2.publish(caker)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass