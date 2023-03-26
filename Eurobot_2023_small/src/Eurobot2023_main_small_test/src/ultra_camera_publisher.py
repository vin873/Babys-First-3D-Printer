#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

browns = [[1.125, 0.725], [1.125, 1.275], [1.875, 0.725], [1.875, 1.275]]
yellows = [[0.775, 0.225], [0.775, 1.775], [2.225, 0.225], [2.225, 1.775]]
pinks = [[0.575, 0.225], [0.575, 1.775], [2.425, 0.225], [2.425, 1.775]]
allCakes = [browns, yellows, pinks]  

def publisher():
    rospy.init_node("ultra_camera_publisher")
    pub = rospy.Publisher('/allCakes', PoseArray, queue_size=1000)
    caker = PoseArray()
    caker.header.stamp = rospy.Time.now()
    for color in allCakes:
        caker.header.frame_id += str(len(color))
        for cakee in color:
            pose = Pose()
            pose.position.x = cakee[0]
            pose.position.y = cakee[1]
            caker.poses.append(pose)
    rospy.sleep(0.3)
    pub.publish(caker)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass