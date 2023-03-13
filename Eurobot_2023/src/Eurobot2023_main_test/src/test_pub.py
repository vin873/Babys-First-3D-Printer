#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
import numpy as np

done = Int32MultiArray()

def publisher():
    rospy.init_node("test_pub")
    pub = rospy.Publisher('/donefullness', Int32MultiArray, queue_size=1000)
    arr = [1, 1 , 1, 1, 1]
    done.data = arr
    rospy.sleep(0.6)
    pub.publish(done)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass