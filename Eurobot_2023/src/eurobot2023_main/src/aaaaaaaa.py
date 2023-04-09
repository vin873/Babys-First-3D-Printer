#! /usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

robot = ''
finished = True

def mission0_feedback(msg):
    global finished, robot
    if robot == '1':
        finished = msg.data[0]
        if finished:
            print("Mission finished !\n")
            print("======================================== \n")

def mission1_feedback(msg):
    global finished, robot
    if robot == '2':
        finished = msg.data[0]
        if finished:
            print("Mission finished !\n")
            print("======================================== \n")

def publisher():
    global finished, robot
    rospy.init_node("aaaaaaaa")
    # rospy.Subscriber('/donefullness0', Int32MultiArray, mission0_feedback)
    # rospy.Subscriber('/donefullness1', Int32MultiArray, mission1_feedback)

    missionStr = String()

    while not rospy.is_shutdown():
        if finished:
            robot = input("robot : ")
            if robot == '-1':
                print("Shut down !!")
                break
            elif robot == '1' or robot == '2':
                pub = rospy.Publisher('/mission'+str(int(robot)-1), String, queue_size = 100)
            else:
                print("Input error !!\n")
                print("======================================== \n")
                continue
            missionStr.data = input("mission : ")
            if missionStr.data[0] != 'b' and missionStr.data[0] != 'y' and missionStr.data[0] != 'p' and missionStr.data[0] != 'c' and missionStr.data[0] != 'o' and missionStr.data[0] != 'h' and missionStr.data[0] != 's' and missionStr.data[0] != 'v' and missionStr.data[0] != 'u' and missionStr.data[0] != 'd' and missionStr.data[0] != 'f':
                print("Input error !!\n")
                print("======================================== \n")
                continue
            elif missionStr.data == 'reset':
                missionStr.data == 'o0'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                missionStr.data == 'o1'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                missionStr.data == 'o2'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                missionStr.data == 'o3'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                rospy.sleep(1)
                missionStr.data == 'c0'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                missionStr.data == 'c1'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                missionStr.data == 'c2'
                rospy.sleep(0.3)
                pub.publish(missionStr)
                missionStr.data == 'c3'
                rospy.sleep(0.3)
                pub.publish(missionStr)
            else:
                pub.publish(missionStr)
                print("Mission Published !!\n")
                # finished = False
                print("======================================== \n")

        rospy.sleep(1)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass