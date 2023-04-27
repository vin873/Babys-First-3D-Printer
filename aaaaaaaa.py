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
            if missionStr.data == 'reset':
                for i in range(4):
                    missionStr.data = 'o' + str(i)
                    rospy.sleep(0.3)
                    pub.publish(missionStr)
                rospy.sleep(1)
                for i in range(4):
                    missionStr.data = 'c' + str(i)
                    rospy.sleep(0.3)
                    pub.publish(missionStr)
            
            elif missionStr.data == 'open':
                for i in range(4):
                    missionStr.data = 'o' + str(i)
                    rospy.sleep(0.3)
                    pub.publish(missionStr)
            
            elif missionStr.data == 'yump':
                for i in range(4):
                    missionStr.data = 'c' + str(i)
                    rospy.sleep(0.3)
                    pub.publish(missionStr)

            elif (missionStr.data[0] == 'b' or missionStr.data[0] == 'y' or missionStr.data[0] == 'p') and len(missionStr.data) == 6:
                ms = missionStr.data
                for i in range(3):
                    missionStr.data = ms[2*i] + ms[2*i+1]
                    rospy.sleep(0.3)
                    pub.publish(missionStr)
                rospy.sleep(1)
                for i in range(3):
                    missionStr.data = 'c' + ms[2*i+1]
                    rospy.sleep(0.3)
                    pub.publish(missionStr)

            elif missionStr.data[0] != 'b' and missionStr.data[0] != 'y' and missionStr.data[0] != 'p' and missionStr.data[0] != 'c' and missionStr.data[0] != 'o' and missionStr.data[0] != 'h' and missionStr.data[0] != 's' and missionStr.data[0] != 'v' and missionStr.data[0] != 'u' and missionStr.data[0] != 'd' and missionStr.data[0] != 'f':
                print("Input error !!\n")
                print("======================================== \n")
                continue
            else:
                pub.publish(missionStr)
                
            print("Mission Published !!\n")
            print("======================================== \n")

        rospy.sleep(1)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass