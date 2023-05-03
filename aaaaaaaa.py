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

missionStr = String()

deliveredAr = False
waitAr = False
deliveredSTM = False
waitSTM = False
pub = rospy.Publisher('/mission0', String, queue_size = 100)

def pub_till_get():
    global deliveredAr, waitAr, deliveredSTM, waitSTM
    print("Mission published!")
    waitAr = True
    if missionStr.data[0] == 'b' or missionStr.data[0] == 'y' or missionStr.data[0] == 'p' or missionStr.data[0] == 'h':
        waitSTM = True
    while (not deliveredAr or not deliveredSTM) and not rospy.is_shutdown():
        if missionStr.data[0] != 'b' and missionStr.data[0] != 'y' and missionStr.data[0] != 'p' and missionStr.data[0] != 'h' and deliveredAr:
            break
        rospy.sleep(0.3)
        pub.publish(missionStr)
    waitAr = False
    waitSTM = False
    deliveredAr = False
    deliveredSTM = False
    print("Mission delivered!")

def handshakerAr0_callback(msg):
    global deliveredAr
    if robot == '1':
        if waitAr and msg.data != "" and msg.data[0] == missionStr.data[0] and msg.data[1] == missionStr.data[1]:
            deliveredAr = True

def handshakerAr1_callback(msg):
    global deliveredAr
    if robot == '2':
        if waitAr and msg.data != "" and msg.data[0] == missionStr.data[0] and msg.data[1] == missionStr.data[1]:
            deliveredAr = True

def handshakerSTM0_callback(msg):
    global deliveredSTM
    if robot == '1':
        if waitSTM and msg.data != "" and msg.data[0] == missionStr.data[0] and msg.data[1] == missionStr.data[1]:
            deliveredSTM = True

def handshakerSTM1_callback(msg):
    global deliveredSTM
    if robot == '2':
        if waitSTM and msg.data != "" and msg.data[0] == missionStr.data[0] and msg.data[1] == missionStr.data[1]:
            deliveredSTM = True

def publisher():
    global finished, robot, pub, missionStr
    rospy.init_node("aaaaaaaa")
    rospy.Subscriber('handshaker0', String, handshakerAr0_callback)
    rospy.Subscriber('handshaker1', String, handshakerAr1_callback)
    rospy.Subscriber('handshakier0', String, handshakerSTM0_callback)
    rospy.Subscriber('handshakier1', String, handshakerSTM1_callback)

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
                    pub_till_get()
                rospy.sleep(1)
                for i in range(4):
                    missionStr.data = 'c' + str(i)
                    pub_till_get()
            
            elif missionStr.data == 'open':
                for i in range(4):
                    missionStr.data = 'o' + str(i)
                    pub_till_get()
            
            elif missionStr.data == 'yump':
                for i in range(4):
                    missionStr.data = 'c' + str(i)
                    pub_till_get()

            elif (missionStr.data[0] == 'b' or missionStr.data[0] == 'y' or missionStr.data[0] == 'p') and len(missionStr.data) == 6:
                ms = missionStr.data
                for i in range(3):
                    missionStr.data = ms[2*i] + ms[2*i+1]
                    pub_till_get()
                rospy.sleep(1)
                for i in range(3):
                    missionStr.data = 'c' + ms[2*i+1]
                    pub_till_get()

            elif missionStr.data[0] != 'b' and missionStr.data[0] != 'y' and missionStr.data[0] != 'p' and missionStr.data[0] != 'c' and missionStr.data[0] != 'o' and missionStr.data[0] != 'h' and missionStr.data[0] != 's' and missionStr.data[0] != 'v' and missionStr.data[0] != 'u' and missionStr.data[0] != 'd' and missionStr.data[0] != 'f':
                print("Input error !!\n")
                print("======================================== \n")
                continue
            else:
                pub_till_get()
                
            print("======================================== \n")

        rospy.sleep(1)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass