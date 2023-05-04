#! /usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

robot = ''
start = False
finished = -1

missionStr = String()

deliveredAr = False
waitAr = False
deliveredSTM = False
waitSTM = False
pub = rospy.Publisher('/mission0', String, queue_size = 100)

robotE = [1, 0]

def euler2quaternion(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return Quaternion(qx, qy, qz, qw)

def pub_till_get():
    global deliveredAr, waitAr, deliveredSTM, waitSTM
    rospy.sleep(0.3)
    pub.publish(missionStr)
    rospy.loginfo("Mission published!")
    # waitAr = True
    # if missionStr.data[0] == 'b' or missionStr.data[0] == 'y' or missionStr.data[0] == 'p' or missionStr.data[0] == 'h':
    #     waitSTM = True
    # while (not deliveredAr or not deliveredSTM) and not rospy.is_shutdown():
    #     if missionStr.data[0] != 'b' and missionStr.data[0] != 'y' and missionStr.data[0] != 'p' and missionStr.data[0] != 'h' and deliveredAr:
    #         break
    #     rospy.sleep(0.3)
    #     pub.publish(missionStr)
    # waitAr = False
    # waitSTM = False
    # deliveredAr = False
    # deliveredSTM = False
    # rospy.loginfo("Mission delivered!")

def finish1_callback(msg):
    global finished, robot
    if robot == 1:
        finished = msg.data
        if finished == 1:
            print("Arrived !\n")
            print("======================================== \n")
        elif finished == 0:
            print("Failed !\n")
            print("======================================== \n")

def finish2_callback(msg):
    global finished, robot
    if robot == 2:
        finished = int(msg.data)
        if finished == 1:
            print("Arrived !\n")
            print("======================================== \n")
        elif finished == 0:
            print("Failed !\n")
            print("======================================== \n")

def start_callback(msg):
    global start
    start = msg.data

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

def initial():
    rospy.init_node("jenlu")
    rospy.Subscriber('/startornot', Bool, start_callback)
    rospy.Subscriber('/robot1/is_finish', Bool, finish1_callback)
    rospy.Subscriber('/robot2/is_finish', Bool, finish2_callback)
    rospy.Subscriber('handshaker0', String, handshakerAr0_callback)
    rospy.Subscriber('handshaker1', String, handshakerAr1_callback)
    rospy.Subscriber('handshakier0', String, handshakerSTM0_callback)
    rospy.Subscriber('handshakier1', String, handshakerSTM1_callback)

def point(m, robotNum, x, y, angle):
    global finished, robot

    if not rospy.is_shutdown():
        robotPose = PoseStamped()

        mode = m
        if mode == 'p' or mode == 'P':
            robotPose.header.frame_id = 'path'
        elif mode == 'd' or mode == 'D':
            robotPose.header.frame_id = 'dock_mov_cake'
        elif mode == 'c' or mode == 'C':
            robotPose.header.frame_id = 'dock_mov_cherry'
        elif mode == 'r' or mode == 'R':
            robotPose.header.frame_id = 'dock_rot'
        elif mode == 'v' or mode == 'V':
            robotPose.header.frame_id = 'dock_vibrate'

        robotPose.header.stamp = rospy.Time.now()
        robot = robotNum

        if robotE[robot-1] == 1:
            if robot == 1 or robot == 2:
                pub = rospy.Publisher('/robot'+str(robot)+'/mission', PoseStamped, queue_size = 100)
            elif robot == 3 or robot == 4:
                pub = rospy.Publisher('/rival'+str(int(robot)-2)+'/mission', PoseStamped, queue_size = 100)

            if mode != 'v' and mode != 'V':
                if mode != 'r' and mode != 'R':
                    robotPose.pose.position.x = float(x)
                    robotPose.pose.position.y = float(y)
                if mode != 'D' and mode != 'd':
                    ang = euler2quaternion(0, 0, float(angle) * math.pi / 180)
                    robotPose.pose.orientation.x = ang.x
                    robotPose.pose.orientation.y = ang.y
                    robotPose.pose.orientation.z = ang.z
                    robotPose.pose.orientation.w = ang.w
            rospy.sleep(0.3)
            pub.publish(robotPose)
            if mode != 'v' and mode != 'V':
                if mode != 'r' and mode != 'R':
                    if mode != 'D' and mode != 'd':
                        print("Heading over to x : [", robotPose.pose.position.x, "] y : [", robotPose.pose.position.x, "] ang : [", angle, "]")
                    else:
                        print("Heading over to x :", robotPose.pose.position.x, "y :", robotPose.pose.position.x)
                else:
                    print("Heading over to ang :", angle)
            else:
                print("Target Published !!\n")
            finished = -1

            while not rospy.is_shutdown():
                if finished == 1 or finished == 0:
                    break

def mission(robotNum, m):
    global finished, robot, missionStr

    if not rospy.is_shutdown():
        missionStr = String()
        
        robot = robotNum
        
        if robotE[robot-1] == 1:
            if robot == 1 or robot == 2:
                pub = rospy.Publisher('/mission'+str(int(robot)-1), String, queue_size = 100)

            missionStr.data = m
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
            else:
                pub_till_get()
                
            print("======================================== \n")

def test():
    while not rospy.is_shutdown():
        if start:
            break
    # t = rospy.get_rostime().to_sec()
    # mission(1, 'y1')
    # mission(2, 'y3')
    # point('p', 1, 1.5, 1, 45)
    # point('p', 1, 1.125, 0.225, 45)
    # point('d', 1, 0.87, 0.225, 45)
    # point('d', 2, 2.13, 0.225, 45)
    # mission(1, 'c1')
    # mission(2, 'c3')
    # rospy.sleep(1)
    # mission(1, 'p0')
    # mission(2, 'p2')
    # point('p', 1, 0.87, 0.225, 135)
    # point('p', 2, 2.13, 0.225, 135)
    # point('d', 1, 0.67, 0.225, 135)
    # point('d', 2, 2.33, 0.225, 135)
    # mission(1, 'c0')
    # mission(2, 'c2')
    # rospy.sleep(1)
    # point('p', 1, 0.225, 0.225, 135)
    # point('p', 2, 2.775, 0.725, 135)
    # mission(1, 'o0')
    # mission(1, 'o1')
    # mission(2, 'o2')
    # mission(2, 'o3')
    # point('d', 1, 0.45, 0.45, 135)
    # point('d', 2, 2.55, 0.5, 135)
    # mission(1, 'c0')
    # mission(1, 'c1')
    # mission(2, 'o2')
    # mission(2, 'o3')
    # rospy.sleep(1)
    # point('p', 1, 1.65, 0.225, 135)
    # point('p', 2, 2.1, 0.225, 135)
    # mission(1, 'f0')
    # mission(2, 'f0')
    # print("Total time : %.1f"%(rospy.get_rostime().to_sec()-t))

    point('p', 1, 1.6, 0.185, 90)
    mission(1, 's3')
    point('c', 1, 1.25, 0.185, 90)
    mission(1, 'v0')
    rospy.sleep(2)
    point('p', 1, 0.4, 0.84, 270)
    mission(1, 's0')
    point('c', 1, 0.13, 0.84, 270)
    mission(1, 'v0')
    rospy.sleep(2)
    point('p', 1, 0.225, 0.225, 180)
    point('d', 1, 0.15, 0.225, 180)
    point('v', 1, 0.15, 0.225, 180)

if __name__=="__main__":
    try:
        initial()
        test()

    except rospy.ROSInterruptException:
        pass