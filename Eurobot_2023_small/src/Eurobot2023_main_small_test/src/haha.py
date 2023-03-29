#! /usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

robot = ''
finished = True

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

def finish1_callback(msg):
    global finished, robot
    if robot == '1':
        finished = msg.data
        if finished:
            print("Arrived !\n")
            print("======================================== \n")

def finish2_callback(msg):
    global finished, robot
    if robot == '2':
        finished = msg.data
        if finished:
            print("Arrived !\n")
            print("======================================== \n")

def finish3_callback(msg):
    global finished, robot
    if robot == '3':
        finished = msg.data
        if finished:
            print("Arrived !\n")
            print("======================================== \n")

def finish4_callback(msg):
    global finished, robot
    if robot == '4':
        finished = msg.data
        if finished:
            print("Arrived !\n")
            print("======================================== \n")

def publisher():
    global finished, robot
    rospy.init_node("hahap")
    rospy.Subscriber('/robot1/is_finish', Bool, finish1_callback)
    rospy.Subscriber('/robot2/is_finish', Bool, finish2_callback)
    rospy.Subscriber('/rival1/is_finish', Bool, finish3_callback)
    rospy.Subscriber('/rival2/is_finish', Bool, finish4_callback)

    robotPose = PoseStamped()

    while not rospy.is_shutdown():
        if finished:
            mode = input("mode : ")
            if mode == '-1':
                print("Shut down !!")
                break
            if mode == 'p' or mode == 'P':
                robotPose.header.frame_id = 'path'
            elif mode == 'd' or mode == 'D':
                robotPose.header.frame_id = 'dock'
            else:
                print("Input error !!\n")
                print("======================================== \n")
                continue

            robotPose.header.stamp = rospy.Time.now()
            robot = input("robot : ")
            
            if robot == '1' or robot == '2':
                pub = rospy.Publisher('/robot'+str(robot)+'/mission', PoseStamped, queue_size = 100)
            elif robot == '3' or robot == '4':
                pub = rospy.Publisher('/rival'+str(int(robot)-2)+'/mission', PoseStamped, queue_size = 100)
            else:
                print("Input error !!\n")
                print("======================================== \n")
                continue

            robotPose.pose.position.x = float(input("x : "))
            robotPose.pose.position.y = float(input("y : "))
            if mode != 'D' and mode != 'd':
                ang = euler2quaternion(0, 0, float(input("ang : ")) * math.pi / 180)
                robotPose.pose.orientation.x = ang.x
                robotPose.pose.orientation.y = ang.y
                robotPose.pose.orientation.z = ang.z
                robotPose.pose.orientation.w = ang.w
            pub.publish(robotPose)
            print("Target Published !!\n")
            finished = False
        rospy.sleep(1)

if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass