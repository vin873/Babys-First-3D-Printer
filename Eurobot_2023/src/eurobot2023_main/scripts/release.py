#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.distance import euclidean
from geometry_msgs.msg import Quaternion
import math
from eurobot2023_main.srv import *

point = [[[[2.800, 1.800, 135], [2.625, 1.625, 135], [2.725, 1.725, 270], [2.575, 1.575, 270]] # B4
        , [[0.225, 0.225, 315], [0.375, 0.375, 315], [0.225, 0.225, 90], [0.375, 0.375, 90]]]  # B0
        ,[[[2.775, 0.175, 225], [2.625, 0.325, 225], [2.775, 0.175, 90], [2.625, 0.325, 90]]   # G3
        , [[0.225, 1.25, 45], [0.325, 1.625, 45], [0.225, 1.225, 270], [0.375, 1.625, 270]]]] # G0

robotNum = 0
side = 0
absAng = [0, 0]
fullness = [0, 0, 0, 0]
headAng = -45

ang = 0
robotAng = Quaternion()
robotPose = PoseArray()

def handle_release(req):
    publisher(req.num)
    return releaseResponse(robotPose)

def mission_callback(msg):
    global fullness
    for i in range(4):
        fullness[i] = msg.data[i+1]

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

def quaternion2euler(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z / math.pi * 180 # in radians

def listener():
    global side, robotNum
    rospy.init_node("release")
    side = rospy.get_param('side')
    robotNum = rospy.get_param('robot')
    rospy.Service('release'+str(robotNum), release, handle_release)
    rospy.Subscriber("/donefullness"+str(robotNum), Int16MultiArray, mission_callback)
    rospy.spin()

def publisher(num):
    global robotPose

    for i in range(4):
        if fullness[i] == 0:
            empty = i
            break
    # print(empty)
    if empty == 0:
        robotPose.header.frame_id = '31'
    elif empty == 1:
         robotPose.header.frame_id = '02'
    elif empty == 2:
         robotPose.header.frame_id = '13'
    elif empty == 3:
         robotPose.header.frame_id = '20'
    robotPose.header.stamp = rospy.Time.now()

    for i in range(4):
        
        ang = (point[side][num][i][2] - empty*90 + headAng) * math.pi / 180
        robotAng = euler2quaternion(0, 0, ang)
    
        pose = Pose()
        pose.position.x = point[side][num][i][0]
        pose.position.y = point[side][num][i][1]
        pose.orientation.x = robotAng.x
        pose.orientation.y = robotAng.y
        pose.orientation.z = robotAng.z
        pose.orientation.w = robotAng.w
        robotPose.poses.append(pose)
        # print((point[side][num][i][2] - empty*90+headAng))

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
