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

point = [[[[2.800, 1.800, 135], [2.625, 1.625, 135], [2.725, 1.725, 265], [2.575, 1.575, 265], 4]  # B4
        , [[2.800, 0.750, 135], [2.625, 0.575, 135], [2.725, 0.675, 265], [2.575, 0.475, 265], 3]  # B3
        , [[1.850, 0.200, 315], [2.025, 0.375, 315], [1.900, 0.250, 85], [2.075, 0.425, 85], 2]    # B2
        , [[1.150, 1.800, 315], [0.975, 1.625, 315], [1.075, 1.725, 85], [0.925, 1.575, 85], 1]    # B1
        , [[0.200, 0.200, 315], [0.375, 0.375, 315], [0.250, 0.250, 85], [0.425, 0.425, 85], 0]]   # B0
        
        ,[[[2.800, 0.200, 225], [2.625, 0.375, 225], [2.725, 0.275, 85], [2.575, 0.425, 85], 4]    # G4
        , [[2.800, 1.250, 225], [2.625, 1.425, 225], [2.725, 1.325, 85], [2.575, 1.525, 85], 3]    # G3
        , [[1.850, 1.800, 225], [2.025, 1.625, 225], [1.900, 1.750, 85], [2.075, 1.575, 85], 2]    # G2
        , [[1.150, 0.200, 225], [0.975, 0.375, 225], [1.075, 0.275, 85], [0.925, 0.425, 85], 1]    # G1
        , [[0.200, 1.800, 45], [0.375, 1.625, 45], [0.250, 1.725, 265], [0.425, 1.575, 265], 0]]]  # G0
        

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
    global robotPose, ang

    if num == 0:
        sideNum = 4
    else:
        sideNum = num

    empty = -1

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
    robotPose.header.frame_id += str(point[side][sideNum][4])
    robotPose.header.stamp = rospy.Time.now()

    for i in range(4):
        
        ang = (point[side][sideNum][i][2] - empty*90 + headAng) * math.pi / 180
        robotAng = euler2quaternion(0, 0, ang)
    
        pose = Pose()

        pose.position.x = point[side][sideNum][i][0]
        pose.position.y = point[side][sideNum][i][1]
        pose.orientation.x = robotAng.x
        pose.orientation.y = robotAng.y
        pose.orientation.z = robotAng.z
        pose.orientation.w = robotAng.w
        robotPose.poses.append(pose)
        # print((point[side][sideNum][i][2] - empty*90+headAng))

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
