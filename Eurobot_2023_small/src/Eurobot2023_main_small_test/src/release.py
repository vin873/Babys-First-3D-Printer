#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray
import numpy as np
from scipy.spatial.distance import euclidean
from geometry_msgs.msg import Quaternion
import math

point = [[0.225, 1.775], [3.775, 0.225]]

# subscribe each enemy's pos
enemies = [[1125, 1775], [1875, 225]]
# subscribe our robots pos
startPos = [[-1, -1],[-1, -1]]

robotNum = 1
absAng = [0, 0]
fullness = [0, 0, 0, 0]

ang = 0
robotAng = Quaternion()
robotPose = PoseStamped()

def mission_callback(msg):
    for i in range(4):
        fullness[i] = msg.data[i+1]

def release_callback(msg):
    if msg.data:
        publisher()

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

def startPos1_callback(msg):
    global startPos, absAng
    startPos[0][0] = msg.pose.pose.position.x * 1000
    startPos[0][1] = msg.pose.pose.position.y * 1000
    absAng[0] = quaternion2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

def startPos2_callback(msg):
    global startPos
    startPos[1][0] = msg.pose.pose.position.x * 1000
    startPos[1][1] = msg.pose.pose.position.y * 1000
    absAng[1] = quaternion2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

def listener():
    rospy.init_node("release")
    rospy.Subscriber("/release"+str(robotNum), Bool, release_callback)
    rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
    rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
    rospy.Subscriber("/donefullness"+str(robotNum), Int32MultiArray, mission_callback)
    rospy.spin()

def publisher():
    pub = rospy.Publisher('/robot'+str(robotNum+1)+"/nav_goal", PoseStamped, queue_size=1000)
    for i in range(4):
        if fullness[i] == 0:
            ang = (225 - i*90) * math.pi / 180
            break
    robotAng = euler2quaternion(0, 0, ang)
    robotPose.header.frame_id = "/robot" + str(robotNum+1) + "/map"
    robotPose.header.stamp = rospy.Time.now()
    robotPose.pose.position.x = point[robotNum][0] * 0.001
    robotPose.pose.position.y = point[robotNum][1] * 0.001
    robotPose.pose.orientation.x = robotAng.x
    robotPose.pose.orientation.y = robotAng.y
    robotPose.pose.orientation.z = robotAng.z
    robotPose.pose.orientation.w = robotAng.w

    pub.publish(robotPose)
    

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass
