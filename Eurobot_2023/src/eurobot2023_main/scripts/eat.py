#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from scipy.spatial.distance import euclidean
from itertools import permutations
from eurobot2023_main.srv import *

# subscribe our robots pos
startPos = [-1, -1]
absAng = 0

fullness = [0, 0, 0, 0]
tempFull = [0, 0, 0, 0]
minAngle = 360
headAng = -45
outAngle = 0
dockDis = 0.240
cakeDis = 0.075
cameraPos = [-1, -1]

robotNum = 0
side = 0
run_mode = ''
position = [-1, -1]
preposition = [-1, -1]
quaternion = Quaternion()
robotPose = PoseArray()

def handle_eat(req):
    publisher(req.color)
    return eatResponse(robotPose)

def tPoint(mode, pos, target):
    # print(pos ,target)
    if target == [-0.001, -0.001]:
        return [-1, -1]
    else:
        if mode == 'd':
            dis = dockDis
        elif mode == 'c':
            dis = cakeDis 
        point = [-1, -1]
        if target[0] == pos[0]:
            if target[1] > pos[1]:
                point = [target[0], target[1]-dis]
            elif target[1] < pos[1]:
                point = [target[0], target[1]+dis]
        elif target[1] == pos[1]:
            if target[0] > pos[0]:
                point = [target[0]-dis, target[1]]
            elif target[0] < pos[0]:
                point = [target[0]+dis, target[1]]
        else:
            if target[0] > pos[0]:
                x = target[0] - pos[0]
            else:
                x = pos[0] - target[0]
            if target[1] > pos[1]:
                y = target[1]-pos[1]
            else:
                y = pos[1]-target[1]
            if target[1] > pos[1]:
                point[1] = target[1] - dis*math.sin(math.atan(y/x))
            else:
                point[1] = target[1] + dis*math.sin(math.atan(y/x))
            if target[0] > pos[0]:
                point[0] = target[0] - dis*math.cos(math.atan(y/x))
            else:
                point[0] = target[0] + dis*math.cos(math.atan(y/x))
        return point

def startPos_callback(msg):
    global startPos, absAng, headAng
    startPos[0] = msg.pose.pose.position.x * 1000
    startPos[1] = msg.pose.pose.position.y * 1000
    absAng = quaternion2euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) - headAng

def camera_callback(msg):
    global cameraPos
    if msg.z==-1:
            cameraPos=[-1,-1]
    # else if msg.z==-1:
    else:
        cameraPos[0], cameraPos[1] = msg.x*1000, msg.y*1000
    print(cameraPos)

def robotPublish(color):
    global robotPose, tempFull, cameraPos
    c = '?'
    if color == 0:
        c = 'b'
    elif color == 1:
        c = 'y'
    elif color == 2:
        c = 'p'
    robotPose.header.frame_id += c
    for full in tempFull:
        if full == 1:
            robotPose.header.frame_id += str(tempFull.index(full))
    robotPose.header.stamp = rospy.Time.now()
    
    pose = Pose()
    pt = tPoint('d', preposition, position)
    # print(pt)
    pose.position.x = pt[0]
    pose.position.y = pt[1]
    pose.orientation.x = quaternion.x
    pose.orientation.y = quaternion.y
    pose.orientation.z = quaternion.z
    pose.orientation.w = quaternion.w
    robotPose.poses.append(pose)
    
    pose2 = Pose()
    pt = tPoint('c', preposition, position)
    pose2.position.x = pt[0]
    pose2.position.y = pt[1]
    pose2.orientation.x = quaternion.x
    pose2.orientation.y = quaternion.y
    pose2.orientation.z = quaternion.z
    pose2.orientation.w = quaternion.w
    robotPose.poses.append(pose2)
    # print("eat : ", robotPose)

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

def howLong(pos):
    dis = 0
    for i in range(len(pos) - 1):
        if pos[i] != [-1, -1] and pos[i+1] != [-1, -1]:
            dis += euclidean(pos[i], pos[i+1])
    return dis

def where2go(pos):
    global fullness, absAng, minAngle, outAngle, headAng, cameraPos, quaternion, tempFull

    tempAng = 0
    anglePos = list(pos)
    tempAng = absAng
    tempFull = list(fullness)

    minAngle = 360
    minAngleNum = -1
    
    tAngle = (np.rad2deg(np.arctan2(cameraPos[1], cameraPos[0])) - tempAng + 360) % 360
    tAngles = [360, 360, 360, 360]
    for i in range(4):
        if tempFull[i] == 0:
            tAngles[i] = (tAngle - i * 90 + 360 - headAng) % 360
    for i in tAngles:
        if abs(i) < abs(minAngle):
            minAngle = i
            minAngleNum = tAngles.index(i)
    outAngle = minAngle + tempAng + 2*headAng
    tempFull[minAngleNum] = 1

    quaternion = euler2quaternion(0, 0, outAngle * math.pi / 180)
    # robotPublish(robotNum)

def listener():
    global side, robotNum, run_mode
    rospy.init_node("eat")
    robotNum = rospy.get_param('robot')
    run_mode = rospy.get_param('run_mode')
    rospy.Subscriber("/onRobot/relative_where", Point, camera_callback)
    rospy.Service('eat'+str(robotNum), eat, handle_eat)
    if run_mode == 'run':
        rospy.Subscriber("/robot" + str(robotNum+1) +"/ekf_pose", PoseWithCovarianceStamped, startPos_callback)
    elif run_mode == 'sim':
        rospy.Subscriber("/robot" + str(robotNum+1) +"/odom", Odometry, startPos_callback)
    rospy.spin()

def publisher(color):
    global quaternion, startPos, robotNum, robotPose, tempFull, minAngle, outAngle, position, quaternion, preposition, cameraPos

    tempFull = [0, 0, 0, 0]
    minAngle = 360
    outAngle = 0
    position = [-1, -1]
    quaternion = Quaternion()
    robotPose = PoseArray()

    if cameraPos != [-1, -1]:
        where2go(startPos)

        if startPos != [-1, -1]:
            robotPose.poses=[]
            robotPose.header.frame_id = ''

            for axis in range(2):
                preposition[axis] = startPos[axis] * 0.001                    
                position[axis] = cameraPos[axis] * 0.001 + preposition[axis]

            quaternion = euler2quaternion(0, 0, outAngle * math.pi / 180)
            robotPublish(color)
        cameraPos = [-1, -1]
        # print(picked)

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass