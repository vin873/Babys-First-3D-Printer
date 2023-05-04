#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from eurobot2023_main.srv import *

robotNum = 0
side = 0

missionStr = String()

deliveredAr = False
waitAr = False
deliveredSTM = False
waitSTM = False

def handle_handshake(req):
    global missionStr
    missionStr.data = req.misStr
    publisher()
    return handshakeResponse()

def handshakerAr_callback(msg):
    global deliveredAr
    if waitAr and msg.data != "" and msg.data[0] == missionStr.data[0] and msg.data[1] == missionStr.data[1]:
        deliveredAr = True

def handshakerSTM_callback(msg):
    global deliveredSTM
    if waitSTM and msg.data != "" and msg.data[0] == missionStr.data[0] and msg.data[1] == missionStr.data[1]:
        deliveredSTM = True

def listener():
    global side, robotNum
    side = rospy.get_param('side')
    robotNum = rospy.get_param('robot')
    rospy.init_node("handshake")
    rospy.Subscriber('handshaker', String, handshakerAr_callback)
    rospy.Subscriber('handshakier', String, handshakerSTM_callback)
    rospy.Service('handshake'+str(robotNum), handshake, handle_handshake)
    rospy.spin()

def publisher():
    

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass