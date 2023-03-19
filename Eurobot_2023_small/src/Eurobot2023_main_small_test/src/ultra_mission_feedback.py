#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

done = Int32MultiArray()
arr = [0, 0, 0, 0, 0]
robotNum = 1

startPos = [[-1, -1], [-1, -1]]

def mission_callback(msg):
    global arr
    # cake
    if msg.data[0] == 'b' or msg.data[0] == 'y' or msg.data[0] == 'p':
        arr[int(msg.data[1])+1] = 1
        arr[0] = 1
    # no feedback
    elif msg.data[0] == 'f' or msg.data[0] == 'd':
        arr[0] = 1
    elif msg.data[0] == 'c' or msg.data[0] == 's':
        arr[0] = 1
        publisher(1)
    elif msg.data[0] == 'o':
        # arr[int(msg.data[1])+1] = 0
        arr[0] = 1
        publisher(1)
    elif msg.data[0] == 'v' or msg.data[0] == 'u':
        arr[0] = 1
        publisher(5)

def startPos1_callback(msg):
    global startPos, absAng
    startPos[0][0] = msg.pose.pose.position.x * 1000
    startPos[0][1] = msg.pose.pose.position.y * 1000

def startPos2_callback(msg):
    global startPos
    startPos[1][0] = msg.pose.pose.position.x * 1000
    startPos[1][1] = msg.pose.pose.position.y * 1000


def listener():
    rospy.init_node("ultra_mission_feedback")
    rospy.Subscriber("/robot1/odom", Odometry, startPos1_callback)
    rospy.Subscriber("/robot2/odom", Odometry, startPos2_callback)
    rospy.Subscriber("/mission"+str(robotNum), String, mission_callback)
    rospy.spin()

def publisher(time):
    global arr
    pub = rospy.Publisher('/donefullness'+str(robotNum), Int32MultiArray, queue_size=1000)
    done.data = arr
    rospy.sleep(time)
    pub.publish(done)

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass