#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

done = Int32MultiArray()
done2 = Int32MultiArray()
arr = [0, 0, 0, 0, 0]
arr2 = [1, 1, 1, 1]
robotNum = 0

startPos = [[-1, -1], [-1, -1]]

def cherryE_callback(msg):
    for i in range(4):
        arr2[i] = msg.data[i]

def mission_callback(msg):
    global arr

    if msg.data[0] == 'b' or msg.data[0] == 'y' or msg.data[0] == 'p':
        arr[int(msg.data[1])+1] = 1
        arr[0] = 1
    elif msg.data[0] == 'c':
        arr[0] = 1
        publisher(1)
    elif msg.data[0] == 'o':
        # arr[int(msg.data[1])+1] = 0
        arr[0] = 1
        publisher(1)
    elif msg.data[0] == 's':
        arr[0] = 1
        arr2[int(msg.data[1])] = 0
        publisher2()
        publisher(3)
    elif msg.data[0] == 'v':
        arr[0] = 1
        publisher(1)
    elif msg.data[0] == 'u':
        arr[0] = 1
        publisher(5)
    elif msg.data[0] == 'f' or msg.data[0] == 'd':
        arr[0] = 1

def listener():
    global robotNum
    rospy.init_node("ultra_mission_feedback")
    robotNum = rospy.get_param('robot')
    rospy.Subscriber("/cherryExistence", Int32MultiArray, cherryE_callback)
    rospy.Subscriber("/rmission"+str(robotNum), String, mission_callback)
    rospy.spin()

def publisher(time):
    global arr
    pub = rospy.Publisher('/rdonefullness'+str(robotNum), Int32MultiArray, queue_size=1000)
    done.data = arr
    rospy.sleep(time)
    pub.publish(done)
    
def publisher2():
    global arr2
    pub2 = rospy.Publisher('/cherryExistence', Int32MultiArray, queue_size=1000)
    done2.data = arr2
    rospy.sleep(0.3)
    pub2.publish(done2)

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass