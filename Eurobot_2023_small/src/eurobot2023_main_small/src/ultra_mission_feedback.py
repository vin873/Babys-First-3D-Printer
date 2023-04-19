#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32MultiArray

done = Int16MultiArray()
done2 = Int32MultiArray()
df = [0, 0, 0, 0, 0]
cherry = [1, 1, 1, 1]
robotNum = 0

startPos = [[-1, -1], [-1, -1]]

def cherryE_callback(msg):
    global cherry
    for i in range(4):
        cherry[i] = msg.data[i]

def mission_callback(msg):
    global df, cherry

    if msg.data[0] == 'b' or msg.data[0] == 'y' or msg.data[0] == 'p':
        df[int(msg.data[1])+1] = 1
        df[0] = 1
    elif msg.data[0] == 'c':
        df[0] = 1
        # publisher(1)
    elif msg.data[0] == 'o':
        df[0] = 1
        # publisher(1)
    elif msg.data[0] == 's':
        df[0] = 1
        if msg.data[1] == '4':
            cherry[0] = 0
        elif msg.data[1] == '5':
            cherry[2] = 0
        else:
            cherry[int(msg.data[1])] = 0
        publisher2()
        # publisher(3)
    elif msg.data[0] == 'v':
        df[0] = 1
        # publisher(1)
    elif msg.data[0] == 'u':
        df[0] = 1
        publisher(5)

        df[0] = 2
        publisher(0.5)
        df[0] = 1

    elif msg.data[0] == 'f' or msg.data[0] == 'd':
        df[0] = 1

def fullness_callback(msg):
    global df
    for i in range(4):
        df[i+1] = msg.data[i+1]

def listener():
    global robotNum
    rospy.init_node("ultra_mission_feedback")
    robotNum = rospy.get_param('robot')
    rospy.Subscriber("/cherryExistence", Int32MultiArray, cherryE_callback)
    rospy.Subscriber("/mission"+str(robotNum), String, mission_callback)
    rospy.Subscriber("/donefullness"+str(robotNum), Int16MultiArray, fullness_callback)
    rospy.spin()

def publisher(time):
    global df, done
    pub = rospy.Publisher('/donefullness'+str(robotNum), Int16MultiArray, queue_size=1000)
    done.data = df
    print("mission callback from ultra_mission_feedback")
    rospy.sleep(time)
    pub.publish(done)
    
def publisher2():
    global cherry, done2
    pub2 = rospy.Publisher('/cherryExistence', Int32MultiArray, queue_size=1000)
    done2.data = cherry
    rospy.sleep(0.3)
    pub2.publish(done2)

if __name__=="__main__":
    try:
        listener()

    except rospy.ROSInterruptException:
        pass