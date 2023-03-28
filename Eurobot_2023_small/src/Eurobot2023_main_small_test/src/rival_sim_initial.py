#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

side = 0

rivalStartPos = [[[1.125, 1.775], [1.875, 0.225]], [[1.125, 0.225], [1.875, 1.775]]]

def publisher():
    global side
    rospy.init_node("rival_sim_initial")
    side = int(not bool(rospy.get_param('side')))
    pub1 = rospy.Publisher('/rival1/mission', PoseStamped, queue_size=1000)
    pub2 = rospy.Publisher('/rival2/mission', PoseStamped, queue_size=1000)
    
    rivalPose = PoseStamped()
    rivalPose.header.frame_id = 'path'
    rivalPose.header.stamp = rospy.Time.now()

    rivalPose.pose.position.x = rivalStartPos[side][0][0]
    rivalPose.pose.position.y = rivalStartPos[side][0][1]
    rivalPose.pose.orientation.x = 0
    rivalPose.pose.orientation.y = 0
    rivalPose.pose.orientation.z = 0
    rivalPose.pose.orientation.w = 1

    rospy.sleep(0.3)
    pub1.publish(rivalPose)

    rivalPose.pose.position.x = rivalStartPos[side][1][0]
    rivalPose.pose.position.y = rivalStartPos[side][1][1]
    rivalPose.pose.orientation.x = 0
    rivalPose.pose.orientation.y = 0
    rivalPose.pose.orientation.z = 0
    rivalPose.pose.orientation.w = 1

    rospy.sleep(0.3)
    pub2.publish(rivalPose)


if __name__=="__main__":
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass