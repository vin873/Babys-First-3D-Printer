// ROS core
#include "ros/ros.h"

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>

// ROS actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> CLIENT;

int main(int argc, char** argv) {
    ros::init(argc, argv, "MoveBaseActionGoal");
    ros::NodeHandle nh;

    CLIENT Client("move_base", true);

    // Duration for Timeout
    while (!Client.waitForServer(ros::Duration(3.0)) && nh.ok()) {
        ROS_INFO("Action Client : Wait for server!!!");
    }

    move_base_msgs::MoveBaseGoal Goal;
    move_base_msgs::MoveBaseFeedback Feedback;
    move_base_msgs::MoveBaseResult Result;

    ros::Rate LoopRate(1);
    while (nh.ok()) {
        // Header
        Goal.target_pose.header.frame_id = "map";
        Goal.target_pose.header.stamp = ros::Time::now() + ros::Duration(2.0);

        // Goal Position
        Goal.target_pose.pose.position.x = 0.352;
        Goal.target_pose.pose.position.y = 1.666;
        Goal.target_pose.pose.orientation.w = 0.6635;

        Client.sendGoal(Goal);
        // if (Client.getState() == actionlib::SimpleClientGoalState::PENDING) {
        //     ROS_INFO("Action Client : Send Goal.");
        // }

        Client.waitForResult(ros::Duration(1.0));

        ROS_INFO("Current State : %s\n", Client.getState().toString().c_str());
        if (Client.getState() == actionlib::SimpleClientGoalState::PENDING) {
            break;
        }

        // if (Client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        //     ROS_INFO("Action Client : Success!");
        // } else {
        //     ROS_INFO("Action Client : Failure!");
        // }

        ros::spinOnce();
        LoopRate.sleep();
    }

    return 0;
}
