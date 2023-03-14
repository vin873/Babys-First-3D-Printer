#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "yaml-cpp/yaml.h"
using namespace std;
double check = 1;

void Check(const std_msgs::Bool::ConstPtr& msg) {
    check = 1;
    if(!msg->data)
    {
        ROS_INFO("script_sim: cannot arrive the goal, next point!");
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_to_point");
    ros::NodeHandle nh;

    string PubName(argv[2]);
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(PubName, 10);

    string SubName(argv[3]);
    ros::Subscriber sub = nh.subscribe(SubName, 10, Check);
    ros::Rate loop_rate(1);

    string PathName(argv[1]);

    geometry_msgs::PoseStamped goal_point;
    YAML::Node pathConfig = YAML::LoadFile(PathName);

    loop_rate.sleep();
    for (auto goal : pathConfig) {
        if (!ros::ok()) {
            break;
        }
        goal_point.header.frame_id = "map";
        goal_point.pose.position.x = goal["xyz"][0].as<double>();
        goal_point.pose.position.y = goal["xyz"][1].as<double>();

        tf2::Quaternion qt;
        qt.setRPY(0, 0, goal["xyz"][2].as<double>());

        goal_point.pose.orientation.x = qt.x();
        goal_point.pose.orientation.y = qt.y();
        goal_point.pose.orientation.z = qt.z();
        goal_point.pose.orientation.w = qt.w();
        pub.publish(goal_point);
        check = 0;
        // cout << "x" << goal["xyz"][0].as<double>() << endl;
        // cout << "y" << goal["xyz"][1].as<double>() << endl;
        // cout << "z" << goal["xyz"][2].as<double>() << endl;
        while (check == 0 && ros::ok()) {
            ros::spinOnce();
        }
        loop_rate.sleep();
    }

    return 0;
}