#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;

std_msgs::Int32MultiArray done;
std_msgs::Int32MultiArray done2;

int arr[5] = {0, 0, 0, 0, 0};
int arr2[4] = {1, 1, 1, 1};
int robotNum = 0;

int startPos[2][2] = {{-1, -1}, {-1, -1}};

class mainFeedback
{
    void cherryE_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        for (int i = 0;i < 4;i++)
        {
            arr2[i] = msg->data.at(i);
        }
    }

    void mission_callback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data.at(0) == 'b' || msg->data.at(0) == 'y' || msg->data.at(0) == 'p')
        {
            arr[int(msg->data.at(1))+1] = 1;
            arr[0] = 1;
        }
        else if (msg->data.at(0) == 'c')
        {
            arr[0] = 1;
            publisher(1);
        }
        else if (msg->data.at(0) == 'o')
        {
            arr[0] = 1;
            publisher(1);
        }
        else if (msg->data.at(0) == 's')
        {
            arr[0] = 1;
            publisher(3);
        }
        else if (msg->data.at(0) == 'v')
        {
            arr[0] = 1;
            arr2[int(msg->data.at(1))] = 0;
            publisher2();
            publisher(1);
        }
        else if (msg->data.at(0) == 'u')
        {
            arr[0] = 1;
            publisher(5);
        }
        else if (msg->data.at(0) == 'f' || msg->data.at(0) == 'd')
        {
            arr[0] = 1;
        }
    }

    void startPos1_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        startPos[0][0] = msg->pose.pose.position.x * 1000;
        startPos[0][1] = msg->pose.pose.position.y * 1000;
    }

    void startPos2_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        startPos[1][0] = msg->pose.pose.position.x * 1000;
        startPos[1][1] = msg->pose.pose.position.y * 1000;
    }

    void publisher(float time)
    {
        for (int i = 0;i < 5;i++)
        {
            done.data.at(i) = arr[i];
        }
        ros::Duration(time).sleep();
        _pub.publish(done);
        ROS_INFO("Hi");
    }

    void publisher2()
    {
        for (int i = 0;i < 4;i++)
        {
            done2.data.at(i) = arr2[i];
        }
        _pub2.publish(done2);
    }

    ros::NodeHandle nh;

    ros::Publisher _pub = nh.advertise<std_msgs::Int32MultiArray>("/donefullness"+to_string(robotNum), 1000);
    ros::Publisher _pub2 = nh.advertise<std_msgs::Int32MultiArray>("/cherryExistence", 1000);
    ros::Subscriber _startPos1 = nh.subscribe<nav_msgs::Odometry>("/robot1/odom", 1000, &mainFeedback::startPos1_callback, this);
    ros::Subscriber _startPos2 = nh.subscribe<nav_msgs::Odometry>("/robot2/odom", 1000, &mainFeedback::startPos2_callback, this);
    ros::Subscriber _cherryExistence = nh.subscribe<std_msgs::Int32MultiArray>("/cherryExistence", 1000, &mainFeedback::cherryE_callback, this);
    ros::Subscriber _mission = nh.subscribe<std_msgs::String>("mission"+to_string(robotNum), 1000, &mainFeedback::mission_callback, this);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultra_mission_feedback");

    mainFeedback mainF;

    ros::Time::init();

    ros::Rate rate(40);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

}