#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace std;

enum Status
{
    SETUP = 0,
    RUN,
    FINISH
};

enum Mode
{
    EMERGENCY = 0,
    NORMAL
};

// Global Variables

int side; // 0 for blue, 1 for green
int now_Status = SETUP;
int now_Mode = NORMAL;
int fullness[4] = {0, 0, 0, 0};

double position_x;
double position_y;
double orientation_z;
double orientation_w;
double startMissionTime;
double go_home_time;

bool moving = false;
bool doing = false;
bool finish_mission = false;
bool going_home = false;
bool pid_closed = false;
bool mission_success = false;

geometry_msgs::PoseStamped target;
geometry_msgs::PointStamped browns[4];
geometry_msgs::PointStamped yellows[4];
geometry_msgs::PointStamped pinks[4];

class mainProgram
{
public:

    void nav_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data && moving && now_Status > SETUP)
        {
            
        }
    }

    void fullness_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        for (int i = 0;i < 4;i++)
        {
            fullness[i] = msg->data.at(i)
        }
    }

    void mission_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            ROS_INFO("Mission Success !");
        }
        else
        {
            ROS_INFO("Mission Failed !");
        }
    }

    void cake_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {

    }

    void myPos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        myPos_x = msg->pose.pose.position.x;
        myPos_y = msg->pose.pose.position.y;
        myOri_z = msg->pose.pose.orientation.z;
        myOri_w = msg->pose.pose.orientation.w;
    }

    void enemiesPos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        enemiesPos_x = msg->pose.pose.position.x;
        enemiesPos_y = msg->pose.pose.position.y;
        enemiesOri_z = msg->pose.pose.orientation.z;
        enemiesOri_w = msg->pose.pose.orientation.w;
    }
    
    ros::NodeHandle nh;

    // chassis
    ros::Publisher _where2go = nh.advertise<geometry_msgs::PoseStamped>("where2go", 1000);
    ros::Subscriber _finishOrNot = nh.subscribe<std_msgs::Bool>("finishornot", 1000, &mainProgram::nav_callback, this);

    // mission
    ros::Publisher _mission = nh.advertise<std_msgs::String>("mission", 1000);
    ros::Subscriber _fullness = nh.subscribe<std_msgs::Int32MultiArray>("fullness", 1000, &mainProgram::fullness_callback, this);
    ros::Subscriber _missionDone = nh.subscribe<std_msgs::Bool>("missionDone", 1000, &mainProgram::mission_callback, this);

    // basket
    ros::Publisher _point_home = nh.advertise<std_msgs::Int32>("point_home", 1000);

    // camera
    ros::Publisher _turnonornot = nh.advertise<std_msgs::Bool>("turnonornot", 1000);
    ros::Subsciber _cake = nh.advertise<geometry_msgs::PointStamped>("cake", 1000, &mainProgram::cake_callback, this);

    // locate
    ros::Subscriber _myPos = nh.subscribe<geometry_msgs::PoseStamped>("myPos", 1000, &mainProgram::myPos_callback, this);
    ros::Subscriber _enemiesPos = nh.subscribe<geometry_msgs::PoseStamped>("enemiesPos", 1000, &mainProgram::enemiesPos_callback, this);
}

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Babies_First_CNC");

    // Node Handling Class Initialize
    mainProgram mainClass;
    ros::Time initialTime = ros::Time::now();
    std_msgs::Float32 timePublish;
    std_msgs::Int32 pointPublish;

    // Main Node Update Frequency
    ros::Rate rate(20);

    while (ros::ok())
    {
        switch (now_Mode)
        {
        case NORMAL:
            switch (now_Status)
            {
            case SETUP:
                mainClass.nh.getParam("side", side);
                mainClass.nh.getParam("go_home_time", go_home_time);

                now_Status = RUN;
                break;

            case RUN:

                break;

            case FINISH:

                break;
            }
            break;
        
        case EMERGENCY:
            ROS_INFO("Emergency State");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}