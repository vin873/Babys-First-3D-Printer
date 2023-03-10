#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>

using namespace std;

enum MissionOrder
{
    CAKE = 0,
    CHERRY,
    BASKET,
    RELEASE,
    STEAL,
    HOME
};

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
int now_Mission = CAKE;
int now_Status = SETUP;
int now_Mode = NORMAL;

double myPos_x;
double myPos_y;
double myOri_z;
double myOri_w;
double enemiesPos_x;
double enemiesPos_y;
double enemiesOri_z;
double enemiesOri_w;
double mission_waitTime;
double startMissionTime;
double go_home_time;

bool moving = false;
bool doing = false;
bool got_cake_picked = false;
bool got_cherry_picked = false;
bool finish_mission = false;
bool going_home = false;
bool pid_closed = false;
bool mission_success = false;
bool fullness[4] = {0, 0, 0, 0}; // {0, 90, 180, 270}
bool plates[5] = {0, 0, 0, 0}; // x y

std_msgs::Bool cake;
std_msgs::Bool cherry;
geometry_msgs::PoseStamped target;
geometry_msgs::PoseStamped cake_picked[3];
geometry_msgs::PoseStamped cherry_picked;

class mainProgram
{
public:

    void pcake_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        for (int i = 0;i < 3;i++)
        {
            cake_picked[i].header.frame_id = msg->header.frame_id;
            cake_picked[i].header.stamp = msg->header.stamp;
            cake_picked[i].pose.position.x = msg->poses[i].position.x;
            cake_picked[i].pose.position.y = msg->poses[i].position.y;
            cake_picked[i].pose.orientation.x = msg->poses[i].orientation.x;
            cake_picked[i].pose.orientation.y = msg->poses[i].orientation.y;
            cake_picked[i].pose.orientation.z = msg->poses[i].orientation.z;
            cake_picked[i].pose.orientation.w = msg->poses[i].orientation.w;
        }
        got_cake_picked = true;
        ROS_INFO("cake_picked got!");
    }

    void pcherry_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        cherry_picked.header.frame_id = msg->header.frame_id;
        cherry_picked.header.stamp = msg->header.stamp;
        cherry_picked.pose.position.x = msg->pose.position.x;
        cherry_picked.pose.position.y = msg->pose.position.y;
        cherry_picked.pose.orientation.x = msg->pose.orientation.x;
        cherry_picked.pose.orientation.y = msg->pose.orientation.y;
        cherry_picked.pose.orientation.z = msg->pose.orientation.z;
        cherry_picked.pose.orientation.w = msg->pose.orientation.w;
        got_cherry_picked = true;
        ROS_INFO("cake_picked got!");
    } 


    void nav_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            ROS_INFO("Arrived!");
        }
        else
        {
            ROS_INFO("Failed to reach goal!");
        }
    }

    void done_fullness_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.at(0))
        {
            ROS_INFO("Mission finished!");
        }
        else
        {
            ROS_INFO("Mission failed!");
        }
        for (int i = 1;i < 5;i++)
        {
            fullness[i-1] = msg->data.at(i);
        }
    }

    void cake_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        ROS_INFO("Cake!");
    }

    void myPos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        myPos_x = msg->pose.position.x;
        myPos_y = msg->pose.position.y;
        myOri_z = msg->pose.orientation.z;
        myOri_w = msg->pose.orientation.w;
    }

    void enemiesPos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        enemiesPos_x = msg->pose.position.x;
        enemiesPos_y = msg->pose.position.y;
        enemiesOri_z = msg->pose.orientation.z;
        enemiesOri_w = msg->pose.orientation.w;
    }
    
    ros::NodeHandle nh;

    // main
    ros::Publisher _better_cake = nh.advertise<std_msgs::Bool>("cake", 1000);
    ros::Publisher _first_cherry = nh.advertise<std_msgs::Bool>("cherry", 1000);
    ros::Publisher _test_cake = nh.advertise<geometry_msgs::PoseStamped>("test_cake", 1000);
    ros::Publisher _test_cherry = nh.advertise<geometry_msgs::PoseStamped>("test_cherry", 1000);
    ros::Subscriber _cake_picked = nh.subscribe<geometry_msgs::PoseArray>("cake_picked", 1000, &mainProgram::pcake_callback, this);
    ros::Subscriber _cherry_picked = nh.subscribe<geometry_msgs::PoseStamped>("cherry_picked", 1000, &mainProgram::pcherry_callback, this);
    
    // chassis
    ros::Publisher _where2go = nh.advertise<geometry_msgs::PoseStamped>("where2go", 1000);
    ros::Subscriber _finishOrNot = nh.subscribe<std_msgs::Bool>("finishornot", 1000, &mainProgram::nav_callback, this);

    // mission
    ros::Publisher _mission = nh.advertise<std_msgs::String>("mission", 1000);
    ros::Subscriber _donefullness = nh.subscribe<std_msgs::Int32MultiArray>("donefullness", 1000, &mainProgram::done_fullness_callback, this);

    // basket
    ros::Publisher _point_home = nh.advertise<std_msgs::Int32>("point_home", 1000);

    // camera
    ros::Publisher _turnonornot = nh.advertise<std_msgs::Bool>("turnonornot", 1000);
    ros::Subscriber _allCakes = nh.subscribe<geometry_msgs::PoseArray>("all_cakes", 1000, &mainProgram::cake_callback, this);

    // locate
    ros::Publisher _obstacles = nh.advertise<geometry_msgs::PoseArray>("obstacles", 1000);
    ros::Subscriber _myPos = nh.subscribe<geometry_msgs::PoseStamped>("myPos", 1000, &mainProgram::myPos_callback, this);
    ros::Subscriber _enemiesPos = nh.subscribe<geometry_msgs::PoseStamped>("enemiesPos", 1000, &mainProgram::enemiesPos_callback, this);
};

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Babies_First_CNC");

    // Node Handling Class Initializeposition
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
                ROS_INFO("SETUP");
                ros::Duration(0.5).sleep();
                cake.data = false;
                cherry.data = false;

                mainClass.nh.getParam("side", side);
                mainClass.nh.getParam("go_home_time", go_home_time);
                now_Status = RUN;
                break;

            case RUN:
                ROS_INFO("RUN");
                switch (now_Mission)
                {
                case CAKE:
                    if (!cake.data)
                    {
                        cake.data = true;
                        mainClass._better_cake.publish(cake);
                    }
                    if (got_cake_picked)
                    {   
                        for (int i = 0;i < 3;i++)
                        {
                            mainClass._test_cake.publish(cake_picked[i]);
                        }
                        now_Mission = CHERRY;
                    }
                    break;
                
                case CHERRY:
                    if (!cherry.data)
                    {
                        cherry.data = true;
                        mainClass._first_cherry.publish(cherry);
                    }
                    if (got_cherry_picked)
                    {   
                        mainClass._test_cherry.publish(cherry_picked);
                        now_Mission = BASKET;
                    }
                    break;

                case BASKET:
                    now_Mission = RELEASE;
                    break;

                case RELEASE:
                    now_Mission = STEAL;
                    break;

                case STEAL:
                    now_Mission = HOME;
                    break;
                
                case HOME:
                    now_Status = FINISH;
                    break;
                }
                break;

            case FINISH:
                if (!finish_mission)
                {
                    finish_mission = true;
                    ROS_INFO("Finish all missions!");
                }
                
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