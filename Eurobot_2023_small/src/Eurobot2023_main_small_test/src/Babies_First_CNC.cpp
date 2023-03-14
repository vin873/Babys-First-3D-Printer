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

int side = 0; // 0 for blue, 1 for green
int robot = 1; // 0 for big, 1 for small
int cakeNum = 0;
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
double mission_timeOut;
double startMissionTime;
double driving_timeOut;
double startDriveTime;
double go_home_time = 90.0;

bool start = false;
bool moving = false;
bool doing = false;
bool arrived = false;
bool mission_success = false;
bool got_cake_picked = false;
bool got_cherry_picked = false;
bool finish_mission = false;
bool going_home = false;
bool fullness[4] = {0, 0, 0, 0}; // {0, 90, 180, 270}
bool plates[5] = {0, 0, 0, 0}; // x y
bool printOnce = false;

string id;
string id_frame = "/robot" + to_string(robot+1) + "/map";

std_msgs::Bool cake;
std_msgs::Bool cherry;
std_msgs::String missionStr;

geometry_msgs::PoseStamped cake_picked[3];
geometry_msgs::PoseStamped cherry_picked;
geometry_msgs::PoseStamped basket_point[2];

class mainProgram
{
public:

    void poseStamped_set(geometry_msgs::PoseStamped &pos, float x, float y, float z, float w)
    {
        pos.header.frame_id = id_frame;
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = x;
        pos.pose.position.y = y;
        pos.pose.orientation.x = 0.0;
        pos.pose.orientation.y = 0.0;
        pos.pose.orientation.z = z;
        pos.pose.orientation.w = w;
    }

    void pcake_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        id = msg->header.frame_id;
        for (int i = 0;i < 3;i++)
        {
            poseStamped_set(cake_picked[i], msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].orientation.z, msg->poses[i].orientation.w);
        }
        got_cake_picked = true;
        ROS_INFO("cake_picked got!");
    }

    void pcherry_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        poseStamped_set(cherry_picked, msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.z, msg->pose.orientation.w);
        got_cherry_picked = true;
        ROS_INFO("cherry_picked got!");
    } 

    void nav_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        moving = false;
        if (msg->data)
        {
            arrived = true;
            ROS_INFO("Arrived!");
        }
        else
        {
            ROS_INFO("Failed to reach goal!");
        }
    }

    void done_fullness_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        doing = false;
        if (msg->data.at(0))
        {
            mission_success = true;
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

    void start_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            start = true;
        }
    }

    void cake_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        ROS_INFO("Cake!");
    }

    void cherry_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        ROS_INFO("Cherry!");
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
    ros::Publisher _better_cake = nh.advertise<std_msgs::Bool>("cake"+to_string(robot), 1000);
    ros::Publisher _first_cherry = nh.advertise<std_msgs::Bool>("cherry"+to_string(robot), 1000);
    ros::Subscriber _cake_picked = nh.subscribe<geometry_msgs::PoseArray>("cake_picked"+to_string(robot), 1000, &mainProgram::pcake_callback, this);
    ros::Subscriber _cherry_picked = nh.subscribe<geometry_msgs::PoseStamped>("cherry_picked"+to_string(robot), 1000, &mainProgram::pcherry_callback, this);

    // chassis
    ros::Publisher _where2go = nh.advertise<geometry_msgs::PoseStamped>("/robot"+to_string(robot+1)+"/nav_goal", 1000);
    ros::Subscriber _finishornot = nh.subscribe<std_msgs::Bool>("/robot"+to_string(robot+1)+"/finishornot", 1000, &mainProgram::nav_callback, this);

    // mission
    ros::Publisher _mission = nh.advertise<std_msgs::String>("mission"+to_string(robot), 1000);
    ros::Subscriber _donefullness = nh.subscribe<std_msgs::Int32MultiArray>("donefullness"+to_string(robot), 1000, &mainProgram::done_fullness_callback, this);
    ros::Subscriber _startornot = nh.subscribe<std_msgs::Bool>("startornot", 1000, &mainProgram::start_callback, this);

    // basket
    ros::Publisher _point_home = nh.advertise<std_msgs::Int32>("point_home", 1000);

    // camera
    ros::Publisher _turnonornot = nh.advertise<std_msgs::Bool>("turnonornot", 1000);
    ros::Subscriber _allCakes = nh.subscribe<geometry_msgs::PoseArray>("allCakes", 1000, &mainProgram::cake_callback, this);
    ros::Subscriber _cherryExistence = nh.subscribe<std_msgs::Int32MultiArray>("cherryExistence", 1000, &mainProgram::cherry_callback, this);

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
                if (!printOnce)
                {
                    ROS_INFO("SETUP");
                }
                printOnce = true;
                cake.data = false;
                cherry.data = false;

                mainClass.poseStamped_set(basket_point[0], 0.225, 0.225, 0, 1);
                mainClass.poseStamped_set(basket_point[1], 0.225, 1.775, 0, 1);

                mainClass.nh.getParam("side", side);
                mainClass.nh.getParam("go_home_time", go_home_time);

                if (start)
                {
                    now_Status = RUN;
                    printOnce = false;
                }
                break;

            case RUN:
                if (!printOnce)
                {
                    initialTime = ros::Time::now();
                    ROS_INFO("RUN");
                }
                printOnce = true;

                switch (now_Mission)
                {
                case CAKE:

                    mission_timeOut = 5;
                    driving_timeOut = 20;

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        now_Mission = HOME;
                        ROS_INFO("===== Time to Go Home !!! =====");
                    }
                    else if (!cake.data)
                    {
                        cake.data = true;
                        mainClass._better_cake.publish(cake);
                    }
                    else if (got_cake_picked)
                    {
                        if (!moving && !doing)
                        {
                            if (!arrived && !mission_success)
                            {
                                missionStr.data = id[2*cakeNum];
                                missionStr.data += id[2*cakeNum+1];
                                mainClass._mission.publish(missionStr);
                                ROS_INFO("Mission [%s] published!", missionStr.data.c_str());

                                mainClass._where2go.publish(cake_picked[cakeNum]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", cake_picked[cakeNum].pose.position.x, cake_picked[cakeNum].pose.position.y);
                                moving = true;
                                startDriveTime = ros::Time::now().toSec();
                            }
                            else if (arrived)
                            {
                                arrived = false;
                                missionStr.data.at(0) = 'c';
                                mainClass._mission.publish(missionStr);
                                ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                doing = true;
                                startMissionTime = ros::Time::now().toSec();
                            }
                            else if (mission_success)
                            {
                                mission_success = false;
                                if (cakeNum < 2)
                                {
                                    cakeNum++;
                                }
                                else
                                {
                                    now_Mission = CHERRY;
                                }
                            }
                        }
                        else if (moving && ros::Time::now().toSec() - startDriveTime >= driving_timeOut)
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            ROS_INFO("===== Can't reach to x:[%.3f] y:[%.3f]! =====", cake_picked[cakeNum].pose.position.x, cake_picked[cakeNum].pose.position.y);
                            if (cakeNum < 2)
                            {
                                cakeNum++;
                            }
                            else
                            {
                                now_Mission = CHERRY;
                            }
                        }
                        else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            ROS_INFO("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                            if (cakeNum < 2)
                            {
                                cakeNum++;
                            }
                            else
                            {
                                now_Mission = CHERRY;
                            }
                        }
                    }
                    break;
                
                case CHERRY:
                    
                    mission_timeOut = 20;
                    driving_timeOut = 20;
                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        now_Mission = HOME;
                        ROS_INFO("===== Time to Go Home !!! =====");
                    }
                    else if (!cherry.data)
                    {
                        cherry.data = true;
                        mainClass._first_cherry.publish(cherry);
                    }
                    else if (got_cherry_picked)
                    {   
                        if (!moving && !doing)
                        {
                            if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(cherry_picked);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", cherry_picked.pose.position.x, cherry_picked.pose.position.y);
                                moving = true;
                                startDriveTime = ros::Time::now().toSec();
                            }
                            else if (arrived)
                            {
                                arrived = false;
                                missionStr.data = "s0";
                                mainClass._mission.publish(missionStr);
                                ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                doing = true;
                                startMissionTime = ros::Time::now().toSec();
                            }
                            else if (mission_success)
                            {
                                mission_success = false;
                                now_Mission = BASKET;
                            }
                        }
                        else if (moving && ros::Time::now().toSec() - startDriveTime >= driving_timeOut)
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            ROS_INFO("===== Can't reach to x:[%.3f] y:[%.3f]! =====",  cherry_picked.pose.position.x, cherry_picked.pose.position.y);
                            now_Mission = BASKET;
                        }
                        else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            ROS_INFO("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                            now_Mission = BASKET;
                        }
                    }
                    break;

                case BASKET:
                    
                    mission_timeOut = 20;
                    driving_timeOut = 20;
                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        now_Mission = HOME;
                        ROS_INFO("===== Time to Go Home !!! =====");
                    }
                    else if (!moving && !doing)
                    {
                        if (!arrived && !mission_success)
                        {
                            mainClass._where2go.publish(basket_point[side]);
                            ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", basket_point[side].pose.position.x, basket_point[side].pose.position.y);
                            moving = true;
                            startDriveTime = ros::Time::now().toSec();
                        }
                        else if (arrived)
                        {
                            arrived = false;
                            missionStr.data = "u0";
                            mainClass._mission.publish(missionStr);
                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                            doing = true;
                            startMissionTime = ros::Time::now().toSec();
                        }
                        else if (mission_success)
                        {
                            mission_success = false;
                            now_Mission = RELEASE;
                        }
                    }
                    else if (moving && ros::Time::now().toSec() - startDriveTime >= driving_timeOut)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        ROS_INFO("===== Can't reach to x:[%.3f] y:[%.3f]! =====", basket_point[side].pose.position.x, basket_point[side].pose.position.y);
                        now_Mission = RELEASE;
                    }
                    else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        ROS_INFO("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                        now_Mission = RELEASE;
                    }
                    break;

                case RELEASE:
                    now_Mission = STEAL;
                    break;

                case STEAL:
                    now_Mission = HOME;
                    break;
                
                case HOME:
                    going_home = true;
                    now_Status = FINISH;
                    printOnce = false;
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