//        _____
//       /  /::\       ___           ___
//      /  /:/\:\     /  /\         /  /\
//     /  /:/  \:\   /  /:/        /  /:/
//    /__/:/ \__\:| /__/::\       /  /:/
//    \  \:\ /  /:/ \__\/\:\__   /  /::\
//     \  \:\  /:/     \  \:\/\ /__/:/\:\
//      \  \:\/:/       \__\::/ \__\/  \:\
//       \  \::/        /__/:/       \  \:\
//        \__\/         \__\/         \__\/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <eurobot2023_main_small/cake.h>
#include <eurobot2023_main_small/cherry.h>
#include <eurobot2023_main_small/release.h>
#include <eurobot2023_main_small/steal.h>
#include <eurobot2023_main_small/eat.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <std_srvs/Empty.h>

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

enum Cake_Mode
{
    NO_CAM = 0,
    USE_CAM
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
int robot = 0; // 0 for big, 1 for small
int cakeNum = 0;
int reCake = 0;
int lastCakeColor = -1;
int cherryNum = 0;
int cherryDelay = 3.5;
int who_basket = -1;
int now_release = -1;
int now_Mission = CAKE;
int now_Camera_Mode = NO_CAM;
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
double go_home_time = 100.0;

bool start = false;
bool moving = false;
bool doing = false;
bool arrived = false;
bool hanoiing = false;
bool cam_pub_once = false;
bool eatornot = false;
bool route_failed = false;
bool mission_success = false;
bool got_cake_picked = false;
bool got_cherry_picked = false;
bool got_release_point = false;
bool got_steal_cake = false;
bool somewhere_once = false;
bool going_home = false;
bool cherryE[4] = {1, 1, 1, 1};
bool fullness[4] = {0, 0, 0, 0}; // {0, 90, 180, 270}
bool plates[5] = {0, 0, 0, 0, 0}; // x y
bool printOnce = false;

string id;
string cid;
string rid;
string id_frame = "path";
string dock_id_frame = "dock";

std_msgs::Int32 release;
std_msgs::Int32 basket_robot;
std_msgs::Int32MultiArray got;
std_msgs::Bool finish_mission;
std_msgs::String missionStr;

geometry_msgs::PoseStamped cake_picked[6];
geometry_msgs::PoseStamped steal_picked;
geometry_msgs::PoseStamped eat_picked[2];
geometry_msgs::PoseStamped cherry_picked[2];
geometry_msgs::PoseStamped basket_point[2];
geometry_msgs::PoseStamped release_point[4];
geometry_msgs::PoseStamped home[2][2];

class mainProgram
{
public:

    void initialize()
    {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    void updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        nh.getParam("robot", robot);
        nh.getParam("side", side);
        _got_cake_color = nh.advertise<std_msgs::Int32MultiArray>("gotcake"+to_string(robot), 1000);
        _release = nh.advertise<std_msgs::Int32>("release"+to_string(robot), 1000);
        _another_release = nh.subscribe<std_msgs::Int32>("release"+to_string(!bool(robot)), 1000, &mainProgram::anothere_callback, this);
        _cake_client = nh.serviceClient<eurobot2023_main_small::cake>("cake"+to_string(robot));
        _cherry_client = nh.serviceClient<eurobot2023_main_small::cherry>("cherry"+to_string(robot));
        _release_client = nh.serviceClient<eurobot2023_main_small::release>("release"+to_string(robot));
        _steal_client = nh.serviceClient<eurobot2023_main_small::steal>("steal"+to_string(robot));
        _eat_client = nh.serviceClient<eurobot2023_main_small::eat>("eat"+to_string(robot));
        _where2go = nh.advertise<geometry_msgs::PoseStamped>("/robot"+to_string(robot+1)+"/mission", 1000);
        _finishornot = nh.subscribe<std_msgs::Bool>("/robot"+to_string(robot+1)+"/is_finish", 1000, &mainProgram::nav_callback, this);
        _mission = nh.advertise<std_msgs::String>("mission"+to_string(robot), 1000);
        _donefullness = nh.subscribe<std_msgs::Int16MultiArray>("donefullness"+to_string(robot), 1000, &mainProgram::done_fullness_callback, this);
        _myPos = nh.subscribe<nav_msgs::Odometry>("/robot"+to_string(robot+1)+"/odom", 1000, &mainProgram::myPos_callback, this);
    }

    void poseStamped_set(bool dock,geometry_msgs::PoseStamped &pos, float x, float y, float z, float w)
    {
        if (dock)
        {
            pos.header.frame_id = dock_id_frame;
        }
        else
        {
            pos.header.frame_id = id_frame;
        }
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = x;
        pos.pose.position.y = y;
        pos.pose.orientation.x = 0.0;
        pos.pose.orientation.y = 0.0;
        pos.pose.orientation.z = z;
        pos.pose.orientation.w = w;
    }

    void what_color_cake_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        got.data = msg->data;
    }

    void basket_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        who_basket = msg->data;
    }

    void anothere_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        release.data = msg->data;
    }

    void finishall_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        finish_mission.data = msg->data;
    }

    void nav_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        moving = false;
        if (msg->data)
        {
            ROS_INFO("Arrived!");
            arrived = true;
        }
        else
        {
            route_failed = true;
            ROS_ERROR("Failed to reach goal!");
        }
    }

    void done_fullness_callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
    {
        if (msg->data.at(0) == 1)
        {
            doing = false;
            mission_success = true;
            ROS_INFO("Mission finished!");
        }
        else if (msg->data.at(0) == 2)
        {
            hanoiing = false;
        }
        else
        {
            doing = false;
            ROS_ERROR("Mission failed!");
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

    void cake_callback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        ROS_WARN("Cake update!");
        got_cake_picked = false;
        moving = false;
    }

    void cherry_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        for (int i = 0;i < 4;i++)
        {
            cherryE[i] = msg->data.at(i);
        }
        ROS_WARN("Cherry update!");
    }

    void myPos_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        myPos_x = msg->pose.pose.position.x;
        myPos_y = msg->pose.pose.position.y;
        myOri_z = msg->pose.pose.orientation.z;
        myOri_w = msg->pose.pose.orientation.w;
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
    ros::Publisher _got_cake_color = nh.advertise<std_msgs::Int32MultiArray>("gotcake"+to_string(robot), 1000);
    ros::Publisher _ibasket = nh.advertise<std_msgs::Int32>("basketornot", 1000);
    ros::Publisher _release = nh.advertise<std_msgs::Int32>("release"+to_string(robot), 1000);
    ros::Publisher _ifinish = nh.advertise<std_msgs::Bool>("finishall", 1000);
    ros::Subscriber _got_what_color = nh.subscribe<std_msgs::Int32MultiArray>("gotcake"+to_string(robot), 1000, &mainProgram::what_color_cake_callback, this);
    ros::Subscriber _basketornot = nh.subscribe<std_msgs::Int32>("basketornot", 1000, &mainProgram::basket_callback, this);
    ros::Subscriber _another_release = nh.subscribe<std_msgs::Int32>("release"+to_string(!bool(robot)), 1000, &mainProgram::anothere_callback, this);
    ros::Subscriber _finishall = nh.subscribe<std_msgs::Bool>("finishall", 1000, &mainProgram::finishall_callback, this);
    ros::ServiceClient _cake_client = nh.serviceClient<eurobot2023_main_small::cake>("cake"+to_string(robot));
    ros::ServiceClient _cherry_client = nh.serviceClient<eurobot2023_main_small::cherry>("cherry"+to_string(robot));
    ros::ServiceClient _release_client = nh.serviceClient<eurobot2023_main_small::release>("release"+to_string(robot));
    ros::ServiceClient _steal_client = nh.serviceClient<eurobot2023_main_small::steal>("steal"+to_string(robot));
    ros::ServiceClient _eat_client = nh.serviceClient<eurobot2023_main_small::eat>("eat"+to_string(robot));

    // chassis
    ros::Publisher _where2go = nh.advertise<geometry_msgs::PoseStamped>("/robot"+to_string(robot+1)+"/mission", 1000);
    ros::Subscriber _finishornot = nh.subscribe<std_msgs::Bool>("/robot"+to_string(robot+1)+"/is_finish", 1000, &mainProgram::nav_callback, this);

    // mission
    ros::Publisher _mission = nh.advertise<std_msgs::String>("mission"+to_string(robot), 1000);
    ros::Subscriber _donefullness = nh.subscribe<std_msgs::Int16MultiArray>("donefullness"+to_string(robot), 1000, &mainProgram::done_fullness_callback, this);
    ros::Subscriber _startornot = nh.subscribe<std_msgs::Bool>("startornot", 1000, &mainProgram::start_callback, this);

    // basket
    ros::Publisher _point_home = nh.advertise<std_msgs::Int32>("point_home", 1000);

    // camera
    ros::Publisher _cam_which_color = nh.advertise<std_msgs::Int32>("cam_which_color", 1000);
    ros::Subscriber _allCakes = nh.subscribe<geometry_msgs::Pose>("adjustCake", 1000, &mainProgram::cake_callback, this);
    ros::Subscriber _cherryExistence = nh.subscribe<std_msgs::Int32MultiArray>("cherryExistence", 1000, &mainProgram::cherry_callback, this);

    // locate
    ros::Publisher _obstacles = nh.advertise<geometry_msgs::PoseArray>("obstacles", 1000);
    ros::Subscriber _myPos = nh.subscribe<nav_msgs::Odometry>("/robot"+to_string(robot+1)+"/odom", 1000, &mainProgram::myPos_callback, this);
    ros::Subscriber _enemiesPos = nh.subscribe<geometry_msgs::PoseStamped>("enemiesPos", 1000, &mainProgram::enemiesPos_callback, this);
};

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Babies_First_CNC");

    mainProgram mainClass;

    // mainClass.nh.getParam("robot", robot);
    // mainClass.nh.getParam("side", side);
    mainClass.initialize();

    ros::Time initialTime = ros::Time::now();
    ros::Time cakeTime = ros::Time::now();
    ros::Time cherryTime = ros::Time::now();
    eurobot2023_main_small::cake srv;
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
                    ROS_WARN("SETUP");
                    cakeTime = ros::Time::now();
                    release.data = 0;
                    finish_mission.data = false;
                    
                    got.data = {0, 0, 0, 0};

                    mainClass.poseStamped_set(0, basket_point[0], 0.245, 0.225, 0, 1);
                    mainClass.poseStamped_set(0, basket_point[1], 0.225, 1.775, 0, 1);
                    mainClass.poseStamped_set(0, home[0][0], 1.125, 1.800, 0, 1);
                    mainClass.poseStamped_set(0, home[0][1], 1.125, 0.200, 0, 1);
                    mainClass.poseStamped_set(0, home[1][0], 1.125, 1.475, 0, 1);
                    mainClass.poseStamped_set(0, home[1][1], 1.125, 0.525, 0, 1);
                }
                printOnce = true;

                if (start)
                {
                    now_Status = RUN;
                    moving = false;
                    doing = false;
                    arrived = false;
                    mission_success = false;
                    printOnce = false; 
                }

                if (ros::Time::now().toSec() - cakeTime.toSec() >= 0.4 && mainClass._cake_client.call(srv))
                {
                    if (srv.response.picked.header.frame_id != "")
                    {
                        id = srv.response.picked.header.frame_id;
                        for (int i = 0;i < 6;i++)
                        {
                            mainClass.poseStamped_set(i%2, cake_picked[i], srv.response.picked.poses[i].position.x, srv.response.picked.poses[i].position.y, srv.response.picked.poses[i].orientation.z, srv.response.picked.poses[i].orientation.w);
                            if (cake_picked[i].pose.position.x == -777)
                            {
                                eatornot = true;
                            }
                        }
                        got_cake_picked = true;
                        cakeTime = ros::Time::now();
                    }
                }

                break;

            case RUN:
                if (!printOnce)
                {
                    initialTime = ros::Time::now();
                    ROS_WARN("RUN");
                    // for (int i = 0;i < 6;i++)
                    // {
                    //     cout << "i : " << i << "  x : " << cake_picked[i].pose.position.x << "  y : " << cake_picked[i].pose.position.y << endl;
                    // }
                }
                printOnce = true;

                switch (now_Mission)
                {
                case CAKE:

                    mission_timeOut = 5;

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                        moving = false;
                    }

                    switch (now_Camera_Mode)
                    {
                    case NO_CAM:
                        if (!got_cake_picked)
                        {
                            if (mainClass._cake_client.call(srv))
                            {
                                if (srv.response.picked.header.frame_id != "")
                                {
                                    id = srv.response.picked.header.frame_id;
                                    for (int i = 0;i < 6;i++)
                                    {
                                        mainClass.poseStamped_set(i%2, cake_picked[i], srv.response.picked.poses[i].position.x, srv.response.picked.poses[i].position.y, srv.response.picked.poses[i].orientation.z, srv.response.picked.poses[i].orientation.w);
                                        if (cake_picked[i].pose.position.x == -777)
                                        {
                                            eatornot = true;
                                        }
                                    }
                                    got_cake_picked = true;
                                    cakeTime = ros::Time::now();
                                }
                            }
                        }
                        else if (got_cake_picked)
                        {
                            if (!moving && !doing)
                            {
                                if (cake_picked[cakeNum].pose.position.x < 0)
                                {
                                    if (cakeNum < 4)
                                    {
                                        cakeNum += 2;
                                    }
                                    else if (eatornot)
                                    {
                                        now_Camera_Mode = USE_CAM;
                                        got_cake_picked = false;
                                        cakeNum = 0;
                                    }
                                    else
                                    {
                                        now_Mission = CHERRY;
                                        missionStr.data = "h0";
                                        mainClass._mission.publish(missionStr);
                                        hanoiing = true;
                                        ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                    }
                                }
                                else {
                                    if (route_failed)
                                    {
                                        route_failed = false;
                                        if (cakeNum < 5)
                                        {
                                            cakeNum += 2;
                                        }
                                        else
                                        {
                                            now_Mission = CHERRY;
                                            missionStr.data = "h0";
                                            mainClass._mission.publish(missionStr);
                                            hanoiing = true;
                                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                        }
                                    }
                                    else if (!arrived && !mission_success)
                                    {
                                        if (cakeNum % 2 == 0)
                                        {
                                            missionStr.data = id[cakeNum];
                                            missionStr.data += id[cakeNum+1];
                                            if (missionStr.data.at(0) == 'b')
                                            {
                                                lastCakeColor = 0;
                                            }
                                            else if (missionStr.data.at(0) == 'y')
                                            {
                                                lastCakeColor = 1;
                                            }
                                            else if (missionStr.data.at(0) == 'p')
                                            {
                                                lastCakeColor = 2;
                                            }
                                            mainClass._mission.publish(missionStr);
                                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                        }

                                        mainClass._where2go.publish(cake_picked[cakeNum]);
                                        ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", cake_picked[cakeNum].pose.position.x, cake_picked[cakeNum].pose.position.y);
                                        moving = true;
                                    }
                                    else if (arrived)
                                    {
                                        arrived = false;
                                        if (cakeNum % 2 == 1)
                                        {
                                            if (missionStr.data.at(0) == 'b')
                                            {
                                                lastCakeColor = 0;
                                                got.data.at(0) = 1;
                                            }
                                            else if (missionStr.data.at(0) == 'y')
                                            {
                                                lastCakeColor = 1;
                                                got.data.at(1) = 1;
                                            }
                                            else if (missionStr.data.at(0) == 'p')
                                            {
                                                lastCakeColor = 2;
                                                got.data.at(2) = 1;
                                            }
                                            mainClass._got_cake_color.publish(got);
                                            missionStr.data.at(0) = 'c';
                                            mainClass._mission.publish(missionStr);
                                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                            doing = true;
                                            startMissionTime = ros::Time::now().toSec();
                                        }
                                        else
                                        {
                                            if (cakeNum < 5)
                                            {
                                                cakeNum++;
                                            }
                                        }
                                    }
                                    else if (mission_success)
                                    {
                                        mission_success = false;
                                        if (cakeNum < 5)
                                        {
                                            cakeNum++;
                                        }
                                        else
                                        {
                                            now_Mission = CHERRY;
                                            missionStr.data = "h0";
                                            mainClass._mission.publish(missionStr);
                                            hanoiing = true;
                                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                        }
                                    }
                                }
                            }
                            else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                            {
                                moving = false;
                                doing = false;
                                arrived = false;
                                mission_success = false;
                                ROS_WARN("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                                if (cakeNum < 5)
                                {
                                    cakeNum ++;
                                }
                                else
                                {
                                    now_Mission = CHERRY;
                                    missionStr.data = "h0";
                                    mainClass._mission.publish(missionStr);
                                    hanoiing = true;
                                    ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                }
                            }
                        }
                        break;
                    
                    case USE_CAM:

                        if (!got_cake_picked)
                        {
                            if (!cam_pub_once)
                            {
                                std_msgs::Int32 camColor;
                                camColor.data = lastCakeColor;
                                mainClass._cam_which_color.publish(camColor);
                                cam_pub_once = true;
                            }
                            eurobot2023_main_small::eat esrv;
                            esrv.request.color = lastCakeColor;
                            if (mainClass._eat_client.call(esrv))
                            {
                                if (esrv.response.picked.header.frame_id != "")
                                {
                                    id = esrv.response.picked.header.frame_id;
                                    if (id[0] == 'b')
                                    {
                                        got.data.at(0) = 1;
                                    }
                                    else if (id[0] == 'y')
                                    {
                                        got.data.at(1) = 1;
                                    }
                                    else if (id[0] == 'p')
                                    {
                                        got.data.at(2) = 1;
                                    }
                                    mainClass._got_cake_color.publish(got);
                                    for (int i = 0;i < 2;i++)
                                    {
                                        mainClass.poseStamped_set(1, cake_picked[i], esrv.response.picked.poses[i].position.x, esrv.response.picked.poses[i].position.y, esrv.response.picked.poses[i].orientation.z, esrv.response.picked.poses[i].orientation.w);
                                    }
                                    got_cake_picked = true;
                                    cakeTime = ros::Time::now();
                                }
                            }
                        }
                        else if (got_cake_picked)
                        {
                            if (!moving && !doing)
                            {
                                if (cake_picked[cakeNum].pose.position.x < 0)
                                {
                                    now_Camera_Mode = NO_CAM;
                                    got_cake_picked = false;
                                    cakeNum = 0;
                                    eatornot = false;
                                    cam_pub_once = false;
                                }
                                else
                                {
                                    if (route_failed)
                                    {
                                        route_failed = false;
                                        if (cakeNum < 1)
                                        {
                                            cakeNum ++;
                                        }
                                        else
                                        {
                                            now_Camera_Mode = NO_CAM;
                                            got_cake_picked = false;
                                            cakeNum = 0;
                                            eatornot = false;
                                            cam_pub_once = false;
                                        }
                                    }
                                    else if (!arrived && !mission_success)
                                    {
                                        if (cakeNum % 2 == 0)
                                        {
                                            missionStr.data = id[cakeNum];
                                            missionStr.data += id[cakeNum+1];
                                            mainClass._mission.publish(missionStr);
                                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                        }
                                        mainClass._where2go.publish(cake_picked[cakeNum]);
                                        ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", cake_picked[cakeNum].pose.position.x, cake_picked[cakeNum].pose.position.y);
                                        moving = true;
                                    }
                                    else if (arrived)
                                    {
                                        arrived = false;
                                        if (cakeNum % 2 == 1)
                                        {
                                            missionStr.data.at(0) = 'c';
                                            mainClass._mission.publish(missionStr);
                                            ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                            doing = true;
                                            startMissionTime = ros::Time::now().toSec();
                                        }
                                        else
                                        {
                                            if (cakeNum < 1)
                                            {
                                                cakeNum++;
                                            }
                                        }
                                    }
                                    else if (mission_success)
                                    {
                                        mission_success = false;
                                        if (cakeNum < 1)
                                        {
                                            cakeNum++;
                                        }
                                        else
                                        {
                                            now_Camera_Mode = NO_CAM;
                                            got_cake_picked = false;
                                            cakeNum = 0;
                                            eatornot = false;
                                            cam_pub_once = false;
                                        }
                                    }
                                }
                            }
                            else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                            {
                                moving = false;
                                doing = false;
                                arrived = false;
                                mission_success = false;
                                ROS_WARN("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                                if (cakeNum < 1)
                                {
                                    cakeNum ++;
                                }
                                else
                                {
                                    now_Camera_Mode = NO_CAM;
                                    got_cake_picked = false;
                                    cakeNum = 0;
                                    eatornot = false;
                                    cam_pub_once = false;
                                }
                            }
                        }
                        break;
                    }
                    
                    break;
                
                case CHERRY:
                    
                    mission_timeOut = 10;
                    
                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                    }
                    else if (!got_cherry_picked)
                    {
                        if (cherryE[0] != 0 || cherryE[1] != 0 || cherryE[2] != 0 || cherryE[3] != 0)
                        {
                            if (ros::Time::now().toSec() - cherryTime.toSec() >= cherryDelay && !got_cherry_picked)
                            {
                                if (mainClass._cherry_client.call(srv))
                                {
                                    if (srv.response.picked.header.frame_id != "")
                                    {
                                        cid = srv.response.picked.header.frame_id;
                                        if (srv.response.picked.poses[0].position.x != cherry_picked[0].pose.position.x)
                                        {
                                            for (int i = 0;i < 2;i++)
                                            {
                                                mainClass.poseStamped_set(i%2, cherry_picked[i], srv.response.picked.poses[i].position.x, srv.response.picked.poses[i].position.y, srv.response.picked.poses[i].orientation.z, srv.response.picked.poses[i].orientation.w);
                                            }
                                        }
                                        else
                                        {
                                            moving = true;
                                        }
                                        got_cherry_picked = true;
                                        cherryTime = ros::Time::now();
                                    }
                                }
                            }
                        }
                        else
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            now_Mission = BASKET;
                        }
                    }
                    else if (got_cherry_picked && cherry_picked[0].pose.position.x < 0)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = BASKET;
                    }
                    else if (got_cherry_picked)
                    {
                        if (ros::Time::now().toSec() - cherryTime.toSec() >= 0.3 && cherryDelay == 0 && moving && !doing)
                        {
                            got_cherry_picked = false;
                            moving = false;
                        }
                        else if (!moving && !doing)
                        {
                            if (route_failed)
                            {
                                route_failed = false;
                                now_Mission = BASKET;
                            }
                            else if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(cherry_picked[cherryNum]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", cherry_picked[cherryNum].pose.position.x, cherry_picked[cherryNum].pose.position.y);
                                moving = true;
                            }
                            else if (arrived)
                            {
                                arrived = false;
                                if (cherryNum == 0)
                                {
                                    missionStr.data = "s"+cid;
                                }
                                else
                                {
                                    missionStr.data = "v0";
                                }
                                mainClass._mission.publish(missionStr);
                                ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                doing = true;
                                startMissionTime = ros::Time::now().toSec();
                            }
                            else if (mission_success)
                            {
                                mission_success = false;
                                if (cherryNum < 1)
                                {
                                    cherryNum++;
                                }
                                else
                                {
                                    now_Mission = BASKET;
                                }
                            }
                        }
                        else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            ROS_WARN("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                            now_Mission = BASKET;
                        }
                    }
                    break;

                case BASKET:
                    
                    mission_timeOut = 15;
                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                        moving = false;
                    }
                    else if (who_basket == -1)
                    {
                        basket_robot.data = robot;
                        mainClass._ibasket.publish(basket_robot);
                    }
                    else if (who_basket != robot)
                    {
                        if (cherryE[0] == 0 && cherryE[1] == 0 && cherryE[2] == 0 && cherryE[3] == 0)
                        {
                            // now_Mission = STEAL;
                        }
                        else
                        {
                            now_Mission = CHERRY;
                            ROS_WARN("Cherry again !!!");
                            cherryNum = 0;
                            got_cherry_picked = false;
                            cherryTime = ros::Time::now();
                        }
                    }
                    else
                    {
                        if (!moving && !doing)
                        {
                            if (route_failed)
                            {
                                route_failed = false;
                                now_Mission = RELEASE;
                                basket_robot.data = -1;
                                mainClass._ibasket.publish(basket_robot);
                            }
                            else if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(basket_point[side]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", basket_point[side].pose.position.x, basket_point[side].pose.position.y);
                                moving = true;
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
                                basket_robot.data = -1;
                                mainClass._ibasket.publish(basket_robot);
                            }
                        }
                        else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                        {
                            moving = false;
                            doing = false;
                            arrived = false;
                            mission_success = false;
                            ROS_WARN("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                            now_Mission = RELEASE;
                            basket_robot.data = -1;
                            mainClass._ibasket.publish(basket_robot);
                        }
                    }
                    break;

                case RELEASE:
                    
                    mission_timeOut = 10;

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                        moving = false;
                    }
                    else if (hanoiing)
                    {
                        if (release.data == 0 && !somewhere_once)
                        {
                            geometry_msgs::PoseStamped somewhere;
                            mainClass.poseStamped_set(0, somewhere, 2.775, 1.775, 0, 1);
                            mainClass._where2go.publish(somewhere);
                            ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", somewhere.pose.position.x, somewhere.pose.position.y);
                            somewhere_once = true;
                        }
                        if (arrived)
                        {
                            arrived = false;
                        }
                    }
                    else if (!got_release_point && !hanoiing)
                    {
                        if (release.data == 0 || release.data == robot+1)
                        {
                            release.data = robot+1;
                            mainClass._release.publish(release);
                            eurobot2023_main_small::release rsrv;
                            rsrv.request.num = 0;
                            if (mainClass._release_client.call(rsrv))
                            {
                                if (rsrv.response.picked.header.frame_id != "")
                                {
                                    rid = rsrv.response.picked.header.frame_id;
                                    for (int i = 0;i < 4;i++)
                                    {
                                        mainClass.poseStamped_set(i%2, release_point[i], rsrv.response.picked.poses[i].position.x, rsrv.response.picked.poses[i].position.y, rsrv.response.picked.poses[i].orientation.z, rsrv.response.picked.poses[i].orientation.w);
                                    }
                                    got_release_point = true;
                                    hanoiing = false;
                                    // cout << "rid : " << rid << endl;
                                }
                            }
                        }
                        else
                        {
                            eurobot2023_main_small::release rsrv;
                            rsrv.request.num = 1;
                            if (mainClass._release_client.call(rsrv))
                            {
                                if (rsrv.response.picked.header.frame_id != "")
                                {
                                    rid = rsrv.response.picked.header.frame_id;
                                    for (int i = 0;i < 4;i++)
                                    {
                                        mainClass.poseStamped_set(i%2, release_point[i], rsrv.response.picked.poses[i].position.x, rsrv.response.picked.poses[i].position.y, rsrv.response.picked.poses[i].orientation.z, rsrv.response.picked.poses[i].orientation.w);
                                    }
                                    got_release_point = true;
                                    hanoiing = false;
                                    // cout << "rid : " << rid << endl;
                                }
                            }
                        }
                    }
                    else if (got_release_point)
                    {
                        if (!moving && !doing)
                        {
                            if (route_failed)
                            {
                                route_failed = false;
                                if (reCake < 3)
                                {
                                    reCake++;
                                }
                                else
                                {
                                    now_Mission = STEAL;
                                }
                            }
                            if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(release_point[reCake]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", release_point[reCake].pose.position.x, release_point[reCake].pose.position.y);
                                moving = true;
                            }
                            else if (arrived)
                            {
                                arrived = false;
                                if (reCake == 0)
                                {
                                    missionStr.data = 'o';
                                    missionStr.data += rid[0];
                                }
                                else if (reCake == 1)
                                {
                                    missionStr.data.at(0) = 'c';
                                    mainClass._mission.publish(missionStr);
                                }
                                else if (reCake == 2)
                                {
                                    missionStr.data = 'o';
                                    missionStr.data += rid[1];
                                    mainClass._mission.publish(missionStr);
                                    ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                    missionStr.data.at(1) = char(int(rid[1])+1);
                                }
                                else if (reCake == 3)
                                {
                                    missionStr.data.at(0) = 'c';
                                    mainClass._mission.publish(missionStr);
                                    ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                                    missionStr.data.at(1) = char(int(rid[1]));
                                }
                                doing = true;
                                startMissionTime = ros::Time::now().toSec();
                                mainClass._mission.publish(missionStr);
                                ROS_INFO("Mission [%s] published!", missionStr.data.c_str());
                            }
                            else if (mission_success)
                            {
                                mission_success = false;
                                if (reCake < 3)
                                {
                                    reCake++;
                                }
                                else
                                {
                                    now_Mission = STEAL;
                                }
                            }
                        }
                    }
                    else if (doing && ros::Time::now().toSec() - startMissionTime >= mission_timeOut)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        ROS_WARN("===== Mission [%s] overtime! =====", missionStr.data.c_str());
                        now_Mission = STEAL;
                    } 
                    break;

                case STEAL:
                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                        moving = false;
                    }
                    else if (!got_steal_cake)
                    {
                        now_Mission = HOME;
                        eurobot2023_main_small::steal ssrv;
                        if (mainClass._steal_client.call(ssrv))
                        {
                            if (srv.response.picked.header.frame_id != "")
                            {
                                mainClass.poseStamped_set(0, steal_picked, ssrv.response.picked.pose.position.x, ssrv.response.picked.pose.position.y, ssrv.response.picked.pose.orientation.z, ssrv.response.picked.pose.orientation.w);
                                got_steal_cake=true;
                            }
                        }
                    }
                    else if (got_steal_cake && !hanoiing)
                    {
                        int emptySide = -1;
                        for (int i = 0;i < 4;i++)
                        {
                            if (fullness[i] == 0)
                            {
                                emptySide = i;
                                break;
                            }
                        }
                    }
                    
                    break;
                
                case HOME:
                    going_home = true;
                    if (!moving)
                    {
                        if (!arrived)
                        {
                            mainClass._where2go.publish(home[robot][side]);
                            ROS_INFO("Heading over to x:[%.3f] y:[%.3f]", home[robot][side].pose.position.x, home[robot][side].pose.position.y);
                            moving = true;
                        }
                        else
                        {
                            missionStr.data = "d0";
                            mainClass._mission.publish(missionStr);
                            now_Status = FINISH;
                            printOnce = false;
                        }
                    }
                    break;
                }
                break;

            case FINISH:
                if (!printOnce)
                {
                    missionStr.data = "f0";
                    mainClass._mission.publish(missionStr);
                    ROS_INFO("Finish all missions!");
                    ROS_INFO("Robot%d finish time : %.1f", robot+1, ros::Time::now().toSec() - initialTime.toSec());
                    if (!finish_mission.data)
                    {
                        finish_mission.data = true;
                        mainClass._ifinish.publish(finish_mission);
                    }
                    else
                    {
                        ROS_INFO("Total time : %.1f", ros::Time::now().toSec() - initialTime.toSec());
                    }
                    printOnce = true;
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