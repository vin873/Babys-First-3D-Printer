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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eurobot2023_main/cake.h>
#include <eurobot2023_main/cherry.h>
#include <eurobot2023_main/release.h>
#include <eurobot2023_main/steal.h>
#include <eurobot2023_main/eat.h>

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
int c_or_d = 0;
int cakeNum = 0;
int reCake = 0;
int lastCakeColor = -1;
int relative_point = -999;
int cherryNum = 0;
int cherryDelay = 1.5;
int who_basket = -1;
int basketNum = 0;
int plates[5] = {0, 0, 0, 0, 0};
int home_num = -1;
int pre_home = 0;
int now_Mission = CAKE;
int now_Camera_Mode = NO_CAM;
int now_Status = SETUP;
int now_Mode = NORMAL;
int doing_mode = CAKE;

int cakeNumber[3] = {-1, -1, -1};

double myPos_x;
double myPos_y;
double myOri_z;
double myOri_w;
double enemiesPos_x;
double enemiesPos_y;
double enemiesOri_z;
double enemiesOri_w;
double pub_time;
double mission_timeOut;
double camera_timeOut;
double startMissionTime;
double startCameraTime;
double releaseDelayTime;
double go_home_time = 90.0;

bool start = false;
bool moving = false;
bool doing = false;
bool arrived = false;
bool waitAr = false;
bool waitSTM = false;
bool deliveredAr = false;
bool deliveredSTM = false;
bool hanoiing = false;
bool cam_pub_once = false;
bool eatornot = false;
bool is_rotate = false;
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
bool printOnce = false;
bool mission_print = false;
bool pub_delay = false;

string id;
string cid;
string rid;
string id_frame = "path_10";
string dock_id_frame = "dock_5_mov_cake";
string dockc_id_frame = "dock_5_mov_cherry";
string dockr_id_frame = "dock_5_rot_cake";
string vibrate_id_frame = "dock_5_vibrate";
string quick_id_frame = "path_2";

std_msgs::Int32 camColor;
std_msgs::Int32 release;
std_msgs::Int32 basket_robot;
std_msgs::Int32 now_home;
std_msgs::Int32MultiArray got;
std_msgs::Bool anotherse;
std_msgs::Bool finish_mission;
std_msgs::String missionStr;

geometry_msgs::PoseStamped somewhere;
geometry_msgs::PoseStamped cake_picked[6];
geometry_msgs::PoseStamped steal_picked;
geometry_msgs::PoseStamped eat_picked[2];
geometry_msgs::PoseStamped cherry_picked[2];
geometry_msgs::PoseStamped basket_point[2];
geometry_msgs::PoseStamped release_point[4];
geometry_msgs::PoseStamped home[2][8];

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
        _all_set_up = nh.advertise<std_msgs::Bool>("allSetUp"+to_string(robot), 1000);
        _another_setup = nh.subscribe<std_msgs::Bool>("allSetUp"+to_string(!bool(robot)), 1000, &mainProgram::anotherse_callback, this);
        _cakeNum = nh.subscribe<std_msgs::Int32MultiArray>("cakeNum"+to_string(robot), 1000, &mainProgram::cakeNum_callback, this);
        _got_cake_color = nh.advertise<std_msgs::Int32MultiArray>("gotcake"+to_string(robot), 1000);
        _relative_where = nh.subscribe<geometry_msgs::Point>("/robot"+to_string(robot+1)+"onRobot/relative_where", 1000, &mainProgram::relative_callback, this);
        _release = nh.advertise<std_msgs::Int32>("release"+to_string(robot), 1000);
        _changeHome = nh.advertise<std_msgs::Int32>("changeHome"+to_string(robot), 1000);
        _another_release = nh.subscribe<std_msgs::Int32>("release"+to_string(!bool(robot)), 1000, &mainProgram::anothere_callback, this);
        _cake_client = nh.serviceClient<eurobot2023_main::cake>("cake"+to_string(robot));
        _cherry_client = nh.serviceClient<eurobot2023_main::cherry>("cherry"+to_string(robot));
        _release_client = nh.serviceClient<eurobot2023_main::release>("release"+to_string(robot));
        _steal_client = nh.serviceClient<eurobot2023_main::steal>("steal"+to_string(robot));
        _eat_client = nh.serviceClient<eurobot2023_main::eat>("eat"+to_string(robot));
        _where2go = nh.advertise<geometry_msgs::PoseStamped>("/robot"+to_string(robot+1)+"/mission", 1000);
        _another_change = nh.subscribe<std_msgs::Int32>("changeHome"+to_string(!bool(robot)), 1000, &mainProgram::anotherch_callback, this);
        _finishornot = nh.subscribe<std_msgs::Bool>("/robot"+to_string(robot+1)+"/is_finish", 1000, &mainProgram::nav_callback, this);
        _mission = nh.advertise<std_msgs::String>("mission"+to_string(robot), 1000);
        _donefullness = nh.subscribe<std_msgs::Int16MultiArray>("donefullness"+to_string(robot), 1000, &mainProgram::done_fullness_callback, this);
        _handshakerAr = nh.subscribe<std_msgs::String>("handshaker"+to_string(robot), 1000, &mainProgram::handshakerAr_callback, this);
        _handshakerSTM = nh.subscribe<std_msgs::String>("handshakier"+to_string(robot), 1000, &mainProgram::handshakerSTM_callback, this);
        _myPos = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot"+to_string(robot+1)+"/ekf_pose", 1000, &mainProgram::myPos_callback, this);
    }


    float q2e(double x, double y, double z, double w)
    {
        double t3, t4, yaw_z;
    
        t3 = +2.0 * (w * z + x * y);
        t4 = +1.0 - 2.0 * (y * y + z * z);
        yaw_z = atan2(t3, t4) / M_PI * 180;
    
        return yaw_z;
    }

    void pub_till_get(std_msgs::String &misStr)
    {
        _mission.publish(misStr);
        ROS_INFO("Mission [%s] published!", misStr.data.c_str());
        // waitAr = true;
        // if (misStr.data.at(0) == 'b' || misStr.data.at(0) == 'y' || misStr.data.at(0) == 'p' || misStr.data.at(0) == 'h')
        // {
        //     waitSTM = true;
        // }
        // ros::Rate rated(20);
        // while ((!deliveredAr || !deliveredSTM) && ros::ok())
        // {
        //     if (misStr.data.at(0) != 'b' && misStr.data.at(0) != 'y' && misStr.data.at(0) != 'p' && misStr.data.at(0) != 'h' && deliveredAr)
        //     {
        //         break;
        //     }
        //     _mission.publish(misStr);
        //     ros::spinOnce();
        //     rated.sleep();
        // }
        // waitAr = false;
        // waitSTM = false;
        // deliveredAr = false;
        // deliveredSTM = false;
        // ROS_INFO("Mission delivered!");
    }

    void handshakerAr_callback(const std_msgs::String::ConstPtr &msg)
    {
        if (waitAr && msg->data != "" && msg->data.at(0) == missionStr.data.at(0) && msg->data.at(1) == missionStr.data.at(1))
        {
            deliveredAr = true;
        }
    }

    void handshakerSTM_callback(const std_msgs::String::ConstPtr &msg)
    {   
        if (waitSTM && msg->data != "" && msg->data.at(0) == missionStr.data.at(0) && msg->data.at(1) == missionStr.data.at(1))
        {
            deliveredSTM = true;
        }
    }

    void poseStamped_set(int dock,geometry_msgs::PoseStamped &pos, float x, float y, float z, float w)
    {
        if (dock == 0)
        {
            pos.header.frame_id = id_frame;
        }
        else if (dock == 1)
        {
            pos.header.frame_id = dock_id_frame;
        }
        else if (dock == 2)
        {
            pos.header.frame_id = dockc_id_frame;
        }
        else if (dock == 3)
        {
            pos.header.frame_id = dockr_id_frame;
        }
        else if (dock == 4)
        {
            pos.header.frame_id = vibrate_id_frame;
        }
        else if (dock == 5)
        {
            pos.header.frame_id = quick_id_frame;
        }
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = x;
        pos.pose.position.y = y;
        pos.pose.orientation.x = 0.0;
        pos.pose.orientation.y = 0.0;
        pos.pose.orientation.z = z;
        pos.pose.orientation.w = w;
    }

    void anotherse_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        anotherse.data = true;
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
        if (moving)
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
    }

    void done_fullness_callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
    {
        if (hanoiing && msg->data.at(0) == 2)
        {
            hanoiing = false;
            for (int i = 1;i < 5;i++)
            {
                fullness[i-1] = msg->data.at(i);
            }
            ROS_WARN("Finished hanoiing!");
        }
        else if (doing)
        {
            if (msg->data.at(0) != 2)
            {
                doing = false;
            }
            if (msg->data.at(0) == 1)
            {
                mission_success = true;
                ROS_INFO("Mission finished!");
            }
            else if (msg->data.at(0) == 0)
            {
                ROS_ERROR("Mission failed!");
            }
        }
    }

    void start_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            start = true;
        }
    }

    void relative_callback(const geometry_msgs::Point::ConstPtr &msg)
    {
        relative_point = msg->z;
    }

    void cakeNum_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        for (int i = 0;i < 3;i++)
        {
            cakeNumber[i] = msg->data.at(i);
            // cout << cakeNumber[i] << endl;
        }
    }

    void cake_callback(const geometry_msgs::Point::ConstPtr &msg)
    {
        // ROS_WARN("Cake update!");
        int num;
        if (cakeNum % 2 == 1)
        {
            // num = (cakeNum-1)/2;
        }
        else
        {
            num = (cakeNum)/2;
            if (got_cake_picked)
            {
                for (int i = num;i < 3;i++)
                {
                    // cout << msg->z << " " << cakeNumber[i] << endl;
                    if (int(msg->z) == cakeNumber[i])
                    {
                        now_Camera_Mode = NO_CAM;
                        got_cake_picked = false;
                        eatornot = false;
                        cam_pub_once = false;
                        cakeNum = 0;
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        break;
                    }
                }
            }
        }
    }

    void cherry_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (now_Status != SETUP)
        {
            for (int i = 0;i < 4;i++)
            {
                cherryE[i] = msg->data.at(i);
            }
            ROS_WARN("Cherry update! [%d, %d, %d, %d]", cherryE[0], cherryE[1], cherryE[2], cherryE[3]);
            int checkId;
            if (cid == "4" || cid == "0")
            {
                checkId = 0;
            }
            else if (cid == "5" || cid == "2")
            {
                checkId = 2;
            }
            else if (cid == "1")
            {
                checkId = 1;
            }
            else if (cid == "3")
            {
                checkId = 3;
            }
            if (now_Mission == CHERRY && got_cherry_picked && cherryE[checkId] == 0 && !doing && !mission_success && cherryNum == 0)
            {
                got_cherry_picked = false;
                moving = false;
                doing = false;
                arrived = false;
                mission_success = false;
                cherryNum = 0;
            }
        }
    }

    void plate_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        for (int i = 0;i < 5;i++)
        {
            plates[i] = msg->data.at(i);
        }
    }

    void anotherch_callback(const std_msgs::Int32::ConstPtr &msg)
    {
        if (now_Mission == HOME)
        {
            if (msg->data == -1)
            {
                home_num = msg->data;
            }
            else
            {
                home_num = msg->data;
                poseStamped_set(0, home[side][home_num], home[side][home_num].pose.position.x, home[side][home_num].pose.position.y, myOri_z, myOri_w);
                _where2go.publish(home[side][home_num]);
                ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", home[side][home_num].pose.position.x, home[side][home_num].pose.position.y, q2e(0, 0, home[side][home_num].pose.orientation.z, home[side][home_num].pose.orientation.w));
                moving = true;
            }
        }
        else
        {
            if (msg->data == -1)
            {
                pre_home = msg->data;
            }
        }
    }

    void myPos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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
    ros::Publisher _all_set_up = nh.advertise<std_msgs::Bool>("allSetUp"+to_string(robot), 1000);
    ros::Publisher _got_cake_color = nh.advertise<std_msgs::Int32MultiArray>("gotcake"+to_string(robot), 1000);
    ros::Publisher _ibasket = nh.advertise<std_msgs::Int32>("basketornot", 1000);
    ros::Publisher _release = nh.advertise<std_msgs::Int32>("release"+to_string(robot), 1000);
    ros::Publisher _plates = nh.advertise<std_msgs::Int32MultiArray>("plates", 1000);
    ros::Publisher _changeHome = nh.advertise<std_msgs::Int32>("changeHome"+to_string(robot), 1000);
    ros::Publisher _ifinish = nh.advertise<std_msgs::Bool>("finishall", 1000);
    ros::Subscriber _another_setup = nh.subscribe<std_msgs::Bool>("allSetUp"+to_string(!bool(robot)), 1000, &mainProgram::anotherse_callback, this);
    ros::Subscriber _cakeNum = nh.subscribe<std_msgs::Int32MultiArray>("cakeNum"+to_string(robot), 1000, &mainProgram::cakeNum_callback, this);
    ros::Subscriber _got_what_color = nh.subscribe<std_msgs::Int32MultiArray>("gotcake"+to_string(robot), 1000, &mainProgram::what_color_cake_callback, this);
    ros::Subscriber _basketornot = nh.subscribe<std_msgs::Int32>("basketornot", 1000, &mainProgram::basket_callback, this);
    ros::Subscriber _another_release = nh.subscribe<std_msgs::Int32>("release"+to_string(!bool(robot)), 1000, &mainProgram::anothere_callback, this);
    ros::Subscriber _plate_full = nh.subscribe<std_msgs::Int32MultiArray>("plates", 1000, &mainProgram::plate_callback, this);
    ros::Subscriber _another_change = nh.subscribe<std_msgs::Int32>("changeHome"+to_string(!bool(robot)), 1000, &mainProgram::anotherch_callback, this);
    ros::Subscriber _finishall = nh.subscribe<std_msgs::Bool>("finishall", 1000, &mainProgram::finishall_callback, this);
    ros::ServiceClient _cake_client = nh.serviceClient<eurobot2023_main::cake>("cake"+to_string(robot));
    ros::ServiceClient _cherry_client = nh.serviceClient<eurobot2023_main::cherry>("cherry"+to_string(robot));
    ros::ServiceClient _release_client = nh.serviceClient<eurobot2023_main::release>("release"+to_string(robot));
    ros::ServiceClient _steal_client = nh.serviceClient<eurobot2023_main::steal>("steal"+to_string(robot));
    ros::ServiceClient _eat_client = nh.serviceClient<eurobot2023_main::eat>("eat"+to_string(robot));

    // chassis
    ros::Publisher _where2go = nh.advertise<geometry_msgs::PoseStamped>("/robot"+to_string(robot+1)+"/mission", 1000);
    ros::Subscriber _finishornot = nh.subscribe<std_msgs::Bool>("/robot"+to_string(robot+1)+"/is_finish", 1000, &mainProgram::nav_callback, this);

    // mission
    ros::Publisher _mission = nh.advertise<std_msgs::String>("mission"+to_string(robot), 1000);
    ros::Subscriber _handshakerAr = nh.subscribe<std_msgs::String>("handshaker"+to_string(robot), 1000, &mainProgram::handshakerAr_callback, this);
    ros::Subscriber _handshakerSTM = nh.subscribe<std_msgs::String>("handshakier"+to_string(robot), 1000, &mainProgram::handshakerSTM_callback, this);
    ros::Subscriber _donefullness = nh.subscribe<std_msgs::Int16MultiArray>("donefullness"+to_string(robot), 1000, &mainProgram::done_fullness_callback, this);
    ros::Subscriber _startornot = nh.subscribe<std_msgs::Bool>("startornot", 1000, &mainProgram::start_callback, this);

    // basket
    ros::Publisher _point_home = nh.advertise<std_msgs::Int32>("point_home", 1000);

    // camera
    ros::Publisher _cam_which_color = nh.advertise<std_msgs::Int32>("cam_which_color", 1000);
    ros::Subscriber _relative_where = nh.subscribe<geometry_msgs::Point>("/robot"+to_string(robot+1)+"onRobot/relative_where", 1000, &mainProgram::relative_callback, this);
    ros::Subscriber _ajustCake = nh.subscribe<geometry_msgs::Point>("adjustCake", 1000, &mainProgram::cake_callback, this);
    ros::Subscriber _cherryExistence = nh.subscribe<std_msgs::Int32MultiArray>("cherryExistence", 1000, &mainProgram::cherry_callback, this);

    // locate
    ros::Publisher _obstacles = nh.advertise<geometry_msgs::PoseArray>("obstacles", 1000);
    ros::Subscriber _myPos = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot"+to_string(robot+1)+"/ekf_pose", 1000, &mainProgram::myPos_callback, this);
    ros::Subscriber _enemiesPos = nh.subscribe<geometry_msgs::PoseStamped>("enemiesPos", 1000, &mainProgram::enemiesPos_callback, this);
};

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Babies_First_CNC");

    mainProgram mainClass;
    mainClass.initialize();

    ros::Time initialTime = ros::Time::now();
    ros::Time cakeTime = ros::Time::now();
    ros::Time cherryTime = ros::Time::now();

    eurobot2023_main::cake csrv;
    eurobot2023_main::cherry srv;
    eurobot2023_main::eat esrv;
    eurobot2023_main::release rsrv;
    eurobot2023_main::steal ssrv;

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

                    mainClass.poseStamped_set(0, basket_point[0], 0.225, 0.225, 1, 0);
                    mainClass.poseStamped_set(0, basket_point[1], 0.225, 1.775, 1, 0);
                    mainClass.poseStamped_set(0, home[0][0], 1.230, 1.775, 0, 1);
                    mainClass.poseStamped_set(0, home[0][1], 0.870, 1.550, 0, 1);
                    mainClass.poseStamped_set(0, home[0][2], 1.770, 0.225, 0, 1);
                    mainClass.poseStamped_set(0, home[0][3], 2.130, 0.450, 0, 1);
                    mainClass.poseStamped_set(0, home[0][4], 2.800, 0.800, 0, 1);
                    mainClass.poseStamped_set(0, home[0][5], 2.500, 0.600, 0, 1);
                    mainClass.poseStamped_set(0, home[0][6], 2.800, 1.800, 0, 1);
                    mainClass.poseStamped_set(0, home[0][7], 2.500, 1.600, 0, 1);

                    mainClass.poseStamped_set(0, home[1][0], 1.230, 0.225, 0, 1);
                    mainClass.poseStamped_set(0, home[1][1], 0.870, 0.450, 0, 1);
                    mainClass.poseStamped_set(0, home[1][2], 1.770, 1.775, 0, 1);
                    mainClass.poseStamped_set(0, home[1][3], 2.130, 1.550, 0, 1);
                    mainClass.poseStamped_set(0, home[1][4], 2.800, 1.200, 0, 1);
                    mainClass.poseStamped_set(0, home[1][5], 2.500, 1.400, 0, 1);
                    mainClass.poseStamped_set(0, home[1][6], 2.800, 0.200, 0, 1);
                    mainClass.poseStamped_set(0, home[1][7], 2.500, 0.400, 0, 1);

                    anotherse.data = false;
                    mainClass._all_set_up.publish(anotherse);
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

                csrv.request.num = c_or_d;
                if (ros::Time::now().toSec() - cakeTime.toSec() >= 0.4 && mainClass._cake_client.call(csrv))
                {
                    if (csrv.response.picked.header.frame_id != "")
                    {
                        id = csrv.response.picked.header.frame_id;
                        for (int i = 0;i < 6;i++)
                        {
                            mainClass.poseStamped_set(i%2, cake_picked[i], csrv.response.picked.poses[i].position.x, csrv.response.picked.poses[i].position.y, csrv.response.picked.poses[i].orientation.z, csrv.response.picked.poses[i].orientation.w);
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

                if (now_Mission != HOME && now_Mission != FINISH && ros::Time::now().toSec() - initialTime.toSec() >= 98)
                {
                    missionStr.data = "d0";
                    mainClass.pub_till_get(missionStr);
                    now_Status = FINISH;
                    printOnce = false;
                }

                switch (now_Mission)
                {
                case CAKE:

                    mission_timeOut = 3;

                    if (!mission_print)
                    {
                        ROS_WARN("===== Cake !!! =====");
                        mission_print = true;
                    }

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                    }

                    switch (now_Camera_Mode)
                    {
                    case NO_CAM:
                        if (!got_cake_picked)
                        {
                            csrv.request.num = c_or_d;
                            if (mainClass._cake_client.call(csrv))
                            {
                                if (csrv.response.picked.header.frame_id != "")
                                {
                                    id = csrv.response.picked.header.frame_id;
                                    for (int i = 0;i < 6;i++)
                                    {
                                        mainClass.poseStamped_set(i%2, cake_picked[i], csrv.response.picked.poses[i].position.x, csrv.response.picked.poses[i].position.y, csrv.response.picked.poses[i].orientation.z, csrv.response.picked.poses[i].orientation.w);
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
                                        mission_print = false;
                                        missionStr.data = "h0";
                                        mainClass.pub_till_get(missionStr);
                                        hanoiing = true;
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
                                            mission_print = false;
                                            missionStr.data = "h0";
                                            mainClass.pub_till_get(missionStr);
                                            hanoiing = true;
                                        }
                                    }
                                    else if (!arrived && !mission_success)
                                    {
                                        if (cakeNum % 2 == 0)
                                        {
                                            missionStr.data = id[cakeNum];
                                            if (missionStr.data.at(0) == 'b')
                                            {
                                                lastCakeColor = 1;
                                            }
                                            else if (missionStr.data.at(0) == 'y')
                                            {
                                                lastCakeColor = 2;
                                            }
                                            else if (missionStr.data.at(0) == 'p')
                                            {
                                                lastCakeColor = 3;
                                            }
                                            if (id[cakeNum+1] != '!')
                                            {
                                                missionStr.data += id[cakeNum+1];
                                                mainClass.pub_till_get(missionStr);
                                            }
                                        }

                                        mainClass._where2go.publish(cake_picked[cakeNum]);
                                        ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", cake_picked[cakeNum].pose.position.x, cake_picked[cakeNum].pose.position.y, mainClass.q2e(0, 0, cake_picked[cakeNum].pose.orientation.z, cake_picked[cakeNum].pose.orientation.w));
                                        moving = true;
                                    }
                                    else if (arrived)
                                    {
                                        arrived = false;
                                        if (cakeNum % 2 == 1)
                                        {
                                            if (missionStr.data.at(0) == 'b')
                                            {
                                                lastCakeColor = 1;
                                                got.data.at(0) = 1;
                                            }
                                            else if (missionStr.data.at(0) == 'y')
                                            {
                                                lastCakeColor = 2;
                                                got.data.at(1) = 1;
                                            }
                                            else if (missionStr.data.at(0) == 'p')
                                            {
                                                lastCakeColor = 3;
                                                got.data.at(2) = 1;
                                            }
                                            doing = true;
                                            missionStr.data.at(0) = 'c';
                                            mainClass.pub_till_get(missionStr);
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
                                            mainClass._got_cake_color.publish(got);
                                            cakeNum++;
                                        }
                                        else
                                        {
                                            now_Mission = CHERRY;
                                            mission_print = false;
                                            missionStr.data = "h0";
                                            mainClass.pub_till_get(missionStr);
                                            hanoiing = true;
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
                                    mission_print = false;
                                    missionStr.data = "h0";
                                    mainClass.pub_till_get(missionStr);
                                    hanoiing = true;
                                }
                            }
                        }
                        break;
                    
                    case USE_CAM:
                        
                        camera_timeOut = 4;

                        if (!got_cake_picked)
                        {
                            if (!cam_pub_once)
                            {
                                camColor.data = lastCakeColor;
                                mainClass._cam_which_color.publish(camColor);
                                ROS_WARN("Using camera !!");
                                startCameraTime = ros::Time::now().toSec();
                                cam_pub_once = true;
                            }
                            else if (ros::Time::now().toSec() - startCameraTime >= camera_timeOut)
                            {
                                c_or_d = 1;
                                ROS_ERROR("Failed to turn on camera!");
                                if (doing_mode == STEAL)
                                {
                                    now_Mission = HOME;
                                    mission_print = false;
                                    camColor.data = 0;
                                    mainClass._cam_which_color.publish(camColor);
                                    break;
                                }
                                now_Camera_Mode = NO_CAM;
                                camColor.data = 0;
                                mainClass._cam_which_color.publish(camColor);
                                got_cake_picked = false;
                                cakeNum = 0;
                                eatornot = false;
                                cam_pub_once = false;
                            }
                            esrv.request.color = lastCakeColor;
                            if (mainClass._eat_client.call(esrv))
                            {
                                if (esrv.response.picked.header.frame_id != "")
                                {
                                    id = esrv.response.picked.header.frame_id;
                                    int dockmode = 0;
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
                                    else if (id[0] == 'x')
                                    {
                                        dockmode = 2;
                                        is_rotate = true;
                                    }
                                    for (int i = 0;i < 2;i++)
                                    {
                                        mainClass.poseStamped_set(dockmode, cake_picked[i], esrv.response.picked.poses[i].position.x, esrv.response.picked.poses[i].position.y, esrv.response.picked.poses[i].orientation.z, esrv.response.picked.poses[i].orientation.w);
                                    }
                                    got_cake_picked = true;
                                    cakeTime = ros::Time::now();
                                }
                            }
                        }
                        else if (got_cake_picked)
                        {
                            // cout << is_rotate << "   " << relative_point << endl;
                            if (relative_point == -2 && is_rotate)
                            {
                                mainClass.poseStamped_set(1, somewhere, myPos_x, myPos_y, myOri_z, myOri_w);
                                mainClass._where2go.publish(somewhere);
                                ROS_WARN("STOP!!!");
                                arrived = false;
                                is_rotate = false;
                                got_cake_picked = false;
                                cakeNum = 0;
                                // cam_pub_once = false;
                                moving = false;
                            }
                            else if (!moving && !doing)
                            {
                                if (cake_picked[cakeNum].pose.position.x < 0)
                                {
                                    if (!is_rotate)
                                    {
                                        if (doing_mode == STEAL)
                                        {
                                            now_Mission = HOME;
                                            mission_print = false;
                                            camColor.data = 0;
                                            mainClass._cam_which_color.publish(camColor);
                                            break;
                                        }
                                        now_Camera_Mode = NO_CAM;
                                        camColor.data = 0;
                                        mainClass._cam_which_color.publish(camColor);
                                        cam_pub_once = false;
                                    }
                                    got_cake_picked = false;
                                    cakeNum = 0;
                                    eatornot = false;
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
                                            if (!is_rotate)
                                            {
                                                if (doing_mode == STEAL)
                                                {
                                                    now_Mission = HOME;
                                                    mission_print = false;
                                                    camColor.data = 0;
                                                    mainClass._cam_which_color.publish(camColor);
                                                    break;
                                                }
                                                now_Camera_Mode = NO_CAM;
                                                camColor.data = 0;
                                                mainClass._cam_which_color.publish(camColor);
                                                cam_pub_once = false;
                                            }
                                            got_cake_picked = false;
                                            cakeNum = 0;
                                            eatornot = false;
                                        }
                                    }
                                    else if (!arrived && !mission_success)
                                    {
                                        if (!is_rotate)
                                        {
                                             if (cakeNum % 2 == 0)
                                            {
                                                missionStr.data = id[cakeNum];
                                                missionStr.data += id[cakeNum+1];
                                                mainClass.pub_till_get(missionStr);
                                            }
                                        }
                                        mainClass._where2go.publish(cake_picked[cakeNum]);
                                        ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", cake_picked[cakeNum].pose.position.x, cake_picked[cakeNum].pose.position.y, mainClass.q2e(0, 0, cake_picked[cakeNum].pose.orientation.z, cake_picked[cakeNum].pose.orientation.w));
                                        moving = true;
                                    }
                                    else if (arrived)
                                    {
                                        arrived = false;
                                        if (!is_rotate)
                                        {
                                            if (cakeNum % 2 == 1)
                                            {
                                                doing = true;
                                                missionStr.data.at(0) = 'c';
                                                mainClass.pub_till_get(missionStr);
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
                                        else
                                        {
                                            if (cakeNum < 1)
                                            {
                                                cakeNum++;
                                            }
                                            else
                                            {
                                                c_or_d = 1;
                                                if (doing_mode == STEAL)
                                                {
                                                    now_Mission = RELEASE;
                                                    mission_print = false;
                                                    camColor.data = 0;
                                                    mainClass._cam_which_color.publish(camColor);
                                                    break;
                                                }
                                                now_Camera_Mode = NO_CAM;
                                                camColor.data = 0;
                                                mainClass._cam_which_color.publish(camColor);
                                                got_cake_picked = false;
                                                cakeNum = 0;
                                                eatornot = false;
                                                cam_pub_once = false;
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
                                            mainClass._got_cake_color.publish(got);
                                            if (!is_rotate)
                                            {
                                                if (doing_mode == STEAL)
                                                {
                                                    now_Mission = RELEASE;
                                                    mission_print = false;
                                                    camColor.data = 0;
                                                    mainClass._cam_which_color.publish(camColor);
                                                    break;
                                                }
                                                now_Camera_Mode = NO_CAM;
                                                camColor.data = 0;
                                                mainClass._cam_which_color.publish(camColor);
                                                cam_pub_once = false;
                                            }
                                            got_cake_picked = false;
                                            cakeNum = 0;
                                            eatornot = false;
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
                                    if (doing_mode == STEAL)
                                    {
                                        now_Mission = RELEASE;
                                        mission_print = false;
                                        camColor.data = 0;
                                        mainClass._cam_which_color.publish(camColor);
                                        cam_pub_once = false;
                                        break;
                                    }
                                    now_Camera_Mode = NO_CAM;
                                    camColor.data = 0;
                                    mainClass._cam_which_color.publish(camColor);
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
                    
                    mission_timeOut = 3;

                    if (!mission_print)
                    {
                        ROS_WARN("===== Cherry !!! =====");
                        mission_print = true;
                    }
                    
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
                                                if (i % 2 == 0)
                                                {
                                                    mainClass.poseStamped_set(0, cherry_picked[i], srv.response.picked.poses[i].position.x, srv.response.picked.poses[i].position.y, srv.response.picked.poses[i].orientation.z, srv.response.picked.poses[i].orientation.w);
                                                }
                                                else
                                                {
                                                    mainClass.poseStamped_set(2, cherry_picked[i], srv.response.picked.poses[i].position.x, srv.response.picked.poses[i].position.y, srv.response.picked.poses[i].orientation.z, srv.response.picked.poses[i].orientation.w);
                                                }
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
                            mission_print = false;
                        }
                    }
                    else if (got_cherry_picked && cherry_picked[0].pose.position.x < 0)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        if (who_basket != -1 && who_basket != robot)
                        {
                            doing_mode = CHERRY;
                            now_Mission = STEAL;
                            mission_print = false;
                        }
                        else
                        {
                            doing_mode = BASKET;
                            now_Mission = BASKET;
                            mission_print = false;
                        }
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
                                mission_print = false;
                            }
                            else if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(cherry_picked[cherryNum]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", cherry_picked[cherryNum].pose.position.x, cherry_picked[cherryNum].pose.position.y, mainClass.q2e(0, 0, cherry_picked[cherryNum].pose.orientation.z, cherry_picked[cherryNum].pose.orientation.w));
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
                                doing = true;
                                mainClass.pub_till_get(missionStr);
                                startMissionTime = ros::Time::now().toSec();
                            }
                            else if (mission_success)
                            {
                                if (cherryNum < 1)
                                {
                                    cherryNum++;
                                }
                                else
                                {
                                    now_Mission = BASKET;
                                    mission_print = false;
                                }
                                mission_success = false;
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
                            mission_print = false;
                        }
                    }
                    break;

                case BASKET:
                    
                    mission_timeOut = 15;

                    if (!mission_print)
                    {
                        ROS_WARN("===== Basket !!! =====");
                        basketNum = 0;
                        mission_print = true;
                    }

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
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
                            doing_mode = CHERRY;
                            now_Mission = STEAL;
                            mission_print = false;
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
                                mission_print = false;
                                basket_robot.data = -1;
                                mainClass._ibasket.publish(basket_robot);
                            }
                            else if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(basket_point[side]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang[%.1f]", basket_point[side].pose.position.x, basket_point[side].pose.position.y, mainClass.q2e(0, 0, basket_point[side].pose.orientation.z, basket_point[side].pose.orientation.y));
                                moving = true;
                            }
                            else if (arrived)
                            {
                                arrived = false;
                                basketNum++;
                                if (basketNum == 1)
                                {
                                    mainClass.poseStamped_set(1, somewhere, 0.18, basket_point[side].pose.position.y, 1, 0);
                                    mainClass._where2go.publish(somewhere);
                                    ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang[%.1f]", somewhere.pose.position.x, somewhere.pose.position.y, mainClass.q2e(0, 0, somewhere.pose.orientation.z, somewhere.pose.orientation.y));
                                    moving = true;
                                }
                                else if (basketNum == 2)
                                {
                                    doing = true;
                                    missionStr.data = "u0";
                                    mainClass.pub_till_get(missionStr);
                                    // mainClass.poseStamped_set(4, somewhere, 0.16, basket_point[side].pose.position.y, 1, 0);
                                    // mainClass._where2go.publish(somewhere);
                                    startMissionTime = ros::Time::now().toSec();
                                }
                            }
                            else if (mission_success)
                            {
                                mission_success = false;
                                now_Mission = RELEASE;
                                mission_print = false;
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
                            mission_print = false;
                            basket_robot.data = -1;
                            mainClass._ibasket.publish(basket_robot);
                        }
                    }
                    break;

                case RELEASE:
                    
                    mission_timeOut = 4;

                    if (!mission_print)
                    {
                        ROS_WARN("===== Release !!! =====");
                        mission_print = true;
                    }

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                    }
                    else if (hanoiing)
                    {
                        if (ros::Time::now().toSec() - initialTime.toSec() >= 72 && !going_home)
                        {
                            ROS_WARN("Hanoiing over time !!");
                            hanoiing = false;
                        }
                        if (release.data == 0 && !somewhere_once)
                        {
                            release.data = robot+1;
                            mainClass._release.publish(release);
                            home_num = 0;
                            mainClass.poseStamped_set(5, somewhere, 2.775, 1.775, 0, 1);
                            mainClass._where2go.publish(somewhere);
                            ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", somewhere.pose.position.x, somewhere.pose.position.y, mainClass.q2e(0, 0, somewhere.pose.orientation.z, somewhere.pose.orientation.w));
                            somewhere_once = true;
                        }
                        else if (release.data != 0 && !somewhere_once)
                        {
                            home_num = 1;
                            mainClass.poseStamped_set(5, somewhere, 0.225, 0.225, 0, 1);
                            mainClass._where2go.publish(somewhere);
                            ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", somewhere.pose.position.x, somewhere.pose.position.y, mainClass.q2e(0, 0, somewhere.pose.orientation.z, somewhere.pose.orientation.w));
                            somewhere_once = true;
                        }
                    }
                    else if (!got_release_point && !hanoiing)
                    {
                        if (doing_mode == STEAL)
                        {
                            for (int i = 0;i < 4;i++)
                            {
                                if (plates[4-i] == 0)
                                {
                                    rsrv.request.num = i;
                                    break;
                                }
                                if (i == 4 && (plates[0] == -1 || plates[0] == 1))
                                {
                                    for (int i = 0;i < 5;i++)
                                    {
                                        if (plates[i] == -1)
                                        {
                                            plates[i] = 0;
                                        }
                                    }
                                }
                            }
                            if (mainClass._release_client.call(rsrv))
                            {
                                if (rsrv.response.picked.header.frame_id != "")
                                {
                                    rid = rsrv.response.picked.header.frame_id;
                                    for (int i = 0;i < 4;i++)
                                    {
                                        if (i == 0)
                                        {
                                            mainClass.poseStamped_set(5, release_point[i], rsrv.response.picked.poses[i].position.x, rsrv.response.picked.poses[i].position.y, rsrv.response.picked.poses[i].orientation.z, rsrv.response.picked.poses[i].orientation.w);
                                        }
                                        else
                                        {
                                            mainClass.poseStamped_set(i%2, release_point[i], rsrv.response.picked.poses[i].position.x, rsrv.response.picked.poses[i].position.y, rsrv.response.picked.poses[i].orientation.z, rsrv.response.picked.poses[i].orientation.w);
                                        }
                                    }
                                    got_release_point = true;
                                }
                            }
                        }
                        else if (release.data == 0 || release.data == robot+1)
                        {
                            release.data = robot+1;
                            mainClass._release.publish(release);
                            home_num = 0;
                            for (int i = 0;i < 4;i++)
                            {
                                if (plates[4-i] == 0)
                                {
                                    // cout << "choose : " << i << endl;
                                    rsrv.request.num = i;
                                    break;
                                }
                                if (i == 3 && (plates[0] == -1 || plates[0] == 1))
                                {
                                    for (int i = 0;i < 5;i++)
                                    {
                                        if (plates[i] == -1)
                                        {
                                            plates[i] = 0;
                                        }
                                    }
                                }
                            }
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
                                    // cout << "rid : " << rid << endl;
                                }
                            }
                        }
                        else
                        {
                            rsrv.request.num = 4;
                            home_num = 1;
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
                                if (reCake == 0)
                                {
                                    got_release_point = false;
                                    plates[int(rid[2])-48] = -1;
                                    // cout << rid << " " << int(rid[2])-48 << endl;
                                    // for (int i = 0;i < 5;i++)
                                    // {
                                    //     cout << plates[i];
                                    // }
                                }
                                else if (reCake < 3)
                                {
                                    reCake++;
                                }
                                else
                                {
                                    now_Mission = STEAL;
                                    mission_print = false;
                                    for (int i = 0;i < 5;i++)
                                    {
                                        if (plates[i] == -1)
                                        {
                                            plates[i] = 0;
                                        }
                                    }
                                }
                            }
                            else if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(release_point[reCake]);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", release_point[reCake].pose.position.x, release_point[reCake].pose.position.y, mainClass.q2e(0, 0, release_point[reCake].pose.orientation.z, release_point[reCake].pose.orientation.w));
                                moving = true;
                            }
                            else if (arrived)
                            {
                                if (!pub_delay)
                                {
                                    if (reCake == 0)
                                    {
                                        missionStr.data = 'o';
                                        missionStr.data += rid[0];
                                    }
                                    else if (reCake == 1)
                                    {
                                        missionStr.data.at(0) = 'c';
                                        mainClass.pub_till_get(missionStr);
                                    }
                                    else if (reCake == 2)
                                    {
                                        missionStr.data = 'o';
                                        missionStr.data += rid[1];
                                        mainClass.pub_till_get(missionStr);
                                        char mc;
                                        if (rid[1] == '3')
                                        {
                                            mc = '0';
                                        }
                                        else
                                        {
                                            mc = char(int(rid[1])+1);
                                        }
                                        missionStr.data.at(1) = mc;
                                    }
                                    else if (reCake == 3)
                                    {
                                        missionStr.data.at(0) = 'c';
                                        mainClass.pub_till_get(missionStr);
                                        missionStr.data.at(1) = char(int(rid[1]));
                                    }
                                    releaseDelayTime = ros::Time::now().toSec();
                                    pub_delay = true;
                                }
                                if (ros::Time::now().toSec() - releaseDelayTime >= 0.25)
                                {
                                    doing = true;
                                    arrived = false;
                                    pub_delay = false;
                                    startMissionTime = ros::Time::now().toSec();
                                    mainClass.pub_till_get(missionStr);
                                }
                            }
                            else if (mission_success)
                            {
                                mission_success = false;
                                if (reCake == 1)
                                {
                                    plates[int(rid[2])-48] = 1;
                                    std_msgs::Int32MultiArray pA;
                                    for (int i = 0;i < 5;i++)
                                    {
                                        pA.data.push_back(plates[i]);
                                    }
                                    mainClass._plates.publish(pA);
                                }
                                if (reCake < 3)
                                {
                                    reCake++;
                                }
                                else
                                {
                                    now_Mission = STEAL;
                                    mission_print = false;
                                    for (int i = 0;i < 5;i++)
                                    {
                                        if (plates[i] == -1)
                                        {
                                            plates[i] = 0;
                                        }
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
                            if (reCake < 3)
                            {
                                reCake++;
                            }
                            else
                            {
                                now_Mission = STEAL;
                                mission_print = false;
                                for (int i = 0;i < 5;i++)
                                {
                                    if (plates[i] == -1)
                                    {
                                        plates[i] = 0;
                                    }
                                }
                            }
                        } 
                    }
                    break;

                case STEAL:

                    if (!mission_print)
                    {
                        ROS_WARN("===== Steal !!! =====");
                        mission_print = true;
                    }

                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("===== Time to Go Home !!! =====");
                    }
                    if (ros::Time::now().toSec() - initialTime.toSec() >= go_home_time-10 && !going_home)
                    {
                        moving = false;
                        doing = false;
                        arrived = false;
                        mission_success = false;
                        now_Mission = HOME;
                        ROS_WARN("No time to steal!");
                    }
                    else if (doing_mode == STEAL)
                    {
                        now_Mission = HOME;
                        mission_print = false;
                        break;
                    }
                    else if (!got_steal_cake)
                    {
                        if (mainClass._steal_client.call(ssrv))
                        {
                            if (ssrv.response.picked.header.frame_id != "")
                            {
                                mainClass.poseStamped_set(0, steal_picked, ssrv.response.picked.pose.position.x, ssrv.response.picked.pose.position.y, ssrv.response.picked.pose.orientation.z, ssrv.response.picked.pose.orientation.w);
                                got_steal_cake = true;
                            }
                        }
                    }
                    else if (got_steal_cake && hanoiing)
                    {
                        ROS_INFO("Still hanoiing !!");
                        if (doing_mode == CHERRY)
                        {
                            now_Mission = BASKET;
                            mission_print = false;
                        }
                    }
                    else if (got_steal_cake && !hanoiing)
                    {
                        if (!moving && !doing)
                        {
                            if (steal_picked.pose.position.x < 0)
                            {
                                ROS_INFO("Nothing to steal !!");
                                if (doing_mode == CHERRY)
                                {
                                    now_Mission = BASKET;
                                    mission_print = false;
                                }
                                else
                                {
                                    now_Mission = HOME;
                                    mission_print = false;
                                }
                            }
                            else if (route_failed)
                            {
                                now_Mission = HOME;
                                mission_print = false;
                            }
                            else if (!arrived && !mission_success)
                            {
                                mainClass._where2go.publish(steal_picked);
                                ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", steal_picked.pose.position.x, steal_picked.pose.position.y, mainClass.q2e(0, 0, steal_picked.pose.orientation.z, steal_picked.pose.orientation.w));
                                moving = true;
                            }
                            else if (arrived)
                            {
                                arrived = false;
                                now_Mission = CAKE;
                                now_Camera_Mode = USE_CAM;
                                lastCakeColor = 3;
                                doing_mode = STEAL;
                                got_cake_picked = false;
                                cakeNum = 0;
                            }
                        }
                    }
                    
                    break;
                
                case HOME:

                    if (!mission_print)
                    {
                        ROS_WARN("===== Home !!! =====");
                        mission_print = true;
                    }

                    going_home = true;
                    if (!moving)
                    {
                        if (route_failed)
                        {
                            route_failed = false;
                            home_num += 2;
                            while (plates[home_num/2+1] == 1)
                            {
                                home_num += 2;
                                if (home_num >= 8)
                                {
                                    break;
                                }
                            }
                            now_home.data = home_num;
                            mainClass._changeHome.publish(now_home);
                        }
                        else if (!arrived)
                        {
                            if (home_num == -1)
                            {
                                home_num = robot;
                            }
                            while (plates[home_num/2+1] == 1)
                            {
                                home_num += 2;
                                if (home_num >= 8)
                                {
                                    home_num -= 2;
                                    break;
                                }
                            }
                            mainClass.poseStamped_set(5, home[side][home_num], home[side][home_num].pose.position.x, home[side][home_num].pose.position.y, myOri_z, myOri_w);
                            mainClass._where2go.publish(home[side][home_num]);
                            ROS_INFO("Heading over to x:[%.3f] y:[%.3f] ang:[%.1f]", home[side][home_num].pose.position.x, home[side][home_num].pose.position.y, mainClass.q2e(0, 0, home[side][home_num].pose.orientation.z, home[side][home_num].pose.orientation.w));
                            moving = true;
                        }
                        else if (arrived)
                        {
                            now_home.data = -1;
                            mainClass._changeHome.publish(now_home);
                            if (pre_home == -1)
                            {
                                home_num = -1;
                            }
                            if ((home_num == -1 && anotherse.data == true) || anotherse.data == false || ros::Time::now().toSec() - initialTime.toSec() >= go_home_time+8)
                            {
                                missionStr.data = "A0";
                                mainClass.pub_till_get(missionStr);
                                missionStr.data = "d0";
                                mainClass.pub_till_get(missionStr);
                                now_Status = FINISH;
                                printOnce = false;
                            }
                        }
                    }
                    break;
                }
                break;

            case FINISH:

                if (!printOnce)
                {
                    ROS_INFO("Finish all missions!");
                    ROS_INFO("Robot%d finish time : %.1f", robot+1, ros::Time::now().toSec() - initialTime.toSec());
                    if (!finish_mission.data)
                    {
                        finish_mission.data = true;
                        mainClass._ifinish.publish(finish_mission);
                    }
                    else if (!anotherse.data || finish_mission.data)
                    {
                        ROS_WARN("Total time : %.1f", ros::Time::now().toSec() - initialTime.toSec());
                    }
                    printOnce = true;
                    
                }
                if (ros::Time::now().toSec() - initialTime.toSec() >= 99)
                {
                    missionStr.data = "f0";
                    mainClass.pub_till_get(missionStr);
                    initialTime = ros::Time::now();
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