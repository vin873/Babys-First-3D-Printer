#include "pathTracker.h"
#include <cmath>
#include <ros/time.h>

using namespace std;

RobotState::RobotState(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double RobotState::distanceTo(RobotState pos)
{
    return sqrt(pow(x_ - pos.x_, 2) + pow(y_ - pos.y_, 2));
}

Eigen::Vector3d RobotState::getVector()
{
    Eigen::Vector3d vec;
    vec << x_, y_, theta_;
    return vec;
}

pathTracker::pathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
{
    nh_ = nh;
    nh_local_ = nh_local;
    std_srvs::Empty empt;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &pathTracker::initializeParams, this);
    initializeParams(empt.request, empt.response);
    initialize();
    t_bef_ = ros::Time::now();
    t_now_ = ros::Time::now();
}

pathTracker::~pathTracker()
{
    nh_local_.deleteParam("active");
    nh_local_.deleteParam("control_frequency");
    nh_local_.deleteParam("lookahead_distance");

    nh_local_.deleteParam("linear_kp");
    nh_local_.deleteParam("linear_max_velocity");
    nh_local_.deleteParam("linear_acceleration");
    nh_local_.deleteParam("linear_brake_distance_ratio");
    nh_local_.deleteParam("linear_min_brake_distance");
    nh_local_.deleteParam("xy_tolerance");
    nh_local_.deleteParam("linear_transition_vel_");
    nh_local_.deleteParam("linear_transition_acc_");
    nh_local_.deleteParam("linear_acceleration_profile");
    nh_local_.deleteParam("linear_deceleration_profile");

    nh_local_.deleteParam("angular_kp");
    nh_local_.deleteParam("angular_max_velocity");
    nh_local_.deleteParam("angular_acceleration");
    nh_local_.deleteParam("angular_brake_distance");
    nh_local_.deleteParam("theta_tolerance");
    nh_local_.deleteParam("angular_transition_vel_");
    nh_local_.deleteParam("angular_transition_acc_");
    nh_local_.deleteParam("angular_acceleration_profile");
    nh_local_.deleteParam("angular_deceleration_profile");
}

void pathTracker::initialize()
{
    if_localgoal_final_reached = false;
    if_globalpath_switched = false;
    if_goal_is_blocked_ = false;
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &pathTracker::timerCallback, this, false, false);
    timer_.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer_.start();
    // timer2_ = nh_.createTimer(ros::Duration(2), &pathTracker::timer2Callback, this, false, false);
    // timer2_.setPeriod(ros::Duration(2), false);
    // timer2_.start();
    workingMode_ = Mode::IDLE;
    workingMode_past_ = Mode::IDLE;
}

bool pathTracker::initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    // load parameter
    bool get_param_ok = true;
    bool prev_active = p_active_;

    get_param_ok = nh_local_.param<bool>("active", p_active_, true);
    get_param_ok = nh_local_.param<string>("robot_type", robot_type_, "omni");
    get_param_ok = nh_local_.param<string>("frame", frame_, "map");
    get_param_ok = nh_local_.param<double>("control_frequency", control_frequency_, 50);
    get_param_ok = nh_local_.param<double>("lookahead_distance", lookahead_d_, 0.2);
    get_param_ok = nh_local_.param<double>("waiting_timeout", waiting_timeout_, 3);
    // linear parameter
    // acceleration
    get_param_ok = nh_local_.param<double>("linear_max_velocity", linear_max_vel_, 0.5);
    get_param_ok = nh_local_.param<double>("linear_acceleration", linear_acceleration_, 0.3);
    get_param_ok = nh_local_.param<string>("linear_acceleration_profile", linear_acceleration_profile_, "linear");
    // transition
    get_param_ok = nh_local_.param<double>("linear_transition_velocity", linear_transition_vel_, 0.15);
    get_param_ok = nh_local_.param<double>("linear_transition_acceleration", linear_transition_acc_, 0.6);
    // deceleration
    get_param_ok = nh_local_.param<double>("linear_kp", linear_kp_, 0.8);
    get_param_ok = nh_local_.param<double>("linear_brake_distance_ratio", linear_brake_distance_ratio_, 0.3);
    get_param_ok = nh_local_.param<double>("linear_min_brake_distance", linear_min_brake_distance_, 0.3);
    get_param_ok = nh_local_.param<string>("linear_deceleration_profile", linear_deceleration_profile_, "linear");

    // angular parameter
    get_param_ok = nh_local_.param<double>("angular_max_velocity", angular_max_vel_, 3);
    get_param_ok = nh_local_.param<double>("angular_acceleration", angular_acceleration_, 0.5);
    get_param_ok = nh_local_.param<double>("angular_brake_distance", angular_brake_distance_, 0.35);
    get_param_ok = nh_local_.param<double>("angular_transition_velocity", angular_transition_vel_, 0.15);
    get_param_ok = nh_local_.param<double>("angular_transition_acceleration", angular_transition_acc_, 0.6);
    get_param_ok = nh_local_.param<double>("angular_kp", angular_kp_, 1.5);
    get_param_ok = nh_local_.param<string>("angular_acceleration_profile", angular_acceleration_profile_, "linear");
    get_param_ok = nh_local_.param<string>("angular_deceleration_profile", angular_deceleration_profile_, "linear");

    get_param_ok = nh_local_.param<double>("xy_tolerance", xy_tolerance_, 0.01);
    get_param_ok = nh_local_.param<double>("theta_tolerance", theta_tolerance_, 0.03);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            poseSub_ = nh_.subscribe("/ekf_pose", 50, &pathTracker::poseCallback, this);
            // poseSub_ = nh_.subscribe("global_filter", 50, &pathTracker::poseCallback, this);
            goalSub_ = nh_.subscribe("nav_goal", 50, &pathTracker::goalCallback, this);
            obstacleSub_ = nh_.subscribe("obstacle_position_array", 50, &pathTracker::obstacleCallbak, this);
            velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
            localgoalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 10);
            posearrayPub_ = nh_.advertise<geometry_msgs::PoseArray>("orientation", 10);
            goalreachedPub_ = nh_.advertise<std_msgs::Bool>("finishornot", 1);
        }
        else
        {
            poseSub_.shutdown();
            goalSub_.shutdown();
            velPub_.shutdown();
            localgoalPub_.shutdown();
            posearrayPub_.shutdown();
            goalreachedPub_.shutdown();
        }
    }

    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Path Tracker]: "
                        << "set param ok");
    }
    else
    {
        ROS_WARN_STREAM("[Path Tracker]: "
                        << "set param failed");
    }
    // cout << "param updated !" << endl;
    return true;
}

void pathTracker::timerCallback(const ros::TimerEvent& e)
{
    // ROS_INFO("%d", workingMode_);
    switch (workingMode_)
    {
        case Mode::GLOBALPATH_RECEIVED: {
            if (workingMode_past_ == Mode::IDLE)
            {
                switchMode(Mode::TRACKING);
                break;
            }
            else if (workingMode_past_ == Mode::TRACKING)
            {
                // Slow down first then start tracking new path
                switchMode(Mode::TRANSITION);
                break;
            }
            else if (workingMode_past_ == Mode::TRANSITION)
            {
                switchMode(Mode::TRACKING);
                break;
            }
        }
        break;

        case Mode::TRACKING: {
            // goal reached
            if (xy_goal_reached(cur_pose_, goal_pose_) && theta_goal_reached(cur_pose_, goal_pose_) && !if_goal_is_blocked_)
            {
                ROS_INFO("Working Mode : GOAL REACHED !");
                switchMode(Mode::IDLE);
                velocity_state_.x_ = 0;
                velocity_state_.y_ = 0;
                velocity_state_.theta_ = 0;
                velocityPublish();

                if_goal_is_blocked_ = false;
                // publish /finishornot
                std_msgs::Bool goalreached;
                goalreached.data = true;
                goalreachedPub_.publish(goalreached);
                break;
            }

            if (workingMode_past_ == Mode::TRANSITION)
            {
                if (if_globalpath_switched == false)
                {
                    if_localgoal_final_reached = false;
                    // plannerClient(cur_pose_, goal_pose_);
                    linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
                    if_globalpath_switched = true;
                }
            }
            // ROS_INFO("Working Mode : TRACKING");
            if (robot_type_ == "omni")
            {
                // dynamic wei
                RobotState local_goal;
                if(!plannerClient(cur_pose_, goal_pose_))
                {
                    local_goal = rollingWindow(cur_pose_, global_path_, lookahead_d_);
                    goal_pose_.x_ = local_goal.x_;
                    goal_pose_.y_ = local_goal.y_;
                    goal_pose_.theta_ = local_goal.theta_;

                    global_path_past_ = global_path_;
                    // ROS_INFO("cur_pose x:%f, y:%f; local_pose x:%f, y:%f", cur_pose_.x_, cur_pose_.y_, local_goal.x_, local_goal.y_);
                    // switchMode(Mode::TRACKING);
                    if_globalpath_switched = false;
                    if_goal_is_blocked_ = true;
                    // publish /finishornot
                    std_msgs::Bool goalreached;
                    goalreached.data = false;
                    goalreachedPub_.publish(goalreached);
                    if_goal_is_blocked_ = false;
                    return;
                }
                local_goal = rollingWindow(cur_pose_, global_path_, lookahead_d_);
                omniController(local_goal, cur_pose_);
            }
            else if (robot_type_ == "diff")
            {
                RobotState local_goal;
                local_goal = rollingWindow(cur_pose_, global_path_, lookahead_d_);
                diffController(local_goal, cur_pose_);
            }
        }
        break;

        case Mode::IDLE: {
            // ROS_INFO("Working Mode : IDLE");
            velocity_state_.x_ = 0;
            velocity_state_.y_ = 0;
            velocity_state_.theta_ = 0;
            velocityPublish();
        }
        break;

        case Mode::TRANSITION: {
            // ROS_INFO("Working Mode : TRANSITION");
            double linear_vel = sqrt(pow(velocity_state_.x_, 2) + pow(velocity_state_.y_, 2));
            double angular_vel = velocity_state_.theta_;

            if (linear_vel <= linear_transition_vel_ && angular_vel <= angular_transition_vel_)
            {
                switchMode(Mode::TRACKING);
                break;
            }

            if (robot_type_ == "omni")
            {

                RobotState local_goal;
                // if the new goal has no path to reach, just change the goal to the local_goal which belong to past_path  
                if(!plannerClient(cur_pose_, goal_pose_))
                {
                    local_goal = rollingWindow(cur_pose_, global_path_, lookahead_d_);
                    goal_pose_.x_ = local_goal.x_;
                    goal_pose_.y_ = local_goal.y_;
                    goal_pose_.theta_ = local_goal.theta_;

                    global_path_past_ = global_path_;
                    // ROS_INFO("cur_pose x:%f, y:%f; local_pose x:%f, y:%f", cur_pose_.x_, cur_pose_.y_, local_goal.x_, local_goal.y_);
                    switchMode(Mode::TRACKING);
                    if_globalpath_switched = false;
                    if_goal_is_blocked_ = true;

                    // publish /finishornot
                    std_msgs::Bool goalreached;
                    goalreached.data = false;
                    goalreachedPub_.publish(goalreached);
                    if_goal_is_blocked_ = false;
                    return;
                }
                local_goal = rollingWindow(cur_pose_, global_path_past_, lookahead_d_);
                omniController(local_goal, cur_pose_);
            }
            else if (robot_type_ == "diff")
            {
                RobotState local_goal;
                local_goal = rollingWindow(cur_pose_, global_path_past_, lookahead_d_);
                diffController(local_goal, cur_pose_);
            }
        }
        break;
    }
}
void pathTracker::timer2Callback(const ros::TimerEvent& e2)
{
    // wei
    //  ros::ServiceClient client2 = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    //  std_srvs::Empty srv;
    //  if (client2.call(srv)){
    //      ROS_INFO("Cleared the map!!");
    //  }
}
void pathTracker::switchMode(Mode next_mode)
{
    workingMode_past_ = workingMode_;
    workingMode_ = next_mode;
}

bool pathTracker::plannerClient(RobotState cur_pos, RobotState goal_pos)
{
    geometry_msgs::PoseStamped cur;
    cur.header.frame_id = frame_;
    cur.pose.position.x = cur_pos.x_;
    cur.pose.position.y = cur_pos.y_;
    cur.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, cur_pos.theta_);
    cur.pose.orientation.x = q.x();
    cur.pose.orientation.y = q.y();
    cur.pose.orientation.z = q.z();
    cur.pose.orientation.w = q.w();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = frame_;
    goal.pose.position.x = goal_pos.x_;
    goal.pose.position.y = goal_pos.y_;
    goal.pose.position.z = 0;

    // tf2::Quaternion q;
    q.setRPY(0, 0, goal_pos.theta_);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    // check if obstacles are on the path

    // bool if_obstacle_on_path = true;
    // if(global_path_.size()){
    //     if_obstacle_on_path = checkObstacle(global_path_);
    // }
    // if(new_goal == false && if_obstacle_on_path == false){
    //     return false;
    // }

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = cur;
    srv.request.goal = goal;

    std::vector<geometry_msgs::PoseStamped> path_msg;

    if (client.call(srv))
    {
        // ******* add by Ben
        new_goal = false;
        if (srv.response.plan.poses.empty())
        {
            ROS_WARN("pathTracker: Got empty plan");
            return false;
        }
        else
        {
            // ROS_INFO("Path received from make_plan service");
        }
        // *******

        // ROS_INFO("1 - Path received from global planner !");
        nav_msgs::Path path_msg;
        path_msg.poses = srv.response.plan.poses;

        global_path_.clear();

        for (const auto& point : path_msg.poses)
        {
            RobotState pose;
            pose.x_ = point.pose.position.x;
            pose.y_ = point.pose.position.y;
            tf2::Quaternion q;
            tf2::fromMsg(point.pose.orientation, q);
            tf2::Matrix3x3 qt(q);
            double _, yaw;
            qt.getRPY(_, _, yaw);
            pose.theta_ = yaw;
            global_path_.push_back(pose);
        }
        global_path_ = orientationFilter(global_path_);

        // add by Ben
        return true;
        // ROS_INFO("2 - Path received from global planner !");

        // print global path
        // ROS_INFO("--- global path ---");
        // for (const auto& point : global_path_)
        // {
        //     ROS_INFO("(%f, %f, %f)", point.x_, point.y_, point.theta_);
        // }
        // ROS_INFO("--- ---");
    }
    else
    {
        ROS_ERROR("Failed to call service make_plan");
        // return 1;
        return false;
    }
}

void pathTracker::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    cur_pose_.x_ = pose_msg->pose.pose.position.x;
    cur_pose_.y_ = pose_msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    cur_pose_.theta_ = yaw;
}

// void pathTracker::poseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
// {
//     cur_pose_.x_ = pose_msg->pose.pose.position.x;
//     cur_pose_.y_ = pose_msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(pose_msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q);
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     cur_pose_.theta_ = yaw;
// }

void pathTracker::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    // add by ben
    ros::ServiceClient client2 = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    t_bef_ = ros::Time::now();

    goal_pose_.x_ = pose_msg->pose.position.x;
    goal_pose_.y_ = pose_msg->pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);

    goal_pose_.theta_ = yaw;
    ROS_INFO("Goal received ! (%f, %f, %f)", goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);

    global_path_past_ = global_path_;
    if (workingMode_ == Mode::IDLE)
    {
        if (!plannerClient(cur_pose_, goal_pose_))
        {
            if_goal_is_blocked_ = true;
            // publish /finishornot
            std_msgs::Bool goalreached;
            goalreached.data = false;
            goalreachedPub_.publish(goalreached);
            
            return;
        }
        if_goal_is_blocked_ = false;

        linear_brake_distance_ = linear_brake_distance_ratio_ * cur_pose_.distanceTo(goal_pose_);
        if (linear_brake_distance_ < linear_min_brake_distance_)
            linear_brake_distance_ = linear_min_brake_distance_;
    }

    if_localgoal_final_reached = false;
    if_globalpath_switched = false;
    switchMode(Mode::GLOBALPATH_RECEIVED);
    new_goal = true;
}

void pathTracker::obstacleCallbak(const geometry_msgs::PoseArray::ConstPtr& poses_msg)
{
    obstacle_pose_.poses.clear();
    obstacle_pose_.header = poses_msg->header;
    for (int i = 0; i < poses_msg->poses.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = poses_msg->poses[i].position.x;
        pose.position.y = poses_msg->poses[i].position.y;
        obstacle_pose_.poses.push_back(pose);
    }
    // std::cout << obstacle_pose_.poses.size() << std::endl;
}

RobotState pathTracker::rollingWindow(RobotState cur_pos, std::vector<RobotState> path, double L_d)
{
    int k = 1;
    int last_k = 0;
    int d_k = 0;
    RobotState a;
    int a_idx = 0;
    RobotState b;
    int b_idx = 0;
    RobotState local_goal;
    bool if_b_asigned = false;
    double r = L_d;

    // ROS_INFO("%ld", path.size());
    for (int i = 0; i < path.size(); i++)
    {
        if (i == 1)
            last_k = 0;
        last_k = k;
        if (cur_pos.distanceTo(path.at(i)) >= r)
            k = 1;
        else
            k = 0;

        d_k = k - last_k;

        if (d_k == 1)
        {
            b = path.at(i);
            if_b_asigned = true;
            b_idx = i;
            a_idx = i - 1;
            break;
        }
    }

    if (!if_b_asigned)
    {
        double min = 1000000;
        for (int i = 0; i < path.size(); i++)
        {
            if (cur_pos.distanceTo(path.at(i)) < min)
            {
                min = cur_pos.distanceTo(path.at(i));
                b_idx = i;
                a_idx = i - 1;
                b = path.at(i);
            }
        }
    }

    if (a_idx == -1)
    {
        local_goal = path.at(b_idx);
    }
    else
    {
        a = path.at(a_idx);
        double d_ca = cur_pos.distanceTo(a);
        double d_cb = cur_pos.distanceTo(b);
        local_goal.x_ = a.x_ + (b.x_ - a.x_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.y_ = a.y_ + (b.y_ - a.y_) * (r - d_ca) / (d_cb - d_ca);
        local_goal.theta_ = a.theta_;
    }

    if (if_localgoal_final_reached)
    {
        // cout << "local goal set to path.back()" << endl;
        local_goal = path.back();
    }

    if (cur_pos.distanceTo(path.back()) < r + 0.01)
        local_goal = path.back();

    if (local_goal.distanceTo(path.back()) < 0.005)
    {
        local_goal = path.back();
        if_localgoal_final_reached = true;
    }

    // for rviz visualization
    geometry_msgs::PoseStamped pos_msg;
    pos_msg.header.frame_id = frame_;
    pos_msg.header.stamp = ros::Time::now();
    pos_msg.pose.position.x = local_goal.x_;
    pos_msg.pose.position.y = local_goal.y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, local_goal.theta_);
    pos_msg.pose.orientation.x = q.x();
    pos_msg.pose.orientation.y = q.y();
    pos_msg.pose.orientation.z = q.z();
    pos_msg.pose.orientation.w = q.w();
    localgoalPub_.publish(pos_msg);
    return local_goal;
}

std::vector<RobotState> pathTracker::orientationFilter(std::vector<RobotState> origin_path)
{
    std::vector<RobotState> path;
    double init_theta = cur_pose_.theta_;
    double goal_theta = goal_pose_.theta_;
    double theta_err = 0;
    double d_theta = 0;
    Eigen::Vector3d init;
    Eigen::Vector3d goal;
    // calculate rotate direction
    init << cos(init_theta), sin(init_theta), 0;
    goal << cos(goal_theta), sin(goal_theta), 0;

    if (init.cross(goal)(2) >= 0)
        rotate_direction_ = 1;
    else
        rotate_direction_ = -1;

    // theta_err = acos(init(0)*goal(0)+init(1)*goal(1));
    theta_err = fabs(angleLimitChecking(goal_theta - init_theta));
    d_theta = rotate_direction_ * theta_err / (origin_path.size() - 1);

    RobotState point(origin_path.at(0).x_, origin_path.at(0).y_, init_theta);
    path.push_back(point);

    for (int i = 0; i < origin_path.size(); i++)
    {
        if (i != 0)
        {
            double theta;
            theta = angleLimitChecking(path.at(i - 1).theta_ + d_theta);
            // cout << "theta = " << theta << endl;
            RobotState point(origin_path.at(i).x_, origin_path.at(i).y_, theta);
            path.push_back(point);
        }
    }

    // Rviz visualize processed path
    geometry_msgs::PoseArray arr_msg;
    arr_msg.header.frame_id = frame_;
    arr_msg.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Pose> poses;

    for (int i = 0; i < path.size(); i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = path.at(i).x_;
        pose.position.y = path.at(i).y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, path.at(i).theta_);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        poses.push_back(pose);
    }
    arr_msg.poses = poses;
    posearrayPub_.publish(arr_msg);

    return path;
}

double pathTracker::angleLimitChecking(double theta)
{
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    return theta;
}

// Path tracker for differential drive robot
void pathTracker::diffController(RobotState local_goal, RobotState cur_pos)
{
}

// Path tracker for omni drive robot
void pathTracker::omniController(RobotState local_goal, RobotState cur_pos)
{
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    int rotate_direction = 0;
    Eigen::Vector3d goal_vec(goal_pose_.x_, goal_pose_.y_, goal_pose_.theta_);
    Eigen::Vector3d cur_vec(cur_pos.x_, cur_pos.y_, cur_pos.theta_);

    if (cur_vec.cross(goal_vec)(2) >= 0)
        rotate_direction = 1;
    else
        rotate_direction = -1;

    // transform local_goal to base_footprint frame
    Eigen::Vector2d goal_base_vec;
    Eigen::Vector2d localgoal_bf;
    Eigen::Matrix2d rot;
    goal_base_vec << (local_goal.x_ - cur_pos.x_), (local_goal.y_ - cur_pos.y_);
    rot << cos(-cur_pos.theta_), -sin(-cur_pos.theta_), sin(-cur_pos.theta_), cos(-cur_pos.theta_);
    localgoal_bf = rot * goal_base_vec;

    t_now_ = ros::Time::now();

    dt_ = (t_now_ - t_bef_).toSec();
    // ROS_INFO("dt: %f", dt_);

    if (xy_goal_reached(cur_pose_, goal_pose_))
    {
        velocity_state_.x_ = 0;
        velocity_state_.y_ = 0;
    }
    else
    {
        linear_velocity =
            velocityProfile(Velocity::linear, cur_pose_, goal_pose_, velocity_state_, linear_acceleration_);
        double direction = atan2(localgoal_bf(1), localgoal_bf(0));
        velocity_state_.x_ = linear_velocity * cos(direction);
        velocity_state_.y_ = linear_velocity * sin(direction);
    }

    if (theta_goal_reached(cur_pose_, goal_pose_))
    {
        velocity_state_.theta_ = 0;
    }
    else
    {
        angular_velocity = velocityProfile(Velocity::angular, cur_pose_, goal_pose_, velocity_state_,
                                           rotate_direction_ * angular_acceleration_);
        velocity_state_.theta_ = angular_velocity;
    }
    velocityPublish();

    t_bef_ = t_now_;
}

double pathTracker::velocityProfile(Velocity vel_type, RobotState cur_pos, RobotState goal_pos, RobotState vel_state_,
                                    double acceleration)
{
    double output_vel = 0;
    if (workingMode_ == Mode::TRACKING)
    {
        if (vel_type == Velocity::linear)
        {
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            // acceleration
            if (linear_acceleration_profile_ == "linear")
            {
                double d_vel = acceleration * dt_;
                output_vel = last_vel + d_vel;
            }
            else if (linear_acceleration_profile_ == "smooth_step")
            {
            }

            double xy_err = cur_pose_.distanceTo(goal_pose_);
            // ROS_INFO("err = %f\n", xy_err);
            // deceleration
            if (xy_err < linear_brake_distance_)
            {
                if (linear_deceleration_profile_ == "linear")
                {
                    double acc = pow(linear_max_vel_, 2) / 2 / linear_brake_distance_;
                    output_vel = sqrt(2 * acc * xy_err);
                    if (output_vel < 0.25)
                    {
                        double output_vel_ = xy_err * linear_kp_;
                        if (output_vel_ < output_vel)
                        {
                            output_vel = output_vel_;
                        }
                    }
                }
                else if (linear_deceleration_profile_ == "p_control")
                {
                    output_vel = cur_pos.distanceTo(goal_pos) * linear_kp_;
                }
                else if (linear_deceleration_profile_ == "smooth_step")
                {
                }
            }

            // Saturation
            if (output_vel > linear_max_vel_)
                output_vel = linear_max_vel_;
            // ROS_INFO("linear vel %f", output_vel);
        }

        if (vel_type == Velocity::angular)
        {
            // double theta_err;
            // // theta_err = fabs(angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
            // theta_err = (angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
            // output_vel = theta_err * angular_kp_;

            // // if (signbit(acceleration))
            // // {
            // //     output_vel *= -1;
            // // }

            // // Saturation
            // if (output_vel > angular_max_vel_)
            //     output_vel = angular_max_vel_;
            // if (output_vel < -angular_max_vel_)
            //     output_vel = -angular_max_vel_;

            // ================= old version =================
            double d_vel = acceleration * dt_;
            output_vel = vel_state_.theta_ + d_vel;
            double theta_err = (angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));

            if (fabs(theta_err) < angular_brake_distance_)
            {
                output_vel = theta_err * angular_kp_;
            }

            // Saturation
            if (output_vel > angular_max_vel_)
                output_vel = angular_max_vel_;
            if (output_vel < -angular_max_vel_)
                output_vel = -angular_max_vel_;
        }
    }
    else if (workingMode_ == Mode::TRANSITION)
    {
        if (vel_type == Velocity::linear)
        {
            double d_vel = linear_transition_acc_ * dt_;
            RobotState _(0, 0, 0);
            double last_vel = vel_state_.distanceTo(_);
            output_vel = last_vel - d_vel;
            if (output_vel < linear_transition_vel_)
                output_vel = linear_transition_vel_;
        }

        if (vel_type == Velocity::angular)
        {
            double d_vel = angular_transition_acc_ * dt_;
            if (output_vel > 0)
            {
                output_vel = vel_state_.theta_ - d_vel;
                if (output_vel < angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
            else
            {
                output_vel = vel_state_.theta_ + d_vel;
                if (output_vel > angular_transition_vel_)
                    output_vel = angular_transition_vel_;
            }
        }
    }

    return output_vel;
}

bool pathTracker::xy_goal_reached(RobotState cur_pos, RobotState goal_pos)
{
    if (cur_pos.distanceTo(goal_pos) < xy_tolerance_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool pathTracker::theta_goal_reached(RobotState cur_pos, RobotState goal_pos)
{
    double theta_err = 0;
    Eigen::Vector2d cur_vec;
    Eigen::Vector2d goal_vec;
    cur_vec << cos(cur_pos.theta_), sin(cur_pos.theta_);
    goal_vec << cos(goal_pos.theta_), sin(goal_pos.theta_);
    theta_err = cur_vec.dot(goal_vec);

    theta_err = fabs(angleLimitChecking(goal_pos.theta_ - cur_pos.theta_));
    if (fabs(theta_err) < theta_tolerance_)
    {
        return true;
    }
    else
        return false;
}

void pathTracker::velocityPublish()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = velocity_state_.x_;
    vel_msg.linear.y = velocity_state_.y_;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = velocity_state_.theta_;
    velPub_.publish(vel_msg);
}

bool pathTracker::checkObstacle(std::vector<RobotState> path)
{
    double dist_thres = 0.1;
    double d;
    for (int i = 0; i < obstacle_pose_.poses.size(); i++)
    {
        for (int j = 0; j < path.size(); j++)
        {
            RobotState obs;
            obs.x_ = obstacle_pose_.poses[i].position.x;
            obs.y_ = obstacle_pose_.poses[i].position.y;
            d = path[j].distanceTo(obs);
            if (d < dist_thres)
                return true;
        }
    }
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathTracker");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    pathTracker pathTracker_inst(nh, nh_local);

    while (ros::ok())
    {
        ros::spin();
    }
}
