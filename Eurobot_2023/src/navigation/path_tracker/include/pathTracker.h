#include <Eigen/Dense>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
// message
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <ros/time.h>

enum class Mode
{
    IDLE,
    TRACKING,
    TRANSITION,
    GLOBALPATH_RECEIVED
};

enum class Velocity
{
    linear,
    angular
};

class RobotState
{
  public:
    RobotState(double x, double y, double theta);
    RobotState()
    {
    }
    double x_;
    double y_;
    double theta_;
    Eigen::Vector3d getVector();
    double distanceTo(RobotState);
};

class pathTracker
{
  public:
    pathTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
    ~pathTracker();
    bool initializeParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void initialize();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;

    // subscriber
    ros::Subscriber poseSub_;
    ros::Subscriber goalSub_;
    ros::Subscriber obstacleSub_;
    ros::Subscriber actionSub_;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
    // void poseCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void obstacleCallbak(const geometry_msgs::PoseArray::ConstPtr& poses_msg);
    // void actionCallback(const std_msgs::Bool::ConstPtr& action_msg);

    geometry_msgs::PoseArray obstacle_pose_;
    geometry_msgs::PoseArray rival_pose_;

    // Publisher
    ros::Publisher velPub_;
    void velocityPublish();
    ros::Publisher localgoalPub_;
    ros::Publisher posearrayPub_;
    ros::Publisher goalreachedPub_;

    // Client
    bool plannerClient(RobotState, RobotState);

    RobotState goal_pose_;
    RobotState cur_pose_;
    RobotState velocity_state_;

    bool if_localgoal_final_reached;
    bool if_reached_target_range;

    bool xy_goal_reached(RobotState cur_pose_, RobotState goal_pose_);
    bool theta_goal_reached(RobotState cur_pose_, RobotState goal_pose_);

    // Goal rquest from Main and Path received from global planner
    std::vector<RobotState> global_path_;
    std::vector<RobotState> global_path_past_;

    // timer setup
    ros::Timer timer_;
    ros::Timer timer2_;
    void timerCallback(const ros::TimerEvent& e);
    void timer2Callback(const ros::TimerEvent& e2);
    Mode workingMode_;
    Mode workingMode_past_;
    void switchMode(Mode next_mode);
    bool if_globalpath_switched;

    // controller parameter
    std::string robot_type_;
    std::string frame_;
    bool p_active_;
    double control_frequency_;
    double lookahead_d_;
    double waiting_timeout_;

    double linear_kp_;
    double linear_max_vel_;
    double linear_acceleration_;
    double linear_brake_distance_;
    double linear_transition_vel_;
    double linear_transition_acc_;
    // double linear_tracking_vel_;
    double xy_tolerance_;
    double linear_min_brake_distance_;
    double linear_brake_distance_ratio_;
    // velocity profile type : linear, smooth_step
    std::string linear_acceleration_profile_;
    std::string linear_deceleration_profile_;

    double angular_kp_;
    double angular_max_vel_;
    double angular_acceleration_;
    double angular_brake_distance_;
    double angular_transition_vel_;
    double angular_transition_acc_;
    double theta_tolerance_;
    double theta_err;
    // velocity profile type : p_control, linear, smooth_step
    std::string angular_acceleration_profile_;
    std::string angular_deceleration_profile_;

    double angleLimitChecking(double theta);
    double velocityProfile(Velocity, RobotState cur_pos, RobotState goal_pos, RobotState velocity_state,
                           double acceleration_sign);
    double max_linear_vel_reached_;

    // Path post process
    std::vector<RobotState> orientationFilter(std::vector<RobotState>);
    int rotate_direction_;

    // rollingWindow method flag
    RobotState rollingWindow(RobotState cur_pos, std::vector<RobotState> global_path, double R);

    void diffController(RobotState local_goal, RobotState cur_pos);
    void omniController(RobotState local_goal, RobotState cur_pos);

    // add by ben
    ros::Time t_bef_;
    ros::Time t_now_;
    double dt_;
    
    /**
     *  @brief check if obstacle is on the current path 
     *  @param path is the current path
     *  accordint to the point line distance to calculate the distance between each obstacles with path
     */
    bool checkObstacle(std::vector<RobotState> path);
    bool new_goal;
    bool if_goal_is_blocked_;
};
