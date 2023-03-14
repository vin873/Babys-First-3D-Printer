// Small

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class pose
{
public:
    double p_x;
    double p_y;
    double p_z;
    double roll;
    double pitch;
    double yaw;
};

class Robot_move
{
public:
    Robot_move(ros::NodeHandle &nh);
    void initialize();

private:
    ros::NodeHandle nh_;
    tf::TransformBroadcaster broadcaster;

    ros::Subscriber pose_sub;
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg); // ekf_pose
    // void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);

    geometry_msgs::TransformStamped trans;

    pose robot_init_tf;
    pose robot_pose;
    void tf_info_initialize();

    ros::Timer timer;
    void timerCallback(const ros::TimerEvent &e);
    double control_frequency_;
};

Robot_move::Robot_move(ros::NodeHandle &nh)
{
    nh_ = nh;
    initialize();
}

void Robot_move::initialize()
{
    pose_sub = nh_.subscribe("ekf_pose", 10, &Robot_move::pose_callback, this);
    // pose_sub = nh_.subscribe("base_pose_ground_truth", 10, &Robot_move::pose_callback, this);

    control_frequency_ = 70; // hz
    timer = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &Robot_move::timerCallback, this, false, false);
    timer.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer.start();

    tf_info_initialize();
}

void Robot_move::tf_info_initialize()
{
    trans.header.frame_id = "base";
    trans.child_frame_id = "robot_Small";

    robot_init_tf.p_x = -0.02;
    robot_init_tf.p_y = -0.125;
    robot_init_tf.p_z = 0.052;
    robot_init_tf.roll = 1.57;
    robot_init_tf.pitch = 0;
    robot_init_tf.yaw = -1.57;

    robot_pose.p_x = 0;
    robot_pose.p_y = 0;
    robot_pose.p_z = 0;
    robot_pose.roll = robot_init_tf.roll;
    robot_pose.pitch = robot_init_tf.pitch;
    robot_pose.yaw = robot_init_tf.yaw;
}

void Robot_move::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg) // ekf_pose
{
    robot_pose.p_x = pose_msg->pose.pose.position.x;
    robot_pose.p_y = pose_msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    robot_pose.roll = robot_init_tf.roll;
    robot_pose.pitch = robot_init_tf.pitch;
    robot_pose.yaw = robot_init_tf.yaw + yaw;
}

// void Robot_move::pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) // base_pose_ground_truth
// {
//     robot_pose.p_x = pose_msg->pose.pose.position.x;
//     robot_pose.p_y = pose_msg->pose.pose.position.y;
//     tf2::Quaternion q;
//     tf2::fromMsg(pose_msg->pose.pose.orientation, q);
//     tf2::Matrix3x3 qt(q);
//     double _, yaw;
//     qt.getRPY(_, _, yaw);
//     robot_pose.roll = robot_init_tf.roll;
//     robot_pose.pitch = 0;
//     robot_pose.yaw = yaw;
// }

void Robot_move::timerCallback(const ros::TimerEvent &e)
{
    tf2::Quaternion q;
    q.setRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);
    trans.header.stamp = ros::Time::now();
    trans.transform.translation.x = robot_init_tf.p_x + robot_pose.p_x;
    trans.transform.translation.y = robot_init_tf.p_y + robot_pose.p_y;
    trans.transform.translation.z = robot_init_tf.p_z;
    trans.transform.rotation.x = q.getX();
    trans.transform.rotation.y = q.getY();
    trans.transform.rotation.z = q.getZ();
    trans.transform.rotation.w = q.getW();
    broadcaster.sendTransform(trans);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_robot_tf");
    ros::NodeHandle nh("");
    Robot_move robot_move(nh);
    while (ros::ok())
    {
        ros::spin();
    }
}