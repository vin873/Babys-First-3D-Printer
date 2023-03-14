// robot : Large
// another robot : Small

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
    tf::TransformBroadcaster another_broadcaster;

    ros::Subscriber pose_sub;
    ros::Subscriber another_pose_sub;
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);         // ekf_pose
    void another_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg); // ally_pose
    // void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg); // base_pose_ground truth

    geometry_msgs::TransformStamped trans;
    geometry_msgs::TransformStamped another_trans;

    pose robot_init_tf;
    pose robot_pose;
    pose another_robot_init_tf;
    pose another_robot_pose;
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
    pose_sub = nh_.subscribe("ekf_pose", 10, &Robot_move::pose_callback, this);                  // large
    another_pose_sub = nh_.subscribe("ally_pose", 10, &Robot_move::another_pose_callback, this); // small
    // pose_sub = nh_.subscribe("base_pose_ground_truth", 10, &Robot_move::pose_callback, this);

    control_frequency_ = 100; // hz
    timer = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &Robot_move::timerCallback, this, false, false);
    timer.setPeriod(ros::Duration(1.0 / control_frequency_), false);
    timer.start();

    tf_info_initialize();
}

void Robot_move::tf_info_initialize()
{
    trans.header.frame_id = "base";
    trans.child_frame_id = "robot_Large";

    robot_init_tf.p_x = 0; // 0.012
    robot_init_tf.p_y = -0.011;
    robot_init_tf.p_z = 0;
    robot_init_tf.roll = 1.57;
    robot_init_tf.pitch = 0;
    robot_init_tf.yaw = -1.57;

    robot_pose.p_x = 0;
    robot_pose.p_y = 0;
    robot_pose.p_z = 0;
    robot_pose.roll = robot_init_tf.roll;
    robot_pose.pitch = robot_init_tf.pitch;
    robot_pose.yaw = robot_init_tf.yaw;

    another_trans.header.frame_id = "base";
    another_trans.child_frame_id = "robot_Small";

    another_robot_init_tf.p_x = -0.02;
    another_robot_init_tf.p_y = -0.125;
    another_robot_init_tf.p_z = 0.052;
    another_robot_init_tf.roll = 1.57;
    another_robot_init_tf.pitch = 0;
    another_robot_init_tf.yaw = -1.57;

    another_robot_pose.p_x = 0;
    another_robot_pose.p_y = 0;
    another_robot_pose.p_z = 0;
    another_robot_pose.roll = another_robot_init_tf.roll;
    another_robot_pose.pitch = another_robot_init_tf.pitch;
    another_robot_pose.yaw = another_robot_init_tf.yaw;
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

void Robot_move::another_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg) // ally_pose
{
    another_robot_pose.p_x = pose_msg->pose.pose.position.x;
    another_robot_pose.p_y = pose_msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose_msg->pose.pose.orientation, q);
    tf2::Matrix3x3 qt(q);
    double _, yaw;
    qt.getRPY(_, _, yaw);
    another_robot_pose.roll = another_robot_init_tf.roll;
    another_robot_pose.pitch = another_robot_init_tf.pitch;
    another_robot_pose.yaw = another_robot_init_tf.yaw + yaw;
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

    tf2::Quaternion q2;
    q2.setRPY(another_robot_pose.roll, another_robot_pose.pitch, another_robot_pose.yaw);
    another_trans.header.stamp = ros::Time::now();
    another_trans.transform.translation.x = another_robot_init_tf.p_x + another_robot_pose.p_x;
    another_trans.transform.translation.y = another_robot_init_tf.p_y + another_robot_pose.p_y;
    another_trans.transform.translation.z = another_robot_init_tf.p_z;
    another_trans.transform.rotation.x = q2.getX();
    another_trans.transform.rotation.y = q2.getY();
    another_trans.transform.rotation.z = q2.getZ();
    another_trans.transform.rotation.w = q2.getW();

    broadcaster.sendTransform(trans);
    another_broadcaster.sendTransform(another_trans);
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