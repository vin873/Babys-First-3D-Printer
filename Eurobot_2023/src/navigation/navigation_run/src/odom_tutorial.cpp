#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


double car[3] = {0};
double x = 0.5;
double y = 0.5;
double th = (double)3.1415926/2;

void vel_callback(const geometry_msgs::Twist& data){
    car[0] = data.linear.x;
    car[1] = data.linear.y;
    car[2] = data.angular.z;
}

void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& data){
    x =  data.pose.pose.position.x;
    y =  data.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(data.pose.pose.orientation, quat);
    double r, p, y;
    tf::Matrix3x3(quat).getRPY(r, p, y);
    th = y;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, vel_callback);
  ros::Subscriber sub_initial_pose = nh.subscribe("initialpose", 1000, initial_pose_callback);

  tf::TransformBroadcaster odom_broadcaster;



  double vx = 0;
  double vy = 0;
  double vth = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  while(nh.ok()){
    vx = car[0];
    vy = car[1];
    vth = car[2];
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}