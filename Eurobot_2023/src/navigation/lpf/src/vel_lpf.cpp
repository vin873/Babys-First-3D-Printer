#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

double beta[3] = {0};
double vx[2] = {0};
double vy[2] = {0};
double vth[2] = {0};

void lpf_callback(const geometry_msgs::Twist& data){
  //Simple low-pass filter
  vx[1] = beta[0]*vx[0] + (1-beta[0])*data.linear.x;
  vy[1] = beta[1]*vy[0] + (1-beta[1])*data.linear.y;
  vth[1] = beta[2]*vth[0] + (1-beta[2])*data.angular.z;

  ROS_INFO("%f %f\n", (ros::Time::now()).toSec(), data.angular.z);
  // ROS_INFO("vy:\t%f\n", vy[1]);
  // ROS_INFO("vth:\t%f\n\n", vth[1]);

  //V(n-1) = V(n)
  vx[0] = vx[1];
  vy[0] = vy[1];
  vth[0] = vth[1];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "vel_lpf");
  
  ros::NodeHandle nh;
  ros::Publisher lpf_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
  ros::Subscriber sub = nh.subscribe("raw_cmd_vel", 1000, lpf_callback);
    
  ros::Rate r(20.0);

  vx[0] = 0;
  vy[0] = 0;
  vth[0] = 0;

  beta[0] = 0.2;
  beta[1] = 0.2;
  beta[2] = 0.2;

  while(ros::ok()){
    ros::spinOnce();
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vx[1];
    cmd_vel.linear.y = vy[1];
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = vth[1];
    lpf_pub.publish(cmd_vel);
    
    r.sleep();
  }
}