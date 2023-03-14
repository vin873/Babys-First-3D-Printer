#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_robot_state");
    ros::NodeHandle nh;
    tf::TransformBroadcaster L_broadcaster;
    tf::TransformBroadcaster S_broadcaster;
    ros::Rate loop_rate(10);

    // message declarations
    geometry_msgs::TransformStamped L_trans;
    geometry_msgs::TransformStamped S_trans;
    L_trans.header.frame_id = "base";
    L_trans.child_frame_id = "robot_Large";

    S_trans.header.frame_id = "base";
    S_trans.child_frame_id = "robot_Small";

    while (ros::ok())
    {
        // update Large transform
        L_trans.header.stamp = ros::Time::now();
        L_trans.transform.translation.x = 0.012;
        L_trans.transform.translation.y = -0.011;
        L_trans.transform.translation.z = 0;
        L_trans.transform.rotation.x = 0.7068252;
        L_trans.transform.rotation.y = 0.0;
        L_trans.transform.rotation.z = 0.0;
        L_trans.transform.rotation.w = 0.7073883;

        // update Small transform
        S_trans.header.stamp = ros::Time::now();
        S_trans.transform.translation.x = -0.016;
        S_trans.transform.translation.y = -0.13;
        S_trans.transform.translation.z = 0.052;
        S_trans.transform.rotation.x = 0.7068252;
        S_trans.transform.rotation.y = 0.0;
        S_trans.transform.rotation.z = 0.0;
        S_trans.transform.rotation.w = 0.7073883;

        // send the transform
        L_broadcaster.sendTransform(L_trans);
        S_broadcaster.sendTransform(S_trans);

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}