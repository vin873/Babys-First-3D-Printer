#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
std::string pic_path = "";

int main(int argc, char **argv)
{
    pic_path=ros::package::getPath("rviz_display");
    pic_path+="/maps/eueu_map/eurobot_table.png";
    ros::init(argc, argv, "publish_image");
    ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(pic_path);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("Eurobot_Image", 10);
    ros::Rate loop_rate(5);

    while (nh.ok())
    {
        pub.publish(ros_image);
        loop_rate.sleep();
    }
}