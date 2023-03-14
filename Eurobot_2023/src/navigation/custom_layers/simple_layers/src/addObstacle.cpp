#include <pluginlib/class_loader.h>
#include <simple_layers/simple_layer.h>
#include <simple_layers/grid_layer.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "addObstacle");
    ros::NodeHandle nh;

    pluginlib::ClassLoader<costmap_2d::Layer> layer_loader("costmap_2d", "costmap_2d::Layer");
    auto grid = layer_loader.createInstance("simple_layer_namespace::GridLayer");
    
    // double robot_x = 1;
    // double robot_y = 1;
    // double robot_yaw = 0;
    // double min_x = 0;
    // double min_y = 0;
    // double max_x = 5;
    // double max_y = 5;

    while(ros::ok()){
        // grid->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y, &max_x, &max_y);
        ros::Duration(1).sleep();
    }
    
    return 0;

}