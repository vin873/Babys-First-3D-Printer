#ifndef PATH_LAYER_H_
#define PATH_LAYER_H_

// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/time.h>

// costmap
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

// msgs
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

// other
#include <algorithm>
#include <cmath>
#include <string>

namespace path_layer_namespace {

enum class Robot_type {
    robot1,
    robot2
};

// public costmap_2d::CostmapLayer
// public costmap_2d::Layer, public costmap_2d::Costmap2D
class PathLayer : public costmap_2d::CostmapLayer {
   public:
    PathLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    bool isDiscretized() {
        return true;
    }

    virtual void matchSize();

   private:
    void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;

    ros::NodeHandle Global_nh;
    double update_frequency_;

    int RobotType;

    // ------------------------- Inflation -------------------------
    // Param
    double CostScalingFactor;
    double InscribedRadius;
    double InflationRadius;
    // double MaxDistance;

    // Function
    void ExpandPointWithCircle(double x, double y, double Radius, double* min_x, double* min_y, double* max_x, double* max_y);
    void InflatePoint(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

    // ------------------------- RobotPath -------------------------
    // Sub
    ros::Subscriber RobotPath_Sub;
    ros::Subscriber RobotOdom_Sub;
    nav_msgs::Odometry RobotOdom;
    nav_msgs::Path RobotPath;

    void RobotPath_CB(const nav_msgs::Path& Path);
    void RobotOdom_CB(const nav_msgs::Odometry& Odom);

    // Time
    ros::Time RobotPathLastTime;
    ros::Time RobotOdomLastTime;
    double RobotPathTimeout;
    double RobotOdomTimeout;
    bool isRobotPath;
    bool isRobotOdom;

    // Param
    int RobotPredictLength;

    // ------------------------- RivalOdom -------------------------
    // Sub
    ros::Subscriber RivalOdom_Sub[2];
    nav_msgs::Odometry RivalOdom[2];

    void RivalOdom1_CB(const nav_msgs::Odometry& Odom);
    void RivalOdom2_CB(const nav_msgs::Odometry& Odom);

    // Time
    ros::Time RivalOdomLastTime[2];
    double RivalOdomTimeout;
    bool isRivalOdom[2];

    // Param
    int RivalOdomPredictLength;
};

}  // namespace path_layer_namespace

#endif