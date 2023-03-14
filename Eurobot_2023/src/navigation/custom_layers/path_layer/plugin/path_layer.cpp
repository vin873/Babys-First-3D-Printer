// plugin
#include <pluginlib/class_list_macros.h>

// include
#include "path_layer/path_layer.h"

PLUGINLIB_EXPORT_CLASS(path_layer_namespace::PathLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace path_layer_namespace {

PathLayer::PathLayer() {
}

void PathLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);

    std::string RobotOdom_CB_TopicName;
    std::string RobotPath_CB_TopicName;
    std::string RivalOdom_CB_TopicName[2];

    // read YAML parameter
    nh.param("update_frequency", update_frequency_, 10.0);
    nh.param("enabled", enabled_, true);
    nh.param("RobotType", RobotType, 1);
    nh.param("RivalOdom_Timeout", RivalOdomTimeout, 1.0);
    nh.param("CostScalingFactor", CostScalingFactor, 10.0);
    nh.param("InscribedRadius", InscribedRadius, 0.1);
    nh.param("InflationRadius", InflationRadius, 0.3);
    nh.param("RobotPath_Timeout", RobotPathTimeout, 1.0);
    nh.param("RobotOdom_Timeout", RobotOdomTimeout, 1.0);
    nh.param("RivalOdom_Timeout", RivalOdomTimeout, 1.0);
    nh.param("RobotPath_PredictLength", RobotPredictLength, 1);
    nh.param("RivalOdom_PredictLength", RivalOdomPredictLength, 1);
    nh.param<std::string>("RobotOdom_TopicName", RobotOdom_CB_TopicName, "/robot1/odom");
    nh.param<std::string>("RobotPath_TopicName", RobotPath_CB_TopicName, "/move_base/GlobalPlanner/plan");
    nh.param<std::string>("RivalOdom1_TopicName", RivalOdom_CB_TopicName[0], "/RivalOdom_1");

    // Subscriber
    RobotPath_Sub = nh.subscribe(RobotPath_CB_TopicName, 1000, &PathLayer::RobotPath_CB, this);
    RobotOdom_Sub = nh.subscribe(RobotOdom_CB_TopicName, 1000, &PathLayer::RobotOdom_CB, this);
    RivalOdom_Sub[0] = nh.subscribe(RivalOdom_CB_TopicName[0], 1000, &PathLayer::RivalOdom1_CB, this);
    RivalOdom_Sub[1] = nh.subscribe(RivalOdom_CB_TopicName[1], 1000, &PathLayer::RivalOdom2_CB, this);

    // Init variable
    isRobotPath = isRobotOdom = isRivalOdom[0] = isRivalOdom[1] = false;
    RobotPathLastTime = ros::Time::now();

    current_ = true;
    default_value_ = NO_INFORMATION;

    // Resize map
    matchSize();

    // Register layer
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&PathLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void PathLayer::matchSize() {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
              master->getResolution(),
              master->getOriginX(), master->getOriginY());
}

void PathLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level) {
    enabled_ = config.enabled;
}

void PathLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                             double* min_x, double* min_y, double* max_x, double* max_y) {
    if (!enabled_)
        return;

    // Clean up the old costmap.
    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());

    // Timeout.
    double CurrentTime = ros::Time::now().toSec();
    if (RobotPathTimeout != -1 && CurrentTime - RobotPathLastTime.toSec() > RobotPathTimeout)
        isRobotPath = false;
    if (RobotOdomTimeout != -1 && CurrentTime - RobotOdomLastTime.toSec() > RobotOdomTimeout)
        isRobotOdom = false;
    for (int i = 0; i < 2; i++) {
        if (RivalOdomTimeout != -1 && CurrentTime - RivalOdomLastTime[i].toSec() > RivalOdomTimeout)
            isRivalOdom[i] = false;
    }

    // Get the Costmap lock. (Optional)
    boost::unique_lock<mutex_t> lock(*(getMutex()));

    // Inflation Robot Odom
    if (isRobotOdom) {
        InflatePoint(RobotOdom.pose.pose.position.x, RobotOdom.pose.pose.position.y, min_x, min_y, max_x, max_y);
    }

    // Inflation Robot Path
    if (isRobotPath) {
        for (int i = 0; i < RobotPredictLength; i++) {
            if (RobotPath.poses.size() <= i)
                break;

            double mark_x = RobotPath.poses[i].pose.position.x;
            double mark_y = RobotPath.poses[i].pose.position.y;
            unsigned int mx;
            unsigned int my;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                InflatePoint(mark_x, mark_y, min_x, min_y, max_x, max_y);
            }
        }
    }

    // Add Rival Path to costmap.
    // TODO : Inflate the point.
    for (int Idx = 0; Idx < 2; Idx++) {
        if (isRivalOdom[Idx]) {
            double mark_x = RivalOdom[Idx].pose.pose.position.x;
            double mark_y = RivalOdom[Idx].pose.pose.position.y;
            unsigned int mx;
            unsigned int my;
            for (int i = 0; i < RivalOdomPredictLength; i++) {
                if (worldToMap(mark_x, mark_y, mx, my)) {
                    *min_x = std::min(*min_x, mark_x);
                    *min_y = std::min(*min_y, mark_y);
                    *max_x = std::max(*max_x, mark_x);
                    *max_y = std::max(*max_y, mark_y);
                    setCost(mx, my, LETHAL_OBSTACLE);
                } else {
                    break;
                }
                mark_x += RivalOdom[Idx].twist.twist.linear.x / 1000.0;
                mark_y += RivalOdom[Idx].twist.twist.linear.y / 1000.0;
            }
        }
    }

    if (isRobotPath || isRobotOdom || isRivalOdom[0] || isRivalOdom[1]) {
        *min_x -= 2;
        *min_y -= 2;
        *max_x += 2;
        *max_y += 2;
    }
}

void PathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    if (!enabled_ || !(isRobotOdom || isRobotPath || isRivalOdom[0] || isRivalOdom[1]))
        return;

    // Get the costmap lock.
    boost::unique_lock<mutex_t> lock(*(getMutex()));

    // updateWithMax(master_grid, 0, 0, getSizeInCellsX(), getSizeInCellsY());
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void PathLayer::ExpandPointWithCircle(double x, double y, double Radius, double* min_x, double* min_y, double* max_x, double* max_y) {
    for (int angle = 0; angle <= 360; angle++) {
        double Rad = angle * M_PI / 180;
        double mark_x = x + Radius * cos(Rad);
        double mark_y = y + Radius * sin(Rad);
        unsigned int mx;
        unsigned int my;
        if (worldToMap(mark_x, mark_y, mx, my)) {
            *min_x = std::min(*min_x, mark_x);
            *min_y = std::min(*min_y, mark_y);
            *max_x = std::max(*max_x, mark_x);
            *max_y = std::max(*max_y, mark_y);
            setCost(mx, my, LETHAL_OBSTACLE);
        }
    }
}

void PathLayer::InflatePoint(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y) {
    // MaxDistance = 6.22258 / CostScalingFactor + InscribedRadius;  // 6.22258 = -ln(0.5/252.0)
    // MaxDistance = (double)(((int)(MaxDistance / resolution_) + resolution_) * resolution_);

    double MaxX = x + InflationRadius;
    double MinX = x - InflationRadius;
    double MaxY;
    double MinY;

    double mark_x = 0.0;
    double mark_y = 0.0;
    unsigned int mx;
    unsigned int my;

    double cost;
    double Distance;

    for (double currentPointX = MinX; currentPointX <= MaxX; currentPointX += resolution_) {
        mark_x = currentPointX;
        MaxY = y + sqrt(pow(InflationRadius, 2) - pow(fabs(currentPointX - x), 2));
        MinY = 2 * y - MaxY;

        for (double currentPointY = MinY; currentPointY <= MaxY; currentPointY += resolution_) {
            mark_y = currentPointY;
            if (worldToMap(mark_x, mark_y, mx, my)) {
                *min_x = std::min(*min_x, mark_x);
                *min_y = std::min(*min_y, mark_y);
                *max_x = std::max(*max_x, mark_x);
                *max_y = std::max(*max_y, mark_y);

                Distance = sqrt(pow(fabs(x - currentPointX), 2) + pow(fabs(y - currentPointY), 2));

                cost = round(252 * exp(-CostScalingFactor * (Distance - InscribedRadius)));
                cost = std::min(cost, 254.0);
                cost = std::max(cost, 0.0);

                if (getCost(mx, my) != NO_INFORMATION) {
                    setCost(mx, my, std::max((unsigned char)cost, getCost(mx, my)));
                } else {
                    setCost(mx, my, cost);
                }
            }
        }
    }
}

void PathLayer::RobotPath_CB(const nav_msgs::Path& Path) {
    this->RobotPath = Path;
    isRobotPath = true;
    RobotPathLastTime = ros::Time::now();
}

void PathLayer::RobotOdom_CB(const nav_msgs::Odometry& Odom) {
    this->RobotOdom = Odom;
    isRobotOdom = true;
    RobotOdomLastTime = ros::Time::now();
}

void PathLayer::RivalOdom1_CB(const nav_msgs::Odometry& Odom) {
    RivalOdom[0] = Odom;
    isRivalOdom[0] = true;
    RivalOdomLastTime[0] = ros::Time::now();
}

void PathLayer::RivalOdom2_CB(const nav_msgs::Odometry& Odom) {
    RivalOdom[1] = Odom;
    isRivalOdom[1] = true;
    RivalOdomLastTime[1] = ros::Time::now();
}

}  // namespace path_layer_namespace