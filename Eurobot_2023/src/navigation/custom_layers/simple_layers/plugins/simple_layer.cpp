#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{
 
SimpleLayer::SimpleLayer() {
  state_ = 0;
  step_ = 0;
  mark_x_bef = 0;
  mark_y_bef = 0;
}
 
void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
 
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}
 
 
void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
 
void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
 
  // mark_x_ = origin_x + cos(origin_yaw);
  // mark_y_ = origin_y + sin(origin_yaw);
 
  // *min_x = std::min(*min_x, mark_x_);
  // *min_y = std::min(*min_y, mark_y_);
  // *max_x = std::max(*max_x, mark_x_);
  // *max_y = std::max(*max_y, mark_y_);
  double extra_obstacle[3][2]={{0.5,0.5},{1,0.5},{1.5,0.5}};
  double obstacle1[2] = {1,1};
  double obstacle2[2] = {2,1};


  int res = 25;
  double mul = (double)step_ / res;

  mark_x_ = obstacle1[0] + (obstacle2[0] - obstacle1[0]) * mul;
  mark_y_ = obstacle1[1] + (obstacle2[1] - obstacle1[1]) * mul;
  if(state_ == 0){
    if(step_ < res){
      step_++;
      // ROS_INFO("1");
    }else if(step_ == res){
      step_--;
      state_ = 1;
      // ROS_INFO("2");
    }
  }else if(state_ == 1){
    if(step_ > 0){
      step_--;
      // ROS_INFO("3");
    }else if(step_ == 0){
      step_++;
      state_ = 0;
      // ROS_INFO("4");
    }
  }
  // ROS_INFO("mark_x: %lf, mark_y: %lf \n", mark_x_, mark_y_);



  *min_x = 0;
  *min_y = 0;
  *max_x = 3;
  *max_y = 2;

  // *min_x = std::min(*min_x, mark_x_);
  // *min_y = std::min(*min_y, mark_y_);
  // *max_x = std::max(*max_x, mark_x_);
  // *max_y = std::max(*max_y, mark_y_);
  
}
 
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned int mx;
  unsigned int my;
  unsigned int mx_bef;
  unsigned int my_bef;
  // if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
  //   master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  //   master_grid.worldToMap(mark_x_bef, mark_y_bef, mx_bef, my_bef);
  //   master_grid.setCost(mx_bef, my_bef, FREE_SPACE);
  // }
  double r = 0.1;
  updateObjCosts(master_grid, mark_x_, mark_y_, r, LETHAL_OBSTACLE);
  updateObjCosts(master_grid, mark_x_bef, mark_y_bef, r, FREE_SPACE);
  mark_x_bef = mark_x_;
  mark_y_bef = mark_y_;
}
 
void SimpleLayer::updateObjCosts(costmap_2d::Costmap2D& master_grid, double mark_x, double mark_y, double r, int cost){
  int n = 20;
  for(int i=0;i<n;i++){
    double x, y;
    unsigned int mx, my;
    x = mark_x + cos(i*(2*M_PI)/n)*r;
    y = mark_y + sin(i*(2*M_PI)/n)*r;
    master_grid.worldToMap(x, y, mx, my);
    master_grid.setCost(mx, my, cost);
  }
}

} // end namespace