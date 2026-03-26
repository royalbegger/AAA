#ifndef UNKNOWN_LAYER_H_
#define UNKNOWN_LAYER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace narrow_nav
{

class UnknownLayerGenerator
{
public:
  UnknownLayerGenerator();
  ~UnknownLayerGenerator();

  void configure(const ros::NodeHandle& pnh);
  void process(const nav_msgs::OccupancyGrid& input_map,
               const nav_msgs::OccupancyGrid& obstacle_layer,
               nav_msgs::OccupancyGrid& unknown_layer);

private:
  int unknown_cost_value_ = 80;
  int narrow_unknown_cost_value_ = 90;
  double narrow_unknown_width_threshold_ = 0.5;
  int narrow_unknown_max_scan_cells_ = 20;
};

}  // namespace narrow_nav

#endif
