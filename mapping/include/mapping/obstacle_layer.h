#ifndef OBSTACLE_LAYER_H_
#define OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace narrow_nav
{

class ObstacleLayerGenerator
{
public:
  ObstacleLayerGenerator();
  ~ObstacleLayerGenerator();

  void configure(const ros::NodeHandle& pnh);
  void process(const nav_msgs::OccupancyGrid& input_map,
               nav_msgs::OccupancyGrid& obstacle_layer);
};

}  // namespace narrow_nav

#endif