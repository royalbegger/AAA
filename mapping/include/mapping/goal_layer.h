#ifndef GOAL_LAYER_H_
#define GOAL_LAYER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

namespace narrow_nav
{

class GoalLayerGenerator
{
public:
  GoalLayerGenerator();
  ~GoalLayerGenerator();

  void configure(const ros::NodeHandle& pnh);
  void process(const nav_msgs::OccupancyGrid& input_map,
               const geometry_msgs::PoseStamped& goal,
               nav_msgs::OccupancyGrid& goal_layer);
};

}  // namespace narrow_nav

#endif