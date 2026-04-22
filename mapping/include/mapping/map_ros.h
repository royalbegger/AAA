#ifndef MAP_ROS_H_
#define MAP_ROS_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include "mapping/obstacle_layer.h"
#include "mapping/unknown_layer.h"
#include "mapping/goal_layer.h"

namespace narrow_nav
{

class MapRosNode
{
public:
  MapRosNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MapRosNode();

  bool init();
  void spin();

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);
  void publishLayers();
  void buildFusedMap();

  static void initEmptyLayerFromMap(const nav_msgs::OccupancyGrid& map,
                                    nav_msgs::OccupancyGrid& layer,
                                    int8_t default_value);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;

  ros::Publisher obstacle_layer_pub_;
  ros::Publisher unknown_layer_pub_;
  ros::Publisher goal_layer_pub_;
  ros::Publisher fused_map_pub_;

  ros::Timer timer_;

  std::string map_topic_;
  std::string goal_topic_;
  std::string obstacle_layer_topic_;
  std::string unknown_layer_topic_;
  std::string goal_layer_topic_;
  std::string fused_map_topic_;

  double process_rate_;

  nav_msgs::OccupancyGrid latest_map_;
  geometry_msgs::PoseStamped latest_goal_;

  bool has_map_;
  bool has_goal_;

  nav_msgs::OccupancyGrid obstacle_layer_;
  nav_msgs::OccupancyGrid unknown_layer_;
  nav_msgs::OccupancyGrid goal_layer_;
  nav_msgs::OccupancyGrid fused_map_;

  std::shared_ptr<ObstacleLayerGenerator> obstacle_generator_;
  std::shared_ptr<UnknownLayerGenerator> unknown_generator_;
  std::shared_ptr<GoalLayerGenerator> goal_generator_;
};

}  // namespace narrow_nav

#endif  // MAP_ROS_H_
