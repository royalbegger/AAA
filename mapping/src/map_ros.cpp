#include "mapping/map_ros.h"

#include <algorithm>

namespace narrow_nav
{
namespace
{

constexpr int kMaxLayerCost = 100;

}  // namespace

MapRosNode::MapRosNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    has_map_(false),
    has_goal_(false),
    process_rate_(10.0)
{
}

MapRosNode::~MapRosNode()
{
}

bool MapRosNode::init()
{
  // parameters
  pnh_.param<std::string>("map_topic", map_topic_, "/map");
  pnh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
  pnh_.param<std::string>("obstacle_layer_topic", obstacle_layer_topic_, "/map_layers/obstacle");
  pnh_.param<std::string>("unknown_layer_topic", unknown_layer_topic_, "/map_layers/unknown");
  pnh_.param<std::string>("goal_layer_topic", goal_layer_topic_, "/map_layers/goal");
  pnh_.param<std::string>("fused_map_topic", fused_map_topic_, "/fusion_map");
  pnh_.param<double>("process_rate", process_rate_, 10.0);

  // subscribers
  map_sub_ = nh_.subscribe(map_topic_, 1, &MapRosNode::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &MapRosNode::goalCallback, this);

  // publishers
  obstacle_layer_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(obstacle_layer_topic_, 1, true);
  unknown_layer_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>(unknown_layer_topic_, 1, true);
  goal_layer_pub_     = nh_.advertise<nav_msgs::OccupancyGrid>(goal_layer_topic_, 1, true);
  fused_map_pub_      = nh_.advertise<nav_msgs::OccupancyGrid>(fused_map_topic_, 1, true);

  // modules
  obstacle_generator_.reset(new ObstacleLayerGenerator());
  unknown_generator_.reset(new UnknownLayerGenerator());
  goal_generator_.reset(new GoalLayerGenerator());

  obstacle_generator_->configure(pnh_);
  unknown_generator_->configure(pnh_);
  goal_generator_->configure(pnh_);

  // timer
  timer_ = nh_.createTimer(ros::Duration(1.0 / process_rate_),
                           &MapRosNode::timerCallback,
                           this);

  ROS_INFO("[map_ros] node initialized.");
  ROS_INFO("[map_ros] fused map topic: %s", fused_map_topic_.c_str());
  return true;
}

void MapRosNode::spin()
{
  ros::spin();
}

void MapRosNode::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (msg == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "[map_ros] null map received.");
    return;
  }

  latest_map_ = *msg;
  has_map_ = true;
}

void MapRosNode::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!msg)
  {
    ROS_WARN_THROTTLE(1.0, "[map_ros] null goal received.");
    return;
  }

  latest_goal_ = *msg;
  has_goal_ = true;
}

void MapRosNode::timerCallback(const ros::TimerEvent& /*event*/)
{
  if (!has_map_)
  {
    ROS_WARN_THROTTLE(2.0, "[map_ros] waiting for map...");
    return;
  }

  // 1. obstacle layer
  obstacle_generator_->process(latest_map_, obstacle_layer_);

  // 2. unknown layer
  unknown_generator_->process(latest_map_, obstacle_layer_, unknown_layer_);

  // 3. goal layer
  if (has_goal_)
  {
    goal_generator_->process(latest_map_, latest_goal_, goal_layer_);
  }
  else
  {
    initEmptyLayerFromMap(latest_map_, goal_layer_, 0);
  }

  // 4. fused map
  buildFusedMap();

  // 5. publish
  publishLayers();
}

void MapRosNode::buildFusedMap()
{
  fused_map_.header = latest_map_.header;
  fused_map_.info = latest_map_.info;
  fused_map_.data.assign(latest_map_.data.size(), 0);

  const std::size_t cell_count = latest_map_.data.size();
  if (obstacle_layer_.data.size() != cell_count ||
      unknown_layer_.data.size() != cell_count ||
      goal_layer_.data.size() != cell_count)
  {
    ROS_WARN_THROTTLE(1.0, "[map_ros] layer size mismatch, skip fused map generation.");
    return;
  }

  for (std::size_t i = 0; i < cell_count; ++i)
  {
    const int fused_cost = static_cast<int>(obstacle_layer_.data[i]) +
                           static_cast<int>(unknown_layer_.data[i]);
    fused_map_.data[i] = static_cast<int8_t>(std::min(kMaxLayerCost, fused_cost));
  }
}

void MapRosNode::publishLayers()
{
  const ros::Time now = ros::Time::now();

  obstacle_layer_.header.stamp = now;
  unknown_layer_.header.stamp = now;
  goal_layer_.header.stamp = now;
  fused_map_.header.stamp = now;

  obstacle_layer_pub_.publish(obstacle_layer_);
  unknown_layer_pub_.publish(unknown_layer_);
  goal_layer_pub_.publish(goal_layer_);
  fused_map_pub_.publish(fused_map_);
}

void MapRosNode::initEmptyLayerFromMap(const nav_msgs::OccupancyGrid& map,
                                       nav_msgs::OccupancyGrid& layer,
                                       int8_t default_value)
{
  layer.header = map.header;
  layer.info = map.info;
  layer.data.assign(map.data.size(), default_value);
}

}  // namespace narrow_nav
