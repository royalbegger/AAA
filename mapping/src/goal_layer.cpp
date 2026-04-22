#include "mapping/goal_layer.h"

#include <cmath>

namespace narrow_nav
{
namespace
{

constexpr int8_t kFreeValue = 0;
constexpr int8_t kGoalValue = 100;

}  // namespace

GoalLayerGenerator::GoalLayerGenerator() = default;

GoalLayerGenerator::~GoalLayerGenerator() = default;

void GoalLayerGenerator::configure(const ros::NodeHandle& pnh)
{
  (void)pnh;
}

void GoalLayerGenerator::process(const nav_msgs::OccupancyGrid& input_map,
                                 const geometry_msgs::PoseStamped& goal,
                                 nav_msgs::OccupancyGrid& goal_layer)
{
  const int width = static_cast<int>(input_map.info.width);
  const int height = static_cast<int>(input_map.info.height);
  const std::size_t cell_count = static_cast<std::size_t>(width * height);

  goal_layer.header = input_map.header;
  goal_layer.info = input_map.info;
  goal_layer.data.assign(cell_count, kFreeValue);

  if (width <= 0 || height <= 0 || input_map.data.size() != cell_count)
  {
    ROS_WARN_THROTTLE(1.0, "Invalid input map for goal layer generation.");
    return;
  }

  const double resolution = input_map.info.resolution;
  if (resolution <= 0.0)
  {
    ROS_WARN_THROTTLE(1.0, "Invalid map resolution for goal layer generation.");
    return;
  }

  const double origin_x = input_map.info.origin.position.x;
  const double origin_y = input_map.info.origin.position.y;
  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;

  const int map_x = static_cast<int>(std::floor((goal_x - origin_x) / resolution));
  const int map_y = static_cast<int>(std::floor((goal_y - origin_y) / resolution));

  if (map_x < 0 || map_x >= width || map_y < 0 || map_y >= height)
  {
    ROS_WARN_THROTTLE(1.0, "Goal is outside map bounds.");
    return;
  }

  const std::size_t idx = static_cast<std::size_t>(map_y * width + map_x);
  goal_layer.data[idx] = kGoalValue;
}

}  // namespace narrow_nav
