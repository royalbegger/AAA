#include "mapping/unknown_layer.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace narrow_nav
{
namespace
{

constexpr int8_t kUnknownValue = -1;
constexpr int8_t kOccupiedValue = 100;

bool isInsideMap(const int x, const int y, const int width, const int height)
{
  return x >= 0 && x < width && y >= 0 && y < height;
}

int findObstacleDistanceAlongDirection(const nav_msgs::OccupancyGrid& input_map,
                                       const int start_x,
                                       const int start_y,
                                       const int dx,
                                       const int dy,
                                       const int max_scan_cells)
{
  const int width = static_cast<int>(input_map.info.width);
  const int height = static_cast<int>(input_map.info.height);

  for (int step = 1; step <= max_scan_cells; ++step)
  {
    const int x = start_x + step * dx;
    const int y = start_y + step * dy;
    if (!isInsideMap(x, y, width, height))
    {
      return -1;
    }

    const int idx = y * width + x;
    if (input_map.data[idx] == kOccupiedValue)
    {
      return step;
    }
  }

  return -1;
}

bool isNarrowUnknownCell(const nav_msgs::OccupancyGrid& input_map,
                         const int x,
                         const int y,
                         const int width_threshold_cells,
                         const int max_scan_cells)
{
  static constexpr std::array<std::array<int, 4>, 4> kDirectionPairs = {{
      {{1, 0, -1, 0}},
      {{0, 1, 0, -1}},
      {{1, 1, -1, -1}},
      {{1, -1, -1, 1}},
  }};

  for (const auto& pair : kDirectionPairs)
  {
    const int positive_distance = findObstacleDistanceAlongDirection(
        input_map, x, y, pair[0], pair[1], max_scan_cells);
    if (positive_distance < 0)
    {
      continue;
    }

    const int negative_distance = findObstacleDistanceAlongDirection(
        input_map, x, y, pair[2], pair[3], max_scan_cells);
    if (negative_distance < 0)
    {
      continue;
    }

    const double step_length_cells =
        (std::abs(pair[0]) + std::abs(pair[1]) == 2) ? std::sqrt(2.0) : 1.0;
    const double width_cells =
        static_cast<double>(positive_distance + negative_distance) * step_length_cells;

    if (width_cells <= static_cast<double>(width_threshold_cells))
    {
      return true;
    }
  }

  return false;
}

}  // namespace

UnknownLayerGenerator::UnknownLayerGenerator() = default;

UnknownLayerGenerator::~UnknownLayerGenerator() = default;

void UnknownLayerGenerator::configure(const ros::NodeHandle& pnh)
{
  pnh.param<int>("unknown_layer/cost_value", unknown_cost_value_, 80);
  pnh.param<int>("unknown_layer/narrow_cost_value", narrow_unknown_cost_value_, 90);
  pnh.param<double>("unknown_layer/narrow_width_threshold",
                    narrow_unknown_width_threshold_,
                    0.5);
  pnh.param<int>("unknown_layer/narrow_max_scan_cells", narrow_unknown_max_scan_cells_, 20);
}

void UnknownLayerGenerator::process(const nav_msgs::OccupancyGrid& input_map,
                                    const nav_msgs::OccupancyGrid& obstacle_layer,
                                    nav_msgs::OccupancyGrid& unknown_layer)
{
  (void)obstacle_layer;

  const int width = static_cast<int>(input_map.info.width);
  const int height = static_cast<int>(input_map.info.height);
  const std::size_t cell_count = static_cast<std::size_t>(width * height);

  unknown_layer.header = input_map.header;
  unknown_layer.info = input_map.info;
  unknown_layer.data.assign(cell_count, 0);

  if (width <= 0 || height <= 0 || input_map.data.size() != cell_count)
  {
    ROS_WARN_THROTTLE(1.0, "Invalid input map for unknown layer generation.");
    return;
  }

  if (input_map.info.resolution <= 0.0)
  {
    ROS_WARN_THROTTLE(1.0, "Invalid map resolution for unknown layer generation.");
    return;
  }

  const int8_t unknown_cost_value =
      static_cast<int8_t>(std::max(0, std::min(100, unknown_cost_value_)));
  const int8_t narrow_unknown_cost_value =
      static_cast<int8_t>(std::max(0, std::min(100, narrow_unknown_cost_value_)));
  const int width_threshold_cells = std::max(
      1, static_cast<int>(std::lround(narrow_unknown_width_threshold_ / input_map.info.resolution)));
  const int max_scan_cells = std::max(
      1, std::max(narrow_unknown_max_scan_cells_, width_threshold_cells));

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const std::size_t idx = static_cast<std::size_t>(y * width + x);
      if (input_map.data[idx] != kUnknownValue)
      {
        continue;
      }

      unknown_layer.data[idx] =
          isNarrowUnknownCell(input_map, x, y, width_threshold_cells, max_scan_cells)
              ? narrow_unknown_cost_value
              : unknown_cost_value;
    }
  }
}

}  // namespace narrow_nav
