#include "mapping/obstacle_layer.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace narrow_nav
{
namespace
{

constexpr float kInf = 1e8f;
constexpr float kDefaultMaxDistance = 1.0f;
constexpr int8_t kOccupiedValue = 100;
constexpr int8_t kUnknownValue = -1;

void distanceTransform1D(const std::vector<float>& f,
                         std::vector<float>& d,
                         int n)
{
  std::vector<int> v(n);
  std::vector<float> z(n + 1);

  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  for (int q = 1; q < n; ++q)
  {
    float s = 0.0f;
    do
    {
      s = ((f[q] + static_cast<float>(q * q)) -
           (f[v[k]] + static_cast<float>(v[k] * v[k]))) /
          (2.0f * static_cast<float>(q - v[k]));
      if (s <= z[k])
      {
        --k;
      }
    } while (s <= z[k]);

    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }

  k = 0;
  for (int q = 0; q < n; ++q)
  {
    while (z[k + 1] < static_cast<float>(q))
    {
      ++k;
    }

    const float dx = static_cast<float>(q - v[k]);
    d[q] = dx * dx + f[v[k]];
  }
}

bool findBoundaryRange(const nav_msgs::OccupancyGrid& input_map,
                       int& x_min,
                       int& x_max)
{
  const int width = static_cast<int>(input_map.info.width);
  const int height = static_cast<int>(input_map.info.height);
  if (width <= 0 || height <= 0)
  {
    return false;
  }

  bool found_known = false;
  x_min = width - 1;
  x_max = 0;

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const int idx = y * width + x;
      const int8_t occ = input_map.data[idx];

      if (occ == kUnknownValue)
      {
        continue;
      }

      found_known = true;
      x_min = std::min(x_min, x);
      x_max = std::max(x_max, x);
    }
  }

  if (!found_known)
  {
    x_min = 0;
    x_max = width - 1;
    return false;
  }

  return true;
}

}  // namespace

ObstacleLayerGenerator::ObstacleLayerGenerator() = default;

ObstacleLayerGenerator::~ObstacleLayerGenerator() = default;

void ObstacleLayerGenerator::configure(const ros::NodeHandle& pnh)
{
  (void)pnh;
}

void ObstacleLayerGenerator::process(const nav_msgs::OccupancyGrid& input_map,
                                     nav_msgs::OccupancyGrid& obstacle_layer)
{
  const int width = static_cast<int>(input_map.info.width);
  const int height = static_cast<int>(input_map.info.height);

  obstacle_layer.header = input_map.header;
  obstacle_layer.info = input_map.info;
  obstacle_layer.data.assign(width * height, 0);

  if (width <= 0 || height <= 0 ||
      input_map.data.size() != static_cast<std::size_t>(width * height))
  {
    ROS_WARN_THROTTLE(1.0, "Invalid input map for obstacle layer generation.");
    return;
  }

  std::vector<float> grid(width * height, kInf);

  int x_min = 0;
  int x_max = width - 1;
  const bool has_boundary_range = findBoundaryRange(input_map, x_min, x_max);

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      const int idx = y * width + x;
      const int8_t occ = input_map.data[idx];

      if ((has_boundary_range && (x < x_min || x > x_max)) ||
          occ == kOccupiedValue)
      {
        grid[idx] = 0.0f;
      }
    }
  }

  std::vector<float> temp(width * height, kInf);
  for (int y = 0; y < height; ++y)
  {
    std::vector<float> f(width);
    std::vector<float> d(width);
    for (int x = 0; x < width; ++x)
    {
      f[x] = grid[y * width + x];
    }

    distanceTransform1D(f, d, width);

    for (int x = 0; x < width; ++x)
    {
      temp[y * width + x] = d[x];
    }
  }

  std::vector<float> dist(width * height, kInf);
  for (int x = 0; x < width; ++x)
  {
    std::vector<float> f(height);
    std::vector<float> d(height);
    for (int y = 0; y < height; ++y)
    {
      f[y] = temp[y * width + x];
    }

    distanceTransform1D(f, d, height);

    for (int y = 0; y < height; ++y)
    {
      dist[y * width + x] = std::sqrt(d[y]) * input_map.info.resolution;
    }
  }

  for (int i = 0; i < width * height; ++i)
  {
    if (input_map.data[i] == kUnknownValue)
    {
      obstacle_layer.data[i] = 0;
      continue;
    }

    if (grid[i] == 0.0f)
    {
      obstacle_layer.data[i] = kOccupiedValue;
      continue;
    }

    const float clipped_distance = std::min(dist[i], kDefaultMaxDistance);
    obstacle_layer.data[i] = static_cast<int8_t>(
        (1.0f - clipped_distance / kDefaultMaxDistance) * 100.0f);
  }
}

}  // namespace narrow_nav
