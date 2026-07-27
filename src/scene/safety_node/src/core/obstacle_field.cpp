/**
 * @file obstacle_field.cpp
 * @brief 点云体素化 → ObstaclePrimitive 列表
 */
#include "safety_node/core/obstacle_field.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace
{

std::pair<int64_t, int64_t> voxelKey(double x, double y, double voxel)
{
  const int64_t ix = static_cast<int64_t>(std::floor(x / voxel));
  const int64_t iy = static_cast<int64_t>(std::floor(y / voxel));
  return {ix, iy};
}

double distanceToOrigin(const ObstaclePrimitive & o)
{
  return std::hypot(o.x, o.y);
}

} // namespace

ObstacleField ObstacleFieldBuilder::buildFromPoints(
  const std::vector<std::tuple<double, double, double>> & points,
  const rclcpp::Time & stamp,
  const AvoidanceParams & params,
  double voxel_size_m)
{
  std::vector<ObstaclePoint> named_points;
  named_points.reserve(points.size());
  for (const auto & pt : points) {
    ObstaclePoint named;
    named.x = std::get<0>(pt);
    named.y = std::get<1>(pt);
    named.z = std::get<2>(pt);
    named.source = "lidar";
    named_points.push_back(named);
  }
  return buildFromPoints(named_points, stamp, params, voxel_size_m, params.max_obstacle_primitives);
}

ObstacleField ObstacleFieldBuilder::buildFromPoints(
  const std::vector<ObstaclePoint> & points,
  const rclcpp::Time & stamp,
  const AvoidanceParams & params,
  double voxel_size_m,
  size_t max_primitives)
{
  ObstacleField field;
  field.stamp = stamp;

  const double voxel = std::max(0.05, voxel_size_m);
  const double radius = std::max(0.0, params.footprint_obstacle_radius_default);

  struct VoxelAcc
  {
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    int count = 0;
    std::string source;
  };
  std::map<std::pair<int64_t, int64_t>, VoxelAcc> voxels;

  for (const auto & pt : points) {
    const double x = pt.x;
    const double y = pt.y;
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    auto & acc = voxels[voxelKey(x, y, voxel)];
    acc.sx += x;
    acc.sy += y;
    acc.sz += std::isfinite(pt.z) ? pt.z : 0.0;
    acc.count += 1;
    if (acc.source.empty()) {
      acc.source = pt.source;
    } else if (acc.source != pt.source) {
      acc.source = "mixed";
    }
  }

  field.obstacles.reserve(voxels.size());
  for (const auto & [key, acc] : voxels) {
    (void)key;
    if (acc.count <= 0) {
      continue;
    }
    ObstaclePrimitive prim;
    prim.x = acc.sx / acc.count;
    prim.y = acc.sy / acc.count;
    prim.z = acc.sz / acc.count;
    prim.radius = radius;
    prim.stamp = stamp;
    prim.valid = true;
    prim.source = acc.source.empty() ? "lidar" : acc.source;
    field.obstacles.push_back(prim);
  }

  const size_t limit = max_primitives > 0U ? max_primitives : params.max_obstacle_primitives;
  if (field.obstacles.size() > limit) {
    std::partial_sort(
      field.obstacles.begin(),
      field.obstacles.begin() + static_cast<std::ptrdiff_t>(limit),
      field.obstacles.end(),
      [](const ObstaclePrimitive & a, const ObstaclePrimitive & b) {
        return distanceToOrigin(a) < distanceToOrigin(b);
      });
    field.obstacles.resize(limit);
  }

  return field;
}
