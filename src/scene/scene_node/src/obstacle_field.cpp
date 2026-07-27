#include "obstacle_field.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scene_node::avoidance
{
namespace
{

bool isFinite2d(double x, double y)
{
  return std::isfinite(x) && std::isfinite(y);
}

double distanceFromBase(const ObstaclePrimitive & obstacle)
{
  return std::hypot(obstacle.x, obstacle.y);
}

}  // namespace

ObstacleField ObstacleFieldBuilder::buildFromClusters(
  const std::map<std::string, std::vector<ClusterInfo>> & radar_clusters,
  const std::map<std::string, rclcpp::Time> & radar_times,
  const rclcpp::Time & now,
  const ObstacleFieldParams & params) const
{
  ObstacleField field;
  field.stamp = now;

  for (const auto & [source, clusters] : radar_clusters) {
    if (!params.include_tilt && (source == "left_tilt" || source == "right_tilt")) {
      continue;
    }
    if (!params.include_back && source == "back") {
      continue;
    }

    auto time_it = radar_times.find(source);
    if (time_it == radar_times.end()) {
      continue;
    }

    const double age_s = (now - time_it->second).seconds();
    if (age_s > params.radar_timeout_s) {
      continue;
    }

    for (const auto & cluster : clusters) {
      auto obstacle = toObstacle(cluster, source, time_it->second, params);
      if (obstacle.valid) {
        field.obstacles.push_back(std::move(obstacle));
      }
    }
  }

  std::sort(
    field.obstacles.begin(), field.obstacles.end(), [](const auto & lhs, const auto & rhs) {
      return distanceFromBase(lhs) < distanceFromBase(rhs);
    });
  if (field.obstacles.size() > params.max_obstacles) {
    field.obstacles.resize(params.max_obstacles);
  }

  return field;
}

ObstaclePrimitive ObstacleFieldBuilder::toObstacle(
  const ClusterInfo & cluster,
  const std::string & source,
  const rclcpp::Time & stamp,
  const ObstacleFieldParams & params) const
{
  ObstaclePrimitive obstacle;
  obstacle.x = cluster.center_x;
  obstacle.y = cluster.center_y;
  obstacle.z = cluster.center_z;
  obstacle.min_x = cluster.min_x;
  obstacle.max_x = cluster.max_x;
  obstacle.min_y = cluster.min_y;
  obstacle.max_y = cluster.max_y;
  obstacle.source = source;
  obstacle.stamp = stamp;

  if (!isFinite2d(obstacle.x, obstacle.y)) {
    return obstacle;
  }

  const double width = std::max(0.0, obstacle.max_y - obstacle.min_y);
  const double depth = std::max(0.0, obstacle.max_x - obstacle.min_x);
  const double size_radius = 0.5 * std::hypot(width, depth);
  obstacle.radius = std::clamp(
    std::max(params.default_radius, size_radius),
    params.min_radius,
    params.max_radius);
  obstacle.valid = true;
  return obstacle;
}

}  // namespace scene_node::avoidance
