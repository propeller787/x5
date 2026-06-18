/**
 * @file swept_footprint_checker.cpp
 * @brief 轨迹扫掠 footprint 与 ObstacleField 碰撞检测
 */
#include "safety_node/core/swept_footprint_checker.h"

#include <cmath>
#include <limits>

namespace
{

void transformObstacleToPoseFrame(
  double ox, double oy,
  const PredictedPose & pose,
  double & lx, double & ly)
{
  const double dx = ox - pose.x;
  const double dy = oy - pose.y;
  const double c = std::cos(-pose.theta);
  const double s = std::sin(-pose.theta);
  lx = c * dx - s * dy;
  ly = s * dx + c * dy;
}

} // namespace

SweptFootprintChecker::SweptFootprintChecker(
  const AvoidanceParams & params,
  VehicleFootprint & footprint)
: params_(params)
  , footprint_(footprint)
{
}

void SweptFootprintChecker::updateParams(const AvoidanceParams & params)
{
  params_ = params;
  footprint_.updateParams(params);
}

CollisionCheckResult SweptFootprintChecker::checkTrajectory(
  const TrajectoryRollout & trajectory,
  const ObstacleField & field) const
{
  CollisionCheckResult result;
  const FootprintPolygon polygon = footprint_.inflatedPolygon();
  double path_distance_m = 0.0;

  for (size_t pi = 0; pi < trajectory.poses.size(); ++pi) {
    const auto & pose = trajectory.poses[pi];
    if (pi > 0U) {
      const auto & previous = trajectory.poses[pi - 1U];
      path_distance_m += std::hypot(pose.x - previous.x, pose.y - previous.y);
    }
    result.path_length_m = path_distance_m;
    for (const auto & obs : field.obstacles) {
      if (!obs.valid) {
        continue;
      }
      double lx = 0.0;
      double ly = 0.0;
      transformObstacleToPoseFrame(obs.x, obs.y, pose, lx, ly);

      const double clearance =
        VehicleFootprint::clearanceToPolygon(lx, ly, obs.radius, polygon);
      if (clearance < result.min_distance) {
        result.min_distance = clearance;
      }
      if (clearance <= params_.hard_stop_distance_m &&
        !std::isfinite(result.hard_stop_path_distance_m))
      {
        result.hard_stop_path_distance_m = path_distance_m;
        result.hard_stop_x = obs.x;
        result.hard_stop_y = obs.y;
      }

      if (VehicleFootprint::pointInPolygon(lx, ly, obs.radius, polygon)) {
        if (!result.collision || pose.t < result.ttc) {
          result.collision = true;
          result.ttc = pose.t;
          result.collision_x = obs.x;
          result.collision_y = obs.y;
          result.source = obs.source;
          result.pose_index = static_cast<int>(pi);
          result.collision_path_distance_m = path_distance_m;
        }
      }
    }
  }

  if (!result.collision) {
    result.ttc = std::numeric_limits<double>::infinity();
  }
  return result;
}
