#include "swept_footprint_checker.h"

#include <algorithm>
#include <cmath>

namespace scene_node::avoidance
{
namespace
{

double clearanceToBox(double local_x, double local_y, double front, double back, double half_width)
{
  double dx = 0.0;
  if (local_x > front) {
    dx = local_x - front;
  } else if (local_x < -back) {
    dx = -back - local_x;
  }

  const double dy = std::max(0.0, std::abs(local_y) - half_width);
  return std::hypot(dx, dy);
}

RiskLevel levelFromTtc(double ttc, const RiskParams & params)
{
  if (ttc <= params.emergency_ttc_s) {
    return RiskLevel::BRAKING;
  }
  if (ttc <= params.limiting_ttc_s) {
    return RiskLevel::LIMITING;
  }
  if (ttc <= params.warning_ttc_s) {
    return RiskLevel::WARNING;
  }
  return RiskLevel::CLEAR;
}

}  // namespace

CollisionRisk SweptFootprintChecker::evaluate(
  const CandidateTrajectory & trajectory,
  const ObstacleField & field,
  const FootprintBox & footprint,
  const FootprintMargins & margins,
  const RiskParams & params) const
{
  CollisionRisk result;

  for (const auto & pose : trajectory.poses) {
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);

    for (const auto & obstacle : field.obstacles) {
      if (!obstacle.valid) {
        continue;
      }

      const double dx = obstacle.x - pose.x;
      const double dy = obstacle.y - pose.y;
      const double local_x = cos_yaw * dx + sin_yaw * dy;
      const double local_y = -sin_yaw * dx + cos_yaw * dy;

      const double front = footprint.front + margins.front + obstacle.radius;
      const double back = footprint.back + margins.back + obstacle.radius;
      const double half_width = footprint.half_width + margins.side + obstacle.radius;

      const double clearance = clearanceToBox(local_x, local_y, front, back, half_width);
      if (clearance < result.min_distance) {
        result.min_distance = clearance;
      }

      const bool in_box = (local_x >= -back && local_x <= front && std::abs(local_y) <= half_width);
      const bool hard_stop = clearance <= params.hard_stop_distance_m;
      if (in_box || hard_stop) {
        result.has_collision = true;
        result.time_to_collision = pose.t;
        result.collision_x = obstacle.x;
        result.collision_y = obstacle.y;
        result.source = obstacle.source;
        result.level = in_box ? levelFromTtc(pose.t, params) : RiskLevel::BRAKING;
        if (result.level == RiskLevel::CLEAR) {
          result.level = RiskLevel::WARNING;
        }
        return result;
      }
    }
  }

  if (result.min_distance <= params.comfort_clearance_m) {
    result.level = RiskLevel::WARNING;
  }
  return result;
}

}  // namespace scene_node::avoidance
