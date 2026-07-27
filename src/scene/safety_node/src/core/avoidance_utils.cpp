/**
 * @file avoidance_utils.cpp
 * @brief 碰撞方向派生、侧向距离统计、干预等级填充
 */
#include "safety_node/core/avoidance_types.h"

#include <algorithm>
#include <cmath>
#include <limits>

CollisionDirection deriveCollisionDirection(double x, double y)
{
  const double ax = std::abs(x);
  const double ay = std::abs(y);
  if (ax < 1e-3 && ay < 1e-3) {
    return CollisionDirection::FORWARD;
  }
  if (ax >= ay) {
    return (x >= 0.0) ? CollisionDirection::FORWARD : CollisionDirection::BACKWARD;
  }
  return (y >= 0.0) ? CollisionDirection::LEFT : CollisionDirection::RIGHT;
}

void estimateLateralDistances(const ObstacleField & field, double & left_dist, double & right_dist)
{
  left_dist = std::numeric_limits<double>::infinity();
  right_dist = std::numeric_limits<double>::infinity();
  for (const auto & o : field.obstacles) {
    if (!o.valid) {
      continue;
    }
    const double edge_y = (o.y >= 0.0) ? (o.y - o.radius) : (o.y + o.radius);
    if (o.y > 0.0) {
      left_dist = std::min(left_dist, edge_y);
    } else if (o.y < 0.0) {
      right_dist = std::min(right_dist, -edge_y);
    }
  }
  if (!std::isfinite(left_dist)) {
    left_dist = 1e6;
  }
  if (!std::isfinite(right_dist)) {
    right_dist = 1e6;
  }
}

void fillInterventionLevels(CollisionRisk & risk, const AvoidanceParams & params)
{
  risk.levels = {};
  if (!risk.has_collision) {
    return;
  }

  risk.primary_direction = risk.worst.collision ?
    deriveCollisionDirection(risk.worst.collision_x, risk.worst.collision_y) :
    CollisionDirection::NONE;
  risk.levels.primary = risk.primary_direction;

  const bool distance_controlled = brakingEnvelopeCanStop(risk);
  const bool emergency = risk.emergency ||
    (params.use_ttc_emergency_stop && risk.min_ttc <= params.emergency_ttc_s) ||
    (params.hard_stop_clearance_enabled &&
    risk.min_clearance <= params.hard_stop_distance_m && !distance_controlled);
  const bool decel = !emergency &&
    (risk.braking_limited ||
    (params.use_ttc_emergency_stop && risk.min_ttc <= params.limiting_ttc_s));

  auto set_dir = [&](CollisionDirection dir) {
      switch (dir) {
        case CollisionDirection::FORWARD:
          if (emergency) {
            risk.levels.forward_brake = true;
          } else if (decel) {
            risk.levels.forward_decel = true;
          }
          break;
        case CollisionDirection::BACKWARD:
          if (emergency) {
            risk.levels.backward_brake = true;
          } else if (decel) {
            risk.levels.backward_decel = true;
          }
          break;
        case CollisionDirection::LEFT:
          if (emergency) {
            risk.levels.left_brake = true;
          } else if (decel) {
            risk.levels.left_decel = true;
          }
          break;
        case CollisionDirection::RIGHT:
          if (emergency) {
            risk.levels.right_brake = true;
          } else if (decel) {
            risk.levels.right_decel = true;
          }
          break;
        default:
          break;
      }
    };

  set_dir(risk.primary_direction);
}
