/**
 * @file safety_filter.cpp
 * @brief 最接近手柄意图的安全候选选择
 */
#include "safety_node/core/safety_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>

SafetyFilter::SafetyFilter(const AvoidanceParams & params, SweptFootprintChecker & checker)
: params_(params)
  , checker_(checker)
{
}

void SafetyFilter::updateParams(const AvoidanceParams & params)
{
  params_ = params;
  checker_.updateParams(params);
}

double SafetyFilter::commandErrorCost(
  const VelocityCandidate & cand,
  const geometry_msgs::msg::Twist & cmd) const
{
  const double dv = std::abs(cand.v - cmd.linear.x);
  const double dw = std::abs(cand.w - cmd.angular.z);
  return dv + params_.safety_filter_angular_weight * dw;
}

SafetyFilterResult SafetyFilter::selectSafeVelocity(
  const geometry_msgs::msg::Twist & cmd,
  const std::vector<TrajectoryRollout> & rollouts,
  const ObstacleField & field,
  double prev_linear_x,
  double prev_angular_z,
  double dt_s) const
{
  SafetyFilterResult out;
  out.safe_twist = cmd;

  if (rollouts.empty()) {
    return out;
  }

  struct ScoredCandidate
  {
    int index = -1;
    CollisionCheckResult check;
    double command_cost = 0.0;
    bool safe = false;
  };

  std::vector<ScoredCandidate> scored;
  scored.reserve(rollouts.size());

  CollisionRisk aggregate;
  aggregate.min_ttc = std::numeric_limits<double>::infinity();
  aggregate.min_clearance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < rollouts.size(); ++i) {
    ScoredCandidate sc;
    sc.index = static_cast<int>(i);
    sc.check = checker_.checkTrajectory(rollouts[i], field);
    sc.command_cost = commandErrorCost(rollouts[i].cmd, cmd);
    sc.safe = !sc.check.collision &&
      footprintCandidateClearanceOk(sc.check.min_distance, params_);

    if (sc.check.ttc < aggregate.min_ttc) {
      aggregate.min_ttc = sc.check.ttc;
    }
    if (sc.check.min_distance < aggregate.min_clearance) {
      aggregate.min_clearance = sc.check.min_distance;
    }
    if (sc.check.collision &&
      (aggregate.worst.ttc > sc.check.ttc ||
      (!aggregate.has_collision && sc.check.collision)))
    {
      aggregate.has_collision = true;
      aggregate.worst = sc.check;
    }
    scored.push_back(sc);
  }

  out.risk = aggregate;

  int best_index = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  for (const auto & sc : scored) {
    if (!sc.safe) {
      continue;
    }
    if (sc.command_cost < best_cost) {
      best_cost = sc.command_cost;
      best_index = sc.index;
    }
  }

  if (best_index < 0) {
    out.all_candidates_collide = true;
    double best_path_distance = -1.0;
    double best_abs_v = std::numeric_limits<double>::infinity();
    for (const auto & sc : scored) {
      double path_distance = brakingEnvelopeAvailableDistance(sc.check, params_);
      if (!std::isfinite(path_distance)) {
        path_distance = sc.check.path_length_m;
      }
      const double abs_v = std::abs(rollouts[static_cast<size_t>(sc.index)].cmd.v);
      if (path_distance > best_path_distance + 1e-6 ||
        (std::abs(path_distance - best_path_distance) <= 1e-6 && abs_v < best_abs_v))
      {
        best_path_distance = path_distance;
        best_abs_v = abs_v;
        best_index = sc.index;
      }
    }
  }

  if (best_index >= 0) {
    const auto & chosen = rollouts[static_cast<size_t>(best_index)].cmd;
    out.safe_twist.linear.x = chosen.v;
    out.safe_twist.angular.z = chosen.w;
    out.selected_candidate_index = best_index;
  }

  if (dt_s > 1e-6) {
    if (params_.max_decel_m_s2 > 0.0) {
      const double max_drop = params_.max_decel_m_s2 * dt_s;
      const double max_rise = params_.max_recover_accel_m_s2 * dt_s;
      double v = out.safe_twist.linear.x;
      const double dv = v - prev_linear_x;
      if (dv < -max_drop) {
        v = prev_linear_x - max_drop;
      } else if (dv > max_rise) {
        v = prev_linear_x + max_rise;
      }
      out.safe_twist.linear.x = v;
    }
    if (params_.max_angular_decel_rad_s2 > 0.0) {
      const double max_drop_w = params_.max_angular_decel_rad_s2 * dt_s;
      const double max_rise_w = params_.max_angular_recover_rad_s2 * dt_s;
      double w = out.safe_twist.angular.z;
      const double dw = w - prev_angular_z;
      if (dw < -max_drop_w) {
        w = prev_angular_z - max_drop_w;
      } else if (dw > max_rise_w) {
        w = prev_angular_z + max_rise_w;
      }
      out.safe_twist.angular.z = w;
    }
  }

  out.safe_twist.linear.x = std::clamp(
    out.safe_twist.linear.x,
    -params_.absolute_max_linear_speed,
    params_.absolute_max_linear_speed);
  out.safe_twist.angular.z = std::clamp(
    out.safe_twist.angular.z,
    -params_.absolute_max_angular_speed,
    params_.absolute_max_angular_speed);

  return out;
}
