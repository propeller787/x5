/**
 * @file emergency_brake.cpp
 * @brief 净空/物理制动能力不足时的硬急停
 */
#include "safety_node/core/emergency_brake.h"

#include <algorithm>
#include <cmath>

EmergencyBrake::EmergencyBrake(const AvoidanceParams & params)
: params_(params)
{
}

void EmergencyBrake::updateParams(const AvoidanceParams & params)
{
  params_ = params;
  clear_time_s_ = 0.0;
}

double EmergencyBrake::effectiveClearance(
  const CollisionRisk & risk,
  const EmergencyBrakeContext * context) const
{
  if (context != nullptr &&
    context->vehicle_stopped &&
    context->direction_command_active &&
    context->directional_clearance_valid)
  {
    return context->directional_clearance;
  }
  return risk.min_clearance;
}

bool EmergencyBrake::shouldEmergencyBrake(
  const CollisionRisk & risk,
  const EmergencyBrakeContext * context) const
{
  const bool controlled_escape =
    context != nullptr &&
    context->direction_command_active &&
    context->controlled_escape_active;
  if (params_.braking_suppress_distance_emergency &&
    params_.min_distance_braking_enabled &&
    context != nullptr &&
    context->forward_approach &&
    risk.braking_limited)
  {
    return risk.emergency;
  }
  if (risk.emergency) {
    return true;
  }
  if (params_.use_ttc_emergency_stop &&
    risk.has_collision && risk.min_ttc <= params_.emergency_ttc_s &&
    !controlled_escape)
  {
    return true;
  }
  const double clearance = effectiveClearance(risk, context);
  if (params_.hard_stop_clearance_enabled &&
    clearance <= params_.hard_stop_distance_m &&
    !brakingEnvelopeCanStop(risk) &&
    !controlled_escape)
  {
    return true;
  }
  return false;
}

bool EmergencyBrake::releaseConditionsMet(
  const CollisionRisk & risk,
  const EmergencyBrakeContext * context) const
{
  const bool controlled_escape =
    context != nullptr &&
    context->direction_command_active &&
    context->controlled_escape_active;
  if (risk.emergency ||
    (params_.use_ttc_emergency_stop && risk.has_collision && !controlled_escape))
  {
    return false;
  }
  if (controlled_escape) {
    return true;
  }

  const double release_ttc =
    std::max(params_.emergency_ttc_s, params_.emergency_ttc_release_s);
  const double release_clearance =
    std::max(params_.hard_stop_distance_m, params_.hard_stop_release_distance_m);

  const bool ttc_clear =
    !params_.use_ttc_emergency_stop ||
    !std::isfinite(risk.min_ttc) || risk.min_ttc >= release_ttc;
  const double clearance = effectiveClearance(risk, context);
  const bool clearance_clear =
    !params_.hard_stop_clearance_enabled ||
    !std::isfinite(clearance) ||
    clearance >= release_clearance;
  return ttc_clear && clearance_clear;
}

bool EmergencyBrake::update(
  const CollisionRisk & risk,
  double dt_s,
  const EmergencyBrakeContext * context)
{
  if (shouldEmergencyBrake(risk, context)) {
    latch();
    return true;
  }
  if (!latched_) {
    return false;
  }
  if (!releaseConditionsMet(risk, context)) {
    clear_time_s_ = 0.0;
    return true;
  }

  const double hold_s = std::max(0.0, params_.emergency_clear_hold_s);
  clear_time_s_ += std::max(0.0, dt_s);
  if (clear_time_s_ + 1e-9 < hold_s) {
    return true;
  }

  reset();
  return false;
}

void EmergencyBrake::latch()
{
  latched_ = true;
  clear_time_s_ = 0.0;
}

void EmergencyBrake::reset()
{
  latched_ = false;
  clear_time_s_ = 0.0;
}

bool EmergencyBrake::latched() const
{
  return latched_;
}

geometry_msgs::msg::Twist EmergencyBrake::applyEmergencyBrake() const
{
  geometry_msgs::msg::Twist z;
  return z;
}
