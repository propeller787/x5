/**
 * @file avoidance_controller.cpp
 * @brief 混合避障控制器实现
 *
 * evaluate() 流程概览：
 *   PASS_THROUGH → 仅 clamp 绝对限速
 *   否则：
 *     采样候选轨迹 → Footprint 碰撞评分 → 排序
 *     → Map lethal veto 选首个可行候选
 *     → 加速度限制 → 急停兜底
 */
#include "safety_node/avoidance_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

/** 候选速度与手柄期望的偏差代价（越小越跟手） */
double commandErrorCost(
  const VelocityCandidate & cand,
  const geometry_msgs::msg::Twist & cmd,
  double angular_weight)
{
  const double dv = std::abs(cand.v - cmd.linear.x);
  const double dw = std::abs(cand.w - cmd.angular.z);
  return dv + angular_weight * dw;
}

double jerkLimitedStoppingDistance(
  double speed,
  double decel,
  double jerk,
  double reaction_delay)
{
  const double v = std::max(0.0, speed);
  const double a = std::max(1e-3, decel);
  const double j = std::max(1e-3, jerk);
  double distance = v * std::max(0.0, reaction_delay);
  const double ramp_time = a / j;
  const double ramp_speed_drop = 0.5 * j * ramp_time * ramp_time;

  if (v <= ramp_speed_drop) {
    const double stop_time = std::sqrt(2.0 * v / j);
    return distance + (2.0 / 3.0) * v * stop_time;
  }

  const double speed_after_ramp = v - ramp_speed_drop;
  distance +=
    v * ramp_time -
    j * ramp_time * ramp_time * ramp_time / 6.0;
  distance += speed_after_ramp * speed_after_ramp / (2.0 * a);
  return distance;
}

EmergencyStopReason classifyEmergencyReason(
  const CollisionRisk & risk,
  const AvoidanceParams & params)
{
  if (params.hard_stop_clearance_enabled &&
    risk.min_clearance <= params.hard_stop_distance_m)
  {
    return EmergencyStopReason::HARD_STOP_DISTANCE;
  }
  if (std::isfinite(risk.braking_available_distance_m) &&
    risk.braking_stopping_distance_m > risk.braking_available_distance_m)
  {
    return EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT;
  }
  if (params.use_ttc_emergency_stop &&
    risk.has_collision && risk.min_ttc <= params.emergency_ttc_s)
  {
    return EmergencyStopReason::EMERGENCY_TTC;
  }
  if (risk.emergency) {
    return EmergencyStopReason::RISK_EMERGENCY;
  }
  return EmergencyStopReason::UNKNOWN;
}

PredictedPose poseAtHorizon(const TrajectoryRollout & rollout, double horizon_s)
{
  const double target_t = std::max(0.0, horizon_s);
  for (const auto & pose : rollout.poses) {
    if (pose.t + 1e-9 >= target_t) {
      return pose;
    }
  }
  if (!rollout.poses.empty()) {
    return rollout.poses.back();
  }
  return PredictedPose{};
}

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

AvoidanceController::AvoidanceController(
  const AvoidanceParams & avoidance,
  const MapConstraintConfig & map_config)
: base_params_(avoidance)
  , params_(avoidance)
  , map_config_(map_config)
  , footprint_(params_)
  , checker_(params_, footprint_)
  , rollout_(params_)
  , safety_filter_(params_, checker_)
  , emergency_brake_(params_)
  , map_checker_(params_, map_config_)
{
}

void AvoidanceController::updateParams(
  const AvoidanceParams & avoidance,
  const MapConstraintConfig & map_config)
{
  base_params_ = avoidance;
  params_ = avoidance;
  map_config_ = map_config;
  footprint_.updateParams(params_);
  checker_.updateParams(params_);
  rollout_.updateParams(params_);
  safety_filter_.updateParams(params_);
  emergency_brake_.updateParams(params_);
  map_checker_.updateFootprintParams(params_);
  map_checker_.updateConfig(map_config_);
  emergency_brake_.reset();
  resetCandidateSpeedUpHold();
  braking_envelope_active_time_s_ = 0.0;
  has_smoothed_braking_speed_limit_ = false;
  resetRecoveryState();
  resetTiltEmergencyState();
  active_avoidance_overrides_enabled_ = false;
}

void AvoidanceController::resetCandidateSpeedUpHold() const
{
  has_held_candidate_linear_ = false;
  held_candidate_linear_ = 0.0;
  candidate_speed_up_clear_time_s_ = 0.0;
}

void AvoidanceController::configureForMode(const ModeProfile & mode_profile) const
{
  if (active_avoidance_overrides_enabled_ ==
    mode_profile.avoidance_overrides_enabled)
  {
    return;
  }

  params_ = applyModeProfile(base_params_, mode_profile);
  footprint_.updateParams(params_);
  checker_.updateParams(params_);
  rollout_.updateParams(params_);
  safety_filter_.updateParams(params_);
  emergency_brake_.updateParams(params_);
  map_checker_.updateFootprintParams(params_);
  active_avoidance_overrides_enabled_ =
    mode_profile.avoidance_overrides_enabled;
  resetCandidateSpeedUpHold();
  braking_envelope_active_time_s_ = 0.0;
  has_smoothed_braking_speed_limit_ = false;
  resetRecoveryState();
  resetTiltEmergencyState();
}

double AvoidanceController::applyCandidateSpeedUpHold(double selected_linear, double dt_s) const
{
  const double hold_s = std::max(0.0, params_.candidate_speed_up_hold_s);
  const double selected_abs = std::abs(selected_linear);
  const double held_abs = std::abs(held_candidate_linear_);
  constexpr double eps = 1e-4;

  if (!has_held_candidate_linear_ || hold_s <= 0.0) {
    has_held_candidate_linear_ = true;
    held_candidate_linear_ = selected_linear;
    candidate_speed_up_clear_time_s_ = 0.0;
    return selected_linear;
  }

  const bool direction_changed = selected_linear * held_candidate_linear_ < -eps;
  const bool starting_from_stop = held_abs < eps;
  const bool not_speeding_up = selected_abs <= held_abs + eps;

  if (direction_changed || starting_from_stop || not_speeding_up) {
    held_candidate_linear_ = selected_linear;
    candidate_speed_up_clear_time_s_ = 0.0;
    return selected_linear;
  }

  candidate_speed_up_clear_time_s_ += std::max(0.0, dt_s);
  if (candidate_speed_up_clear_time_s_ + 1e-9 < hold_s) {
    return std::copysign(held_abs, selected_linear);
  }

  held_candidate_linear_ = selected_linear;
  candidate_speed_up_clear_time_s_ = 0.0;
  return selected_linear;
}

TrajectoryRollout AvoidanceController::buildDistanceProbe(
  const geometry_msgs::msg::Twist & cmd) const
{
  TrajectoryRollout probe;
  probe.cmd.v = cmd.linear.x;
  probe.cmd.w = cmd.angular.z;

  const double speed = std::abs(cmd.linear.x);
  if (speed < 1e-4) {
    probe.poses.push_back(PredictedPose{});
    return probe;
  }

  const double max_distance = std::max(0.0, params_.braking_lookahead_m);
  const double step = std::clamp(
    params_.braking_path_step_m, 0.005, std::max(0.005, max_distance));
  const double direction = std::copysign(1.0, cmd.linear.x);
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double distance = 0.0;
  probe.poses.push_back({x, y, theta, 0.0});

  while (distance + 1e-9 < max_distance) {
    const double ds = std::min(step, max_distance - distance);
    const double dt = ds / speed;
    x += direction * std::cos(theta) * ds;
    y += direction * std::sin(theta) * ds;
    theta += cmd.angular.z * dt;
    distance += ds;
    probe.poses.push_back({x, y, theta, distance / speed});
  }
  return probe;
}

TrajectoryRollout AvoidanceController::buildTiltProbe(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear) const
{
  TrajectoryRollout probe;
  probe.cmd.v = cmd.linear.x;
  probe.cmd.w = cmd.angular.z;

  const double command_speed = std::abs(cmd.linear.x);
  const double speed = params_.tilt_ttc.use_command_velocity_for_ttc ?
    std::max(command_speed, std::abs(fb_linear)) :
    std::abs(fb_linear);
  if (speed < 1e-4 || std::abs(cmd.linear.x) < 1e-4) {
    probe.poses.push_back(PredictedPose{});
    return probe;
  }

  const double direction = std::copysign(1.0, cmd.linear.x);
  const double max_distance = std::max(
    0.0,
    std::min(
      params_.tilt_ttc.max_probe_distance_m,
      speed * std::max(0.0, params_.tilt_ttc.horizon_s)));
  const double step = std::clamp(
    params_.braking_path_step_m, 0.005, std::max(0.005, max_distance));

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double distance = 0.0;
  probe.poses.push_back({x, y, theta, 0.0});

  while (distance + 1e-9 < max_distance) {
    const double ds = std::min(step, max_distance - distance);
    const double dt = ds / speed;
    x += direction * std::cos(theta) * ds;
    y += direction * std::sin(theta) * ds;
    theta += cmd.angular.z * dt;
    distance += ds;
    probe.poses.push_back({x, y, theta, distance / speed});
  }
  return probe;
}

AvoidanceController::TiltIntervention AvoidanceController::evaluateTiltIntervention(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear,
  const ObstacleField * tilt_field,
  double dt_s) const
{
  TiltIntervention out;
  const auto reset_tilt_latch = [this]() {
      tilt_emergency_latched_ = false;
      tilt_emergency_clear_time_s_ = 0.0;
      tilt_emergency_reason_ = EmergencyStopReason::NONE;
    };
  const auto apply_latched_hold = [&](bool release_clear) {
      if (!tilt_emergency_latched_) {
        return false;
      }
      if (release_clear) {
        tilt_emergency_clear_time_s_ += std::max(0.0, std::isfinite(dt_s) ? dt_s : 0.0);
      } else {
        tilt_emergency_clear_time_s_ = 0.0;
      }
      if (tilt_emergency_clear_time_s_ + 1e-9 <
        std::max(0.0, params_.tilt_ttc.emergency_clear_hold_s))
      {
        out.active = true;
        out.limiting = true;
        out.emergency = true;
        out.speed_limit_m_s = 0.0;
        out.reason = tilt_emergency_reason_ == EmergencyStopReason::NONE ?
          EmergencyStopReason::TILT_NEAR_DISTANCE :
          tilt_emergency_reason_;
        return true;
      }
      reset_tilt_latch();
      return false;
    };

  if (!params_.tilt_ttc.enabled) {
    reset_tilt_latch();
    return out;
  }

  const double min_command = std::max(0.0, params_.recovery_min_command);
  if (cmd.linear.x <= min_command) {
    reset_tilt_latch();
    return out;
  }

  if (tilt_field == nullptr || tilt_field->obstacles.empty()) {
    apply_latched_hold(true);
    return out;
  }

  const TrajectoryRollout probe = buildTiltProbe(cmd, fb_linear);
  const CollisionCheckResult check = checker_.checkTrajectory(probe, *tilt_field);
  out.source = check.source;
  out.min_ttc_s = check.ttc;

  double nearest_forward_edge_m = std::numeric_limits<double>::infinity();
  std::string nearest_source;
  for (const auto & obs : tilt_field->obstacles) {
    if (!obs.valid || obs.x <= 0.0) {
      continue;
    }
    const double edge_distance = std::max(0.0, obs.x - std::max(0.0, obs.radius));
    if (edge_distance < nearest_forward_edge_m) {
      nearest_forward_edge_m = edge_distance;
      nearest_source = obs.source;
    }
  }

  if (std::isfinite(check.collision_path_distance_m)) {
    out.min_distance_m = check.collision_path_distance_m;
  } else if (std::isfinite(check.hard_stop_path_distance_m)) {
    out.min_distance_m = check.hard_stop_path_distance_m;
  } else {
    out.min_distance_m = nearest_forward_edge_m;
  }
  if (out.source.empty()) {
    out.source = nearest_source;
  }

  const double ttc_speed = std::max(std::abs(cmd.linear.x), std::abs(fb_linear));
  if (!std::isfinite(out.min_ttc_s) &&
    std::isfinite(out.min_distance_m) &&
    ttc_speed >= std::max(0.0, params_.tilt_ttc.min_speed_for_ttc_m_s))
  {
    out.min_ttc_s = out.min_distance_m / std::max(1e-3, ttc_speed);
  }

  const bool distance_emergency =
    std::isfinite(out.min_distance_m) &&
    out.min_distance_m <= params_.tilt_ttc.hard_stop_distance_m;
  const bool ttc_enabled =
    ttc_speed >= std::max(0.0, params_.tilt_ttc.min_speed_for_ttc_m_s);
  const bool ttc_emergency =
    ttc_enabled &&
    std::isfinite(out.min_ttc_s) &&
    out.min_ttc_s <= params_.tilt_ttc.emergency_ttc_s;

  out.emergency = check.collision || distance_emergency || ttc_emergency;
  if (out.emergency) {
    out.active = true;
    out.limiting = true;
    out.speed_limit_m_s = 0.0;
    out.reason = ttc_emergency ? EmergencyStopReason::TILT_TTC :
      EmergencyStopReason::TILT_NEAR_DISTANCE;
    tilt_emergency_latched_ = true;
    tilt_emergency_clear_time_s_ = 0.0;
    tilt_emergency_reason_ = out.reason;
    return out;
  }

  const bool distance_release_clear =
    !std::isfinite(out.min_distance_m) ||
    out.min_distance_m >= std::max(
    params_.tilt_ttc.hard_stop_distance_m,
    params_.tilt_ttc.release_distance_m);
  const bool ttc_release_clear =
    !ttc_enabled ||
    !std::isfinite(out.min_ttc_s) ||
    out.min_ttc_s >= std::max(
    params_.tilt_ttc.emergency_ttc_s,
    params_.tilt_ttc.emergency_ttc_release_s);
  if (apply_latched_hold(distance_release_clear && ttc_release_clear && !check.collision)) {
    return out;
  }

  const bool distance_limited =
    std::isfinite(out.min_distance_m) &&
    out.min_distance_m <= params_.tilt_ttc.limiting_distance_m;
  const bool ttc_limited =
    ttc_enabled &&
    std::isfinite(out.min_ttc_s) &&
    out.min_ttc_s <= params_.tilt_ttc.limiting_ttc_s;
  if (!distance_limited && !ttc_limited) {
    return out;
  }

  const double comfort_decel = std::max(1e-3, params_.braking_comfort_decel_m_s2);
  const double jerk = std::max(1e-3, params_.braking_max_jerk_m_s3);
  const double reaction_delay = std::max(0.0, params_.braking_reaction_delay_s);
  const double usable_distance = std::max(
    0.0,
    out.min_distance_m -
    params_.tilt_ttc.hard_stop_distance_m -
    std::max(0.0, params_.braking_stop_buffer_m));
  double lower_speed = 0.0;
  double upper_speed = std::max(std::abs(cmd.linear.x), params_.absolute_max_linear_speed);
  for (int i = 0; i < 48; ++i) {
    const double candidate_speed = 0.5 * (lower_speed + upper_speed);
    if (jerkLimitedStoppingDistance(
        candidate_speed, comfort_decel, jerk, reaction_delay) <= usable_distance)
    {
      lower_speed = candidate_speed;
    } else {
      upper_speed = candidate_speed;
    }
  }

  double ttc_cap = std::numeric_limits<double>::infinity();
  if (ttc_limited) {
    ttc_cap = std::max(0.0, (out.min_ttc_s - reaction_delay) * comfort_decel);
  }

  out.active = true;
  out.limiting = true;
  out.speed_limit_m_s = std::min({lower_speed, ttc_cap, std::abs(cmd.linear.x)});
  return out;
}

void AvoidanceController::mergeTiltIntervention(
  AvoidanceResult & result,
  const TiltIntervention & tilt) const
{
  result.hybrid_log.tilt_min_distance_m = tilt.min_distance_m;
  result.hybrid_log.tilt_min_ttc = tilt.min_ttc_s;
  result.hybrid_log.tilt_speed_limit_m_s = tilt.speed_limit_m_s;
  result.hybrid_log.tilt_collision_source = tilt.source;
  if (!tilt.active) {
    result.hybrid_log.tilt_intervention = "none";
    return;
  }

  result.hybrid_log.tilt_intervention = tilt.emergency ? "emergency" : "limiting";
  if (tilt.emergency) {
    result.risk.has_collision = true;
    result.risk.emergency = true;
    result.emergency_reason = tilt.reason;
    return;
  }

  if (!std::isfinite(tilt.speed_limit_m_s)) {
    return;
  }
  const double current_linear = result.safe_twist.linear.x;
  if (current_linear <= 0.0) {
    return;
  }
  const double capped_linear = std::min(current_linear, tilt.speed_limit_m_s);
  const double scale = std::abs(current_linear) > 1e-6 ?
    std::clamp(capped_linear / current_linear, 0.0, 1.0) :
    0.0;
  result.safe_twist.linear.x = capped_linear;
  result.safe_twist.angular.z *= scale;
}

AvoidanceController::BrakingEnvelope AvoidanceController::evaluateBrakingEnvelope(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear,
  double prev_output_linear_x,
  const ObstacleField & field,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile,
  double control_dt_s) const
{
  BrakingEnvelope envelope;
  const double requested_speed = std::abs(cmd.linear.x);
  if (!params_.braking_envelope_enabled || requested_speed < 1e-4 ||
    params_.braking_lookahead_m <= 0.0)
  {
    return envelope;
  }

  const TrajectoryRollout probe = buildDistanceProbe(cmd);
  envelope.footprint_check = checker_.checkTrajectory(probe, field);
  envelope.available_distance_m =
    brakingEnvelopeAvailableDistance(envelope.footprint_check, params_);
  const double min_motion_command = std::max(0.0, params_.recovery_min_command);
  const double max_escape_linear = std::max(
    min_motion_command, params_.recovery_max_linear_speed);
  const bool reverse_escape_from_front_hard_stop =
    fb_linear <= 0.02 &&
    prev_output_linear_x <= 0.02 &&
    std::abs(fb_linear) <= max_escape_linear + 1e-9 &&
    std::abs(prev_output_linear_x) <= max_escape_linear + 1e-9 &&
    cmd.linear.x < -min_motion_command &&
    std::isfinite(envelope.footprint_check.hard_stop_path_distance_m) &&
    envelope.footprint_check.hard_stop_path_distance_m <= 1e-6 &&
    envelope.footprint_check.hard_stop_x >= 0.0;
  if (reverse_escape_from_front_hard_stop) {
    envelope.controlled_escape_only = true;
  }

  const bool map_enabled = mode_profile.map_constraint_enabled &&
    map_config_.enabled &&
    map_config_.veto_on_lethal &&
    local_map != nullptr &&
    !local_map->probability.empty();
  if (map_enabled) {
    MapConstraintResult map_result;
    if (map_checker_.trajectoryLethal(
        *local_map,
        probe.poses,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        map_result))
    {
      const double max_linear = std::max(
        params_.recovery_min_command, params_.recovery_max_linear_speed);
      const double scale = std::min(1.0, max_linear / requested_speed);
      VelocityCandidate escape_candidate;
      escape_candidate.v = cmd.linear.x * scale;
      escape_candidate.w = 0.0;
      const TrajectoryRollout escape_rollout =
        rollout_.rolloutCandidate(escape_candidate, fb_linear, 0.0);
      MapConstraintResult escape_result;
      const double grid_clearance_time_s =
        std::max(0.0, local_map->resolution_m) / std::max(1e-3, max_linear);
      const bool controlled_escape =
        std::abs(fb_linear) <= max_linear + 1e-9 &&
        map_checker_.trajectoryEscapesInitialLethal(
        *local_map,
        escape_rollout.poses,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        std::max(
          0.02,
          params_.recovery_lookahead_s + grid_clearance_time_s),
        escape_result);
      if (controlled_escape) {
        envelope.controlled_escape_only = true;
        envelope.map_escape_only = true;
      } else {
        envelope.available_distance_m = std::min(
          envelope.available_distance_m,
          map_result.first_lethal_t * requested_speed);
      }
    }
  }

  if (!std::isfinite(envelope.available_distance_m)) {
    return envelope;
  }

  const double usable_distance = std::max(
    0.0,
    envelope.available_distance_m -
    std::max(0.0, params_.braking_stop_buffer_m) -
    0.5 * std::max(0.0, params_.braking_path_step_m));
  const double jerk = std::max(1e-3, params_.braking_max_jerk_m_s3);
  const double reaction_delay = std::max(0.0, params_.braking_reaction_delay_s);
  const double envelope_delay =
    reaction_delay +
    4.0 * std::clamp(control_dt_s, 0.0, 0.10);
  const double comfort_decel = std::max(1e-3, params_.braking_comfort_decel_m_s2);
  double lower_speed = 0.0;
  double upper_speed = std::max(requested_speed, params_.absolute_max_linear_speed);
  for (int i = 0; i < 48; ++i) {
    const double candidate_speed = 0.5 * (lower_speed + upper_speed);
    if (jerkLimitedStoppingDistance(
        candidate_speed, comfort_decel, jerk, envelope_delay) <= usable_distance)
    {
      lower_speed = candidate_speed;
    } else {
      upper_speed = candidate_speed;
    }
  }
  envelope.speed_limit_m_s = lower_speed;
  envelope.active = envelope.speed_limit_m_s + 1e-6 < requested_speed;

  const double current_speed = std::max(std::abs(fb_linear), std::abs(prev_output_linear_x));
  const double max_decel = std::max(1e-3, params_.max_decel_m_s2);
  const double valid_control_dt = std::clamp(control_dt_s, 1e-3, 0.10);
  const bool braking_established =
    braking_envelope_active_time_s_ >= 4.0 * valid_control_dt &&
    std::abs(fb_linear) <= std::abs(prev_output_linear_x) + 0.005;
  envelope.stopping_distance_m = current_speed * reaction_delay;
  if (braking_established) {
    envelope.stopping_distance_m +=
      current_speed * current_speed / (2.0 * max_decel);
  } else {
    envelope.stopping_distance_m =
      jerkLimitedStoppingDistance(current_speed, max_decel, jerk, reaction_delay);
  }
  envelope.emergency =
    !envelope.controlled_escape_only &&
    current_speed > 1e-3 &&
    envelope.stopping_distance_m > envelope.available_distance_m + 1e-6;
  return envelope;
}

AvoidanceController::FootprintIntrusion AvoidanceController::analyzeFootprintIntrusion(
  const ObstacleField & field) const
{
  FootprintIntrusion intrusion;
  intrusion.external_field.stamp = field.stamp;
  if (!params_.footprint_intrusion_check_enabled) {
    intrusion.external_field = field;
    return intrusion;
  }

  // 侵入判定仅用物理轮廓；安全余量仍由 inflatedPolygon 用于轨迹 Footprint 扫掠。
  const FootprintPolygon polygon = footprint_.physicalPolygon();

  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    const double clearance =
      VehicleFootprint::clearanceToPolygon(obs.x, obs.y, obs.radius, polygon);
    const bool inside =
      VehicleFootprint::pointInPolygon(obs.x, obs.y, obs.radius, polygon);
    if (!inside) {
      intrusion.external_field.obstacles.push_back(obs);
      continue;
    }

    intrusion.active = true;
    if (clearance < intrusion.min_clearance) {
      intrusion.min_clearance = clearance;
      intrusion.worst.collision = true;
      intrusion.worst.ttc = 0.0;
      intrusion.worst.min_distance = clearance;
      intrusion.worst.collision_x = obs.x;
      intrusion.worst.collision_y = obs.y;
      intrusion.worst.source = obs.source;
      intrusion.worst.pose_index = 0;
    }
  }

  return intrusion;
}

void AvoidanceController::enrichProximityBlock(
  FootprintIntrusion & intrusion,
  const ObstacleField & field,
  double fb_linear,
  double prev_output_linear_x) const
{
  const bool feedback_stopped =
    std::abs(fb_linear) < 0.02 && std::abs(prev_output_linear_x) < 0.02;
  if (intrusion.active || !feedback_stopped || !params_.recovery_mode_enabled) {
    return;
  }

  if (!params_.hard_stop_clearance_enabled) {
    return;
  }

  const FootprintPolygon inflated = footprint_.inflatedPolygon();
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    min_clearance = std::min(
      min_clearance,
      VehicleFootprint::clearanceToPolygon(obs.x, obs.y, obs.radius, inflated));
  }

  if (std::isfinite(min_clearance) &&
    min_clearance <= params_.hard_stop_distance_m)
  {
    intrusion.proximity_blocked = true;
    intrusion.min_clearance = min_clearance;
    intrusion.external_field = field;
    intrusion.external_field.stamp = field.stamp;
  }
}

double AvoidanceController::smoothBrakingSpeedLimit(
  double raw_limit_m_s,
  double dt_s) const
{
  if (!std::isfinite(raw_limit_m_s)) {
    has_smoothed_braking_speed_limit_ = false;
    return raw_limit_m_s;
  }

  const double valid_dt_s = std::max(1e-3, std::isfinite(dt_s) ? dt_s : 0.0);
  const double max_drop = std::max(1e-3, params_.max_decel_m_s2) * valid_dt_s;
  const double max_rise = std::max(1e-3, params_.max_recover_accel_m_s2) * valid_dt_s;
  if (!has_smoothed_braking_speed_limit_) {
    has_smoothed_braking_speed_limit_ = true;
    smoothed_braking_speed_limit_m_s_ = raw_limit_m_s;
    return raw_limit_m_s;
  }

  double next = smoothed_braking_speed_limit_m_s_;
  if (raw_limit_m_s + 1e-9 < next) {
    next = std::max(raw_limit_m_s, next - max_drop);
  } else if (raw_limit_m_s > next + 1e-9) {
    next = std::min(raw_limit_m_s, next + max_rise);
  } else {
    next = raw_limit_m_s;
  }
  smoothed_braking_speed_limit_m_s_ = next;
  return next;
}

EmergencyBrakeContext AvoidanceController::buildEmergencyBrakeContext(
  const geometry_msgs::msg::Twist & cmd,
  const geometry_msgs::msg::Twist & output_cmd,
  double fb_linear,
  double fb_angular,
  double prev_output_linear_x,
  const ObstacleField & field) const
{
  EmergencyBrakeContext context;
  const double min_command = std::max(0.0, params_.recovery_min_command);
  context.vehicle_stopped =
    std::abs(fb_linear) < 0.02 && std::abs(prev_output_linear_x) < 0.02;
  const geometry_msgs::msg::Twist & direction_cmd =
    (std::abs(output_cmd.linear.x) >= min_command ||
    std::abs(output_cmd.angular.z) >= min_command) ?
    output_cmd :
    cmd;
  context.direction_command_active =
    std::abs(direction_cmd.linear.x) >= min_command ||
    std::abs(direction_cmd.angular.z) >= min_command;
  if (!context.vehicle_stopped || !context.direction_command_active) {
    return context;
  }

  VelocityCandidate dir_cmd;
  dir_cmd.v = direction_cmd.linear.x;
  dir_cmd.w = direction_cmd.angular.z;
  const TrajectoryRollout dir_rollout =
    rollout_.rolloutCandidate(dir_cmd, fb_linear, fb_angular);
  const PredictedPose future_pose = poseAtHorizon(
    dir_rollout,
    std::min(params_.recovery_lookahead_s, params_.rollout_horizon_s));
  const FootprintPolygon clearance_polygon = footprint_.inflatedPolygon();
  double future_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    double lx = 0.0;
    double ly = 0.0;
    transformObstacleToPoseFrame(obs.x, obs.y, future_pose, lx, ly);
    future_clearance = std::min(
      future_clearance,
      VehicleFootprint::clearanceToPolygon(lx, ly, obs.radius, clearance_polygon));
  }
  context.directional_clearance = future_clearance;
  context.directional_clearance_valid = std::isfinite(future_clearance);
  return context;
}

bool AvoidanceController::isDirectionalEscapeViable(
  const EmergencyBrakeContext & context) const
{
  if (!params_.hard_stop_clearance_enabled) {
    return false;
  }
  return context.vehicle_stopped &&
         context.direction_command_active &&
         context.directional_clearance_valid &&
         context.directional_clearance >= params_.hard_stop_release_distance_m;
}

bool AvoidanceController::isControlledMapEscape(
  const TrajectoryRollout & rollout,
  double fb_linear,
  double prev_output_linear_x,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile,
  double max_escape_time_s,
  MapConstraintResult & out) const
{
  if (local_map == nullptr || local_map->probability.empty()) {
    return false;
  }

  const double min_command = std::max(0.0, params_.recovery_min_command);
  const double max_linear = std::max(min_command, params_.recovery_max_linear_speed);
  const double max_angular = std::max(min_command, params_.recovery_max_angular_speed);
  if (std::abs(rollout.cmd.v) < min_command ||
    std::abs(rollout.cmd.v) > max_linear + 1e-9 ||
    std::abs(rollout.cmd.w) > max_angular + 1e-9 ||
    std::abs(fb_linear) > max_linear + 1e-9 ||
    std::abs(prev_output_linear_x) > max_linear + 1e-9)
  {
    return false;
  }

  const double grid_clearance_time_s =
    std::max(0.0, local_map->resolution_m) / std::max(1e-3, max_linear);
  return map_checker_.trajectoryEscapesInitialLethal(
    *local_map,
    rollout.poses,
    mode_profile.lethal_threshold,
    map_config_.check_footprint_corners,
    std::max(0.02, max_escape_time_s + grid_clearance_time_s),
    out);
}

AvoidanceController::RecoveryCandidate AvoidanceController::evaluateRecoveryCandidate(
  const VelocityCandidate & candidate,
  const geometry_msgs::msg::Twist & requested_cmd,
  double fb_linear,
  double fb_angular,
  const ObstacleField & field,
  const FootprintIntrusion & intrusion,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile) const
{
  RecoveryCandidate evaluated;
  evaluated.command = candidate;
  evaluated.command_cost =
    commandErrorCost(candidate, requested_cmd, params_.safety_filter_angular_weight);

  const bool proximity_only = intrusion.proximity_blocked && !intrusion.active;
  const FootprintPolygon clearance_polygon = proximity_only ?
    footprint_.inflatedPolygon() :
    footprint_.physicalPolygon();
  const ObstacleField & clearance_field = proximity_only ? field : field;

  const TrajectoryRollout recovery_rollout =
    rollout_.rolloutCandidate(candidate, fb_linear, fb_angular);
  const PredictedPose future_pose =
    poseAtHorizon(recovery_rollout, params_.recovery_lookahead_s);
  const double allowed_transient_loss =
    std::max(0.0, params_.recovery_max_transient_clearance_loss_m);

  double previous_clearance = intrusion.min_clearance;
  for (const auto & pose : recovery_rollout.poses) {
    if (pose.t > params_.recovery_lookahead_s + 1e-9) {
      break;
    }
    double pose_min_clearance = std::numeric_limits<double>::infinity();
    for (const auto & obs : clearance_field.obstacles) {
      if (!obs.valid) {
        continue;
      }
      if (!proximity_only &&
        !VehicleFootprint::pointInPolygon(obs.x, obs.y, obs.radius, clearance_polygon))
      {
        continue;
      }
      double lx = 0.0;
      double ly = 0.0;
      transformObstacleToPoseFrame(obs.x, obs.y, pose, lx, ly);
      pose_min_clearance = std::min(
        pose_min_clearance,
        VehicleFootprint::clearanceToPolygon(lx, ly, obs.radius, clearance_polygon));
    }
    if (!std::isfinite(pose_min_clearance) ||
      pose_min_clearance + allowed_transient_loss < previous_clearance)
    {
      return evaluated;
    }
    previous_clearance = pose_min_clearance;
  }

  double future_min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : clearance_field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    if (!proximity_only &&
      !VehicleFootprint::pointInPolygon(obs.x, obs.y, obs.radius, clearance_polygon))
    {
      continue;
    }
    double lx = 0.0;
    double ly = 0.0;
    transformObstacleToPoseFrame(obs.x, obs.y, future_pose, lx, ly);
    future_min_clearance = std::min(
      future_min_clearance,
      VehicleFootprint::clearanceToPolygon(lx, ly, obs.radius, clearance_polygon));
  }

  const double required_improvement =
    std::max(0.0, params_.recovery_min_clearance_improvement_m);
  if (!std::isfinite(future_min_clearance) ||
    future_min_clearance <= intrusion.min_clearance + required_improvement)
  {
    return evaluated;
  }

  const ObstacleField & trajectory_field =
    proximity_only ? field : intrusion.external_field;
  const CollisionCheckResult external_check =
    checker_.checkTrajectory(recovery_rollout, trajectory_field);
  if (external_check.collision) {
    return evaluated;
  }
  if (!proximity_only &&
    params_.hard_stop_clearance_enabled &&
    external_check.min_distance <= params_.hard_stop_distance_m)
  {
    return evaluated;
  }
  if (proximity_only &&
    external_check.min_distance + allowed_transient_loss <
    intrusion.min_clearance)
  {
    return evaluated;
  }

  const bool map_enabled = mode_profile.map_constraint_enabled &&
    map_config_.enabled &&
    map_config_.veto_on_lethal &&
    local_map != nullptr &&
    !local_map->probability.empty();
  if (map_enabled) {
    MapConstraintResult map_result;
    if (map_checker_.trajectoryLethal(
        *local_map,
        recovery_rollout.poses,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        map_result))
    {
      MapConstraintResult escape_result;
      const double max_linear = std::max(
        params_.recovery_min_command, params_.recovery_max_linear_speed);
      const double grid_clearance_time_s =
        std::max(0.0, local_map->resolution_m) / std::max(1e-3, max_linear);
      if (!map_checker_.trajectoryEscapesInitialLethal(
          *local_map,
          recovery_rollout.poses,
          mode_profile.lethal_threshold,
          map_config_.check_footprint_corners,
          std::max(
            0.02,
            params_.recovery_lookahead_s + grid_clearance_time_s),
          escape_result))
      {
        return evaluated;
      }
    }
  }

  evaluated.valid = true;
  evaluated.future_clearance = future_min_clearance;
  evaluated.clearance_gain = future_min_clearance - intrusion.min_clearance;
  return evaluated;
}

bool AvoidanceController::trySelectRecoveryCommand(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear,
  double fb_angular,
  const ObstacleField & field,
  const FootprintIntrusion & intrusion,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile,
  double dt_s,
  geometry_msgs::msg::Twist & recovery_twist,
  double & clearance_gain) const
{
  if (!params_.recovery_mode_enabled || !intrusion.recoveryEligible()) {
    return false;
  }

  const double min_command = std::max(0.0, params_.recovery_min_command);
  if (std::abs(cmd.linear.x) < min_command &&
    std::abs(cmd.angular.z) < min_command)
  {
    return false;
  }

  const double candidate_hold_s = std::max(0.0, params_.recovery_candidate_hold_s);
  if (has_recovery_candidate_ &&
    recovery_candidate_hold_time_s_ + 1e-9 < candidate_hold_s)
  {
    const RecoveryCandidate held = evaluateRecoveryCandidate(
      held_recovery_candidate_, cmd, fb_linear, fb_angular, field, intrusion,
      local_map, mode_profile);
    if (held.valid) {
      recovery_candidate_hold_time_s_ += std::max(0.0, dt_s);
      recovery_twist = geometry_msgs::msg::Twist{};
      recovery_twist.linear.x = held.command.v;
      recovery_twist.angular.z = held.command.w;
      clearance_gain = held.clearance_gain;
      return true;
    }
    has_recovery_candidate_ = false;
    recovery_candidate_hold_time_s_ = 0.0;
  }

  const double linear_speed = std::min(
    std::abs(params_.recovery_max_linear_speed),
    std::abs(params_.absolute_max_linear_speed));
  const double angular_speed = std::min(
    std::abs(params_.recovery_max_angular_speed),
    std::abs(params_.absolute_max_angular_speed));

  std::vector<VelocityCandidate> candidates;
  candidates.reserve(10);
  auto append_unique = [&](double v, double w) {
      VelocityCandidate candidate {
        std::clamp(v, -linear_speed, linear_speed),
        std::clamp(w, -angular_speed, angular_speed)};
      if (std::abs(candidate.v) < min_command &&
        std::abs(candidate.w) < min_command)
      {
        return;
      }
      const auto duplicate = std::find_if(
        candidates.begin(), candidates.end(), [&](const VelocityCandidate & existing) {
          return std::abs(existing.v - candidate.v) < 1e-6 &&
          std::abs(existing.w - candidate.w) < 1e-6;
        });
      if (duplicate == candidates.end()) {
        candidates.push_back(candidate);
      }
    };

  append_unique(cmd.linear.x, cmd.angular.z);
  if (params_.recovery_auto_select_enabled) {
    append_unique(linear_speed, 0.0);
    append_unique(-linear_speed, 0.0);
    append_unique(0.0, angular_speed);
    append_unique(0.0, -angular_speed);
    append_unique(linear_speed, angular_speed);
    append_unique(linear_speed, -angular_speed);
    append_unique(-linear_speed, angular_speed);
    append_unique(-linear_speed, -angular_speed);
  }

  RecoveryCandidate best;
  for (const auto & candidate : candidates) {
    const RecoveryCandidate evaluated = evaluateRecoveryCandidate(
      candidate, cmd, fb_linear, fb_angular, field, intrusion, local_map, mode_profile);
    if (!evaluated.valid) {
      continue;
    }
    const bool greater_clearance =
      !best.valid || evaluated.future_clearance > best.future_clearance + 1e-6;
    const bool equal_clearance =
      best.valid && std::abs(evaluated.future_clearance - best.future_clearance) <= 1e-6;
    if (greater_clearance ||
      (equal_clearance && evaluated.command_cost < best.command_cost))
    {
      best = evaluated;
    }
  }

  if (!best.valid) {
    has_recovery_candidate_ = false;
    recovery_candidate_hold_time_s_ = 0.0;
    return false;
  }

  held_recovery_candidate_ = best.command;
  has_recovery_candidate_ = true;
  recovery_candidate_hold_time_s_ = std::max(0.0, dt_s);
  recovery_twist = geometry_msgs::msg::Twist{};
  recovery_twist.linear.x = best.command.v;
  recovery_twist.angular.z = best.command.w;
  clearance_gain = best.clearance_gain;
  return true;
}

void AvoidanceController::resetRecoveryState() const
{
  recovery_intrusion_confirm_time_s_ = 0.0;
  recovery_elapsed_s_ = 0.0;
  recovery_no_progress_time_s_ = 0.0;
  recovery_feedback_stall_time_s_ = 0.0;
  recovery_best_clearance_ = -std::numeric_limits<double>::infinity();
  recovery_abort_latched_ = false;
  recovery_abort_reason_ = RecoveryState::IDLE;
  has_recovery_candidate_ = false;
  held_recovery_candidate_ = VelocityCandidate{};
  recovery_candidate_hold_time_s_ = 0.0;
}

void AvoidanceController::resetTiltEmergencyState() const
{
  tilt_emergency_latched_ = false;
  tilt_emergency_clear_time_s_ = 0.0;
  tilt_emergency_reason_ = EmergencyStopReason::NONE;
}

void AvoidanceController::latchRecoveryAbort(RecoveryState reason) const
{
  recovery_abort_latched_ = true;
  recovery_abort_reason_ = reason;
  has_recovery_candidate_ = false;
  recovery_candidate_hold_time_s_ = 0.0;
}

AvoidanceResult AvoidanceController::evaluate(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear,
  double fb_angular,
  const ObstacleField & field,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile,
  double prev_output_linear_x,
  double prev_output_angular_z,
  double dt_s,
  const AvoidanceContext & context,
  const ObstacleField * tilt_field) const
{
  const auto t0 = std::chrono::steady_clock::now();
  configureForMode(mode_profile);
  AvoidanceResult result;
  result.safe_twist = cmd;
  result.hybrid_log.mode = context.safety_mode;

  if (context.safety_mode == SafetyMode::ESTOP) {
    emergency_brake_.latch();
    resetCandidateSpeedUpHold();
    braking_envelope_active_time_s_ = 0.0;
    resetRecoveryState();
    resetTiltEmergencyState();
    result.safe_twist = emergency_brake_.applyEmergencyBrake();
    result.emergency_brake = true;
    result.emergency_reason = EmergencyStopReason::MODE_ESTOP;
    result.risk.has_collision = true;
    result.risk.emergency = true;
    const auto t1 = std::chrono::steady_clock::now();
    result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.hybrid_log.tick_ms = result.tick_duration_ms;
    return result;
  }

  // ── 透传模式：不做 Footprint / Map，仅绝对限速 ──
  if (!mode_profile.footprint_selector_enabled) {
    emergency_brake_.reset();
    resetCandidateSpeedUpHold();
    braking_envelope_active_time_s_ = 0.0;
    resetRecoveryState();
    resetTiltEmergencyState();
    result.safe_twist.linear.x = std::clamp(
      cmd.linear.x,
      -params_.absolute_max_linear_speed,
      params_.absolute_max_linear_speed);
    result.safe_twist.angular.z = std::clamp(
      cmd.angular.z,
      -params_.absolute_max_angular_speed,
      params_.absolute_max_angular_speed);
    const auto t1 = std::chrono::steady_clock::now();
    result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
  }

  geometry_msgs::msg::Twist eval_cmd = cmd;
  const geometry_msgs::msg::Twist requested_cmd = cmd;
  if (context.cruise_active && context.cruise_linear_cap > 0.0 && eval_cmd.linear.x > 0.01) {
    eval_cmd.linear.x = std::min(eval_cmd.linear.x, context.cruise_linear_cap);
  }

  FootprintIntrusion intrusion = analyzeFootprintIntrusion(field);
  enrichProximityBlock(intrusion, field, fb_linear, prev_output_linear_x);
  const double min_motion_command = std::max(0.0, params_.recovery_min_command);
  const bool feedback_stopped =
    std::abs(fb_linear) < 0.02 && std::abs(prev_output_linear_x) < 0.02;
  const bool parked =
    feedback_stopped &&
    std::abs(eval_cmd.linear.x) < min_motion_command &&
    std::abs(eval_cmd.angular.z) < min_motion_command;
  BrakingEnvelope braking_envelope;
  if (!intrusion.active) {
    braking_envelope = evaluateBrakingEnvelope(
      eval_cmd, fb_linear, prev_output_linear_x, field, local_map, mode_profile, dt_s);
    if (braking_envelope.active) {
      braking_envelope_active_time_s_ += std::max(0.0, dt_s);
    } else {
      braking_envelope_active_time_s_ = 0.0;
    }
    if (braking_envelope.active && !parked &&
      !braking_envelope.controlled_escape_only)
    {
      const double requested_speed = std::abs(eval_cmd.linear.x);
      const double smoothed_limit = smoothBrakingSpeedLimit(
        braking_envelope.speed_limit_m_s, dt_s);
      braking_envelope.speed_limit_m_s = smoothed_limit;
      const double scale = requested_speed > 1e-6 ?
        std::clamp(smoothed_limit / requested_speed, 0.0, 1.0) :
        0.0;
      eval_cmd.linear.x = std::copysign(smoothed_limit, eval_cmd.linear.x);
      eval_cmd.angular.z *= scale;
    } else {
      has_smoothed_braking_speed_limit_ = false;
    }
  }
  if (intrusion.active) {
    braking_envelope_active_time_s_ = 0.0;
  }

  if (braking_envelope.emergency && mode_profile.emergency_stop_enabled) {
    emergency_brake_.latch();
    resetCandidateSpeedUpHold();
    resetRecoveryState();
    result.safe_twist = emergency_brake_.applyEmergencyBrake();
    result.emergency_brake = true;
    result.emergency_reason = EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT;
    result.risk.has_collision = true;
    result.risk.emergency = true;
    result.risk.braking_limited = true;
    result.risk.braking_available_distance_m = braking_envelope.available_distance_m;
    result.risk.braking_speed_limit_m_s = braking_envelope.speed_limit_m_s;
    result.risk.braking_stopping_distance_m = braking_envelope.stopping_distance_m;
    result.risk.min_ttc = braking_envelope.footprint_check.ttc;
    result.risk.min_clearance = braking_envelope.footprint_check.min_distance;
    result.risk.worst = braking_envelope.footprint_check;
    const auto t1 = std::chrono::steady_clock::now();
    result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.hybrid_log.tick_ms = result.tick_duration_ms;
    return result;
  }
  const bool braking_control_feasible =
    braking_envelope.active &&
    !braking_envelope.controlled_escape_only &&
    braking_envelope.available_distance_m >
    std::max(1e-3, 0.5 * params_.braking_path_step_m) &&
    braking_envelope.stopping_distance_m <=
    braking_envelope.available_distance_m + 1e-6;

  const EmergencyBrakeContext directional_ctx = buildEmergencyBrakeContext(
    requested_cmd, geometry_msgs::msg::Twist{}, fb_linear, fb_angular,
    prev_output_linear_x, field);
  const bool directional_escape_viable =
    !intrusion.active && isDirectionalEscapeViable(directional_ctx);

  const bool map_enabled = mode_profile.map_constraint_enabled &&
    map_config_.enabled &&
    local_map != nullptr &&
    !local_map->probability.empty();

  // ── 步骤 1：生成候选轨迹集 Q ──
  auto rollouts = rollout_.rolloutAll(eval_cmd, fb_linear, fb_angular);
  if (map_enabled && map_config_.veto_on_lethal &&
    std::abs(eval_cmd.linear.x) >= min_motion_command)
  {
    MapConstraintResult current_map_result;
    if (map_checker_.trajectoryLethal(
        *local_map,
        {PredictedPose{}},
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        current_map_result))
    {
      VelocityCandidate escape_candidate;
      escape_candidate.v = std::clamp(
        eval_cmd.linear.x,
        -params_.recovery_max_linear_speed,
        params_.recovery_max_linear_speed);
      escape_candidate.w = 0.0;
      const bool duplicate = std::any_of(
        rollouts.begin(), rollouts.end(),
        [&](const TrajectoryRollout & rollout) {
          return std::abs(rollout.cmd.v - escape_candidate.v) < 1e-9 &&
          std::abs(rollout.cmd.w - escape_candidate.w) < 1e-9;
        });
      if (!duplicate) {
        rollouts.push_back(
          rollout_.rolloutCandidate(escape_candidate, fb_linear, fb_angular));
      }
    }
  }

  struct ScoredCandidate
  {
    int index = -1;
    CollisionCheckResult check;
    double command_cost = 0.0;
    bool footprint_safe = false;
    bool controlled_escape = false;
    bool map_escape = false;
  };

  std::vector<ScoredCandidate> scored;
  scored.reserve(rollouts.size());

  CollisionRisk aggregate;
  aggregate.min_ttc = std::numeric_limits<double>::infinity();
  aggregate.min_clearance = std::numeric_limits<double>::infinity();
  aggregate.braking_limited = braking_envelope.active;
  aggregate.braking_available_distance_m = braking_envelope.available_distance_m;
  aggregate.braking_speed_limit_m_s = braking_envelope.speed_limit_m_s;
  aggregate.braking_stopping_distance_m = braking_envelope.stopping_distance_m;

  // ── 步骤 2：Footprint 扫掠碰撞，记录每条候选结果 ──
  for (size_t i = 0; i < rollouts.size(); ++i) {
    ScoredCandidate sc;
    sc.index = static_cast<int>(i);
    sc.check = checker_.checkTrajectory(rollouts[i], field);
    sc.command_cost = commandErrorCost(
      rollouts[i].cmd, eval_cmd, params_.safety_filter_angular_weight);
    const bool follows_braking_envelope =
      braking_control_feasible && i == 0U;
    const double min_command = std::max(0.0, params_.recovery_min_command);
    const double max_escape_linear = std::max(
      min_command, params_.recovery_max_linear_speed);
    const bool controlled_reverse_escape_state =
      fb_linear <= 0.02 &&
      prev_output_linear_x <= 0.02 &&
      std::abs(fb_linear) <= max_escape_linear + 1e-9 &&
      std::abs(prev_output_linear_x) <= max_escape_linear + 1e-9;
    const bool front_block_collision =
      (sc.check.collision && sc.check.collision_x >= 0.0) ||
      (!sc.check.collision &&
      std::isfinite(sc.check.hard_stop_path_distance_m) &&
      sc.check.hard_stop_x >= 0.0);
    const bool escape_from_forward_block =
      controlled_reverse_escape_state &&
      !parked &&
      requested_cmd.linear.x < -min_command &&
      rollouts[i].cmd.v < -min_command &&
      front_block_collision;
    const bool escape_from_rear_side_block =
      directional_escape_viable &&
      feedback_stopped &&
      !parked &&
      requested_cmd.linear.x > min_command &&
      rollouts[i].cmd.v > min_command;
    sc.footprint_safe =
      follows_braking_envelope ||
      escape_from_rear_side_block ||
      escape_from_forward_block ||
      (!sc.check.collision &&
      footprintCandidateClearanceOk(sc.check.min_distance, params_));
    sc.controlled_escape = escape_from_forward_block;

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

  // 安全候选优先，同等安全下选手柄偏差最小者
  std::sort(
    scored.begin(), scored.end(), [](const ScoredCandidate & a, const ScoredCandidate & b) {
      if (a.footprint_safe != b.footprint_safe) {
        return a.footprint_safe && !b.footprint_safe;
      }
      return a.command_cost < b.command_cost;
    });

  int selected_index = -1;
  int footprint_best = -1;
  bool selected_controlled_escape = false;
  bool selected_map_escape = false;

  // ── 步骤 3：Map 硬约束 veto — 从优到劣尝试，首个 map_ok 者胜出 ──
  for (auto & sc : scored) {
    if (!sc.footprint_safe) {
      continue;
    }
    if (footprint_best < 0) {
      footprint_best = sc.index;
    }

    bool map_ok = true;
    if (map_enabled && map_config_.veto_on_lethal &&
      !(braking_control_feasible && sc.index == 0))
    {
      MapConstraintResult map_result;
      const double lethal_threshold = mode_profile.lethal_threshold;
      const bool check_corners = map_config_.check_footprint_corners;
      const bool map_lethal = map_checker_.trajectoryLethal(
        *local_map,
        rollouts[static_cast<size_t>(sc.index)].poses,
        lethal_threshold,
        check_corners,
        map_result);
      map_ok = !map_lethal;
      if (map_lethal) {
        MapConstraintResult escape_result;
        map_ok = isControlledMapEscape(
          rollouts[static_cast<size_t>(sc.index)],
          fb_linear,
          prev_output_linear_x,
          local_map,
          mode_profile,
          params_.recovery_lookahead_s,
          escape_result);
        sc.map_escape = map_ok;
      }
      if (!map_ok && footprint_best == sc.index) {
        result.hybrid_log.map_veto_applied = true;
        result.hybrid_log.map_ttc = map_result.first_lethal_t;
      }
    }

    if (map_ok) {
      selected_index = sc.index;
      selected_controlled_escape = sc.controlled_escape;
      selected_map_escape = sc.map_escape;
      break;
    }
  }

  result.hybrid_log.footprint_selected_candidate = footprint_best;

  const bool need_recovery =
    params_.recovery_mode_enabled &&
    (intrusion.active ||
    (intrusion.proximity_blocked && selected_index < 0 && !directional_escape_viable));

  if (need_recovery) {
    result.risk = aggregate;
    result.risk.has_collision = true;
    if (intrusion.active) {
      result.risk.worst = intrusion.worst;
    }
    result.risk.min_clearance = std::min(result.risk.min_clearance, intrusion.min_clearance);

    const double valid_dt_s = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;
    recovery_intrusion_confirm_time_s_ += valid_dt_s;
    const double confirmation_s = std::max(0.0, params_.recovery_intrusion_confirm_s);

    auto apply_recovery_stop = [&](bool physical_intrusion) {
        emergency_brake_.latch();
        resetCandidateSpeedUpHold();
        result.emergency_brake = true;
        result.emergency_reason = classifyEmergencyReason(result.risk, params_);
        result.safe_twist = emergency_brake_.applyEmergencyBrake();
        if (physical_intrusion) {
          result.risk.emergency = true;
        }
      };

    if (recovery_abort_latched_) {
      result.recovery_state = recovery_abort_reason_;
      if (intrusion.active || !directional_escape_viable) {
        apply_recovery_stop(intrusion.active);
      } else {
        resetRecoveryState();
      }
    } else if (recovery_intrusion_confirm_time_s_ + 1e-9 < confirmation_s) {
      result.recovery_state = RecoveryState::CONFIRMING;
      apply_recovery_stop(intrusion.active);
    } else {
      recovery_elapsed_s_ += valid_dt_s;
      const double timeout_s = std::max(0.0, params_.recovery_timeout_s);
      if (timeout_s > 0.0 && recovery_elapsed_s_ + 1e-9 >= timeout_s) {
        result.recovery_state = RecoveryState::ABORTED_TIMEOUT;
        if (intrusion.active) {
          latchRecoveryAbort(RecoveryState::ABORTED_TIMEOUT);
          apply_recovery_stop(true);
        } else {
          resetRecoveryState();
          apply_recovery_stop(false);
        }
      } else {
        geometry_msgs::msg::Twist recovery_twist;
        double clearance_gain = 0.0;
        const bool recovery_found = trySelectRecoveryCommand(
          requested_cmd, fb_linear, fb_angular, field, intrusion, local_map, mode_profile,
          valid_dt_s, recovery_twist, clearance_gain);

        if (!recovery_found) {
          result.recovery_state = RecoveryState::BLOCKED;
          apply_recovery_stop(intrusion.active);
        } else {
          const double progress_epsilon =
            std::max(1e-4, 0.5 * params_.recovery_min_clearance_improvement_m);
          if (!std::isfinite(recovery_best_clearance_)) {
            recovery_best_clearance_ = intrusion.min_clearance;
            recovery_no_progress_time_s_ = 0.0;
          } else if (intrusion.min_clearance >
            recovery_best_clearance_ + progress_epsilon)
          {
            recovery_best_clearance_ = intrusion.min_clearance;
            recovery_no_progress_time_s_ = 0.0;
          } else if (recovery_elapsed_s_ >
            std::max(0.0, params_.recovery_motion_grace_s))
          {
            recovery_no_progress_time_s_ += valid_dt_s;
          }

          const double no_progress_timeout_s =
            std::max(0.0, params_.recovery_no_progress_timeout_s);
          if (recovery_elapsed_s_ >
            std::max(0.0, params_.recovery_motion_grace_s))
          {
            const bool expects_linear =
              std::abs(recovery_twist.linear.x) >= params_.recovery_min_command;
            const bool expects_angular =
              std::abs(recovery_twist.angular.z) >= params_.recovery_min_command;
            const bool feedback_moving =
              (expects_linear &&
              std::abs(fb_linear) >=
              std::max(0.0, params_.recovery_feedback_linear_min_m_s)) ||
              (expects_angular &&
              std::abs(fb_angular) >=
              std::max(0.0, params_.recovery_feedback_angular_min_rad_s));
            if (feedback_moving) {
              recovery_feedback_stall_time_s_ = 0.0;
            } else {
              recovery_feedback_stall_time_s_ += valid_dt_s;
            }
          }

          const double feedback_stall_timeout_s =
            std::max(0.0, params_.recovery_feedback_stall_timeout_s);
          const bool no_clearance_progress =
            no_progress_timeout_s > 0.0 &&
            recovery_no_progress_time_s_ + 1e-9 >= no_progress_timeout_s;
          const bool feedback_stalled =
            feedback_stall_timeout_s > 0.0 &&
            recovery_feedback_stall_time_s_ + 1e-9 >= feedback_stall_timeout_s;
          if (no_clearance_progress || feedback_stalled) {
            const RecoveryState abort_state =
              feedback_stalled ?
              RecoveryState::ABORTED_FEEDBACK_STALL :
              RecoveryState::ABORTED_NO_PROGRESS;
            result.recovery_state = abort_state;
            if (intrusion.active) {
              latchRecoveryAbort(abort_state);
              apply_recovery_stop(true);
            } else {
              resetRecoveryState();
              apply_recovery_stop(false);
            }
          } else {
            emergency_brake_.reset();
            resetCandidateSpeedUpHold();
            result.safe_twist = recovery_twist;
            result.recovery_active = true;
            result.recovery_state = RecoveryState::ACTIVE;
            result.recovery_clearance_gain_m = clearance_gain;
            result.hybrid_log.final_candidate = -2;
          }
        }
      }
    }
    result.recovery_elapsed_s = recovery_elapsed_s_;
  } else {
    resetRecoveryState();
  }

  if (!result.recovery_active && !result.emergency_brake && selected_index < 0) {
    // ── 无可行候选：急停或 SafetyFilter 尽力降级 ──
    result.risk = aggregate;
    result.risk.has_collision = true;
    const bool all_footprint_safe_candidates_map_vetoed =
      map_enabled && map_config_.veto_on_lethal && footprint_best >= 0;
    if (all_footprint_safe_candidates_map_vetoed) {
      emergency_brake_.latch();
      resetCandidateSpeedUpHold();
      result.emergency_brake = true;
      result.emergency_reason = EmergencyStopReason::MAP_VETO;
      result.safe_twist = emergency_brake_.applyEmergencyBrake();
      result.risk.emergency = true;
    } else if (mode_profile.emergency_stop_enabled) {
      const EmergencyBrakeContext pre_context = buildEmergencyBrakeContext(
        requested_cmd, eval_cmd, fb_linear, fb_angular, prev_output_linear_x, field);
      if (emergency_brake_.shouldEmergencyBrake(result.risk, &pre_context)) {
        emergency_brake_.latch();
        resetCandidateSpeedUpHold();
        result.emergency_brake = true;
        result.emergency_reason = classifyEmergencyReason(result.risk, params_);
        result.safe_twist = emergency_brake_.applyEmergencyBrake();
        result.risk.emergency = true;
      } else {
        const SafetyFilterResult filtered = safety_filter_.selectSafeVelocity(
          eval_cmd, rollouts, field, prev_output_linear_x, prev_output_angular_z, dt_s);
        result.safe_twist = filtered.safe_twist;
        result.risk = filtered.risk;
        result.risk.has_collision = filtered.all_candidates_collide || aggregate.has_collision;
      }
    }
  } else if (!result.recovery_active && !result.emergency_brake) {
    // ── 选中候选：应用速度并做周期内加减速限制 ──
    const auto & chosen = rollouts[static_cast<size_t>(selected_index)].cmd;
    result.safe_twist.linear.x = applyCandidateSpeedUpHold(chosen.v, dt_s);
    result.safe_twist.angular.z = chosen.w;
    result.hybrid_log.final_candidate = selected_index;
    result.risk = aggregate;

    if (dt_s > 1e-6) {
      if (params_.max_decel_m_s2 > 0.0) {
        const double max_drop = params_.max_decel_m_s2 * dt_s;
        const double max_rise = params_.max_recover_accel_m_s2 * dt_s;
        double v = result.safe_twist.linear.x;
        const double dv = v - prev_output_linear_x;
        if (dv < -max_drop) {
          v = prev_output_linear_x - max_drop;
        } else if (dv > max_rise) {
          v = prev_output_linear_x + max_rise;
        }
        result.safe_twist.linear.x = v;
      }
      if (params_.max_angular_decel_rad_s2 > 0.0) {
        const double max_drop_w = params_.max_angular_decel_rad_s2 * dt_s;
        const double max_rise_w = params_.max_angular_recover_rad_s2 * dt_s;
        double w = result.safe_twist.angular.z;
        const double dw = w - prev_output_angular_z;
        if (dw < -max_drop_w) {
          w = prev_output_angular_z - max_drop_w;
        } else if (dw > max_rise_w) {
          w = prev_output_angular_z + max_rise_w;
        }
        result.safe_twist.angular.z = w;
      }
    }

    result.safe_twist.linear.x = std::clamp(
      result.safe_twist.linear.x,
      -params_.absolute_max_linear_speed,
      params_.absolute_max_linear_speed);
    result.safe_twist.angular.z = std::clamp(
      result.safe_twist.angular.z,
      -params_.absolute_max_angular_speed,
      params_.absolute_max_angular_speed);
  }

  // 对控制器最终将要发布的命令重新做完整检查，不能沿用已淘汰候选的聚合风险。
  bool final_follows_controlled_escape = false;
  if (!result.recovery_active && !result.emergency_brake &&
    mode_profile.footprint_selector_enabled)
  {
    VelocityCandidate final_cmd;
    final_cmd.v = result.safe_twist.linear.x;
    final_cmd.w = result.safe_twist.angular.z;
    const TrajectoryRollout final_rollout =
      rollout_.rolloutCandidate(final_cmd, fb_linear, fb_angular);
    const CollisionCheckResult final_check = checker_.checkTrajectory(final_rollout, field);
    bool final_follows_map_escape = false;
    if (selected_map_escape && selected_index >= 0) {
      const VelocityCandidate & selected_cmd =
        rollouts[static_cast<size_t>(selected_index)].cmd;
      const bool linear_follows =
        final_cmd.v * selected_cmd.v > 0.0 &&
        std::abs(final_cmd.v) <= std::abs(selected_cmd.v) + 1e-9;
      const bool angular_follows =
        (std::abs(selected_cmd.w) < 1e-9 && std::abs(final_cmd.w) < 1e-9) ||
        (final_cmd.w * selected_cmd.w >= 0.0 &&
        std::abs(final_cmd.w) <= std::abs(selected_cmd.w) + 1e-9);
      final_follows_map_escape = linear_follows && angular_follows;
    }
    if (selected_controlled_escape && selected_index >= 0) {
      const VelocityCandidate & selected_cmd =
        rollouts[static_cast<size_t>(selected_index)].cmd;
      const bool linear_follows =
        final_cmd.v * selected_cmd.v > 0.0 &&
        std::abs(final_cmd.v) <= std::abs(selected_cmd.v) + 1e-9;
      const bool angular_follows =
        (std::abs(selected_cmd.w) < 1e-9 && std::abs(final_cmd.w) < 1e-9) ||
        (final_cmd.w * selected_cmd.w >= 0.0 &&
        std::abs(final_cmd.w) <= std::abs(selected_cmd.w) + 1e-9);
      final_follows_controlled_escape = linear_follows && angular_follows;
    }

    result.risk = CollisionRisk{};
    result.risk.min_ttc = final_check.ttc;
    result.risk.min_clearance = final_check.min_distance;
    if (final_check.collision) {
      result.risk.has_collision = true;
      result.risk.worst = final_check;
    }

    if (map_enabled && map_config_.veto_on_lethal &&
      !braking_control_feasible)
    {
      MapConstraintResult map_result;
      if (map_checker_.trajectoryLethal(
          *local_map,
          final_rollout.poses,
          mode_profile.lethal_threshold,
          map_config_.check_footprint_corners,
          map_result))
      {
        MapConstraintResult escape_result;
        if (!final_follows_map_escape &&
          !isControlledMapEscape(
            final_rollout,
            fb_linear,
            prev_output_linear_x,
            local_map,
            mode_profile,
            selected_map_escape ?
            params_.rollout_horizon_s :
            params_.recovery_lookahead_s,
            escape_result))
        {
          result.hybrid_log.map_veto_applied = true;
          result.hybrid_log.map_ttc = map_result.first_lethal_t;
          emergency_brake_.latch();
          resetCandidateSpeedUpHold();
          result.emergency_brake = true;
          result.emergency_reason = EmergencyStopReason::MAP_VETO;
          result.safe_twist = emergency_brake_.applyEmergencyBrake();
          result.risk.has_collision = true;
          result.risk.emergency = true;
        }
      }
    }
  }

  estimateLateralDistances(
    field, result.risk.min_lateral_left_distance, result.risk.min_lateral_right_distance);

  if (result.risk.has_collision && result.risk.worst.collision) {
    result.risk.primary_direction = deriveCollisionDirection(
      result.risk.worst.collision_x, result.risk.worst.collision_y);
  }
  result.risk.braking_limited = braking_envelope.active;
  result.risk.braking_available_distance_m = braking_envelope.available_distance_m;
  result.risk.braking_speed_limit_m_s = braking_envelope.speed_limit_m_s;
  result.risk.braking_stopping_distance_m = braking_envelope.stopping_distance_m;
  const TiltIntervention tilt_intervention =
    evaluateTiltIntervention(requested_cmd, fb_linear, tilt_field, dt_s);
  mergeTiltIntervention(result, tilt_intervention);
  if (!result.recovery_active && params_.use_ttc_emergency_stop &&
    !final_follows_controlled_escape &&
    result.risk.min_ttc <= params_.limiting_ttc_s)
  {
    result.risk.emergency = (result.risk.min_ttc <= params_.emergency_ttc_s);
  }
  fillInterventionLevels(result.risk, params_);

  // ── 最后一道硬闸：TTC / 净空触线则强制零速 ──
  EmergencyBrakeContext eb_context = buildEmergencyBrakeContext(
    requested_cmd, result.safe_twist, fb_linear, fb_angular, prev_output_linear_x, field);
  eb_context.controlled_escape_active =
    !result.recovery_active &&
    !result.emergency_brake &&
    selected_controlled_escape &&
    final_follows_controlled_escape;
  if (!result.recovery_active && mode_profile.emergency_stop_enabled) {
    const bool immediate_brake =
      emergency_brake_.shouldEmergencyBrake(result.risk, &eb_context);
    const bool brake_active =
      emergency_brake_.update(result.risk, dt_s, &eb_context);
    if (brake_active) {
      resetCandidateSpeedUpHold();
      result.emergency_brake = true;
      if (result.emergency_reason == EmergencyStopReason::NONE) {
        result.emergency_reason =
          immediate_brake ? classifyEmergencyReason(result.risk, params_) :
          EmergencyStopReason::BRAKE_HOLD;
      }
      result.safe_twist = emergency_brake_.applyEmergencyBrake();
      result.risk.emergency = true;
    } else {
      result.emergency_brake = false;
      if (result.emergency_reason == EmergencyStopReason::BRAKE_HOLD ||
        result.emergency_reason == EmergencyStopReason::HARD_STOP_DISTANCE)
      {
        result.emergency_reason = EmergencyStopReason::NONE;
      }
    }
  } else {
    emergency_brake_.reset();
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  result.hybrid_log.tick_ms = result.tick_duration_ms;
  return result;
}
