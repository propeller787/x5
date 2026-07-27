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

/**
 * 按当前接近速度计算斜雷达「最近停车边距」。
 *
 * 低速时使用 hard_stop_distance_m（如 0.12 m）；随速度升高二次插值到
 * distance_emergency_m（如 0.45 m @ 8 km/h），避免高速晚检出时反应过晚。
 *
 * @param speed  接近速度 (m/s)，取 max(|命令|, |反馈|)
 * @return 触发 emergency 的前向净空阈值 (m)
 */
double tiltNearestStopDistance(
  double speed,
  const AvoidanceParams & params)
{
  const double hard_stop = std::max(0.0, params.tilt_ttc.hard_stop_distance_m);
  const double max_speed_stop = std::max(
    hard_stop,
    params.tilt_ttc.distance_emergency_m);
  const double reference_speed = std::max(
    1e-3,
    std::abs(params.absolute_max_linear_speed));
  // ratio² 使低速区更贴近 hard_stop，高速区更快逼近 distance_emergency_m
  const double ratio = std::clamp(std::max(0.0, speed) / reference_speed, 0.0, 1.0);
  return hard_stop + (max_speed_stop - hard_stop) * ratio * ratio;
}

double computeEffectiveBrakingRemaining(
  double path_remaining_m,
  double min_clearance_m,
  const AvoidanceParams & params,
  bool forward_motion)
{
  if (!params.min_distance_braking_enabled || !forward_motion) {
    return path_remaining_m;
  }
  const double clearance_remaining = std::isfinite(min_clearance_m) ?
    std::max(0.0, min_clearance_m - params.hard_stop_distance_m) :
    std::numeric_limits<double>::infinity();
  const double current_clearance_cap_distance = std::max(
    params.hard_stop_distance_m,
    params.hard_stop_release_distance_m);
  // Side clearance outside the hard-stop release band is passable margin, not braking distance.
  if (!std::isfinite(min_clearance_m) ||
    min_clearance_m > current_clearance_cap_distance)
  {
    return path_remaining_m;
  }
  if (!std::isfinite(path_remaining_m)) {
    return clearance_remaining;
  }
  return std::min(path_remaining_m, clearance_remaining);
}

/**
 * 转弯合成速度包络：外缘线速度 |v|+|ω|·r 与横向加速度 |v·ω| 任一超限时，
 * v、ω 等比缩放（保持转弯半径）。返回缩放系数 k∈(0,1]，未生效为 1。
 * 等比缩放下外缘按 k 线性收敛、横加按 k² 收敛，故取 min(k_edge, √(cap/lat))
 * 可同时满足两判据。直线（ω≈0）与默认参数下的纯旋转均不触发。
 */
double applyTurnSpeedEnvelope(
  double & linear_x,
  double & angular_z,
  const AvoidanceParams & params)
{
  if (!params.turn_speed_envelope_enabled) {
    return 1.0;
  }
  double k = 1.0;
  const double r = std::max(0.0, params.turn_envelope_edge_radius_m);
  const double edge = std::abs(linear_x) + std::abs(angular_z) * r;
  if (params.turn_envelope_max_edge_speed_m_s > 0.0 &&
    edge > params.turn_envelope_max_edge_speed_m_s)
  {
    k = std::min(k, params.turn_envelope_max_edge_speed_m_s / edge);
  }
  const double lat = std::abs(linear_x * angular_z);
  if (params.turn_envelope_max_lat_accel_m_s2 > 0.0 &&
    lat > params.turn_envelope_max_lat_accel_m_s2)
  {
    k = std::min(k, std::sqrt(params.turn_envelope_max_lat_accel_m_s2 / lat));
  }
  if (k < 1.0) {
    linear_x *= k;
    angular_z *= k;
  }
  return k;
}

/**
 * 急速转向线速度硬帽：|ω| ≥ trigger_angular_rad_s 判定「急速转向」时，把线速度钳到
 * max_linear_m_s（只压 v、不动 ω，不保转弯半径）。返回本次是否生效。
 *
 * 与 turn_speed_envelope 的分工：包络按外缘/横加等比缩放 v 与 ω 保持转弯半径，其 k 有下界
 * （bag speed_test_tilt_on_2 临界帧 v1.57/ω1.04 只得 k≈0.86 → v≈1.35 m/s，仍高于已撞的
 * ~1.3 m/s）；本层不保半径，直接把前向逼近速度压到停得住的量级，给净空/EB 留出制动距离。
 * 两层并存取更严者：包络管「转弯外圈别太快」，本层管「急转时整体别太快」。
 *
 * 直线与缓转（|ω| < 阈值）不介入；trigger ≤ 0 视为未配置、不介入（validateParams 告警）。
 * 阈值是硬门限，跨越时 v 上限有阶跃，由下游 OutputShaper 的加加速度整形吸收。
 */
bool applyRapidSteerLinearCap(
  double & linear_x,
  double angular_z,
  const AvoidanceParams & params)
{
  if (!params.rapid_steer_cap_enabled) {
    return false;
  }
  const double trigger = params.rapid_steer_trigger_angular_rad_s;
  if (!(trigger > 0.0) || std::abs(angular_z) < trigger) {
    return false;
  }
  const double cap = std::max(0.0, params.rapid_steer_max_linear_m_s);
  if (std::abs(linear_x) <= cap) {
    return false;
  }
  linear_x = std::clamp(linear_x, -cap, cap);
  return true;
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
  , sudden_detector_(params_.sudden_obstacle)
  , map_checker_(params_, map_config_)
  , narrow_planner_(params_)
  , closing_estimator_(params_.closing_speed)
{
  pulse_comp_.updateParams(params_.precise_stop);
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
  sudden_detector_.updateParams(params_.sudden_obstacle);
  map_checker_.updateFootprintParams(params_);
  map_checker_.updateConfig(map_config_);
  pulse_comp_.updateParams(params_.precise_stop);
  narrow_planner_.updateParams(params_);
  closing_estimator_.updateParams(params_.closing_speed);
  emergency_brake_.reset();
  sudden_detector_.reset();
  resetCandidateSpeedUpHold();
  braking_envelope_active_time_s_ = 0.0;
  has_smoothed_braking_speed_limit_ = false;
  resetRecoveryState();
  resetPinnedEscape();
  resetTiltEmergencyState();
  resetForwardStopHold();
  resetStandstillGuard();
  resetSharedControlState();
  hard_stop_reached_ = false;
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
  sudden_detector_.updateParams(params_.sudden_obstacle);
  map_checker_.updateFootprintParams(params_);
  pulse_comp_.updateParams(params_.precise_stop);
  narrow_planner_.updateParams(params_);
  closing_estimator_.updateParams(params_.closing_speed);
  active_avoidance_overrides_enabled_ =
    mode_profile.avoidance_overrides_enabled;
  resetCandidateSpeedUpHold();
  braking_envelope_active_time_s_ = 0.0;
  has_smoothed_braking_speed_limit_ = false;
  resetRecoveryState();
  resetPinnedEscape();
  resetTiltEmergencyState();
  resetForwardStopHold();
  resetStandstillGuard();
  resetSharedControlState();
  hard_stop_reached_ = false;
}

void AvoidanceController::updateMinDistanceBrakingLatch(
  const BrakingEnvelope & envelope,
  const geometry_msgs::msg::Twist & eval_cmd,
  double min_clearance_m,
  double effective_remaining_m) const
{
  if (!params_.min_distance_braking_enabled) {
    hard_stop_reached_ = false;
    return;
  }

  const double min_command = std::max(0.0, params_.recovery_min_command);
  const bool at_boundary =
    envelope.at_min_stop ||
    (std::isfinite(effective_remaining_m) &&
    effective_remaining_m <= params_.braking_at_stop_distance_eps_m);
  if (hard_stop_reached_) {
    // 前向净空为非有限值 = 方向掩码判定前向无近障威胁（走廊完全畅通）。
    // 此时必须与「净空 > 释放距离」同样构成解除依据：否则 direction_mask 启用后
    // forward_clearance_m 恒为 +inf，isfinite 判据永远为假，硬停自锁只能靠倒车解除，
    // 驾驶员持续前推即被永久钉死（表现为前推蠕行/卡顿）。
    const bool forward_clearance_released =
      !at_boundary &&
      (!std::isfinite(min_clearance_m) ||
      min_clearance_m > params_.hard_stop_release_distance_m);
    if (eval_cmd.linear.x < -min_command || forward_clearance_released) {
      hard_stop_reached_ = false;
    }
  }

  if (params_.braking_no_forward_reaccel_at_hard_stop &&
    eval_cmd.linear.x > min_command &&
    (at_boundary ||
    (envelope.active && envelope.speed_limit_m_s <= 1e-6)))
  {
    hard_stop_reached_ = true;
  }
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
  // 累计航向变化上限：紧转弯（半径 v/w 极小）时，沿固定弧长积分会原地打转数圈，
  // 使膨胀 footprint 扫遍车周并把后/侧障误判为前向即时硬停而冻结整车。一旦航向累计超过
  // 此值即截断——打转弧长不算前向可停距离，紧转弯净空交由 turn_clearance_guard /
  // direction_mask 角轴处理。≤0 表示不截断（旧行为）。
  const double max_heading =
    params_.braking_probe_max_heading_rad > 0.0 ?
    params_.braking_probe_max_heading_rad :
    std::numeric_limits<double>::infinity();
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double distance = 0.0;
  probe.poses.push_back({x, y, theta, 0.0});

  while (distance + 1e-9 < max_distance && std::abs(theta) < max_heading) {
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

/**
 * 构建斜雷达 TTC 短窗探测轨迹。
 *
 * 与水平主路径的长时域 rollout 不同，此处仅沿当前前向命令做直线/弧线积分，
 * 长度 = min(max_probe_distance_m, speed × horizon_s)，适配 0.03–0.7 m 有效量程。
 * 轨迹供 SweptFootprintChecker 计算碰撞 TTC 与路径距离。
 */
TrajectoryRollout AvoidanceController::buildTiltProbe(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear) const
{
  TrajectoryRollout probe;
  probe.cmd.v = cmd.linear.x;
  probe.cmd.w = cmd.angular.z;

  const double command_speed = std::abs(cmd.linear.x);
  // 取命令与反馈较大者，避免命令突增但车体尚未加速时 TTC 过于乐观
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

  // 航向截断保护：低速大转向时 dt=ds/speed 极大，theta 积分会生成螺旋轨迹，
  // 使 SweptFootprintChecker 扫遍车周把侧后方障碍误判为前向碰撞。
  // 复用 braking_probe_max_heading_rad（与 buildDistanceProbe 同一意图）。
  const double max_heading =
    params_.braking_probe_max_heading_rad > 0.0 ?
    params_.braking_probe_max_heading_rad :
    std::numeric_limits<double>::infinity();

  while (distance + 1e-9 < max_distance && std::abs(theta) < max_heading) {
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

/**
 * 计算单个斜雷达障碍图元的前向净空。
 *
 * 仅统计 x > 0 的前向障碍（贴地/低矮障碍常落在车体前方低处）。
 * 净空 = 图元中心 x − 图元半径 − 膨胀 Footprint 前缘 x_max。
 * SIM-016 等场景优先用此边距而非轨迹碰撞距离，避免侧向点误导。
 */
double AvoidanceController::tiltForwardClearance(const ObstaclePrimitive & obstacle) const
{
  if (!obstacle.valid || obstacle.x <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double front_x = footprint_.inflatedRect().x_max;
  return obstacle.x - std::max(0.0, obstacle.radius) - front_x;
}

/**
 * 斜雷达近距 TTC 通道：单周期评估。
 *
 * 执行流程：
 *  1. 前置检查：通道开关、前向命令、障碍场非空
 *  2. 短窗轨迹扫掠 → min_ttc_s；遍历图元 → min_distance_m（前向净空优先）
 *  3. Emergency：边距 ≤ nearest_stop(speed)，或 TTC 急停窗口，或碰撞且无有效前向边距
 *  4. 锁存保持：已 emergency 时须持续清空 emergency_clear_hold_s 才释放
 *  5. Limiting：边距/TTC 进入舒适区 → jerk 二分搜索 speed_limit_m_s
 *
 * 倒车（cmd.linear.x ≤ recovery_min_command）直接返回且不锁存。
 */
AvoidanceController::TiltIntervention AvoidanceController::evaluateTiltIntervention(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear,
  const ObstacleField * tilt_field,
  double dt_s) const
{
  TiltIntervention out;

  // ── 锁存状态机：释放急停锁存并重置计时 ──
  const auto reset_tilt_latch = [this]() {
      tilt_emergency_latched_ = false;
      tilt_emergency_clear_time_s_ = 0.0;
      tilt_emergency_reason_ = EmergencyStopReason::NONE;
    };
  // 若已锁存：release_clear 为 true 时累加清空时间，否则打断计时；未达 hold 则继续 emergency
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

  // 倒车/静止：斜雷达补盲仅服务前向近距，不参与后向
  const double min_command = std::max(0.0, params_.recovery_min_command);
  if (cmd.linear.x <= min_command) {
    reset_tilt_latch();
    return out;
  }

  // 无障碍：尝试释放锁存（障碍离开视场）
  if (tilt_field == nullptr || tilt_field->obstacles.empty()) {
    apply_latched_hold(true);
    return out;
  }

  // ── 轨迹 TTC 与最近前向边距 ──
  const TrajectoryRollout probe = buildTiltProbe(cmd, fb_linear);
  const CollisionCheckResult check = checker_.checkTrajectory(probe, *tilt_field);
  out.source = check.source;
  out.min_ttc_s = check.ttc;

  double nearest_forward_clearance_m = std::numeric_limits<double>::infinity();
  std::string nearest_source;
  for (const auto & obs : tilt_field->obstacles) {
    const double clearance_m = tiltForwardClearance(obs);
    if (!std::isfinite(clearance_m)) {
      continue;
    }
    if (clearance_m < nearest_forward_clearance_m) {
      nearest_forward_clearance_m = clearance_m;
      nearest_source = obs.source;
    }
  }

  // min_distance 优先级：前向净空 > 轨迹碰撞距离 > hard_stop 路径距离
  if (std::isfinite(nearest_forward_clearance_m)) {
    out.min_distance_m = nearest_forward_clearance_m;
  } else if (std::isfinite(check.collision_path_distance_m)) {
    out.min_distance_m = check.collision_path_distance_m;
  } else {
    out.min_distance_m = check.hard_stop_path_distance_m;
  }
  if (out.source.empty()) {
    out.source = nearest_source;
  }

  const double approach_speed = std::max(std::abs(cmd.linear.x), std::abs(fb_linear));
  // 扫掠未给出 TTC 时，用 d/v 近似（仅当速度 ≥ min_speed_for_ttc_m_s）
  if (!std::isfinite(out.min_ttc_s) &&
    std::isfinite(out.min_distance_m) &&
    approach_speed >= std::max(0.0, params_.tilt_ttc.min_speed_for_ttc_m_s))
  {
    out.min_ttc_s = std::max(0.0, out.min_distance_m) / std::max(1e-3, approach_speed);
  }

  // ── Emergency 判据 ──
  const double hard_stop = std::max(0.0, params_.tilt_ttc.hard_stop_distance_m);
  const double nearest_stop_distance = tiltNearestStopDistance(approach_speed, params_);
  const bool ttc_enabled =
    approach_speed >= std::max(0.0, params_.tilt_ttc.min_speed_for_ttc_m_s);
  const bool inside_nearest_stop =
    std::isfinite(out.min_distance_m) && out.min_distance_m <= nearest_stop_distance;
  const bool ttc_emergency_window =
    ttc_enabled &&
    std::isfinite(out.min_ttc_s) &&
    out.min_ttc_s <= params_.tilt_ttc.emergency_ttc_s;

  const auto latch_tilt_emergency = [&]() {
      out.active = true;
      out.limiting = true;
      out.emergency = true;
      out.speed_limit_m_s = 0.0;
      out.reason = EmergencyStopReason::TILT_NEAR_DISTANCE;
      tilt_emergency_latched_ = true;
      tilt_emergency_clear_time_s_ = 0.0;
      tilt_emergency_reason_ = out.reason;
    };

  const bool has_forward_clearance = std::isfinite(nearest_forward_clearance_m);
  // 轨迹碰撞但无法算前向边距（如纯侧向点）→ 保守急停
  const bool collision_without_clearance = check.collision && !has_forward_clearance;
  // TTC 急停须同时满足短 TTC 与距离在 emergency 窗口内
  const bool ttc_emergency_distance =
    ttc_emergency_window &&
    std::isfinite(out.min_distance_m) &&
    out.min_distance_m <= std::max(
    nearest_stop_distance,
    params_.tilt_ttc.distance_emergency_m);

  if (inside_nearest_stop || collision_without_clearance || ttc_emergency_distance) {
    latch_tilt_emergency();
    return out;
  }

  // ── 锁存释放：距离与 TTC 均进入滞回带，且无轨迹碰撞 ──
  const double release_distance = std::max(
    {hard_stop, nearest_stop_distance, params_.tilt_ttc.release_distance_m});
  const bool distance_release_clear =
    !std::isfinite(out.min_distance_m) || out.min_distance_m >= release_distance;
  const bool ttc_release_clear =
    !ttc_enabled ||
    !std::isfinite(out.min_ttc_s) ||
    out.min_ttc_s >= std::max(
    params_.tilt_ttc.emergency_ttc_s,
    params_.tilt_ttc.emergency_ttc_release_s);
  if (apply_latched_hold(distance_release_clear && ttc_release_clear && !check.collision)) {
    return out;
  }

  // ── Limiting 判据：进入舒适减速区但未达 emergency ──
  const bool distance_limited =
    std::isfinite(out.min_distance_m) &&
    out.min_distance_m <= std::max(
    params_.tilt_ttc.limiting_distance_m,
    nearest_stop_distance);
  const bool ttc_limited =
    ttc_enabled &&
    std::isfinite(out.min_ttc_s) &&
    out.min_ttc_s <= params_.tilt_ttc.limiting_ttc_s;
  const bool collision_limited = check.collision && has_forward_clearance;
  if (!distance_limited && !ttc_limited && !collision_limited) {
    return out;
  }

  // ── 限速计算：jerk 制动包络二分搜索 + TTC 上限，取更保守者 ──
  const double comfort_decel = std::max(1e-3, params_.braking_comfort_decel_m_s2);
  const double jerk = std::max(1e-3, params_.braking_max_jerk_m_s3);
  const double reaction_delay = std::max(0.0, params_.braking_reaction_delay_s);
  const double valid_dt_s = std::isfinite(dt_s) ? std::clamp(dt_s, 0.0, 0.10) : 0.0;
  const double envelope_delay = reaction_delay + 4.0 * valid_dt_s;

  double distance_cap = std::numeric_limits<double>::infinity();
  if (std::isfinite(out.min_distance_m)) {
    // 可用制动距离 = 前向边距 − 最近停车边距 − 停车缓冲
    const double usable_distance = std::max(
      0.0,
      out.min_distance_m - nearest_stop_distance - std::max(0.0, params_.braking_stop_buffer_m));
    double lower_speed = 0.0;
    double upper_speed = std::max(std::abs(cmd.linear.x), params_.absolute_max_linear_speed);
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
    distance_cap = lower_speed;
  }

  double ttc_cap = std::numeric_limits<double>::infinity();
  if (ttc_limited && std::isfinite(out.min_distance_m)) {
    ttc_cap = out.min_distance_m / std::max(1e-3, params_.tilt_ttc.limiting_ttc_s);
  }

  const double capped_linear = std::min({distance_cap, ttc_cap, std::abs(cmd.linear.x)});
  if (capped_linear + 1e-6 >= std::abs(cmd.linear.x)) {
    return out;
  }

  out.active = true;
  out.limiting = true;
  out.speed_limit_m_s = capped_linear;
  return out;
}

/**
 * 将斜雷达 TTC 评估结果合并入主路径输出。
 *
 * - limiting：在水平主路径 safe_twist 基础上压低线速度，角速度同比例缩放
 * - emergency：置 risk.emergency，后续 EmergencyBrake 统一硬闸置零
 * - 与水平链路取更保守者：merge 在 evaluate 末尾执行，limiting 用 min 裁剪
 */
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
  // P1 转向保真：限速只压线速度，保留驾驶员航向速率——
  // 斜雷达通道判据是前向净空，转弯避开障碍恰恰依赖转向不被削弱。
  if (params_.shared_control.enabled && params_.shared_control.preserve_steering) {
    result.safe_twist.linear.x = capped_linear;
    return;
  }
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
  double control_dt_s,
  const DirectionBlockMask & direction_mask,
  double dynamic_closing_reserve_m_s) const
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
    fb_linear <= params_.standstill_speed_eps_m_s &&
    prev_output_linear_x <= params_.standstill_speed_eps_m_s &&
    std::abs(fb_linear) <= max_escape_linear + 1e-9 &&
    std::abs(prev_output_linear_x) <= max_escape_linear + 1e-9 &&
    cmd.linear.x < -min_motion_command &&
    std::isfinite(envelope.footprint_check.hard_stop_path_distance_m) &&
    envelope.footprint_check.hard_stop_path_distance_m <= 1e-6 &&
    envelope.footprint_check.hard_stop_x >= 0.0;
  if (reverse_escape_from_front_hard_stop) {
    envelope.controlled_escape_only = true;
  }

  // braking_use_local_map=false 时制动包络不查 LocalMap：制动距离仅由激光 footprint
  // 链决定，规避「探针冲出 6m 地图前边界被误判致命」的恒定限速 bug。Map veto 另路不受影响。
  const bool map_enabled = params_.braking_use_local_map &&
    mode_profile.map_constraint_enabled &&
    map_config_.enabled &&
    map_config_.veto_on_lethal &&
    local_map != nullptr &&
    !local_map->probability.empty();
  if (map_enabled) {
    MapConstraintResult map_result;
    const std::vector<MapExemptCircle> probe_exemptions =
      collectMapExemptCircles(field, probe.poses);
    if (map_checker_.trajectoryLethal(
        *local_map,
        probe.poses,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        map_result,
        probe_exemptions.empty() ? nullptr : &probe_exemptions))
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
          0.02,   // 逃逸时间下限 (s)，非速度门控
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

  const bool forward_motion = cmd.linear.x > 1e-4;
  // L1 定向净空钳制：只有「前向运动会推向物理接触」的近障净空才压缩前向制动距离，
  // 硬停带内的侧向/后方近障不再迫使前向蠕行（旧行为为全向最近净空）。
  const double current_clearance_m = params_.direction_mask_enabled ?
    direction_mask.forward_clearance_m :
    currentInflatedClearance(field);
  envelope.effective_remaining_m = computeEffectiveBrakingRemaining(
    envelope.available_distance_m,
    current_clearance_m,
    params_,
    forward_motion);
  envelope.at_min_stop =
    params_.min_distance_braking_enabled &&
    forward_motion &&
    std::isfinite(envelope.effective_remaining_m) &&
    envelope.effective_remaining_m <= params_.braking_at_stop_distance_eps_m;

  const double discretization =
    0.5 * std::max(0.0, params_.braking_path_step_m);
  const double buffer =
    (params_.min_distance_braking_enabled && params_.braking_exact_stop_enabled) ?
    discretization :
    std::max(0.0, params_.braking_stop_buffer_m) + discretization;
  const double usable_distance = std::max(
    0.0,
    envelope.effective_remaining_m - buffer);

  const double jerk = std::max(1e-3, params_.braking_max_jerk_m_s3);
  const double reaction_delay = std::max(0.0, params_.braking_reaction_delay_s);
  const double envelope_delay =
    reaction_delay +
    4.0 * std::clamp(control_dt_s, 0.0, 0.10);
  const double comfort_decel = std::max(1e-3, params_.braking_comfort_decel_m_s2);
  // P0-A：动态障碍接近速度储备。假设障碍在整个停车过程中仍以 closing_reserve 继续逼近，
  // 额外预留其行进距离 closing_reserve × 停车总时间。停车时间随候选速度单调增，
  // 与 jerkLimitedStoppingDistance 同为速度的单调增函数，二分判据仍成立。
  // 方案 5：静态参数降级为下限，与实测逼近速度估计取 max——障碍逼近更快时
  // 预留自动加大，静态环境不再白留距离（估计未启用时恒等旧行为）。
  const double closing_reserve = std::max(
    std::max(0.0, params_.braking_obstacle_closing_reserve_m_s),
    std::max(0.0, dynamic_closing_reserve_m_s));
  if (envelope.at_min_stop) {
    envelope.speed_limit_m_s = 0.0;
    envelope.active = requested_speed > 1e-6;
  } else {
    double lower_speed = 0.0;
    double upper_speed = std::max(requested_speed, params_.absolute_max_linear_speed);
    for (int i = 0; i < 48; ++i) {
      const double candidate_speed = 0.5 * (lower_speed + upper_speed);
      // 停车总时间近似 = 反应/跟踪延迟 + 匀减速段 v/a（略保守，jerk 建立段已计入距离）。
      const double closing_distance = closing_reserve > 0.0 ?
        closing_reserve * (envelope_delay + candidate_speed / comfort_decel) :
        0.0;
      if (jerkLimitedStoppingDistance(
          candidate_speed, comfort_decel, jerk, envelope_delay) +
        closing_distance <= usable_distance)
      {
        lower_speed = candidate_speed;
      } else {
        upper_speed = candidate_speed;
      }
    }
    envelope.speed_limit_m_s = lower_speed;
    // ── P0 近距指数速度律：s→0 时 v→0 渐近软着陆，消除终端阈值振荡 ──
    // 中远距仍由上方 jerk 包络（二次函数区）主导；进入 near_zone 后与
    // 指数律取 min，保证整体单调、无过冲地收敛到 hard-stop 目标线。
    if (params_.precise_stop.enabled &&
      params_.min_distance_braking_enabled &&
      forward_motion &&
      std::isfinite(envelope.effective_remaining_m) &&
      envelope.effective_remaining_m <= params_.precise_stop.near_zone_m)
    {
      const double s = std::max(
        0.0,
        envelope.effective_remaining_m - params_.precise_stop.stop_tolerance_m);
      const double lambda = std::max(1e-3, params_.precise_stop.exp_lambda_m);
      const double v_exp =
        std::max(0.0, params_.precise_stop.near_speed_scale_m_s) *
        (1.0 - std::exp(-s / lambda));
      envelope.speed_limit_m_s = std::min(envelope.speed_limit_m_s, v_exp);
    }
    envelope.active = envelope.speed_limit_m_s + 1e-6 < requested_speed;
  }

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
  const double emergency_distance_m =
    params_.min_distance_braking_enabled ?
    envelope.effective_remaining_m :
    envelope.available_distance_m;
  envelope.emergency =
    !envelope.controlled_escape_only &&
    !params_.braking_suppress_distance_emergency &&
    current_speed > 1e-3 &&
    envelope.stopping_distance_m > emergency_distance_m + 1e-6;
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
    std::abs(fb_linear) < params_.standstill_speed_eps_m_s &&
    std::abs(prev_output_linear_x) < params_.standstill_speed_eps_m_s;
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

double AvoidanceController::currentInflatedClearance(const ObstacleField & field) const
{
  const FootprintPolygon polygon = footprint_.inflatedPolygon();
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    min_clearance = std::min(
      min_clearance,
      VehicleFootprint::clearanceToPolygon(obs.x, obs.y, obs.radius, polygon));
  }
  return min_clearance;
}

double AvoidanceController::currentPhysicalClearance(const ObstacleField & field) const
{
  // 用物理车体轮廓（非膨胀）度量最近净空，供 P0-B 停稳看护判定「是否即将触碰车体」。
  const FootprintPolygon polygon = footprint_.physicalPolygon();
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    min_clearance = std::min(
      min_clearance,
      VehicleFootprint::clearanceToPolygon(obs.x, obs.y, obs.radius, polygon));
  }
  return min_clearance;
}

double AvoidanceController::currentForwardClearance(
  const ObstacleField & field, double corridor_half_m) const
{
  // L0-EB 前向走廊净空：只取车体前缘之前、横向落在走廊内的障碍到物理车体的最近净空，
  // 避免转弯时侧向横穿的障碍产生假触发。
  const FootprintPolygon polygon = footprint_.physicalPolygon();
  const double front_x = footprint_.physicalRect().x_max;
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    if (obs.x <= front_x) {
      continue;    // 仅前方
    }
    if (std::abs(obs.y) > corridor_half_m) {
      continue;    // 走廊裁剪
    }
    min_clearance = std::min(
      min_clearance,
      VehicleFootprint::clearanceToPolygon(obs.x, obs.y, obs.radius, polygon));
  }
  return min_clearance;
}

double AvoidanceController::frontCorridorClearance(
  const ObstacleField & field, double corridor_margin_m) const
{
  const double corridor_half =
    footprint_.physicalRect().y_max + std::max(0.0, corridor_margin_m);
  return currentForwardClearance(field, corridor_half);
}

double AvoidanceController::smoothBrakingSpeedLimit(
  double raw_limit_m_s,
  double dt_s) const
{
  if (!std::isfinite(raw_limit_m_s)) {
    has_smoothed_braking_speed_limit_ = false;
    return raw_limit_m_s;
  }

  if (params_.braking_no_forward_reaccel_at_hard_stop && hard_stop_reached_) {
    raw_limit_m_s = 0.0;
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
    std::abs(fb_linear) < params_.standstill_speed_eps_m_s &&
    std::abs(prev_output_linear_x) < params_.standstill_speed_eps_m_s;
  const geometry_msgs::msg::Twist & direction_cmd =
    (std::abs(output_cmd.linear.x) >= min_command ||
    std::abs(output_cmd.angular.z) >= min_command) ?
    output_cmd :
    cmd;
  context.direction_command_active =
    std::abs(direction_cmd.linear.x) >= min_command ||
    std::abs(direction_cmd.angular.z) >= min_command;
  context.forward_approach = direction_cmd.linear.x > min_command;
  if (!context.vehicle_stopped || !context.direction_command_active) {
    return context;
  }

  VelocityCandidate dir_cmd;
  dir_cmd.v = direction_cmd.linear.x;
  // 方向净空的旋转分量钳到 recovery_max_angular_speed:命令角速度随摇杆无界,
  // 否则原地大转向的 rollout 过度旋转、角点扫向近障，方向净空恒被压小、硬闸永不释放。
  dir_cmd.w = std::clamp(
    direction_cmd.angular.z,
    -std::abs(params_.recovery_max_angular_speed),
    std::abs(params_.recovery_max_angular_speed));
  const TrajectoryRollout dir_rollout =
    rollout_.rolloutCandidate(dir_cmd, fb_linear, fb_angular);
  // 旋转主导时,威胁窗口截断到累计航向达 direction_mask_probe_angle_rad 为止——与
  // direction mask 的有界角探针取齐,使「掩码判旋转安全」与「硬闸释放」结论一致;
  // 线速主导保持原时程行为不变。
  std::vector<PredictedPose> dir_poses = dir_rollout.poses;
  const double probe_angle = std::max(0.0, params_.direction_mask_probe_angle_rad);
  const bool rotation_dominant =
    std::abs(dir_cmd.w) > 1e-3 &&
    std::abs(dir_cmd.v) <= std::max(0.0, params_.recovery_min_command);
  if (rotation_dominant && probe_angle > 0.0) {
    for (size_t i = 1; i < dir_poses.size(); ++i) {
      if (std::abs(dir_poses[i].theta) > probe_angle) {
        dir_poses.resize(i + 1);
        break;
      }
    }
  }
  const PredictedPose future_pose =
    (rotation_dominant && !dir_poses.empty()) ?
    dir_poses.back() :
    poseAtHorizon(
      dir_rollout,
      std::min(params_.recovery_lookahead_s, params_.rollout_horizon_s));
  const FootprintPolygon clearance_polygon = footprint_.inflatedPolygon();
  const bool directional = params_.direction_mask_enabled;
  const FootprintPolygon physical_polygon =
    directional ? footprint_.physicalPolygon() : FootprintPolygon{};
  double future_clearance = std::numeric_limits<double>::infinity();
  for (const auto & obs : field.obstacles) {
    if (!obs.valid) {
      continue;
    }
    // L1 方向解耦：命令方向不将其推向物理接触的近障不参与方向净空，
    // 否则贴障停稳后（如前障 0.10 m）原地转向会被无关近障压住方向净空而误触发硬闸。
    if (directional &&
      direction_mask::isNearObstacle(obs, clearance_polygon, params_.hard_stop_distance_m) &&
      !direction_mask::obstacleThreatenedAlongPoses(
        obs, dir_poses, physical_polygon,
        params_.direction_mask_contact_buffer_m,
        params_.direction_mask_approach_eps_m))
    {
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
  // L1：全部近障被方向豁免（future_clearance 保持 +inf）时同样视为有效方向净空，
  // 使 isDirectionalEscapeViable / 释放判据按「无威胁」处理而非退回全向 min_clearance。
  context.directional_clearance_valid = std::isfinite(future_clearance) || directional;
  return context;
}

std::vector<MapExemptCircle> AvoidanceController::collectMapExemptCircles(
  const ObstacleField & field,
  const std::vector<PredictedPose> & poses) const
{
  std::vector<MapExemptCircle> circles;
  fillMapExemptCircles(circles, field, poses);
  return circles;
}

void AvoidanceController::fillMapExemptCircles(
  std::vector<MapExemptCircle> & circles,
  const ObstacleField & field,
  const std::vector<PredictedPose> & poses) const
{
  circles.clear();
  if (!params_.direction_mask_enabled || poses.empty()) {
    return;
  }
  const FootprintPolygon physical_polygon = footprint_.physicalPolygon();

  // ── 近障子集缓存：isNearObstacle 仅依赖 field+footprint，与 poses 无关。
  // 单次 evaluate() 周期内 7 处调用复用同一份近障子集，避免重复全量遍历。
  if (!near_obstacle_cache_valid_) {
    const FootprintPolygon inflated_polygon = footprint_.inflatedPolygon();
    near_obstacle_cache_.clear();
    near_obstacle_cache_.reserve(field.obstacles.size());
    for (const auto & obs : field.obstacles) {
      if (direction_mask::isNearObstacle(
          obs, inflated_polygon, params_.hard_stop_distance_m))
      {
        near_obstacle_cache_.push_back(&obs);
      }
    }
    near_obstacle_cache_valid_ = true;
  }

  for (const auto * obs_ptr : near_obstacle_cache_) {
    const auto & obs = *obs_ptr;
    if (direction_mask::obstacleThreatenedAlongPoses(
        obs, poses, physical_polygon,
        params_.direction_mask_contact_buffer_m,
        params_.direction_mask_approach_eps_m))
    {
      continue;
    }
    circles.push_back(
      MapExemptCircle{
        obs.x, obs.y,
        std::max(0.0, obs.radius) + params_.direction_mask_map_exempt_radius_m});
  }
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
    std::max(0.02, max_escape_time_s + grid_clearance_time_s),  // 逃逸时间下限 (s)，非速度门控
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

  // L1：脱困进度度量改用符号净空（外正内负、跨界单调）。
  // 旧 V 形 clearanceToPolygon 在障碍穿越轮廓边界时先降后升，
  // 浅侵入（贴边）情形下真实逃逸命令会因「瞬时净空损失」被误拒；
  // 符号净空对逃逸单调递增、对加深单调递减，方向归因正确。
  const bool directional = params_.direction_mask_enabled;
  const auto clearance_metric = [&](double lx, double ly, double radius) {
      return directional ?
             VehicleFootprint::signedClearanceToPolygon(lx, ly, radius, clearance_polygon) :
             VehicleFootprint::clearanceToPolygon(lx, ly, radius, clearance_polygon);
    };
  const auto pose_clearance = [&](const PredictedPose & pose) {
      double min_clearance = std::numeric_limits<double>::infinity();
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
        min_clearance = std::min(
          min_clearance,
          clearance_metric(lx, ly, obs.radius));
      }
      return min_clearance;
    };

  // 基线：directional 用符号净空在当前位姿重算（与逐位姿度量同尺度），否则沿用 V 形值
  const double baseline_clearance = directional ?
    pose_clearance(PredictedPose{}) :
    intrusion.min_clearance;

  double previous_clearance = baseline_clearance;
  for (const auto & pose : recovery_rollout.poses) {
    if (pose.t > params_.recovery_lookahead_s + 1e-9) {
      break;
    }
    const double pose_min_clearance = pose_clearance(pose);
    if (!std::isfinite(pose_min_clearance) ||
      pose_min_clearance + allowed_transient_loss < previous_clearance)
    {
      return evaluated;
    }
    previous_clearance = pose_min_clearance;
  }

  const double future_min_clearance = pose_clearance(future_pose);

  const double required_improvement =
    std::max(0.0, params_.recovery_min_clearance_improvement_m);
  if (!std::isfinite(future_min_clearance) ||
    future_min_clearance <= baseline_clearance + required_improvement)
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
    const std::vector<MapExemptCircle> recovery_exemptions =
      collectMapExemptCircles(field, recovery_rollout.poses);
    if (map_checker_.trajectoryLethal(
        *local_map,
        recovery_rollout.poses,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        map_result,
        recovery_exemptions.empty() ? nullptr : &recovery_exemptions))
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
            0.02,   // 逃逸时间下限 (s)，非速度门控
            params_.recovery_lookahead_s + grid_clearance_time_s),
          escape_result))
      {
        return evaluated;
      }
    }
  }

  evaluated.valid = true;
  evaluated.future_clearance = future_min_clearance;
  evaluated.clearance_gain = future_min_clearance - baseline_clearance;
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

bool AvoidanceController::directionalEscapeTwist(
  const geometry_msgs::msg::Twist & requested_cmd,
  const DirectionBlockMask & mask,
  geometry_msgs::msg::Twist & escape) const
{
  if (!params_.direction_mask_enabled) {
    return false;
  }
  const double min_command = std::max(0.0, params_.recovery_min_command);
  const double max_linear = std::min(
    std::abs(params_.recovery_max_linear_speed),
    std::abs(params_.absolute_max_linear_speed));
  const double max_angular = std::min(
    std::abs(params_.recovery_max_angular_speed),
    std::abs(params_.absolute_max_angular_speed));

  // 按轴施加方向掩码：命令方向被判「将近障推向物理接触」时清零，只留放行分量。
  double vx = requested_cmd.linear.x;
  double wz = requested_cmd.angular.z;
  if ((vx > 0.0 && mask.forward) || (vx < 0.0 && mask.backward)) {
    vx = 0.0;
  }
  if ((wz > 0.0 && mask.left) || (wz < 0.0 && mask.right)) {
    wz = 0.0;
  }
  vx = std::clamp(vx, -max_linear, max_linear);
  wz = std::clamp(wz, -max_angular, max_angular);
  if (std::abs(vx) < min_command && std::abs(wz) < min_command) {
    return false;
  }
  escape = geometry_msgs::msg::Twist{};
  escape.linear.x = vx;
  escape.angular.z = wz;
  return true;
}

bool AvoidanceController::tryPinnedEscape(
  const geometry_msgs::msg::Twist & requested_cmd,
  double fb_linear,
  double fb_angular,
  const ObstacleField & field,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile,
  double prev_output_linear_x,
  double dt_s,
  geometry_msgs::msg::Twist & escape,
  double & clearance_gain) const
{
  const PinnedEscapeParams & pe = params_.pinned_escape;
  if (!pe.enabled) {
    return false;
  }

  const double min_command = std::max(0.0, params_.recovery_min_command);
  // 前置门控：车速须低于「脱困蠕行速度」量级，且驾驶员确实在给令（否则重置去抖并放弃，
  // 回退硬停）。门控放宽到蠕行速度而非严格零，使脱困启动后能逐帧续动、不至于「动一帧停一帧」；
  // 同时仍排除正常行驶速度（>蠕行）——那属于在进入夹死点前的减速阶段，等减到蠕行量级再计时。
  const double speed_eps = std::max(0.0, pe.standstill_speed_eps_m_s);
  const double linear_gate = std::max(speed_eps, pe.creep_linear_m_s + speed_eps);
  const double angular_gate =
    std::max(std::max(speed_eps, 0.05), pe.creep_angular_rad_s + speed_eps);
  const bool near_standstill =
    std::abs(fb_linear) <= linear_gate &&
    std::abs(fb_angular) <= angular_gate &&
    std::abs(prev_output_linear_x) <= linear_gate;
  const bool driver_commanding =
    std::abs(requested_cmd.linear.x) >= min_command ||
    std::abs(requested_cmd.angular.z) >= min_command;
  if (!near_standstill || !driver_commanding) {
    pinned_escape_confirm_time_s_ = 0.0;
    return false;
  }

  // 去抖：连续处于夹死死状态达 confirm_s 才尝试脱困。
  const double valid_dt_s = (std::isfinite(dt_s) && dt_s > 0.0) ? dt_s : 0.0;
  pinned_escape_confirm_time_s_ += valid_dt_s;
  if (pinned_escape_confirm_time_s_ + 1e-9 < std::max(0.0, pe.confirm_s)) {
    return false;
  }

  // LocalMap 是本层的授权闸；地图不可用则不放行（回退硬停）。
  const bool map_enabled = mode_profile.map_constraint_enabled &&
    map_config_.enabled && map_config_.veto_on_lethal &&
    local_map != nullptr && !local_map->probability.empty();
  if (!map_enabled) {
    return false;
  }

  const double escape_margin = std::max(params_.hard_stop_distance_m, pe.escape_margin_m);
  const double baseline_clearance = currentInflatedClearance(field);
  const double required_gain = std::max(0.0, pe.min_clearance_gain_m);

  const double v = std::max(0.0, pe.creep_linear_m_s);
  const double w = std::max(0.0, pe.creep_angular_rad_s);
  // 低速微动作原语集：±前后、±原地转、±前后弧。每个都用现有闸复验。
  const VelocityCandidate primitives[] = {
    {+v, 0.0}, {-v, 0.0}, {0.0, +w}, {0.0, -w},
    {+v, +w}, {+v, -w}, {-v, +w}, {-v, -w},
  };

  bool found = false;
  VelocityCandidate best_cmd{};
  double best_clearance = -std::numeric_limits<double>::infinity();
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & cand : primitives) {
    if (std::abs(cand.v) < 1e-9 && std::abs(cand.w) < 1e-9) {
      continue;
    }
    const TrajectoryRollout rollout =
      rollout_.rolloutCandidate(cand, fb_linear, fb_angular);
    // ① 当前帧 Footprint 扫掠：无碰撞且全程净空不低于地板。
    const CollisionCheckResult check = checker_.checkTrajectory(rollout, field);
    if (check.collision) {
      continue;
    }
    if (!std::isfinite(check.min_distance) || check.min_distance < escape_margin) {
      continue;
    }
    // ② 严格改善：微动作须使最近净空相对当前变大，保证脱困有进展（近障场景才约束）。
    if (std::isfinite(baseline_clearance) &&
      check.min_distance < baseline_clearance + required_gain)
    {
      continue;
    }
    // ③ LocalMap veto 复验（同一把闸）：整条扫掠必须无 lethal（严于常规候选，不用逃逸放宽）。
    MapConstraintResult map_result;
    const std::vector<MapExemptCircle> exemptions =
      collectMapExemptCircles(field, rollout.poses);
    if (map_checker_.trajectoryLethal(
        *local_map,
        rollout.poses,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        map_result,
        exemptions.empty() ? nullptr : &exemptions))
    {
      continue;
    }
    // 通过：择净空最大者，同净空择最贴驾驶员意图者。
    const double cost = commandErrorCost(
      cand, requested_cmd, params_.safety_filter_angular_weight);
    if (!found ||
      check.min_distance > best_clearance + 1e-6 ||
      (std::abs(check.min_distance - best_clearance) <= 1e-6 && cost < best_cost))
    {
      found = true;
      best_cmd = cand;
      best_clearance = check.min_distance;
      best_cost = cost;
    }
  }

  if (!found) {
    return false;
  }

  const double max_linear = std::min(
    std::abs(params_.recovery_max_linear_speed),
    std::abs(params_.absolute_max_linear_speed));
  const double max_angular = std::min(
    std::abs(params_.recovery_max_angular_speed),
    std::abs(params_.absolute_max_angular_speed));
  escape = geometry_msgs::msg::Twist{};
  escape.linear.x = std::clamp(best_cmd.v, -max_linear, max_linear);
  escape.angular.z = std::clamp(best_cmd.w, -max_angular, max_angular);
  clearance_gain =
    std::isfinite(baseline_clearance) ? (best_clearance - baseline_clearance) : 0.0;
  return true;
}

void AvoidanceController::resetPinnedEscape() const
{
  pinned_escape_confirm_time_s_ = 0.0;
}

bool AvoidanceController::tryRotationEscape(
  const geometry_msgs::msg::Twist & requested_cmd,
  const ObstacleField & field,
  const ObstacleField * tilt_field,
  const RollingLocalMapGrid * local_map,
  const ModeProfile & mode_profile,
  geometry_msgs::msg::Twist & escape) const
{
  const RotationEscapeParams & re = params_.rotation_escape;
  if (!re.enabled) {
    return false;
  }

  // ── 仅纯原地转向意图：线速度分量近零、角速度显著（|w| ≥ 门控）──
  const double min_command = std::max(0.0, params_.recovery_min_command);
  if (std::abs(requested_cmd.linear.x) > min_command) {
    return false;
  }
  const double w_cmd = requested_cmd.angular.z;
  const double angular_gate = std::max(min_command, std::max(0.0, re.min_angular_rad_s));
  if (std::abs(w_cmd) < angular_gate) {
    return false;
  }

  // ── LocalMap 是本层授权闸：地图不可用则不放行（回退硬停），与 pinned_escape 一致 ──
  const bool map_enabled = mode_profile.map_constraint_enabled &&
    map_config_.enabled && map_config_.veto_on_lethal &&
    local_map != nullptr && !local_map->probability.empty();
  if (!map_enabled) {
    return false;
  }

  // ── 沿实际转向方向的有界角探针（复用 direction_mask 同款生成器，使用 rotation_escape 独立步长）──
  const double sign = w_cmd >= 0.0 ? 1.0 : -1.0;
  const double probe_step = re.probe_step_rad > 0.0 ?
    re.probe_step_rad : params_.direction_mask_probe_step_rad;
  const std::vector<PredictedPose> probe = direction_mask::angularProbePoses(
    sign, re.probe_angle_rad, probe_step);
  if (probe.size() < 2) {
    return false;
  }

  // ① 当前帧 Footprint 扫掠（复用 direction_mask 威胁谓词）：转向不得把任一近障推向物理
  //    接触。「不恶化」语义——非趋近即豁免，已在接触带内也允许维持（不要求净空改善）。
  //    水平主障场与斜雷达体素障碍场（tilt_field，独立于 field）都要检，避免转向扫入
  //    仅斜雷达可见的前侧近障。
  const FootprintPolygon physical_polygon = footprint_.physicalPolygon();
  const FootprintPolygon inflated_polygon = footprint_.inflatedPolygon();
  const auto rotationThreatensField =
    [&](const ObstacleField & obstacle_field) -> bool {
      for (const auto & obs : obstacle_field.obstacles) {
        if (!direction_mask::isNearObstacle(
            obs, inflated_polygon, params_.hard_stop_distance_m))
        {
          continue;
        }
        if (direction_mask::obstacleThreatenedAlongPoses(
            obs, probe, physical_polygon,
            params_.direction_mask_contact_buffer_m,
            params_.direction_mask_approach_eps_m))
        {
          return true;    // 该转向把某近障推向接触 → 恶化
        }
      }
      return false;
    };
  if (rotationThreatensField(field)) {
    return false;
  }
  if (tilt_field != nullptr && rotationThreatensField(*tilt_field)) {
    return false;
  }

  // ② LocalMap 复验：转向不得踏入任何「起始姿势未占用」的新 lethal 栅格（不恶化）。
  MapConstraintResult map_result;
  if (!map_checker_.trajectoryNoNewLethal(
      *local_map, probe,
      mode_profile.lethal_threshold,
      map_config_.check_footprint_corners,
      map_result))
  {
    return false;    // 转向踏入新 lethal → 恶化,不放行
  }

  // 通过：受控低速原地转向脱困（线速度恒零，钳到脱困角速度上限）。
  const double max_angular = std::min(
    std::abs(re.max_angular_rad_s),
    std::abs(params_.absolute_max_angular_speed));
  escape = geometry_msgs::msg::Twist{};
  escape.angular.z = std::clamp(w_cmd, -max_angular, max_angular);
  return std::abs(escape.angular.z) > 1e-9;
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

/** 清除斜雷达急停锁存及关联计时/原因码 */
void AvoidanceController::resetTiltEmergencyState() const
{
  tilt_emergency_latched_ = false;
  tilt_emergency_clear_time_s_ = 0.0;
  tilt_emergency_reason_ = EmergencyStopReason::NONE;
}

void AvoidanceController::resetForwardStopHold() const
{
  forward_stop_hold_latched_ = false;
  forward_stop_hold_confirm_time_s_ = 0.0;
}

void AvoidanceController::resetStandstillGuard() const
{
  standstill_guard_latched_ = false;
  standstill_guard_confirm_time_s_ = 0.0;
  // 使用 NaN 而非 +infinity 表示「无历史净空数据」。
  // std::isfinite(NaN)=false，与现有守卫配合使首帧 floor_breach / approach
  // 不触发。NaN 语义比 infinity 更清晰——后者可能被误读为「净空无限大」。
  standstill_prev_physical_clearance_m_ =
    std::numeric_limits<double>::quiet_NaN();
}

/** 清除 P1 接管系数平滑状态与 P0 脉冲状态（模式切换/参数更新/透传时调用） */
void AvoidanceController::resetSharedControlState() const
{
  smoothed_authority_alpha_ = 0.0;
  has_authority_alpha_ = false;
  pulse_comp_.reset();
}

double AvoidanceController::updateAuthorityAlpha(
  const CollisionCheckResult & driver_check,
  double dt_s) const
{
  const SharedControlParams & sc = params_.shared_control;

  // TTC 通道：仅在驾驶员轨迹实际碰撞（ttc 有限）时贡献 α
  double alpha_ttc = 0.0;
  const double ttc_zero = std::max(sc.alpha_ttc_zero_s, sc.alpha_ttc_full_s + 1e-3);
  if (std::isfinite(driver_check.ttc)) {
    alpha_ttc = std::clamp(
      (ttc_zero - driver_check.ttc) / (ttc_zero - sc.alpha_ttc_full_s),
      0.0, 1.0);
  }

  // 净空通道：轨迹全程最小净空归一化
  double alpha_clearance = 0.0;
  const double clr_zero = std::max(
    sc.alpha_clearance_zero_m, sc.alpha_clearance_full_m + 1e-3);
  if (std::isfinite(driver_check.min_distance)) {
    alpha_clearance = std::clamp(
      (clr_zero - driver_check.min_distance) /
      (clr_zero - sc.alpha_clearance_full_m),
      0.0, 1.0);
  }

  const double raw = std::max(alpha_ttc, alpha_clearance);

  // 上升瞬时（安全响应不延迟），回落限速形成滞回，抑制临界点附近抖动
  const double dt = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;
  if (!has_authority_alpha_ || raw >= smoothed_authority_alpha_) {
    smoothed_authority_alpha_ = raw;
  } else {
    smoothed_authority_alpha_ = std::max(
      raw,
      smoothed_authority_alpha_ -
      std::max(0.0, sc.alpha_release_rate_per_s) * dt);
  }
  has_authority_alpha_ = true;
  return smoothed_authority_alpha_;
}

bool AvoidanceController::updateForwardStopHold(
  const BrakingEnvelope & envelope,
  const geometry_msgs::msg::Twist & requested_cmd,
  double fb_linear,
  double prev_output_linear_x,
  bool recovery_active,
  bool emergency_brake,
  double dt_s) const
{
  if (!params_.forward_stop_hold_enabled ||
    !params_.min_distance_braking_enabled ||
    recovery_active ||
    emergency_brake)
  {
    resetForwardStopHold();
    return false;
  }

  const double min_command = std::max(0.0, params_.recovery_min_command);
  if (requested_cmd.linear.x < -min_command) {
    resetForwardStopHold();
    return false;
  }

  const bool release_by_distance =
    std::isfinite(envelope.effective_remaining_m) &&
    envelope.effective_remaining_m >= std::max(
    params_.forward_stop_hold_release_distance_m,
    params_.hard_stop_release_distance_m);
  if (!envelope.active || envelope.controlled_escape_only || release_by_distance) {
    resetForwardStopHold();
    return false;
  }

  const bool forward_request = requested_cmd.linear.x > min_command;
  if (!forward_request) {
    forward_stop_hold_confirm_time_s_ = 0.0;
    return false;
  }

  const double stopped_threshold =
    std::max(0.0, params_.forward_stop_hold_feedback_speed_m_s);
  const bool vehicle_stopped =
    std::abs(fb_linear) <= stopped_threshold &&
    std::abs(prev_output_linear_x) <= stopped_threshold;
  const bool low_speed_limit =
    std::isfinite(envelope.speed_limit_m_s) &&
    envelope.speed_limit_m_s <= std::max(0.0, params_.forward_stop_hold_speed_limit_m_s);
  const bool near_stop_distance =
    std::isfinite(envelope.effective_remaining_m) &&
    envelope.effective_remaining_m <= std::max(0.0, params_.forward_stop_hold_remaining_m);

  if (!forward_stop_hold_latched_) {
    if (!vehicle_stopped || (!low_speed_limit && !near_stop_distance)) {
      forward_stop_hold_confirm_time_s_ = 0.0;
      return false;
    }

    forward_stop_hold_confirm_time_s_ += std::max(
      0.0,
      std::isfinite(dt_s) ? dt_s : 0.0);
    if (forward_stop_hold_confirm_time_s_ + 1e-9 <
      std::max(0.0, params_.forward_stop_hold_confirm_s))
    {
      return false;
    }
    forward_stop_hold_latched_ = true;
  }

  return true;
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
  // ── 近障子集缓存失效：每个 evaluate() 周期开始时标记未初始化，
  // 首次调用 collectMapExemptCircles 时按本周期 field 重建。──
  near_obstacle_cache_valid_ = false;
  AvoidanceResult result;
  result.safe_twist = cmd;
  result.hybrid_log.mode = context.safety_mode;

  // ── 方案 1 步骤 A：影子仲裁（纯观测，不改变任何输出）──
  // 各安全层在原地生效的同时旁路记录 LayerVerdict；主路径结束前用
  // SafetyArbiter 复算输出并与真实输出比对，不一致计入 hybrid_log。
  // ESTOP / 透传早退路径跳过（无层间交互，无比对价值）。
  std::vector<LayerVerdict> shadow_verdicts;
  const bool shadow_enabled = params_.arbiter_shadow_enabled;
  if (shadow_enabled) {
    shadow_verdicts.reserve(16);   // 安全层枚举约 10+，预分配避免热路径多次扩容
  }
  auto record_verdict = [&](const LayerVerdict & verdict) {
      if (shadow_enabled) {
        shadow_verdicts.push_back(verdict);
      }
    };
  auto run_arbiter_shadow =
    [&](AvoidanceResult & res, const geometry_msgs::msg::Twist & base_cmd) {
      if (!shadow_enabled) {
        return;
      }
      const ArbitrationResult arb =
        safety_arbiter::arbitrate(base_cmd, shadow_verdicts);
      const bool match =
        std::abs(arb.twist.linear.x - res.safe_twist.linear.x) < 1e-6 &&
        std::abs(arb.twist.angular.z - res.safe_twist.angular.z) < 1e-6;
      res.hybrid_log.arbiter_shadow_mismatch = !match;
    };

  if (context.safety_mode == SafetyMode::ESTOP) {
    emergency_brake_.latch();
    resetCandidateSpeedUpHold();
    braking_envelope_active_time_s_ = 0.0;
    resetRecoveryState();
    resetTiltEmergencyState();
    resetForwardStopHold();
    resetStandstillGuard();
    resetSharedControlState();
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
    sudden_detector_.reset();
    resetCandidateSpeedUpHold();
    braking_envelope_active_time_s_ = 0.0;
    resetRecoveryState();
    resetTiltEmergencyState();
    resetForwardStopHold();
    resetStandstillGuard();
    resetSharedControlState();
    result.safe_twist.linear.x = std::clamp(
      cmd.linear.x,
      -params_.absolute_max_linear_speed,
      params_.absolute_max_linear_speed);
    result.safe_twist.angular.z = std::clamp(
      cmd.angular.z,
      -params_.absolute_max_angular_speed,
      params_.absolute_max_angular_speed);
    // 透传模式同样约束转弯合成速度（外缘/横加），直线与原地旋转不受影响。
    result.hybrid_log.turn_envelope_scale = applyTurnSpeedEnvelope(
      result.safe_twist.linear.x, result.safe_twist.angular.z, params_);
    // 透传模式同样施加急速转向线速度硬帽（只压 v，不改转向意图）。
    result.hybrid_log.rapid_steer_cap_active = applyRapidSteerLinearCap(
      result.safe_twist.linear.x, result.safe_twist.angular.z, params_);
    const auto t1 = std::chrono::steady_clock::now();
    result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
  }

  geometry_msgs::msg::Twist eval_cmd = cmd;
  const geometry_msgs::msg::Twist requested_cmd = cmd;
  if (context.cruise_active && context.cruise_linear_cap > 0.0 && eval_cmd.linear.x > 0.01) {
    eval_cmd.linear.x = std::min(eval_cmd.linear.x, context.cruise_linear_cap);
  }
  // ── 斜雷达前向近障软限速：前向走廊探到障碍即钳前向速度上限（见 front_tilt_speed_cap.h）。
  //    与 cruise_linear_cap 同款仅压正向 eval_cmd（含巡航保持速度），不改转向/后退；
  //    +inf 时分支不进入，输出等价旧行为。requested_cmd 保持原始不受影响。
  if (std::isfinite(context.front_tilt_speed_cap_m_s) && eval_cmd.linear.x > 0.0) {
    eval_cmd.linear.x = std::min(eval_cmd.linear.x, context.front_tilt_speed_cap_m_s);
  }
  // ── 转弯合成速度包络：在入口整体等比降速（保持转弯半径），使制动包络、候选
  // 采样、rollout 全链路评估的是限幅后的真实命令，而非合成无界的原始摇杆。──
  result.hybrid_log.turn_envelope_scale = applyTurnSpeedEnvelope(
    eval_cmd.linear.x, eval_cmd.angular.z, params_);
  // ── 急速转向线速度硬帽：紧接包络之后、进入评估链路之前压低 v，使制动包络、候选采样、
  // rollout 全链路评估的都是「急转已降速」后的命令（bag speed_test_tilt_on_2 病根）。──
  // 影子仲裁（方案 1）暂不为本层记 verdict：SafetyLayerId 枚举序即优先级且按 COUNT 定长，
  // 复用 TURN_ENVELOPE 会一 tick 内重复记录并错标层次。观测经 hybrid_log.rapid_steer_cap_active，
  // 待 PRODUCTION_HARDENING_PLAN 步骤 B「限速层 verdict 即真源」时再补独立枚举值。
  result.hybrid_log.rapid_steer_cap_active = applyRapidSteerLinearCap(
    eval_cmd.linear.x, eval_cmd.angular.z, params_);
  if (shadow_enabled && result.hybrid_log.turn_envelope_scale < 1.0) {
    LayerVerdict verdict;
    verdict.id = SafetyLayerId::TURN_ENVELOPE;
    verdict.active = true;
    verdict.forward_cap = std::abs(eval_cmd.linear.x);
    verdict.backward_cap = std::abs(eval_cmd.linear.x);
    verdict.left_cap = std::abs(eval_cmd.angular.z);
    verdict.right_cap = std::abs(eval_cmd.angular.z);
    record_verdict(verdict);
  }

  // ── 方案 4 传感器降级矩阵：分轴封锁 + 全局限速（与 direction_mask 同款语义）──
  // 失效侧盲区不再被当作自由空间跑全速；封锁/限速由节点层按 per-source 新鲜度
  // 与 degradation: 配置合成后经 context 传入。全局 INPUT_FAULT 兜底仍在节点层。
  if (context.degradation_active) {
    if (context.degradation_block_forward && eval_cmd.linear.x > 0.0) {
      eval_cmd.linear.x = 0.0;
    }
    if (context.degradation_block_backward && eval_cmd.linear.x < 0.0) {
      eval_cmd.linear.x = 0.0;
    }
    if (context.degradation_block_left && eval_cmd.angular.z > 0.0) {
      eval_cmd.angular.z = 0.0;
    }
    if (context.degradation_block_right && eval_cmd.angular.z < 0.0) {
      eval_cmd.angular.z = 0.0;
    }
    if (std::isfinite(context.degradation_speed_cap_m_s) &&
      context.degradation_speed_cap_m_s >= 0.0 &&
      std::abs(eval_cmd.linear.x) > context.degradation_speed_cap_m_s)
    {
      eval_cmd.linear.x =
        std::copysign(context.degradation_speed_cap_m_s, eval_cmd.linear.x);
    }
    result.hybrid_log.degradation_active = true;
    LayerVerdict verdict;
    verdict.id = SafetyLayerId::DEGRADATION;
    verdict.active = true;
    verdict.stop_axes.forward = context.degradation_block_forward;
    verdict.stop_axes.backward = context.degradation_block_backward;
    verdict.stop_axes.left = context.degradation_block_left;
    verdict.stop_axes.right = context.degradation_block_right;
    verdict.forward_cap = context.degradation_speed_cap_m_s;
    verdict.backward_cap = context.degradation_speed_cap_m_s;
    record_verdict(verdict);
  }

  FootprintIntrusion intrusion = analyzeFootprintIntrusion(field);
  enrichProximityBlock(intrusion, field, fb_linear, prev_output_linear_x);
  const double min_motion_command = std::max(0.0, params_.recovery_min_command);

  // ── L1 方向掩码：近障按轴封锁，仅裁剪受威胁分量（见 direction_mask.h）──
  // 前障+前进左转 → 只清前进保留左转；左障+前进左转 → 只清左转保留前进；
  // 后障+后退右转 → 只清后退保留右转。被裁剪后的 eval_cmd 进入后续所有链路
  //（制动包络、候选采样、停稳看护逃逸意图判定）。
  DirectionBlockMask direction_mask;
  if (params_.direction_mask_enabled) {
    direction_mask = direction_mask::computeDirectionBlockMask(
      field, footprint_.physicalPolygon(), footprint_.inflatedPolygon(), params_);
    if (direction_mask.forward && eval_cmd.linear.x > 0.0) {
      eval_cmd.linear.x = 0.0;
    }
    if (direction_mask.backward && eval_cmd.linear.x < 0.0) {
      eval_cmd.linear.x = 0.0;
    }
    if (direction_mask.left && eval_cmd.angular.z > 0.0) {
      eval_cmd.angular.z = 0.0;
    }
    if (direction_mask.right && eval_cmd.angular.z < 0.0) {
      eval_cmd.angular.z = 0.0;
    }
    result.hybrid_log.direction_mask_forward = direction_mask.forward;
    result.hybrid_log.direction_mask_backward = direction_mask.backward;
    result.hybrid_log.direction_mask_left = direction_mask.left;
    result.hybrid_log.direction_mask_right = direction_mask.right;
    if (shadow_enabled && direction_mask.any()) {
      LayerVerdict verdict;
      verdict.id = SafetyLayerId::DIRECTION_MASK;
      verdict.active = true;
      verdict.stop_axes.forward = direction_mask.forward;
      verdict.stop_axes.backward = direction_mask.backward;
      verdict.stop_axes.left = direction_mask.left;
      verdict.stop_axes.right = direction_mask.right;
      record_verdict(verdict);
    }
  }

  // ── 转弯净空护栏（turn clearance guard，见 avoidance_types.h）──
  // 方向解耦令前向制动包络对侧向近障免疫；转弯时旋转车体会侧扫入前侧近障，前向净空
  // 与 ±v/±w 探针都漏判（bag「转弯加速撞车」：1.3 m/s 大转弯撞前侧障，仅 map veto
  // 在接触时才响应）。此处仅在 |w| 显著时，取沿实际 (v,w) 曲线扫掠 Footprint 的最近
  // 净空——SweptFootprintChecker 已含近障方向豁免，弧线掠离的障碍自动豁免、掠向的
  // 如实计入——按可停净空钳制线速度、角速度同比缩放（保持转弯半径），使既有 map veto
  // / 制动兜底来得及。|w|<门控（直线/微转）完全不介入，不改变既有直线行为。
  {
    double turn_guard_cap = std::numeric_limits<double>::infinity();
    // 方案 2.1 门控升级：min_curvature_1_m > 0 时按曲率 |w|/max(|v|,0.1) 门控
    //（转弯半径足够小才介入）——低速大转弯（tilt_on_2）比固定角速度门控介入更
    // 及时，直行高速不受影响；=0 回退旧的固定角速度门控（回滚开关）。
    const double curvature_gate = params_.turn_clearance_guard_min_curvature_1_m;
    const bool turning_gate = curvature_gate > 0.0 ?
      (std::abs(eval_cmd.angular.z) >= 0.1 &&
      std::abs(eval_cmd.angular.z) /
      std::max(std::abs(eval_cmd.linear.x), 0.1) >= curvature_gate) :
      (std::abs(eval_cmd.angular.z) >=
      params_.turn_clearance_guard_min_angular_rad_s);
    const bool turning =
      params_.turn_clearance_guard_enabled &&
      mode_profile.emergency_stop_enabled &&
      !intrusion.active &&
      eval_cmd.linear.x > min_motion_command &&
      turning_gate;
    if (turning) {
      const double arc_clearance =
        checker_.checkTrajectory(buildDistanceProbe(eval_cmd), field).min_distance;
      if (std::isfinite(arc_clearance) &&
        arc_clearance <= params_.turn_clearance_guard_band_m)
      {
        const double decel = std::max(1e-3, params_.braking_comfort_decel_m_s2);
        const double stoppable = std::max(0.0, arc_clearance - params_.hard_stop_distance_m);
        turn_guard_cap = std::sqrt(2.0 * decel * stoppable);
        if (eval_cmd.linear.x > turn_guard_cap) {
          const double scale = eval_cmd.linear.x > 1e-6 ? turn_guard_cap / eval_cmd.linear.x : 0.0;
          eval_cmd.linear.x = turn_guard_cap;
          eval_cmd.angular.z *= scale;   // 同比缩放保持转弯半径
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::TURN_CLEARANCE_GUARD;
            verdict.active = true;
            verdict.forward_cap = turn_guard_cap;
            verdict.left_cap = std::abs(eval_cmd.angular.z);
            verdict.right_cap = std::abs(eval_cmd.angular.z);
            record_verdict(verdict);
          }
        }
      }
    }
    result.hybrid_log.turn_guard_speed_cap_m_s = turn_guard_cap;
  }

  const bool feedback_stopped =
    std::abs(fb_linear) < params_.standstill_speed_eps_m_s &&
    std::abs(prev_output_linear_x) < params_.standstill_speed_eps_m_s;
  const bool parked =
    feedback_stopped &&
    std::abs(eval_cmd.linear.x) < min_motion_command &&
    std::abs(eval_cmd.angular.z) < min_motion_command;

  // ── P0-B 停稳看护：停稳后障碍逼近保护，防止减速停车后被侵入车体 ──
  // 制动包络/MDB/TTC 均以自车前进为门控，停稳后全部失效；此处独立跟踪最近物理净空。
  // 只在净空「正在缩小」（有物体逼近）或「刚跌破地板」（动态切入）时锁存急停并上报，
  // 静止贴墙/窄道停车净空不变，不会误触发。倒车意图始终放行，供驾驶员/脱困退让。
  {
    const double physical_clearance = currentPhysicalClearance(field);
    const double prev_clearance = standstill_prev_physical_clearance_m_;
    standstill_prev_physical_clearance_m_ = physical_clearance;   // 逐周期更新净空历史
    result.hybrid_log.standstill_min_clearance_m = physical_clearance;

    const bool reverse_intent = eval_cmd.linear.x < -min_motion_command;
    // L1：掩码裁剪后仍存活的命令分量即为未受威胁方向的逃逸意图，与倒车同等放行。
    // 旧行为仅放行倒车，左障逼近时前向/右转逃逸会被全向看护误锁。
    const bool masked_escape_intent =
      params_.direction_mask_enabled &&
      (std::abs(eval_cmd.linear.x) > min_motion_command ||
      std::abs(eval_cmd.angular.z) > min_motion_command);
    const bool guard_enabled =
      params_.standstill_guard_enabled &&
      mode_profile.emergency_stop_enabled &&
      !intrusion.active;   // 已侵入交由脱困 recovery 处理

    if (!guard_enabled || reverse_intent || masked_escape_intent) {
      standstill_guard_confirm_time_s_ = 0.0;
      standstill_guard_latched_ = false;   // 让位/脱困：不看护并解除已有锁存
    } else {
      const double dt = std::max(1e-3, std::max(0.0, dt_s));
      double approach_rate =
        (std::isfinite(prev_clearance) && std::isfinite(physical_clearance)) ?
        (prev_clearance - physical_clearance) / dt : 0.0;
      // dt 下界 1ms 防止控制环抖动导致 approach_rate 噪声放大；
      // 再施加上界截断，避免异常帧产生不合理的极大逼近速率。
      approach_rate = std::clamp(
        approach_rate,
        -params_.absolute_max_linear_speed,
        params_.absolute_max_linear_speed);
      const bool in_band =
        std::isfinite(physical_clearance) &&
        physical_clearance <= params_.standstill_guard_band_m;
      const bool approaching =
        in_band && approach_rate >= params_.standstill_guard_approach_rate_m_s;
      // 仅「刚跌破地板」= 动态切入才触发；静止已在地板内（贴墙/窄道）不触发。
      const bool floor_breach =
        std::isfinite(physical_clearance) &&
        physical_clearance <= params_.standstill_guard_floor_m &&
        std::isfinite(prev_clearance) &&
        prev_clearance > params_.standstill_guard_floor_m;
      const bool release_by_clearance =
        !std::isfinite(physical_clearance) ||
        physical_clearance >= params_.standstill_guard_release_m;

      if (standstill_guard_latched_) {
        if (release_by_clearance) {
          standstill_guard_latched_ = false;
          standstill_guard_confirm_time_s_ = 0.0;
        }
      } else if (feedback_stopped && floor_breach) {
        // 动态切入至地板（净空刚跌破 floor）：即将触碰车体，立即锁存，不等确认。
        standstill_guard_latched_ = true;
        standstill_guard_confirm_time_s_ = 0.0;
      } else if (feedback_stopped && approaching) {
        // 持续逼近：满足确认时间后锁存，过滤停车瞬间的自车运动尾差与抖动。
        standstill_guard_confirm_time_s_ += dt;
        if (standstill_guard_confirm_time_s_ >= params_.standstill_guard_confirm_s) {
          standstill_guard_latched_ = true;
        }
      } else {
        standstill_guard_confirm_time_s_ = 0.0;
      }
    }

    if (standstill_guard_latched_) {
      emergency_brake_.latch();
      resetCandidateSpeedUpHold();
      resetRecoveryState();
      result.safe_twist = emergency_brake_.applyEmergencyBrake();
      result.emergency_brake = true;
      result.emergency_reason = EmergencyStopReason::STANDSTILL_GUARD;
      result.risk.has_collision = true;
      result.risk.emergency = true;
      result.hybrid_log.standstill_guard_active = true;
      if (shadow_enabled) {
        LayerVerdict verdict;
        verdict.id = SafetyLayerId::STANDSTILL_GUARD;
        verdict.active = true;
        verdict.request_full_stop = true;
        verdict.has_collision_stop = true;
        verdict.reason = EmergencyStopReason::STANDSTILL_GUARD;
        verdict.bypass_smoothing = true;
        record_verdict(verdict);
        run_arbiter_shadow(result, eval_cmd);
      }
      const auto t1 = std::chrono::steady_clock::now();
      result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      result.hybrid_log.tick_ms = result.tick_duration_ms;
      return result;
    }
  }

  // ── 方案 5 动态障碍逼近速度估计：前向走廊净空差分（扣除自车前进分量）──
  // 输出接入制动包络的动态预留与 L0-EB 的相对速度；未启用时恒为 0（旧行为）。
  const double sudden_corridor_half =
    footprint_.physicalRect().y_max +
    std::max(0.0, params_.sudden_obstacle.corridor_margin_m);
  const double forward_corridor_clearance =
    currentForwardClearance(field, sudden_corridor_half);
  const double closing_rate_m_s = params_.closing_speed.enabled ?
    closing_estimator_.update(
    forward_corridor_clearance, std::max(0.0, fb_linear), dt_s) :
    0.0;
  result.hybrid_log.closing_speed_m_s = closing_rate_m_s;
  result.hybrid_log.forward_corridor_clearance_m = forward_corridor_clearance;

  // ── L0-EB 突发障碍动态紧急制动 ──
  // 在制动包络之前判定：所需减速度 a_req 超舒适档 = MDB 没能提前压住 = 突发/切入，
  // 立即锁存急停（reason 走 bypassOutputSmoothing 绕过 max_decel 限幅，由底盘全力制动）。
  // 与停稳看护同层、正交：仅在自车前进且前向净空进入判定窗口时参与。
  {
    const bool eb_enabled =
      params_.sudden_obstacle.enabled &&
      mode_profile.emergency_stop_enabled &&
      !intrusion.active;
    if (!eb_enabled) {
      sudden_detector_.reset();
    } else {
      const double forward_clearance = forward_corridor_clearance;
      const double d_safe =
        std::max(0.0, params_.hard_stop_distance_m) +
        std::max(0.0, params_.braking_stop_buffer_m);
      // 方案 5：a_req 用相对速度 v_rel = 自车前向速度 + 障碍逼近速度，
      // 对向切入时触发显著提前；估计未启用/未确认时 closing_rate=0（旧行为）。
      const double v_rel = std::max(0.0, fb_linear) + closing_rate_m_s;
      const SuddenObstacleResult sudden =
        sudden_detector_.update(forward_clearance, v_rel, d_safe, dt_s);
      result.hybrid_log.sudden_a_req_m_s2 = sudden.a_req;
      result.hybrid_log.sudden_brake_saturated = sudden.saturated;
      if (sudden.trigger) {
        // 方案 2.2 影子模式：触发条件满足只写日志，不实际急停（灰度期）。
        if (params_.sudden_obstacle.shadow_mode) {
          result.hybrid_log.sudden_obstacle_shadow = true;
        } else {
          emergency_brake_.latch();
          resetCandidateSpeedUpHold();
          resetRecoveryState();
          result.safe_twist = emergency_brake_.applyEmergencyBrake();
          result.emergency_brake = true;
          result.emergency_reason = sudden.saturated ?
            EmergencyStopReason::BRAKE_SATURATED :
            EmergencyStopReason::SUDDEN_OBSTACLE;
          result.risk.has_collision = true;
          result.risk.emergency = true;
          result.risk.min_clearance = forward_clearance;
          result.hybrid_log.sudden_obstacle_active = true;
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::SUDDEN_OBSTACLE;
            verdict.active = true;
            verdict.request_full_stop = true;
            verdict.has_collision_stop = true;
            verdict.reason = result.emergency_reason;
            verdict.bypass_smoothing = true;
            record_verdict(verdict);
            run_arbiter_shadow(result, eval_cmd);
          }
          const auto t1 = std::chrono::steady_clock::now();
          result.tick_duration_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
          result.hybrid_log.tick_ms = result.tick_duration_ms;
          return result;
        }
      }
    }
  }

  BrakingEnvelope braking_envelope;
  if (!intrusion.active) {
    braking_envelope = evaluateBrakingEnvelope(
      eval_cmd, fb_linear, prev_output_linear_x, field, local_map, mode_profile, dt_s,
      direction_mask, closing_rate_m_s);
    if (braking_envelope.active) {
      braking_envelope_active_time_s_ += std::max(0.0, dt_s);
    } else {
      braking_envelope_active_time_s_ = 0.0;
    }

    auto apply_braking_limit = [&](double raw_limit_m_s) {
        const double requested_speed = std::abs(eval_cmd.linear.x);
        const double smoothed_limit = smoothBrakingSpeedLimit(raw_limit_m_s, dt_s);
        braking_envelope.speed_limit_m_s = smoothed_limit;
        eval_cmd.linear.x = std::copysign(smoothed_limit, eval_cmd.linear.x);
        // 转向保真：前向制动限速只压线速度，保留驾驶员角速度，不随线速度同比清零。
        // 否则前向净空一受限（含探针误判/真前障）就把纯转向意图也一并冻结，驾驶员无法
        // 靠转向绕开；转弯自身的净空由 turn_clearance_guard / direction_mask 角轴另行约束。
        if (!(params_.shared_control.enabled && params_.shared_control.preserve_steering)) {
          const double scale = requested_speed > 1e-6 ?
            std::clamp(smoothed_limit / requested_speed, 0.0, 1.0) :
            0.0;
          eval_cmd.angular.z *= scale;
        }
      };

    const bool can_shape_braking =
      !parked && !braking_envelope.controlled_escape_only;
    if (braking_envelope.active && can_shape_braking) {
      apply_braking_limit(braking_envelope.speed_limit_m_s);
      if (shadow_enabled) {
        LayerVerdict verdict;
        verdict.id = SafetyLayerId::BRAKING_ENVELOPE;
        verdict.active = true;
        verdict.forward_cap = braking_envelope.speed_limit_m_s;
        verdict.backward_cap = braking_envelope.speed_limit_m_s;
        if (!(params_.shared_control.enabled &&
          params_.shared_control.preserve_steering))
        {
          verdict.left_cap = std::abs(eval_cmd.angular.z);
          verdict.right_cap = std::abs(eval_cmd.angular.z);
        }
        record_verdict(verdict);
      }
    } else {
      has_smoothed_braking_speed_limit_ = false;
    }
    updateMinDistanceBrakingLatch(
      braking_envelope,
      eval_cmd,
      params_.direction_mask_enabled ?
      direction_mask.forward_clearance_m :
      currentInflatedClearance(field),
      braking_envelope.effective_remaining_m);
  }
  if (intrusion.active) {
    braking_envelope_active_time_s_ = 0.0;
  }

  result.hybrid_log.effective_remaining_d_m = braking_envelope.effective_remaining_m;
  result.hybrid_log.at_min_stop = braking_envelope.at_min_stop;
  result.hybrid_log.hard_stop_reached_latched = hard_stop_reached_;

  if (braking_envelope.emergency &&
    mode_profile.emergency_stop_enabled &&
    !params_.braking_suppress_distance_emergency)
  {
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
    result.risk.braking_effective_remaining_m = braking_envelope.effective_remaining_m;
    result.risk.braking_at_min_stop = braking_envelope.at_min_stop;
    result.risk.braking_speed_limit_m_s = braking_envelope.speed_limit_m_s;
    result.risk.braking_stopping_distance_m = braking_envelope.stopping_distance_m;
    result.risk.min_ttc = braking_envelope.footprint_check.ttc;
    result.risk.min_clearance = braking_envelope.footprint_check.min_distance;
    result.risk.worst = braking_envelope.footprint_check;
    if (shadow_enabled) {
      LayerVerdict verdict;
      verdict.id = SafetyLayerId::BRAKING_ENVELOPE;
      verdict.active = true;
      verdict.request_full_stop = true;
      verdict.has_collision_stop = true;
      verdict.reason = EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT;
      record_verdict(verdict);
      run_arbiter_shadow(result, eval_cmd);
    }
    const auto t1 = std::chrono::steady_clock::now();
    result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.hybrid_log.tick_ms = result.tick_duration_ms;
    return result;
  }
  const double braking_control_distance_m =
    params_.min_distance_braking_enabled ?
    braking_envelope.effective_remaining_m :
    braking_envelope.available_distance_m;
  const bool braking_control_feasible =
    braking_envelope.active &&
    !braking_envelope.controlled_escape_only &&
    braking_control_distance_m >
    std::max(1e-3, 0.5 * params_.braking_path_step_m) &&
    braking_envelope.stopping_distance_m <=
    braking_control_distance_m + 1e-6;

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
    const std::vector<PredictedPose> current_pose{PredictedPose{}};
    fillMapExemptCircles(exempt_circles_buf_, field, current_pose);
    if (map_checker_.trajectoryLethal(
        *local_map,
        current_pose,
        mode_profile.lethal_threshold,
        map_config_.check_footprint_corners,
        current_map_result,
        exempt_circles_buf_.empty() ? nullptr : &exempt_circles_buf_))
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

  // ── P2 窄道导引：检出有效走廊时注入 Bézier 居中候选 ──
  // 候选仍需通过 Footprint 扫掠与 Map veto 全部安全检查，本步只提供导引。
  int narrow_candidate_index = -1;
  if (params_.narrow_planner.enabled &&
    (mode_profile.avoidance_overrides_enabled ||
    params_.narrow_planner.auto_detect_enabled) &&
    !intrusion.active && !intrusion.proximity_blocked &&
    eval_cmd.linear.x > min_motion_command)
  {
    const FootprintRect body_rect = footprint_.physicalRect();
    const NarrowCorridorPlan narrow_plan = narrow_planner_.plan(
      field, eval_cmd, body_rect.y_max - body_rect.y_min);
    if (narrow_plan.valid) {
      rollouts.push_back(
        rollout_.rolloutCandidate(narrow_plan.candidate, fb_linear, fb_angular));
      narrow_candidate_index = static_cast<int>(rollouts.size()) - 1;
      result.hybrid_log.narrow_planner_active = true;
      result.hybrid_log.narrow_corridor_width_m = narrow_plan.corridor_width_m;
      result.hybrid_log.narrow_speed_cap_m_s = narrow_plan.speed_cap_m_s;
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
  aggregate.braking_effective_remaining_m = braking_envelope.effective_remaining_m;
  aggregate.braking_at_min_stop = braking_envelope.at_min_stop;
  aggregate.braking_speed_limit_m_s = braking_envelope.speed_limit_m_s;
  aggregate.braking_stopping_distance_m = braking_envelope.stopping_distance_m;

  // ── 步骤 2：Footprint 扫掠碰撞，记录每条候选结果 ──
  // P1：候选 0 即驾驶员当前命令轨迹，其风险用于计算连续接管系数 α
  CollisionCheckResult driver_cmd_check;
  for (size_t i = 0; i < rollouts.size(); ++i) {
    ScoredCandidate sc;
    sc.index = static_cast<int>(i);
    sc.check = checker_.checkTrajectory(rollouts[i], field);
    sc.command_cost = commandErrorCost(
      rollouts[i].cmd, eval_cmd, params_.safety_filter_angular_weight);
    if (sc.index == 0) {
      driver_cmd_check = sc.check;
    }
    if (sc.index == narrow_candidate_index) {
      // P2 居中候选偏置：窄道内同等安全下优先跟随 Bézier 中线导引
      sc.command_cost = std::max(
        0.0, sc.command_cost - std::max(0.0, params_.narrow_planner.cost_bias));
    }
    const bool follows_braking_envelope =
      braking_control_feasible && i == 0U;
    const double min_command = std::max(0.0, params_.recovery_min_command);
    const double max_escape_linear = std::max(
      min_command, params_.recovery_max_linear_speed);
    const bool controlled_reverse_escape_state =
      fb_linear <= params_.standstill_speed_eps_m_s &&
      prev_output_linear_x <= params_.standstill_speed_eps_m_s &&
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
      (braking_envelope.at_min_stop &&
      i == 0U &&
      !intrusion.proximity_blocked &&
      !intrusion.active) ||
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
      // L1：近障（含其膨胀栅格）已由 Footprint 威胁谓词按该候选方向判定，
      // 未受威胁者以豁免圆撤出 map veto，避免同一近障被地图层二次全向否决。
      fillMapExemptCircles(
        exempt_circles_buf_, field, rollouts[static_cast<size_t>(sc.index)].poses);
      const bool map_lethal = map_checker_.trajectoryLethal(
        *local_map,
        rollouts[static_cast<size_t>(sc.index)].poses,
        lethal_threshold,
        check_corners,
        map_result,
        exempt_circles_buf_.empty() ? nullptr : &exempt_circles_buf_);
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
        if (shadow_enabled) {
          LayerVerdict verdict;
          verdict.id = SafetyLayerId::EMERGENCY_GATE;
          verdict.active = true;
          verdict.request_full_stop = true;
          verdict.has_collision_stop = true;
          verdict.reason = result.emergency_reason;
          record_verdict(verdict);
        }
      };

    // L1 定向脱困释放阀：物理侵入下 recovery 放弃(abort/blocked)时，若驾驶员正朝
    // 方向掩码未封锁的方向给令，放行该方向的受控逃逸而非清零。前撞后障碍进入物理
    // 车体 → intrusion.active → recovery，一旦超时/无进展 abort，旧逻辑对所有轴无条件
    // 清零并被自锁钉死，连掩码明确放行的后退方向也一并锁死（deadlock：不动 → 侵入
    // 不解 → 不动）。此处先取掩码放行的分量作受控逃逸，使车能退出侵入并令 need_recovery
    // 自然转假解锁。命令方向被判向物理接触逼近或未给令时仍回退硬停。
    auto release_or_stop = [&](bool physical_intrusion) {
        geometry_msgs::msg::Twist escape;
        if (physical_intrusion &&
          directionalEscapeTwist(requested_cmd, direction_mask, escape))
        {
          emergency_brake_.reset();
          resetCandidateSpeedUpHold();
          result.safe_twist = escape;
          result.recovery_active = true;
          result.recovery_state = RecoveryState::DIRECTIONAL_ESCAPE;
          result.emergency_brake = false;
          result.hybrid_log.final_candidate = -3;
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::RECOVERY_ESCAPE;
            verdict.active = true;
            verdict.has_suggestion = true;
            verdict.suggestion = escape;
            verdict.escape_grant = true;
            record_verdict(verdict);
          }
          return;
        }
        apply_recovery_stop(physical_intrusion);
      };

    if (recovery_abort_latched_) {
      result.recovery_state = recovery_abort_reason_;
      if (intrusion.active || !directional_escape_viable) {
        release_or_stop(intrusion.active);
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
          release_or_stop(true);
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
          release_or_stop(intrusion.active);
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
              release_or_stop(true);
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
            if (shadow_enabled) {
              LayerVerdict verdict;
              verdict.id = SafetyLayerId::RECOVERY_ESCAPE;
              verdict.active = true;
              verdict.has_suggestion = true;
              verdict.suggestion = recovery_twist;
              verdict.escape_grant = true;
              record_verdict(verdict);
            }
          }
        }
      }
    }
    result.recovery_elapsed_s = recovery_elapsed_s_;
  } else {
    resetRecoveryState();
  }

  if (!result.recovery_active && !result.emergency_brake && selected_index < 0) {
    if (params_.min_distance_braking_enabled &&
      braking_envelope.at_min_stop &&
      !map_enabled &&
      !result.hybrid_log.map_veto_applied &&
      !intrusion.proximity_blocked &&
      !intrusion.active)
    {
      result.risk = aggregate;
      result.safe_twist = geometry_msgs::msg::Twist{};
      result.emergency_brake = false;
      result.emergency_reason = EmergencyStopReason::NONE;
      result.hybrid_log.final_candidate = 0;
      if (shadow_enabled) {
        LayerVerdict verdict;
        verdict.id = SafetyLayerId::CANDIDATE_SELECT;
        verdict.active = true;
        verdict.has_suggestion = true;
        verdict.suggestion = geometry_msgs::msg::Twist{};
        record_verdict(verdict);
      }
    } else {
      // ── 无可行候选：急停或 SafetyFilter 尽力降级 ──
      result.risk = aggregate;
      result.risk.has_collision = true;
      const bool all_footprint_safe_candidates_map_vetoed =
        map_enabled && map_config_.veto_on_lethal && footprint_best >= 0;
      if (all_footprint_safe_candidates_map_vetoed) {
        // ── L2 被困脱困：四向候选全被 LocalMap veto 的夹死死状态下，在硬停前尝试
        // 低速微动作脱困（默认关；每个微动作经 Footprint 扫掠 + LocalMap veto 复验，
        // 均不过则回退到与原先完全一致的 MAP_VETO 硬停）。──
        geometry_msgs::msg::Twist pinned_escape_twist;
        double pinned_clearance_gain = 0.0;
        const bool pinned_found = tryPinnedEscape(
          requested_cmd, fb_linear, fb_angular, field, local_map, mode_profile,
          prev_output_linear_x, dt_s, pinned_escape_twist, pinned_clearance_gain);
        // 方案 2.3 影子模式：微动作复验通过只写日志，仍回退到与原先一致的硬停。
        if (pinned_found && params_.pinned_escape.shadow_mode) {
          result.hybrid_log.pinned_escape_shadow = true;
        }
        if (pinned_found && !params_.pinned_escape.shadow_mode) {
          emergency_brake_.reset();
          resetCandidateSpeedUpHold();
          result.safe_twist = pinned_escape_twist;
          result.recovery_active = true;
          result.recovery_state = RecoveryState::PINNED_ESCAPE;
          result.recovery_clearance_gain_m = pinned_clearance_gain;
          result.emergency_brake = false;
          result.hybrid_log.final_candidate = -4;
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::RECOVERY_ESCAPE;
            verdict.active = true;
            verdict.has_suggestion = true;
            verdict.suggestion = pinned_escape_twist;
            verdict.escape_grant = true;
            record_verdict(verdict);
          }
        } else {
          emergency_brake_.latch();
          resetCandidateSpeedUpHold();
          result.emergency_brake = true;
          result.emergency_reason = EmergencyStopReason::MAP_VETO;
          result.safe_twist = emergency_brake_.applyEmergencyBrake();
          result.risk.emergency = true;
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::MAP_VETO;
            verdict.active = true;
            verdict.request_full_stop = true;
            verdict.has_collision_stop = true;
            verdict.reason = EmergencyStopReason::MAP_VETO;
            record_verdict(verdict);
          }
        }
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
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::EMERGENCY_GATE;
            verdict.active = true;
            verdict.request_full_stop = true;
            verdict.has_collision_stop = true;
            verdict.reason = result.emergency_reason;
            record_verdict(verdict);
          }
        } else {
          const SafetyFilterResult filtered = safety_filter_.selectSafeVelocity(
            eval_cmd, rollouts, field, prev_output_linear_x, prev_output_angular_z, dt_s);
          result.safe_twist = filtered.safe_twist;
          result.risk = filtered.risk;
          result.risk.has_collision = filtered.all_candidates_collide || aggregate.has_collision;
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::CANDIDATE_SELECT;
            verdict.active = true;
            verdict.has_suggestion = true;
            verdict.suggestion = filtered.safe_twist;
            record_verdict(verdict);
          }
        }
      }
    }
  } else if (!result.recovery_active && !result.emergency_brake) {
    // ── 选中候选：应用速度并做周期内加减速限制 ──
    const auto & chosen = rollouts[static_cast<size_t>(selected_index)].cmd;
    double target_linear = chosen.v;
    double target_angular = chosen.w;
    // ── P1 连续接管：output = driver + α·(safety − driver) ──
    // α=0 决策空间内完全跟手；风险逼近时 α 平滑上升软接管；α=1 完全过载。
    // 受控逃逸/地图逃逸候选须精确跟随（下游按选中命令核对），不参与融合。
    // eval_cmd 已含制动包络限速与巡航上限，融合不会放宽任何速度硬约束。
    if (params_.shared_control.enabled &&
      !selected_controlled_escape && !selected_map_escape)
    {
      const double alpha = updateAuthorityAlpha(driver_cmd_check, dt_s);
      result.hybrid_log.authority_alpha = alpha;
      target_linear = eval_cmd.linear.x + alpha * (chosen.v - eval_cmd.linear.x);
      target_angular = eval_cmd.angular.z + alpha * (chosen.w - eval_cmd.angular.z);
    }
    result.safe_twist.linear.x = applyCandidateSpeedUpHold(target_linear, dt_s);
    result.safe_twist.angular.z = target_angular;
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
    // ── 转弯合成速度包络兜底：入口已限幅的命令此处 k=1 恒等通过，仅约束
    // narrow planner / escape 等内部合成且未经入口的输出。──
    result.hybrid_log.turn_envelope_scale = std::min(
      result.hybrid_log.turn_envelope_scale,
      applyTurnSpeedEnvelope(
        result.safe_twist.linear.x, result.safe_twist.angular.z, params_));
    // 急速转向线速度硬帽兜底：入口已钳过的命令此处恒等通过，仅约束 narrow planner /
    // escape 等内部合成且未经入口的输出。生效标志取或，保留入口已置位的记录。
    result.hybrid_log.rapid_steer_cap_active =
      applyRapidSteerLinearCap(
      result.safe_twist.linear.x, result.safe_twist.angular.z, params_) ||
      result.hybrid_log.rapid_steer_cap_active;
    if (shadow_enabled) {
      LayerVerdict verdict;
      verdict.id = SafetyLayerId::CANDIDATE_SELECT;
      verdict.active = true;
      verdict.has_suggestion = true;
      verdict.suggestion = result.safe_twist;
      record_verdict(verdict);
    }
  }

  // 对控制器最终将要发布的命令重新做完整检查，不能沿用已淘汰候选的聚合风险。
  const bool forward_stop_hold_active = updateForwardStopHold(
    braking_envelope,
    requested_cmd,
    fb_linear,
    prev_output_linear_x,
    result.recovery_active,
    result.emergency_brake,
    dt_s);
  result.hybrid_log.forward_stop_hold_active = forward_stop_hold_active;
  if (forward_stop_hold_active && !result.recovery_active && !result.emergency_brake) {
    if (result.safe_twist.linear.x > 0.0) {
      result.safe_twist.linear.x = 0.0;
      // L1 分轴停车：前向保持只停前向轴，存活的转向分量保留（掩码已裁剪受威胁转向）
      if (!params_.direction_mask_enabled) {
        result.safe_twist.angular.z = 0.0;
      }
    }
    resetCandidateSpeedUpHold();
    if (shadow_enabled) {
      LayerVerdict verdict;
      verdict.id = SafetyLayerId::FORWARD_STOP_HOLD;
      verdict.active = true;
      verdict.stop_axes.forward = true;
      if (!params_.direction_mask_enabled) {
        verdict.stop_axes.left = true;
        verdict.stop_axes.right = true;
      }
      record_verdict(verdict);
    }
  }

  // ── P0 脉冲补偿：底盘死区导致停短（残差 > 容差却停住）时，发有界
  // 前向微脉冲逼近 hard-stop 目标线。输出仍经下方全量安全复查与硬闸。──
  if (params_.precise_stop.enabled && params_.precise_stop.pulse_enabled &&
    params_.min_distance_braking_enabled &&
    mode_profile.footprint_selector_enabled &&
    !result.emergency_brake && !result.recovery_active &&
    !forward_stop_hold_active &&    // 贴障停稳保持期间不再补偿蠕动，安全"保持"压过精度"补偿"
    !intrusion.active && !intrusion.proximity_blocked &&
    !braking_envelope.at_min_stop &&
    !braking_envelope.controlled_escape_only &&
    braking_envelope.active)
  {
    PulseCompensator::Input pulse_in;
    pulse_in.residual_m = braking_envelope.effective_remaining_m;
    pulse_in.fb_linear = fb_linear;
    pulse_in.forward_intent = requested_cmd.linear.x > min_motion_command;
    pulse_in.safe_to_creep = !tilt_emergency_latched_;
    if (pulse_comp_.update(pulse_in, dt_s)) {
      const double creep = std::max(0.0, params_.precise_stop.pulse_creep_speed_m_s);
      if (result.safe_twist.linear.x < creep) {
        // 直线微动：脉冲期间不带角速度，避免扫掠包络横向增大
        result.safe_twist.linear.x = creep;
        result.safe_twist.angular.z = 0.0;
        resetCandidateSpeedUpHold();
        if (shadow_enabled) {
          // 脉冲是受控且有界的微逃逸：suggestion + 令牌，豁免更低优先级的
          // 制动包络 cap（否则仲裁器会把 creep 重新钳回停短状态）。
          LayerVerdict verdict;
          verdict.id = SafetyLayerId::PULSE_COMP;
          verdict.active = true;
          verdict.has_suggestion = true;
          verdict.suggestion = result.safe_twist;
          verdict.escape_grant = true;
          record_verdict(verdict);
        }
      }
      result.hybrid_log.pulse_comp_active = true;
    }
  } else {
    pulse_comp_.reset();
  }

  bool final_follows_controlled_escape = false;
  bool final_follows_map_escape = false;
  if (!result.recovery_active && !result.emergency_brake &&
    mode_profile.footprint_selector_enabled)
  {
    VelocityCandidate final_cmd;
    final_cmd.v = result.safe_twist.linear.x;
    final_cmd.w = result.safe_twist.angular.z;
    const TrajectoryRollout final_rollout =
      rollout_.rolloutCandidate(final_cmd, fb_linear, fb_angular);
    const CollisionCheckResult final_check = checker_.checkTrajectory(final_rollout, field);
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
      fillMapExemptCircles(exempt_circles_buf_, field, final_rollout.poses);
      if (map_checker_.trajectoryLethal(
          *local_map,
          final_rollout.poses,
          mode_profile.lethal_threshold,
          map_config_.check_footprint_corners,
          map_result,
          exempt_circles_buf_.empty() ? nullptr : &exempt_circles_buf_))
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
          if (shadow_enabled) {
            LayerVerdict verdict;
            verdict.id = SafetyLayerId::MAP_VETO;
            verdict.active = true;
            verdict.request_full_stop = true;
            verdict.has_collision_stop = true;
            verdict.reason = EmergencyStopReason::MAP_VETO;
            record_verdict(verdict);
          }
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
  result.risk.braking_effective_remaining_m = braking_envelope.effective_remaining_m;
  result.risk.braking_at_min_stop = braking_envelope.at_min_stop;
  result.risk.braking_speed_limit_m_s = braking_envelope.speed_limit_m_s;
  result.risk.braking_stopping_distance_m = braking_envelope.stopping_distance_m;

  // ── 斜雷达 TTC 近距通道：主路径完成后叠加，与水平链路取更保守输出 ──
  // 【deprecated，方案 2.5】生产已由 treat_as_horizontal 并入水平主链替代，
  // 本通道配置关闭（tilt_ttc.enabled=false 时直接返回）。物理删除（含两个大
  // 测试套件重写）作为带编译环境的独立 commit 执行，勿再向该通道加新功能。
  const TiltIntervention tilt_intervention =
    evaluateTiltIntervention(requested_cmd, fb_linear, tilt_field, dt_s);
  mergeTiltIntervention(result, tilt_intervention);
  if (shadow_enabled && tilt_intervention.active && !tilt_intervention.emergency &&
    std::isfinite(tilt_intervention.speed_limit_m_s))
  {
    LayerVerdict verdict;
    verdict.id = SafetyLayerId::BRAKING_ENVELOPE;
    verdict.active = true;
    verdict.forward_cap = tilt_intervention.speed_limit_m_s;
    if (!(params_.shared_control.enabled && params_.shared_control.preserve_steering)) {
      verdict.left_cap = std::abs(result.safe_twist.angular.z);
      verdict.right_cap = std::abs(result.safe_twist.angular.z);
    }
    record_verdict(verdict);
  }

  if (!result.recovery_active && params_.use_ttc_emergency_stop &&
    !final_follows_controlled_escape &&
    result.risk.min_ttc <= params_.limiting_ttc_s)
  {
    result.risk.emergency = (result.risk.min_ttc <= params_.emergency_ttc_s);
  }
  fillInterventionLevels(result.risk, params_);

  const bool mdb_at_boundary_stop =
    params_.min_distance_braking_enabled &&
    braking_envelope.at_min_stop &&
    !result.hybrid_log.map_veto_applied &&
    !result.recovery_active;

  // ── 最后一道硬闸：TTC / 净空触线则强制零速 ──
  EmergencyBrakeContext eb_context = buildEmergencyBrakeContext(
    requested_cmd, result.safe_twist, fb_linear, fb_angular, prev_output_linear_x, field);
  // L2 退离逃逸（默认关）：对已通过「单调退离」校验的 map-escape 候选，硬停同样按
  // 受控逃逸豁免，避免选中后又被硬停净空判据重新钉零。关闭时此项恒为原行为。
  eb_context.controlled_escape_active =
    !result.recovery_active &&
    !result.emergency_brake &&
    ((selected_controlled_escape && final_follows_controlled_escape) ||
    (map_config_.retreat_escape_enabled && selected_map_escape && final_follows_map_escape));
  if (mdb_at_boundary_stop) {
    emergency_brake_.reset();
    result.emergency_brake = false;
    if (result.emergency_reason == EmergencyStopReason::BRAKE_HOLD ||
      result.emergency_reason == EmergencyStopReason::HARD_STOP_DISTANCE)
    {
      result.emergency_reason = EmergencyStopReason::NONE;
    }
    result.risk.emergency = false;
    if (std::abs(result.safe_twist.linear.x) > 1e-6 &&
      std::abs(requested_cmd.linear.x) > min_motion_command)
    {
      result.safe_twist.linear.x = 0.0;
      // L1 分轴停车：MDB 边界只停前向轴；角速度分量来自掩码裁剪后的命令，
      // 受威胁转向已在入口被清零，存活的转向保留（前障贴停仍可原地转向脱离）。
      if (!params_.direction_mask_enabled) {
        result.safe_twist.angular.z = 0.0;
      }
      if (shadow_enabled) {
        LayerVerdict verdict;
        verdict.id = SafetyLayerId::FORWARD_STOP_HOLD;
        verdict.active = true;
        verdict.stop_axes.forward = true;
        verdict.stop_axes.backward = true;
        if (!params_.direction_mask_enabled) {
          verdict.stop_axes.left = true;
          verdict.stop_axes.right = true;
        }
        record_verdict(verdict);
      }
    }
  } else if (!result.recovery_active && mode_profile.emergency_stop_enabled) {
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
      const double surviving_w = result.safe_twist.angular.z;   // 掩码已裁剪受威胁转向
      result.safe_twist = emergency_brake_.applyEmergencyBrake();  // 线速归零
      // L1 分轴硬停:前向/净空触线的急停,保留掩码认可的原地转向(对应旋转向未被封锁),
      // 与前向保持(2716)/MDB 边界(2895)分轴一致——避免走廊贴墙原地调头被整帧钉零。
      // 真实碰撞风险(has_collision / 全向 emergency)仍整帧停,不放宽。
      if (params_.direction_mask_enabled && !result.risk.has_collision &&
        ((surviving_w > 0.0 && !direction_mask.left) ||
        (surviving_w < 0.0 && !direction_mask.right)))
      {
        result.safe_twist.angular.z = surviving_w;
      }
      result.risk.emergency = true;
      if (shadow_enabled) {
        LayerVerdict verdict;
        verdict.id = SafetyLayerId::EMERGENCY_GATE;
        verdict.active = true;
        verdict.reason = result.emergency_reason;
        if (result.risk.has_collision || !params_.direction_mask_enabled ||
          std::abs(result.safe_twist.angular.z) <= 1e-9)
        {
          verdict.request_full_stop = true;
          verdict.has_collision_stop = result.risk.has_collision;
        } else {
          // 分轴硬停：线速轴全停，掩码认可的原地转向保留
          verdict.stop_axes.forward = true;
          verdict.stop_axes.backward = true;
        }
        record_verdict(verdict);
      }
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

  // ── 方案 1 步骤 A：主路径末尾影子比对（纯观测，见文件头与 safety_verdict.h）──
  run_arbiter_shadow(result, eval_cmd);

  // ── 原地转向豁免（rotation escape，默认关，见 tryRotationEscape / avoidance_types.h）──
  // 急停已把输出整帧清零（applyEmergencyBrake 全零）。若本帧确为急停全停、且驾驶员在给
  // 纯原地转向，且该转向经 Footprint 扫掠 + LocalMap「不进新 lethal」复验证明不恶化，则以
  // recovery 输出契约放行受控低速原地转向脱困（线速度仍恒零，急停解除使 OutputShaper 正常
  // 整形转向而非收敛到零）。仅作用于急停全停帧；正常候选/recovery/限速帧不满足前置条件。
  // 早返回的 standstill_guard / sudden_obstacle / input_fault / mode_estop 急停不经此处，
  // 故「有物主动逼近或系统失效」的冻结不被放宽。enabled=false 时短路，输出与旧行为一致。
  if (params_.rotation_escape.enabled &&
    result.emergency_brake &&
    std::abs(result.safe_twist.linear.x) <= 1e-9 &&
    std::abs(result.safe_twist.angular.z) <= 1e-9)
  {
    geometry_msgs::msg::Twist rot_escape;
    if (tryRotationEscape(
        requested_cmd, field, tilt_field, local_map, mode_profile, rot_escape))
    {
      emergency_brake_.reset();
      resetCandidateSpeedUpHold();
      result.safe_twist = rot_escape;
      result.emergency_brake = false;
      result.emergency_reason = EmergencyStopReason::NONE;
      result.recovery_active = true;
      result.recovery_state = RecoveryState::ROTATION_ESCAPE;
      result.risk.emergency = false;
      result.hybrid_log.rotation_escape_active = true;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  result.tick_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  result.hybrid_log.tick_ms = result.tick_duration_ms;
  result.hybrid_log.effective_remaining_d_m = braking_envelope.effective_remaining_m;
  result.hybrid_log.at_min_stop = braking_envelope.at_min_stop;
  result.hybrid_log.hard_stop_reached_latched = hard_stop_reached_;
  return result;
}
