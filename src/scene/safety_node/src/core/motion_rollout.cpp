/**
 * @file motion_rollout.cpp
 * @brief 候选速度采样与 unicycle 前向积分
 */
#include "safety_node/core/motion_rollout.h"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double kLinearSampleRatios[] = {1.0, 0.8, 0.6, 0.4, 0.2, 0.0};
constexpr double kAngularSampleRatios[] = {1.0, 0.75, 0.5, 0.25, 0.0};
constexpr int kMaxTrajectoryCandidates = 20;

bool isInPlaceRotation(double v_cmd, double w_cmd)
{
  return std::abs(v_cmd) < 0.05 && std::abs(w_cmd) > 0.05;
}

bool isArcMotion(double v_cmd, double w_cmd)
{
  return std::abs(v_cmd) >= 0.05 && std::abs(w_cmd) >= 0.10;
}

double stepToward(
  double current,
  double target,
  double decel,
  double recover_accel,
  double dt)
{
  const bool reducing_magnitude =
    current * target < 0.0 || std::abs(target) < std::abs(current);
  const double rate = reducing_magnitude ? decel : recover_accel;
  if (rate <= 0.0) {
    return target;
  }
  const double max_delta = rate * dt;
  return current + std::clamp(target - current, -max_delta, max_delta);
}

} // namespace

MotionRollout::MotionRollout(const AvoidanceParams & params)
: params_(params)
{
}

void MotionRollout::updateParams(const AvoidanceParams & params)
{
  params_ = params;
}

std::vector<VelocityCandidate> MotionRollout::sampleCandidates(
  const geometry_msgs::msg::Twist & cmd,
  double,
  double) const
{
  const double v_cmd = std::clamp(
    cmd.linear.x,
    -params_.absolute_max_linear_speed,
    params_.absolute_max_linear_speed);
  const double w_cmd = std::clamp(
    cmd.angular.z,
    -params_.absolute_max_angular_speed,
    params_.absolute_max_angular_speed);

  std::vector<VelocityCandidate> out;
  out.reserve(static_cast<size_t>(params_.rollout_speed_samples * params_.rollout_angular_samples));

  if (isInPlaceRotation(v_cmd, w_cmd)) {
    const int n = std::clamp(params_.rollout_angular_samples, 1, 5);
    for (int i = 0; i < n; ++i) {
      VelocityCandidate c;
      c.v = 0.0;
      c.w = w_cmd * kAngularSampleRatios[i];
      out.push_back(c);
    }
    return out;
  }

  const int speed_n = std::clamp(params_.rollout_speed_samples, 1, 6);
  for (int i = 0; i < speed_n; ++i) {
    VelocityCandidate c;
    c.v = v_cmd * kLinearSampleRatios[i];
    c.w = w_cmd;
    out.push_back(c);
  }

  if (isArcMotion(v_cmd, w_cmd)) {
    const int angular_n = std::min(3, std::clamp(params_.rollout_angular_samples, 1, 5));
    for (int i = 0; i < angular_n && static_cast<int>(out.size()) < kMaxTrajectoryCandidates; ++i) {
      VelocityCandidate c;
      c.v = v_cmd;
      c.w = w_cmd * kAngularSampleRatios[i];
      out.push_back(c);
    }
  }

  if (static_cast<int>(out.size()) > kMaxTrajectoryCandidates) {
    out.resize(static_cast<size_t>(kMaxTrajectoryCandidates));
  }
  return out;
}

TrajectoryRollout MotionRollout::rolloutCandidate(const VelocityCandidate & candidate) const
{
  return rolloutCandidate(candidate, candidate.v, candidate.w);
}

TrajectoryRollout MotionRollout::rolloutCandidate(
  const VelocityCandidate & candidate,
  double initial_linear,
  double initial_angular) const
{
  TrajectoryRollout tr;
  tr.cmd = candidate;

  const double dt = std::max(0.02, params_.rollout_dt_s);
  const double horizon = std::max(dt, params_.rollout_horizon_s);
  const int steps = static_cast<int>(std::ceil(horizon / dt));

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double t = 0.0;
  double v = initial_linear;
  double w = initial_angular;

  tr.poses.reserve(static_cast<size_t>(steps + 1));
  tr.poses.push_back({x, y, theta, t});

  for (int i = 0; i < steps; ++i) {
    v = stepToward(
      v,
      candidate.v,
      params_.max_decel_m_s2,
      params_.max_recover_accel_m_s2,
      dt);
    w = stepToward(
      w,
      candidate.w,
      params_.max_angular_decel_rad_s2,
      params_.max_angular_recover_rad_s2,
      dt);
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta += w * dt;
    t += dt;
    tr.poses.push_back({x, y, theta, t});
    if (t >= horizon - 1e-9) {
      break;
    }
  }
  return tr;
}

std::vector<TrajectoryRollout> MotionRollout::rolloutAll(
  const geometry_msgs::msg::Twist & cmd,
  double fb_linear,
  double fb_angular) const
{
  // NaN/Inf 反馈速度会污染整条轨迹积分，导致碰撞检测返回 +inf 净空，
  // 候选被误选为最优安全速度。非有限值时回退为 0（视为静止）。
  const double safe_fb_linear = std::isfinite(fb_linear) ? fb_linear : 0.0;
  const double safe_fb_angular = std::isfinite(fb_angular) ? fb_angular : 0.0;

  const auto candidates = sampleCandidates(cmd, safe_fb_linear, safe_fb_angular);
  std::vector<TrajectoryRollout> rollouts;
  rollouts.reserve(candidates.size());
  for (const auto & c : candidates) {
    rollouts.push_back(rolloutCandidate(c, safe_fb_linear, safe_fb_angular));
  }
  return rollouts;
}
