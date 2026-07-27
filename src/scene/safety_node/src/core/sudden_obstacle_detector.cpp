/**
 * @file sudden_obstacle_detector.cpp
 * @brief L0-EB 突发障碍动态紧急制动检测器实现
 */
#include "safety_node/core/sudden_obstacle_detector.h"

#include <algorithm>
#include <cmath>

SuddenObstacleDetector::SuddenObstacleDetector(const SuddenObstacleParams & params)
: params_(params)
{
}

void SuddenObstacleDetector::updateParams(const SuddenObstacleParams & params)
{
  params_ = params;
}

void SuddenObstacleDetector::reset()
{
  a_req_filtered_ = 0.0;
  confirm_count_ = 0;
  latched_ = false;
  clear_time_s_ = 0.0;
}

SuddenObstacleResult SuddenObstacleDetector::update(
  double forward_clearance_m,
  double forward_speed,
  double d_safe_m,
  double dt_s)
{
  SuddenObstacleResult result;
  if (!params_.enabled) {
    reset();
    return result;
  }

  const double dt = std::max(0.0, dt_s);
  const bool moving_forward = forward_speed >= params_.min_forward_speed_m_s;
  const bool clear_finite = std::isfinite(forward_clearance_m);
  const bool in_window =
    moving_forward && clear_finite &&
    forward_clearance_m < params_.emergency_distance_m;

  // 所需减速度 a_req：把速度在 d_eff 内降到零。d_eff 预留执行器延迟滑行 + 几何余量。
  double a_req = 0.0;
  if (in_window) {
    const double floor = std::max(1e-3, params_.d_eff_floor_m);
    const double d_eff = std::max(
      floor,
      forward_clearance_m - forward_speed * params_.t_react_s - std::max(0.0, d_safe_m));
    a_req = (forward_speed * forward_speed) / (2.0 * d_eff);
  }

  // 一阶低通抑制 d_min 噪声导致的 a_req 抖动。
  const double alpha = std::clamp(params_.a_req_filter_alpha, 0.0, 1.0);
  a_req_filtered_ = alpha * a_req + (1.0 - alpha) * a_req_filtered_;
  result.a_req = a_req_filtered_;
  result.saturated = a_req_filtered_ > params_.emergency_decel_m_s2;

  // a_req 超过舒适档 = MDB 没能提前压住 = 突发/切入。confirm_frames 防单帧毛刺。
  const bool raw_trigger =
    in_window && a_req_filtered_ > params_.comfort_decel_m_s2;
  if (raw_trigger) {
    confirm_count_ = std::min(confirm_count_ + 1, std::max(1, params_.confirm_frames));
    if (confirm_count_ >= std::max(1, params_.confirm_frames)) {
      latched_ = true;
      clear_time_s_ = 0.0;
    }
  } else {
    confirm_count_ = 0;
  }

  // 滞回释放：净空恢复到 release 以上持续 clear_hold_s 才解锁。
  if (latched_) {
    const bool cleared =
      !clear_finite || forward_clearance_m >= params_.release_distance_m;
    if (cleared) {
      clear_time_s_ += dt;
      if (clear_time_s_ >= params_.clear_hold_s) {
        latched_ = false;
        clear_time_s_ = 0.0;
        confirm_count_ = 0;
      }
    } else {
      clear_time_s_ = 0.0;
    }
  }

  result.trigger = latched_;
  return result;
}
