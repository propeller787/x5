/**
 * @file closing_speed_estimator.cpp
 * @brief 动态障碍逼近速度估计实现
 */
#include "safety_node/core/closing_speed_estimator.h"

#include <algorithm>
#include <cmath>

ClosingSpeedEstimator::ClosingSpeedEstimator(const ClosingSpeedParams & params)
: params_(params)
{
}

void ClosingSpeedEstimator::updateParams(const ClosingSpeedParams & params)
{
  params_ = params;
  reset();
}

void ClosingSpeedEstimator::reset()
{
  has_prev_ = false;
  prev_clearance_m_ = 0.0;
  filtered_rate_m_s_ = 0.0;
  confirm_count_ = 0;
  confirmed_ = false;
}

double ClosingSpeedEstimator::update(
  double clearance_m,
  double ego_forward_speed,
  double dt_s)
{
  if (!params_.enabled) {
    reset();
    return 0.0;
  }
  const double dt = std::isfinite(dt_s) ? dt_s : 0.0;
  // 净空非有限（无障碍/首帧）：不外推，直接复位
  if (!std::isfinite(clearance_m) || dt <= 1e-6) {
    if (!std::isfinite(clearance_m)) {
      reset();
    }
    return rate();
  }
  if (!has_prev_) {
    has_prev_ = true;
    prev_clearance_m_ = clearance_m;
    return rate();
  }

  const double delta = prev_clearance_m_ - clearance_m;   // 正 = 净空缩小
  // 净空增大突变：漏检恢复 / 障碍离开走廊 → 历史差分不再可信，复位
  if (-delta > std::max(0.0, params_.jump_reset_m)) {
    reset();
    has_prev_ = true;
    prev_clearance_m_ = clearance_m;
    return 0.0;
  }
  prev_clearance_m_ = clearance_m;

  // 扣除自车前进分量：净空缩小速率 − 自车速度 = 障碍自身逼近速度
  const double raw_obstacle_rate =
    delta / dt - std::max(0.0, ego_forward_speed);
  const double clamped = std::clamp(
    raw_obstacle_rate, 0.0, std::max(0.0, params_.max_rate_m_s));

  const double alpha = std::clamp(params_.filter_alpha, 0.0, 1.0);
  filtered_rate_m_s_ = alpha * clamped + (1.0 - alpha) * filtered_rate_m_s_;

  // 连续确认帧数：滤单帧体素中心抖动导致的伪逼近
  constexpr double kActiveEps = 0.02;   // m/s，低于此视为静止观测
  if (clamped > kActiveEps) {
    confirm_count_ = std::min(confirm_count_ + 1, 1000000);
  } else {
    confirm_count_ = 0;
    confirmed_ = false;
    if (clamped <= 0.0) {
      filtered_rate_m_s_ = std::min(filtered_rate_m_s_, kActiveEps);
    }
  }
  if (confirm_count_ >= std::max(1, params_.confirm_frames)) {
    confirmed_ = true;
  }
  return rate();
}
