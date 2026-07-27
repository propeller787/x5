/**
 * @file output_shaper.cpp
 * @brief 输出斜率限制实现
 */
#include "safety_node/output_shaper.h"

#include <algorithm>
#include <cmath>

OutputShaper::OutputShaper() = default;

geometry_msgs::msg::Twist OutputShaper::shapeTowardTarget(
  const geometry_msgs::msg::Twist & target,
  double dt_s)
{
  geometry_msgs::msg::Twist out = target;
  if (!has_last_) {
    last_ = target;
    has_last_ = true;
    last_linear_accel_m_s2_ = 0.0;
    return out;
  }

  const double valid_dt_s = std::max(
    1e-3, std::isfinite(dt_s) ? dt_s : 0.0);
  double linear_target = out.linear.x;
  if (linear_target * last_.linear.x < 0.0) {
    linear_target = 0.0;
  }

  const bool reducing_magnitude =
    linear_target * last_.linear.x < 0.0 ||
    std::abs(linear_target) < std::abs(last_.linear.x);
  const double accel_limit = reducing_magnitude ?
    max_linear_decel_m_s2_ : max_linear_accel_m_s2_;
  const double desired_accel = std::clamp(
    (linear_target - last_.linear.x) / valid_dt_s,
    -accel_limit,
    accel_limit);
  const double max_accel_change = max_linear_jerk_m_s3_ * valid_dt_s;
  double next_accel = std::clamp(
    desired_accel,
    last_linear_accel_m_s2_ - max_accel_change,
    last_linear_accel_m_s2_ + max_accel_change);
  double next_linear = last_.linear.x + next_accel * valid_dt_s;
  const double target_delta = linear_target - last_.linear.x;
  const double shaped_delta = next_linear - last_.linear.x;
  if (std::abs(target_delta) < 1e-12 ||
    (target_delta > 0.0 && shaped_delta >= target_delta) ||
    (target_delta < 0.0 && shaped_delta <= target_delta))
  {
    next_linear = linear_target;
    next_accel = 0.0;
  }
  out.linear.x = next_linear;
  last_linear_accel_m_s2_ = next_accel;

  out.angular.z = slewAngularToward(out.angular.z, valid_dt_s);

  last_ = out;
  return out;
}

double OutputShaper::slewAngularToward(double target, double dt_s) const
{
  // 方向反转时不再直接归零，而是按角减速度向零方向 slew，下一周期再从零向反方向爬升。
  // 这消除了旧实现中方向切换时的单周期零速顿挫，同时仍保证过零过程受控。
  const double valid_dt = std::max(1e-3, std::isfinite(dt_s) ? dt_s : 0.0);
  const double current = last_.angular.z;

  if (target * current < 0.0) {
    // 方向反转：先向零方向按减速 slew rate 收敛
    const double step = max_angular_decel_rad_s2_ * valid_dt;
    if (current > 0.0) {
      return std::max(0.0, current - step);
    }
    return std::min(0.0, current + step);
  }

  if (std::abs(target) > std::abs(current)) {
    // 增大幅值：按加速 slew rate 限制每周期变化量
    const double max_delta = max_angular_accel_rad_s2_ * valid_dt;
    const double dw = target - current;
    if (dw > max_delta) {
      return current + max_delta;
    }
    if (dw < -max_delta) {
      return current - max_delta;
    }
  }
  // 减小幅值或同向小增量：不限速，直接跟目标
  return target;
}

geometry_msgs::msg::Twist OutputShaper::shape(
  const geometry_msgs::msg::Twist & input,
  double dt_s,
  bool emergency_stop,
  bool recovery_active,
  bool hard_emergency_stop)
{
  if (emergency_stop && hard_emergency_stop) {
    last_ = geometry_msgs::msg::Twist{};
    has_last_ = true;
    last_linear_accel_m_s2_ = 0.0;
    emergency_holding_ = true;
    emergency_clear_time_s_ = 0.0;
    return last_;
  }

  if (emergency_stop) {
    const geometry_msgs::msg::Twist stopped =
      shapeTowardTarget(geometry_msgs::msg::Twist{}, dt_s);
    const bool still_moving =
      std::abs(stopped.linear.x) > 1e-4 ||
      std::abs(stopped.angular.z) > 1e-4;
    if (still_moving) {
      emergency_holding_ = false;
      return stopped;
    }
    last_ = geometry_msgs::msg::Twist{};
    has_last_ = true;
    last_linear_accel_m_s2_ = 0.0;
    emergency_holding_ = true;
    emergency_clear_time_s_ = 0.0;
    return last_;
  }

  if (recovery_active) {
    emergency_holding_ = false;
    emergency_clear_time_s_ = 0.0;
  } else if (emergency_holding_) {
    const double valid_dt_s = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;
    emergency_clear_time_s_ += valid_dt_s;
    if (emergency_clear_time_s_ < emergency_release_delay_s_) {
      last_ = geometry_msgs::msg::Twist{};
      has_last_ = true;
      last_linear_accel_m_s2_ = 0.0;
      return last_;
    }
    emergency_holding_ = false;
    emergency_clear_time_s_ = 0.0;
  }

  return shapeTowardTarget(input, dt_s);
}

geometry_msgs::msg::Twist OutputShaper::lowPassToZero(
  double dt_s,
  double time_constant_s)
{
  if (!has_last_) {
    last_ = geometry_msgs::msg::Twist{};
    has_last_ = true;
    last_linear_accel_m_s2_ = 0.0;
    return last_;
  }

  const double valid_dt_s = std::max(
    1e-3, std::isfinite(dt_s) ? dt_s : 0.0);
  const double valid_time_constant_s =
    std::max(1e-3, std::isfinite(time_constant_s) ? time_constant_s : 0.30);
  const double alpha = std::exp(-valid_dt_s / valid_time_constant_s);

  geometry_msgs::msg::Twist out;
  out.linear.x = alpha * last_.linear.x;
  out.angular.z = alpha * last_.angular.z;

  constexpr double kLinearStopThreshold = 0.01;
  constexpr double kAngularStopThreshold = 0.02;
  if (std::abs(out.linear.x) <= kLinearStopThreshold) {
    out.linear.x = 0.0;
  }
  if (std::abs(out.angular.z) <= kAngularStopThreshold) {
    out.angular.z = 0.0;
  }

  last_ = out;
  // 低通减速不向常规 jerk 整形器遗留较大的负加速度状态，
  // 避免驾驶员再次推杆时输出继续向负方向变化。
  last_linear_accel_m_s2_ = 0.0;
  emergency_holding_ = false;
  emergency_clear_time_s_ = 0.0;
  return last_;
}

geometry_msgs::msg::Twist OutputShaper::lowPassLinearToZero(
  double dt_s,
  double time_constant_s,
  double angular_target)
{
  if (!has_last_) {
    last_ = geometry_msgs::msg::Twist{};
    has_last_ = true;
    last_linear_accel_m_s2_ = 0.0;
    return last_;
  }

  const double valid_dt_s = std::max(
    1e-3, std::isfinite(dt_s) ? dt_s : 0.0);
  const double valid_time_constant_s =
    std::max(1e-3, std::isfinite(time_constant_s) ? time_constant_s : 0.30);
  const double alpha = std::exp(-valid_dt_s / valid_time_constant_s);

  geometry_msgs::msg::Twist out;
  out.linear.x = alpha * last_.linear.x;
  constexpr double kLinearStopThreshold = 0.01;
  if (std::abs(out.linear.x) <= kLinearStopThreshold) {
    out.linear.x = 0.0;
  }
  // 角速度不参与低通：按目标限幅下发，保留原地/边走边转的脱离能力。
  out.angular.z = slewAngularToward(
    std::isfinite(angular_target) ? angular_target : 0.0, valid_dt_s);

  last_ = out;
  // 与 lowPassToZero 同理：不向 jerk 整形器遗留负加速度状态。
  last_linear_accel_m_s2_ = 0.0;
  emergency_holding_ = false;
  emergency_clear_time_s_ = 0.0;
  return last_;
}

geometry_msgs::msg::Twist OutputShaper::fastStopToZero(
  double dt_s,
  double max_decel_m_s2,
  double max_jerk_m_s3)
{
  if (!has_last_) {
    last_ = geometry_msgs::msg::Twist{};
    has_last_ = true;
    last_linear_accel_m_s2_ = 0.0;
    return last_;
  }

  const double valid_dt_s = std::max(
    1e-3, std::isfinite(dt_s) ? dt_s : 0.0);
  const double decel = std::isfinite(max_decel_m_s2) ?
    std::max(1e-3, max_decel_m_s2) :
    max_linear_decel_m_s2_;
  const double jerk = std::isfinite(max_jerk_m_s3) ?
    std::max(1e-3, max_jerk_m_s3) :
    max_linear_jerk_m_s3_;

  geometry_msgs::msg::Twist out;
  const double current = last_.linear.x;
  if (std::abs(current) <= 1e-4) {
    out.linear.x = 0.0;
    last_linear_accel_m_s2_ = 0.0;
  } else {
    const double desired_accel = current > 0.0 ? -decel : decel;
    const double max_accel_change = jerk * valid_dt_s;
    double next_accel = std::clamp(
      desired_accel,
      last_linear_accel_m_s2_ - max_accel_change,
      last_linear_accel_m_s2_ + max_accel_change);
    double next_linear = current + next_accel * valid_dt_s;
    if (current * next_linear <= 0.0) {
      next_linear = 0.0;
      next_accel = 0.0;
    }
    out.linear.x = next_linear;
    last_linear_accel_m_s2_ = next_accel;
  }

  // 角速度按减速 slew rate 向零收敛（与 slewAngularToward 过零逻辑一致）
  const double angular_step = max_angular_decel_rad_s2_ * valid_dt_s;
  if (last_.angular.z > 0.0) {
    out.angular.z = std::max(0.0, last_.angular.z - angular_step);
  } else if (last_.angular.z < 0.0) {
    out.angular.z = std::min(0.0, last_.angular.z + angular_step);
  }

  last_ = out;
  // fastStopToZero 是方向反转快速刹停，不是急停释放。
  // 不清除 emergency_holding_，避免绕过 emergency_release_delay_s_ 等待时间。
  // 后续 shape() 调用会通过正常流程判断是否释放急停锁存。
  return out;
}

void OutputShaper::setEmergencyReleaseDelay(double delay_s)
{
  emergency_release_delay_s_ =
    std::isfinite(delay_s) ? std::max(0.0, delay_s) : 0.15;
}

void OutputShaper::setMotionLimits(
  double max_accel_m_s2,
  double max_decel_m_s2,
  double max_jerk_m_s3)
{
  max_linear_accel_m_s2_ =
    std::isfinite(max_accel_m_s2) ? std::max(1e-3, max_accel_m_s2) : 1.2;
  max_linear_decel_m_s2_ =
    std::isfinite(max_decel_m_s2) ? std::max(1e-3, max_decel_m_s2) : 1.2;
  max_linear_jerk_m_s3_ =
    std::isfinite(max_jerk_m_s3) ? std::max(1e-3, max_jerk_m_s3) : 2.5;
}

void OutputShaper::setAngularSlewRate(
  double max_angular_accel_rad_s2,
  double max_angular_decel_rad_s2)
{
  max_angular_accel_rad_s2_ =
    std::isfinite(max_angular_accel_rad_s2) ?
    std::max(1e-3, max_angular_accel_rad_s2) : 2.0;
  max_angular_decel_rad_s2_ =
    std::isfinite(max_angular_decel_rad_s2) ?
    std::max(1e-3, max_angular_decel_rad_s2) : 2.0;
}

void OutputShaper::reset()
{
  last_ = geometry_msgs::msg::Twist{};
  has_last_ = false;
  emergency_holding_ = false;
  emergency_clear_time_s_ = 0.0;
  last_linear_accel_m_s2_ = 0.0;
}
