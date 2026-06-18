/**
 * @file output_shaper.h
 * @brief 控制输出斜率限制（OutputShaper）
 *
 * 对 AvoidanceController 输出的 Twist 做速率限制，避免突变：
 * - 线速度：每周期变化量 + 加速度上限
 * - 角速度：每周期变化量上限
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>

class OutputShaper {
public:
  OutputShaper();

  /**
   * @brief 整形输出
   * @param input 避障后的原始 Twist
   * @param dt_s  控制周期，用于加速度限制
   * @param emergency_stop 紧急停止时向零速收敛
   * @param recovery_active 脱困命令绕过急停释放等待
   * @param hard_emergency_stop 真急停（ESTOP/输入故障）时立即清零
   */
  geometry_msgs::msg::Twist shape(
    const geometry_msgs::msg::Twist & input,
    double dt_s,
    bool emergency_stop = false,
    bool recovery_active = false,
    bool hard_emergency_stop = false);

  /**
   * @brief 原始摇杆回中时，使用一阶低通平滑收敛到零速
   * @param dt_s 控制周期
   * @param time_constant_s 一阶低通时间常数，越大刹停越柔和
   */
  geometry_msgs::msg::Twist lowPassToZero(double dt_s, double time_constant_s);

  void setEmergencyReleaseDelay(double delay_s);
  void setMotionLimits(
    double max_accel_m_s2,
    double max_decel_m_s2,
    double max_jerk_m_s3);
  void reset();

private:
  geometry_msgs::msg::Twist shapeTowardTarget(
    const geometry_msgs::msg::Twist & target,
    double dt_s);

  geometry_msgs::msg::Twist last_;
  bool has_last_ = false;
  bool emergency_holding_ = false;
  double emergency_clear_time_s_ = 0.0;
  double emergency_release_delay_s_ = 0.15;
  double max_angular_delta_ = 0.05;      ///< 每周期 |Δωz| 上限 (rad/s)
  double max_linear_accel_m_s2_ = 1.2;     ///< 线加速度上限 (m/s²)
  double max_linear_decel_m_s2_ = 1.2;
  double max_linear_jerk_m_s3_ = 2.5;
  double last_linear_accel_m_s2_ = 0.0;
};
