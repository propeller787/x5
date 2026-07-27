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

  /**
   * @brief 只把线速度一阶低通收敛到零，角速度仍按目标下发（保留转向脱离出路）
   *
   * 供前向近障低通刹停使用：车前贴身有障碍时压掉线速度即可，若连角速度一并衰减，
   * 驾驶员连「原地转个向脱离」都做不到（rotation_escape 要解决的正是这类冻结）。
   * 角速度走与 shapeTowardTarget 相同的每周期 |Δωz| 限幅，不做额外放宽。
   *
   * @param dt_s 控制周期
   * @param time_constant_s 线速度一阶低通时间常数
   * @param angular_target 本周期角速度目标（应取已过安全裁剪的候选角速度）
   */
  geometry_msgs::msg::Twist lowPassLinearToZero(
    double dt_s,
    double time_constant_s,
    double angular_target);

  /** Fast stop toward zero for an explicit direction reversal request. */
  geometry_msgs::msg::Twist fastStopToZero(
    double dt_s,
    double max_decel_m_s2,
    double max_jerk_m_s3);

  void setEmergencyReleaseDelay(double delay_s);
  void setMotionLimits(
    double max_accel_m_s2,
    double max_decel_m_s2,
    double max_jerk_m_s3);
  /**
   * @brief 设置角速度 slew rate（角加速度上限）
   * @param max_angular_accel_rad_s2 角速度增大方向（加速旋转）的每秒变化率上限
   * @param max_angular_decel_rad_s2 角速度减小方向（减速旋转 / 过零）的每秒变化率上限
   *
   * 替代旧硬编码 max_angular_delta_=0.05。调用方传入 AvoidanceParams 中已配置的
   * max_angular_decel_rad_s2 / max_angular_recover_rad_s2，使角速度响应特性
   * 可通过 YAML 调整且与控制周期解耦（内部按 dt_s 换算每周期步长）。
   */
  void setAngularSlewRate(
    double max_angular_accel_rad_s2,
    double max_angular_decel_rad_s2);
  void reset();

private:
  geometry_msgs::msg::Twist shapeTowardTarget(
    const geometry_msgs::msg::Twist & target,
    double dt_s);

  /** 角速度每周期限幅：方向反转时按 slew rate 平滑过零，增大幅值时限 |Δωz|，减小幅值不限速。
   *  dt_s 由调用方（shapeTowardTarget / lowPassLinearToZero）传入，使限幅与控制周期解耦。 */
  double slewAngularToward(double target, double dt_s) const;

  geometry_msgs::msg::Twist last_;
  bool has_last_ = false;
  bool emergency_holding_ = false;
  double emergency_clear_time_s_ = 0.0;
  double emergency_release_delay_s_ = 0.15;
  ///< 角速度增大方向（加速旋转）的每秒变化率上限 (rad/s²)
  double max_angular_accel_rad_s2_ = 2.0;
  ///< 角速度减小方向（减速旋转 / 过零）的每秒变化率上限 (rad/s²)
  double max_angular_decel_rad_s2_ = 2.0;
  double max_linear_accel_m_s2_ = 1.2;     ///< 线加速度上限 (m/s²)
  double max_linear_decel_m_s2_ = 1.2;
  double max_linear_jerk_m_s3_ = 2.5;
  double last_linear_accel_m_s2_ = 0.0;
};
