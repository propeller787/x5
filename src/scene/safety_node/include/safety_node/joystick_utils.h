/**
 * @file joystick_utils.h
 * @brief 原始摇杆 → Twist 映射与差速轨迹积分（调试用）
 */
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <interface/msg/joystick.hpp>

#include <vector>

struct JoystickTwistMapping
{
  int deadzone = 5;
  double max_linear_m_s = 2.22;
  double max_angular_rad_s = 0.8;
};

class JoystickUtils {
public:
  /** 将 /sensor/joystick 原始 rocker 映射为 base_link 下期望 (v, w) */
  static geometry_msgs::msg::Twist toTwist(
    const interface::msg::Joystick & joystick,
    const JoystickTwistMapping & mapping,
    bool fresh);

  /** 恒速差速模型积分，返回 LINE_STRIP 用点列（首点为原点） */
  static std::vector<geometry_msgs::msg::Point> integrateUnicycleTrajectory(
    double linear_x,
    double angular_z,
    double horizon_s,
    double dt_s,
    double z = 0.06);
};
