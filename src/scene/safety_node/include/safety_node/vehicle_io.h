/**
 * @file vehicle_io.h
 * @brief 车辆输入输出接口：手柄期望 + 轮速反馈
 *
 * 订阅话题（见 common/utils/topic.h）：
 *   /handle/control  — 驾驶员期望 Twist
 *   /sensor/joystick — 原始摇杆位置，用于识别驾驶员回中意图
 *   /wheel/twist     — 车体速度反馈，linear.x 为线速度，angular.z 为角速度
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <interface/msg/joystick.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

/** 车体速度反馈快照 */
struct WheelFeedback
{
  double linear_velocity = 0.0;      ///< 车体线速度 (m/s)
  double angular_velocity = 0.0;     ///< 车体角速度 (rad/s)
  rclcpp::Time timestamp {rclcpp::Time(0, 0, RCL_ROS_TIME)};
  bool received = false;
  bool fresh = false;
};

/** 手柄控制快照 */
struct ControlCommand
{
  geometry_msgs::msg::Twist twist;
  rclcpp::Time timestamp {rclcpp::Time(0, 0, RCL_ROS_TIME)};
  bool received = false;
  bool fresh = false;
  bool driver_neutral = false;
};

class VehicleIo {
public:
  explicit VehicleIo(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  /**
   * 最新手柄期望；过期或新鲜摇杆处于中心死区时 twist 自动归零。
   * driver_neutral=true 表示应忽略 /handle/control 的缓慢衰减并进入平滑回中刹停。
   */
  ControlCommand desiredControl(
    const rclcpp::Time & now,
    double timeout_s,
    double joystick_timeout_s = 0.25,
    int joystick_deadzone = 5) const;

  /** 最新车体速度反馈；过期时速度自动归零 */
  WheelFeedback wheelFeedback(const rclcpp::Time & now, double timeout_s) const;

private:
  static bool isFresh(
    bool received,
    const rclcpp::Time & timestamp,
    const rclcpp::Time & now,
    double timeout_s);

  void onHandle(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onJoystick(const interface::msg::Joystick::SharedPtr msg);
  void onWheel(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription < geometry_msgs::msg::Twist > ::SharedPtr handle_sub_;
  rclcpp::Subscription < interface::msg::Joystick > ::SharedPtr joystick_sub_;
  rclcpp::Subscription < geometry_msgs::msg::TwistStamped > ::SharedPtr wheel_sub_;
  mutable std::mutex mutex_;
  ControlCommand control_;
  interface::msg::Joystick joystick_;
  rclcpp::Time joystick_timestamp_ {rclcpp::Time(0, 0, RCL_ROS_TIME)};
  bool joystick_received_ = false;
  WheelFeedback wheels_;
};
