/**
 * @file vehicle_io.cpp
 * @brief 手柄与轮速话题订阅实现
 */
#include "safety_node/vehicle_io.h"

#include "../../../common/utils/topic.h"

#include <algorithm>
#include <cmath>

VehicleIo::VehicleIo(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node)
{
  // 将输入订阅放入独立 callback group，避免与控制定时器串行阻塞
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;

  handle_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    handleControlTopic, 10,
    std::bind(&VehicleIo::onHandle, this, std::placeholders::_1), sub_options);

  joystick_sub_ = node_->create_subscription<interface ::msg::Joystick>(
    joystickTopic, 10,
    std::bind(&VehicleIo::onJoystick, this, std::placeholders::_1), sub_options);

  wheel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    carWheelSpeedStaTopic, 10,
    std::bind(&VehicleIo::onWheel, this, std::placeholders::_1), sub_options);

  car_info_sub_ = node_->create_subscription<interface::msg::CarInfo>(
    carInfoTopic, 10,
    std::bind(&VehicleIo::onCarInfo, this, std::placeholders::_1), sub_options);
}

bool VehicleIo::isFresh(
  bool received,
  const rclcpp::Time & timestamp,
  const rclcpp::Time & now,
  double timeout_s)
{
  // 容忍跨节点时钟的轻微超前（与 LidarPerception 保持一致），
  // 避免发布端时钟略快时把正常反馈误判为过期而触发输入故障急停。
  constexpr double kClockSkewToleranceS = 0.05;
  if (!received) {
    return false;
  }
  if (timeout_s <= 0.0) {
    return true;
  }
  const double age_s = (now - timestamp).seconds();
  return std::isfinite(age_s) && age_s >= -kClockSkewToleranceS && age_s <= timeout_s;
}

ControlCommand VehicleIo::desiredControl(
  const rclcpp::Time & now,
  double timeout_s,
  double joystick_timeout_s,
  int joystick_deadzone,
  int joystick_reverse_threshold) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ControlCommand out = control_;
  out.fresh = isFresh(out.received, out.timestamp, now, timeout_s);
  if (!out.fresh) {
    out.twist = geometry_msgs::msg::Twist{};
  }

  const bool joystick_fresh = isFresh(
    joystick_received_, joystick_timestamp_, now, joystick_timeout_s);
  const int deadzone = std::max(0, joystick_deadzone);
  const int reverse_threshold = std::max(0, joystick_reverse_threshold);
  out.driver_neutral =
    joystick_fresh &&
    std::abs(static_cast<int>(joystick_.rocker_x)) <= deadzone &&
    std::abs(static_cast<int>(joystick_.rocker_y)) <= deadzone;
  out.driver_reverse_request =
    joystick_fresh &&
    !out.driver_neutral &&
    static_cast<int>(joystick_.rocker_x) <= -reverse_threshold;
  if (out.driver_neutral) {
    out.twist = geometry_msgs::msg::Twist{};
  }
  return out;
}

WheelFeedback VehicleIo::wheelFeedback(const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  WheelFeedback out = wheels_;
  out.fresh = isFresh(out.received, out.timestamp, now, timeout_s);
  if (!out.fresh) {
    out.linear_velocity = 0.0;
    out.angular_velocity = 0.0;
  }
  return out;
}

JoystickSnapshot VehicleIo::latestJoystick(
  const rclcpp::Time & now,
  double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  JoystickSnapshot out;
  out.msg = joystick_;
  out.timestamp = joystick_timestamp_;
  out.received = joystick_received_;
  out.fresh = isFresh(joystick_received_, joystick_timestamp_, now, timeout_s);
  return out;
}

CarInfoState VehicleIo::carInfo(const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  CarInfoState out;
  out.received = car_info_received_;
  out.fresh = isFresh(car_info_received_, car_info_timestamp_, now, timeout_s);
  out.gear = static_cast<int>(car_info_.gear);
  out.mode_type = static_cast<int>(car_info_.mode_type);
  out.brake_engaged =
    car_info_.left_brakests == 1 || car_info_.right_brakests == 1;
  return out;
}

void VehicleIo::onHandle(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  control_.twist = *msg;
  control_.timestamp = node_->now();
  control_.received = true;
}

void VehicleIo::onJoystick(const interface ::msg::Joystick::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  joystick_ = *msg;
  joystick_timestamp_ = node_->now();
  joystick_received_ = true;
}

void VehicleIo::onWheel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  rclcpp::Time timestamp = node_->now();
  if (rclcpp::Time(msg->header.stamp).nanoseconds() > 0) {
    timestamp = rclcpp::Time(
      msg->header.stamp, node_->get_clock()->get_clock_type());
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (wheels_.received && timestamp <= wheels_.timestamp) {
    return;
  }
  wheels_.linear_velocity = msg->twist.linear.x;
  wheels_.angular_velocity = msg->twist.angular.z;
  wheels_.timestamp = timestamp;
  wheels_.received = true;
}

void VehicleIo::onCarInfo(const interface::msg::CarInfo::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  car_info_ = *msg;
  car_info_timestamp_ = node_->now();
  car_info_received_ = true;
}
