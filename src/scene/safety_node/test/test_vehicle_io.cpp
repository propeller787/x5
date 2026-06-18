/**
 * @file test_vehicle_io.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/vehicle_io.h"

#include "../../../common/utils/topic.h"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class VehicleIoTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(VehicleIoTest, ControlCommandExpiresAfterTimeout)
{
  auto node = std::make_shared<rclcpp::Node>("vehicle_io_control_timeout_test");
  VehicleIo io(node);
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(handleControlTopic, 10);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.8;
  publisher->publish(command);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  ControlCommand snapshot;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    snapshot = io.desiredControl(node->now(), 0.25);
    if (snapshot.received) {
      break;
    }
    publisher->publish(command);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(snapshot.received);
  ASSERT_TRUE(snapshot.fresh);
  EXPECT_DOUBLE_EQ(snapshot.twist.linear.x, 0.8);

  const ControlCommand expired =
    io.desiredControl(snapshot.timestamp + rclcpp::Duration::from_seconds(0.3), 0.25);
  EXPECT_FALSE(expired.fresh);
  EXPECT_DOUBLE_EQ(expired.twist.linear.x, 0.0);

  const ControlCommand clock_rollback =
    io.desiredControl(snapshot.timestamp - rclcpp::Duration::from_seconds(0.1), 0.25);
  EXPECT_FALSE(clock_rollback.fresh);
  EXPECT_DOUBLE_EQ(clock_rollback.twist.linear.x, 0.0);
}

TEST_F(VehicleIoTest, FreshCenteredJoystickOverridesDecayingControl)
{
  auto node = std::make_shared<rclcpp::Node>("vehicle_io_joystick_neutral_test");
  VehicleIo io(node);
  auto control_publisher =
    node->create_publisher<geometry_msgs::msg::Twist>(handleControlTopic, 10);
  auto joystick_publisher =
    node->create_publisher<interface ::msg::Joystick>(joystickTopic, 10);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.8;
  interface ::msg::Joystick joystick;
  joystick.rocker_x = 3;
  joystick.rocker_y = -4;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  ControlCommand snapshot;
  while (std::chrono::steady_clock::now() < deadline) {
    control_publisher->publish(command);
    joystick_publisher->publish(joystick);
    executor.spin_some();
    snapshot = io.desiredControl(node->now(), 0.25, 0.25, 5);
    if (snapshot.received && snapshot.driver_neutral) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(snapshot.received);
  ASSERT_TRUE(snapshot.fresh);
  ASSERT_TRUE(snapshot.driver_neutral);
  EXPECT_DOUBLE_EQ(snapshot.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.twist.angular.z, 0.0);
}

TEST_F(VehicleIoTest, WheelFeedbackExpiresAfterTimeout)
{
  auto node = std::make_shared<rclcpp::Node>("vehicle_io_wheel_timeout_test");
  VehicleIo io(node);
  auto publisher =
    node->create_publisher<geometry_msgs::msg::TwistStamped>(carWheelSpeedStaTopic, 10);

  geometry_msgs::msg::TwistStamped feedback;
  feedback.twist.linear.x = 0.7;
  feedback.twist.angular.z = 0.9;
  publisher->publish(feedback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  WheelFeedback snapshot;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    snapshot = io.wheelFeedback(node->now(), 0.25);
    if (snapshot.received) {
      break;
    }
    publisher->publish(feedback);
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(snapshot.received);
  ASSERT_TRUE(snapshot.fresh);
  EXPECT_DOUBLE_EQ(snapshot.linear_velocity, 0.7);
  EXPECT_DOUBLE_EQ(snapshot.angular_velocity, 0.9);

  const WheelFeedback expired =
    io.wheelFeedback(snapshot.timestamp + rclcpp::Duration::from_seconds(0.3), 0.25);
  EXPECT_FALSE(expired.fresh);
  EXPECT_DOUBLE_EQ(expired.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(expired.angular_velocity, 0.0);
}

TEST_F(VehicleIoTest, FrozenWheelTimestampExpiresDespiteRepeatedDelivery)
{
  auto node = std::make_shared<rclcpp::Node>("vehicle_io_frozen_wheel_stamp_test");
  VehicleIo io(node);
  auto publisher =
    node->create_publisher<geometry_msgs::msg::TwistStamped>(carWheelSpeedStaTopic, 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  geometry_msgs::msg::TwistStamped feedback;
  feedback.header.stamp = node->now();
  feedback.twist.linear.x = 0.7;
  feedback.twist.angular.z = 0.7;
  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline) {
    publisher->publish(feedback);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  const WheelFeedback snapshot = io.wheelFeedback(node->now(), 0.12);
  EXPECT_TRUE(snapshot.received);
  EXPECT_FALSE(snapshot.fresh);
  EXPECT_DOUBLE_EQ(snapshot.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.angular_velocity, 0.0);
}

TEST_F(VehicleIoTest, OutOfOrderWheelFeedbackDoesNotReplaceLatestValue)
{
  auto node = std::make_shared<rclcpp::Node>("vehicle_io_out_of_order_test");
  VehicleIo io(node);
  auto publisher =
    node->create_publisher<geometry_msgs::msg::TwistStamped>(carWheelSpeedStaTopic, 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  geometry_msgs::msg::TwistStamped latest;
  latest.header.stamp = node->now();
  latest.twist.linear.x = 0.8;
  latest.twist.angular.z = 0.2;
  publisher->publish(latest);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  geometry_msgs::msg::TwistStamped older = latest;
  older.header.stamp =
    rclcpp::Time(latest.header.stamp) - rclcpp::Duration::from_seconds(0.02);
  older.twist.linear.x = 0.1;
  older.twist.angular.z = 0.1;
  publisher->publish(older);
  for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  const WheelFeedback snapshot = io.wheelFeedback(node->now(), 0.25);
  ASSERT_TRUE(snapshot.fresh);
  EXPECT_DOUBLE_EQ(snapshot.linear_velocity, 0.8);
  EXPECT_DOUBLE_EQ(snapshot.angular_velocity, 0.2);
}
