#include <gtest/gtest.h>

#include "safety_node/joystick_utils.h"

#include <cmath>

TEST(JoystickUtilsTest, DeadzoneZerosTwist)
{
  interface::msg::Joystick joystick;
  joystick.rocker_x = 3;
  joystick.rocker_y = -4;

  JoystickTwistMapping mapping;
  mapping.deadzone = 5;
  mapping.max_linear_m_s = 2.0;
  mapping.max_angular_rad_s = 1.0;

  const auto twist = JoystickUtils::toTwist(joystick, mapping, true);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(JoystickUtilsTest, ForwardAndRightStickMapToTwist)
{
  interface::msg::Joystick joystick;
  joystick.rocker_x = 50;
  joystick.rocker_y = 100;

  JoystickTwistMapping mapping;
  mapping.deadzone = 5;
  mapping.max_linear_m_s = 2.0;
  mapping.max_angular_rad_s = 1.0;

  const auto twist = JoystickUtils::toTwist(joystick, mapping, true);
  EXPECT_NEAR(twist.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(twist.angular.z, -1.0, 1e-9);
}

TEST(JoystickUtilsTest, StaleJoystickProducesZeroTwist)
{
  interface::msg::Joystick joystick;
  joystick.rocker_x = 80;
  joystick.rocker_y = 0;

  JoystickTwistMapping mapping;
  const auto twist = JoystickUtils::toTwist(joystick, mapping, false);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(JoystickUtilsTest, IntegrateStraightTrajectory)
{
  const auto points = JoystickUtils::integrateUnicycleTrajectory(
    1.0, 0.0, 0.5, 0.1, 0.06);
  ASSERT_EQ(points.size(), 6u);
  EXPECT_NEAR(points.front().x, 0.0, 1e-9);
  EXPECT_NEAR(points.back().x, 0.5, 1e-6);
  EXPECT_NEAR(points.back().y, 0.0, 1e-6);
}

TEST(JoystickUtilsTest, IntegrateArcTrajectory)
{
  const auto points = JoystickUtils::integrateUnicycleTrajectory(
    1.0, 1.0, 0.2, 0.1, 0.06);
  ASSERT_GE(points.size(), 2u);
  EXPECT_GT(points.back().y, 0.0);
}
