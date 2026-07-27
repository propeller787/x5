/**
 * @file test_output_shaper.cpp
 */
#include <gtest/gtest.h>

#include "safety_node/output_shaper.h"

#include <cmath>

TEST(OutputShaperTest, EmergencyStopUsesRateLimitedDeceleration)
{
  OutputShaper shaper;
  shaper.setMotionLimits(1.0, 0.8, 2.5);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  moving.angular.z = 0.5;
  shaper.shape(moving, 0.025);

  const geometry_msgs::msg::Twist stopping =
    shaper.shape(geometry_msgs::msg::Twist{}, 0.025, true, false, false);

  EXPECT_LT(stopping.linear.x, 1.0);
  EXPECT_GT(stopping.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(stopping.angular.z, 0.0);
}

TEST(OutputShaperTest, HardEmergencyStopStillInstant)
{
  OutputShaper shaper;

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  moving.angular.z = 0.5;
  shaper.shape(moving, 0.025);

  const geometry_msgs::msg::Twist stopped =
    shaper.shape(geometry_msgs::msg::Twist{}, 0.025, true, false, true);

  EXPECT_DOUBLE_EQ(stopped.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(stopped.angular.z, 0.0);
}

TEST(OutputShaperTest, RecoveryAfterEmergencyStartsFromZero)
{
  OutputShaper shaper;
  shaper.setEmergencyReleaseDelay(0.04);
  shaper.setMotionLimits(1.0, 0.8, 2.5);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  shaper.shape(moving, 0.025);
  for (int i = 0; i < 80; ++i) {
    shaper.shape(geometry_msgs::msg::Twist{}, 0.025, true, false, false);
  }

  const geometry_msgs::msg::Twist held = shaper.shape(moving, 0.025);
  EXPECT_DOUBLE_EQ(held.linear.x, 0.0);

  const geometry_msgs::msg::Twist recovering = shaper.shape(moving, 0.025);
  EXPECT_NEAR(recovering.linear.x, 0.0015625, 1e-9);
}

TEST(OutputShaperTest, ActiveRecoveryBypassesEmergencyReleaseDelay)
{
  OutputShaper shaper;
  shaper.setEmergencyReleaseDelay(0.50);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = -0.12;
  moving.angular.z = 0.35;
  shaper.shape(geometry_msgs::msg::Twist{}, 0.025, true);

  const geometry_msgs::msg::Twist recovery =
    shaper.shape(moving, 0.025, false, true);

  EXPECT_LT(recovery.linear.x, 0.0);
  EXPECT_NEAR(recovery.linear.x, -0.0015625, 1e-9);
  EXPECT_GT(recovery.angular.z, 0.0);
}

TEST(OutputShaperTest, EmergencyReleaseRequiresContinuousClearTime)
{
  OutputShaper shaper;
  shaper.setEmergencyReleaseDelay(0.09);
  shaper.setMotionLimits(1.0, 0.8, 2.5);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  shaper.shape(moving, 0.025);
  for (int i = 0; i < 80; ++i) {
    shaper.shape(geometry_msgs::msg::Twist{}, 0.025, true, false, false);
  }

  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(shaper.shape(moving, 0.025).linear.x, 0.0);
    EXPECT_DOUBLE_EQ(
      shaper.shape(geometry_msgs::msg::Twist{}, 0.025, true, false, false).linear.x, 0.0);
  }

  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(shaper.shape(moving, 0.025).linear.x, 0.0);
  }
  EXPECT_NEAR(shaper.shape(moving, 0.025).linear.x, 0.0015625, 1e-9);
}

TEST(OutputShaperTest, SafetyReductionUsesJerkLimitedDeceleration)
{
  OutputShaper shaper;
  shaper.setMotionLimits(1.0, 1.0, 2.0);

  geometry_msgs::msg::Twist moving;
  moving.linear.x = 1.0;
  moving.angular.z = 0.5;
  shaper.shape(moving, 0.1);

  geometry_msgs::msg::Twist reduced;
  reduced.linear.x = 0.2;
  reduced.angular.z = 0.1;
  const geometry_msgs::msg::Twist first = shaper.shape(reduced, 0.1);
  const geometry_msgs::msg::Twist second = shaper.shape(reduced, 0.1);

  EXPECT_NEAR(first.linear.x, 0.98, 1e-9);
  EXPECT_NEAR(second.linear.x, 0.94, 1e-9);
  EXPECT_DOUBLE_EQ(first.angular.z, reduced.angular.z);
}

TEST(OutputShaperTest, NormalAccelerationIsRateLimited)
{
  OutputShaper shaper;
  shaper.shape(geometry_msgs::msg::Twist{}, 0.025);
  geometry_msgs::msg::Twist target;
  target.linear.x = 1.0;
  target.angular.z = 1.0;
  const auto output = shaper.shape(target, 0.025);
  EXPECT_NEAR(output.linear.x, 0.0015625, 1e-9);
  EXPECT_DOUBLE_EQ(output.angular.z, 0.05);
}

TEST(OutputShaperTest, DirectionChangesStopBeforeReversing)
{
  OutputShaper shaper;
  geometry_msgs::msg::Twist positive;
  positive.linear.x = 0.2;
  positive.angular.z = 0.2;
  shaper.shape(positive, 0.025);
  geometry_msgs::msg::Twist negative;
  negative.linear.x = -1.0;
  negative.angular.z = -1.0;
  auto stopped = shaper.shape(negative, 0.025);
  EXPECT_GT(stopped.linear.x, 0.0);
  // 方向反转时角速度按 slew rate 平滑过零，不再单周期归零
  // 0.2 - max_angular_decel_rad_s2_(2.0) * dt(0.025) = 0.2 - 0.05 = 0.15
  EXPECT_NEAR(stopped.angular.z, 0.15, 1e-9);
  for (int i = 0; i < 100 && stopped.linear.x > 1e-9; ++i) {
    stopped = shaper.shape(negative, 0.025);
    EXPECT_GE(stopped.linear.x, 0.0);
  }
  EXPECT_NEAR(stopped.linear.x, 0.0, 1e-9);
  EXPECT_LT(shaper.shape(negative, 0.025).linear.x, 0.0);

  shaper.reset();
  geometry_msgs::msg::Twist initial;
  initial.linear.x = -0.1;
  initial.angular.z = -0.1;
  shaper.shape(initial, 0.025);
  const auto limited = shaper.shape(negative, 0.025);
  EXPECT_NEAR(limited.linear.x, -0.1015625, 1e-9);
  // 同向增幅（-0.1 → -1.0 均为负，非反转）：按加速 slew rate 限幅
  // max_angular_accel_rad_s2_(2.0) * dt(0.025) = 0.05 → -0.1 - 0.05 = -0.15
  EXPECT_NEAR(limited.angular.z, -0.15, 1e-9);
}

TEST(OutputShaperTest, LowPassLinearToZeroPreservesSteering)
{
  OutputShaper shaper;
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.4;
  moving.angular.z = 0.3;
  shaper.shape(moving, 0.05);

  const auto out = shaper.lowPassLinearToZero(0.05, 0.30, 0.3);

  // 线速度按 exp(-dt/τ) 衰减
  EXPECT_NEAR(out.linear.x, 0.4 * std::exp(-0.05 / 0.30), 1e-9);
  // 角速度不参与衰减：前障贴身时仍要能转向脱离
  EXPECT_DOUBLE_EQ(out.angular.z, 0.3);
}

TEST(OutputShaperTest, LowPassToZeroStillDecaysSteering)
{
  OutputShaper shaper;
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.4;
  moving.angular.z = 0.3;
  shaper.shape(moving, 0.05);

  // 对照组：摇杆回中用的双轴低通仍旧把角速度一起收敛
  const auto out = shaper.lowPassToZero(0.05, 0.30);
  EXPECT_NEAR(out.angular.z, 0.3 * std::exp(-0.05 / 0.30), 1e-9);
}

TEST(OutputShaperTest, LowPassLinearToZeroRateLimitsAngularTarget)
{
  OutputShaper shaper;
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.4;
  moving.angular.z = 0.1;
  shaper.shape(moving, 0.05);

  // 保留转向不等于放开转向：仍走 slewAngularToward 的 |Δωz| 限幅
  // 默认 max_angular_accel_rad_s2_=2.0, dt=0.05 → max_delta=0.1
  // 从 0.1 增大到 1.0：0.1 + 0.1 = 0.2
  const auto out = shaper.lowPassLinearToZero(0.05, 0.30, 1.0);
  EXPECT_NEAR(out.angular.z, 0.2, 1e-9);
}

// 低通滑行距离闭式解 d = τ·v₀：0.35 m/s + τ=0.30 应停在约 10 cm 内，
// 即 16 cm 触发时仍留得住 stop_buffer。
TEST(OutputShaperTest, LowPassLinearToZeroSlideDistanceMatchesTauTimesSpeed)
{
  OutputShaper shaper;
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.35;
  shaper.shape(moving, 0.05);

  constexpr double kDt = 0.05;
  constexpr double kTau = 0.30;
  double distance = 0.0;
  geometry_msgs::msg::Twist out = moving;
  int steps = 0;
  for (; steps < 500 && out.linear.x > 0.0; ++steps) {
    out = shaper.lowPassLinearToZero(kDt, kTau, 0.0);
    distance += out.linear.x * kDt;
  }

  EXPECT_DOUBLE_EQ(out.linear.x, 0.0);
  EXPECT_LT(steps, 500);
  // 连续解 τ·(v₀ − 停止阈值) = 0.102 m；离散求和略小。
  EXPECT_GT(distance, 0.08);
  EXPECT_LT(distance, 0.11);
}

TEST(OutputShaperTest, LinearAccelerationChangeIsJerkBounded)
{
  OutputShaper shaper;
  shaper.setMotionLimits(1.0, 1.0, 2.0);
  shaper.shape(geometry_msgs::msg::Twist{}, 0.1);

  geometry_msgs::msg::Twist target;
  target.linear.x = 1.0;
  const double first_velocity = shaper.shape(target, 0.1).linear.x;
  const double second_velocity = shaper.shape(target, 0.1).linear.x;
  const double first_accel = first_velocity / 0.1;
  const double second_accel = (second_velocity - first_velocity) / 0.1;

  EXPECT_NEAR(first_accel, 0.2, 1e-9);
  EXPECT_NEAR(second_accel, 0.4, 1e-9);
  EXPECT_LE(std::abs(second_accel - first_accel), 2.0 * 0.1 + 1e-9);
}
