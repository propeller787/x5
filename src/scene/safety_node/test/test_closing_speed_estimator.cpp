/**
 * @file test_closing_speed_estimator.cpp
 * @brief 方案 5 逼近速度估计单测：收敛 / 自车扣除 / 跳变复位 / 关闭态
 */
#include <gtest/gtest.h>

#include "safety_node/core/closing_speed_estimator.h"

#include <cmath>

namespace
{

ClosingSpeedParams enabledParams()
{
  ClosingSpeedParams params;
  params.enabled = true;
  params.filter_alpha = 0.4;
  params.confirm_frames = 3;
  params.max_rate_m_s = 2.0;
  params.jump_reset_m = 0.30;
  return params;
}

}  // namespace

TEST(ClosingSpeedEstimator, ConvergesToApproachRate)
{
  ClosingSpeedEstimator estimator(enabledParams());
  const double dt = 0.02;
  const double true_rate = 0.5;   // 障碍以 0.5 m/s 逼近，自车静止
  double clearance = 2.0;
  double rate = 0.0;
  for (int i = 0; i < 100; ++i) {
    clearance -= true_rate * dt;
    rate = estimator.update(clearance, 0.0, dt);
  }
  EXPECT_NEAR(rate, true_rate, 0.2 * true_rate);   // 收敛误差 < 20%
}

TEST(ClosingSpeedEstimator, StaticObstacleWithEgoMotionIsZero)
{
  ClosingSpeedEstimator estimator(enabledParams());
  const double dt = 0.02;
  const double ego_speed = 0.8;   // 自车前进，障碍静止：净空缩小全由自车贡献
  double clearance = 3.0;
  double rate = 0.0;
  for (int i = 0; i < 100; ++i) {
    clearance -= ego_speed * dt;
    rate = estimator.update(clearance, ego_speed, dt);
  }
  EXPECT_NEAR(rate, 0.0, 0.05);
}

TEST(ClosingSpeedEstimator, JumpResetsEstimate)
{
  ClosingSpeedEstimator estimator(enabledParams());
  const double dt = 0.02;
  double clearance = 1.0;
  for (int i = 0; i < 50; ++i) {
    clearance -= 0.5 * dt;
    estimator.update(clearance, 0.0, dt);
  }
  EXPECT_GT(estimator.rate(), 0.1);
  // 净空突增（漏检恢复/障碍离开）→ 估计复位为零
  const double rate = estimator.update(clearance + 0.5, 0.0, dt);
  EXPECT_DOUBLE_EQ(rate, 0.0);
}

TEST(ClosingSpeedEstimator, NonFiniteClearanceResets)
{
  ClosingSpeedEstimator estimator(enabledParams());
  const double dt = 0.02;
  double clearance = 1.0;
  for (int i = 0; i < 50; ++i) {
    clearance -= 0.5 * dt;
    estimator.update(clearance, 0.0, dt);
  }
  const double rate = estimator.update(
    std::numeric_limits<double>::infinity(), 0.0, dt);
  EXPECT_DOUBLE_EQ(rate, 0.0);
}

TEST(ClosingSpeedEstimator, SingleFrameNoiseFilteredByConfirm)
{
  ClosingSpeedEstimator estimator(enabledParams());
  const double dt = 0.02;
  estimator.update(1.0, 0.0, dt);
  // 单帧 3cm 抖动（等效 1.5 m/s）：未达确认帧数，不得输出
  const double rate = estimator.update(0.97, 0.0, dt);
  EXPECT_DOUBLE_EQ(rate, 0.0);
}

TEST(ClosingSpeedEstimator, OutputClampedToMaxRate)
{
  ClosingSpeedEstimator estimator(enabledParams());
  const double dt = 0.02;
  double clearance = 5.0;
  double rate = 0.0;
  for (int i = 0; i < 100; ++i) {
    clearance -= 5.0 * dt;   // 疯狂逼近 5 m/s
    rate = estimator.update(clearance, 0.0, dt);
  }
  EXPECT_LE(rate, 2.0 + 1e-9);
}

TEST(ClosingSpeedEstimator, DisabledAlwaysZero)
{
  ClosingSpeedParams params = enabledParams();
  params.enabled = false;
  ClosingSpeedEstimator estimator(params);
  const double dt = 0.02;
  double clearance = 1.0;
  for (int i = 0; i < 50; ++i) {
    clearance -= 0.5 * dt;
    EXPECT_DOUBLE_EQ(estimator.update(clearance, 0.0, dt), 0.0);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
