/**
 * @file test_front_tilt_speed_cap.cpp
 * @brief 斜雷达前向近障软限速闸单测，见 core/front_tilt_speed_cap.h
 */
#include <gtest/gtest.h>

#include "safety_node/core/front_tilt_speed_cap.h"

#include <cmath>
#include <limits>

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

FrontTiltSpeedCapParams defaultParams()
{
  FrontTiltSpeedCapParams p;    // 结构体默认值即 yaml 默认值
  p.enabled = true;
  p.confirm_frames = 2;
  return p;
}

FrontTiltSpeedCapInput makeInput(double clearance_m, double dt_s = 0.05)
{
  FrontTiltSpeedCapInput in;
  in.forward_clearance_m = clearance_m;
  in.dt_s = dt_s;
  return in;
}

}  // namespace

// 关闭时恒不限速，cap=+inf，与旧行为等价。
TEST(FrontTiltSpeedCapTest, DisabledNeverCaps)
{
  FrontTiltSpeedCapParams p;
  p.enabled = false;
  FrontTiltSpeedCapGate gate(p);
  const auto r = gate.update(makeInput(0.5));
  EXPECT_FALSE(r.active);
  EXPECT_FALSE(std::isfinite(r.speed_cap_m_s));    // +inf
  EXPECT_FALSE(gate.latched());
}

// 走廊内无障碍（+inf）不触发。
TEST(FrontTiltSpeedCapTest, NoObstacleNeverCaps)
{
  FrontTiltSpeedCapGate gate(defaultParams());
  for (int i = 0; i < 10; ++i) {
    const auto r = gate.update(makeInput(kInf));
    EXPECT_FALSE(r.active);
  }
}

// 净空 ≤ trigger_distance_m 连续 confirm_frames 帧后锁存并输出限速。
TEST(FrontTiltSpeedCapTest, LatchesAfterConfirmFrames)
{
  FrontTiltSpeedCapParams p = defaultParams();    // trigger=2.0, cap=1.25, confirm=2
  FrontTiltSpeedCapGate gate(p);

  const auto r1 = gate.update(makeInput(1.5));    // 第 1 帧在带内
  EXPECT_FALSE(r1.active);
  EXPECT_FALSE(r1.just_latched);

  const auto r2 = gate.update(makeInput(1.5));    // 第 2 帧 → 锁存
  EXPECT_TRUE(r2.active);
  EXPECT_TRUE(r2.just_latched);
  EXPECT_NEAR(r2.speed_cap_m_s, 1.25, 1e-9);
  EXPECT_TRUE(gate.latched());
}

// 触发带外的单帧抖动会清零确认计数，不误触发。
TEST(FrontTiltSpeedCapTest, ConfirmCounterResetsOnGap)
{
  FrontTiltSpeedCapGate gate(defaultParams());
  EXPECT_FALSE(gate.update(makeInput(1.5)).active);    // 计数 1
  EXPECT_FALSE(gate.update(makeInput(kInf)).active);   // 带外 → 清零
  EXPECT_FALSE(gate.update(makeInput(1.5)).active);    // 计数重新从 1 起
  EXPECT_TRUE(gate.update(makeInput(1.5)).active);     // 计数 2 → 锁存
}

// 滞回：净空恢复到 release 以上须持续 clear_hold_s 才解除；期间仍限速。
TEST(FrontTiltSpeedCapTest, ReleasesWithHysteresisHold)
{
  FrontTiltSpeedCapParams p = defaultParams();    // trigger=2.0 release=2.3 clear_hold=0.15
  p.confirm_frames = 1;
  FrontTiltSpeedCapGate gate(p);

  ASSERT_TRUE(gate.update(makeInput(1.0)).active);    // 立即锁存

  // 净空落在滞回带内（trigger<clearance<release）不算清空，保持限速。
  const auto mid = gate.update(makeInput(2.1, 0.05));
  EXPECT_TRUE(mid.active);
  EXPECT_FALSE(mid.just_released);

  // 净空 ≥ release：累计清空时间，未到 clear_hold_s 前仍限速。
  const auto c1 = gate.update(makeInput(2.5, 0.05));    // 累计 0.05s
  EXPECT_TRUE(c1.active);
  const auto c2 = gate.update(makeInput(2.5, 0.05));    // 累计 0.10s
  EXPECT_TRUE(c2.active);
  const auto c3 = gate.update(makeInput(2.5, 0.05));    // 累计 0.15s → 解除
  EXPECT_FALSE(c3.active);
  EXPECT_TRUE(c3.just_released);
  EXPECT_FALSE(gate.latched());
}

// 清空计时被带内回落打断后须重新累计，不会提前解除。
TEST(FrontTiltSpeedCapTest, ClearTimerResetsWhenObstacleReturns)
{
  FrontTiltSpeedCapParams p = defaultParams();
  p.confirm_frames = 1;
  FrontTiltSpeedCapGate gate(p);

  ASSERT_TRUE(gate.update(makeInput(1.0)).active);
  EXPECT_TRUE(gate.update(makeInput(2.5, 0.05)).active);    // 累计 0.05s
  EXPECT_TRUE(gate.update(makeInput(1.0, 0.05)).active);    // 障碍回落 → 计时清零
  EXPECT_TRUE(gate.update(makeInput(2.5, 0.05)).active);    // 重新累计 0.05s
  EXPECT_TRUE(gate.update(makeInput(2.5, 0.05)).active);    // 0.10s，仍未到 0.15s
}

// +inf（走廊清空）满足释放条件（≥ release）。
TEST(FrontTiltSpeedCapTest, InfinityClearanceCountsAsCleared)
{
  FrontTiltSpeedCapParams p = defaultParams();
  p.confirm_frames = 1;
  p.clear_hold_s = 0.0;    // 立即解除
  FrontTiltSpeedCapGate gate(p);

  ASSERT_TRUE(gate.update(makeInput(1.0)).active);
  const auto r = gate.update(makeInput(kInf, 0.05));
  EXPECT_FALSE(r.active);
  EXPECT_TRUE(r.just_released);
}
