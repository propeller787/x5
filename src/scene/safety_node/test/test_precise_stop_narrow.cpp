/**
 * @file test_precise_stop_narrow.cpp
 * @brief P0 PulseCompensator / P2 NarrowPassagePlanner 单元测试
 */
#include <gtest/gtest.h>

#include <cmath>

#include "safety_node/core/narrow_passage_planner.h"
#include "safety_node/core/pulse_compensator.h"

namespace
{

PreciseStopParams makePulseParams()
{
  PreciseStopParams p;
  p.enabled = true;
  p.pulse_enabled = true;
  p.stop_tolerance_m = 0.015;
  p.pulse_creep_speed_m_s = 0.08;
  p.pulse_max_duration_s = 0.25;
  p.pulse_stall_speed_eps_m_s = 0.02;
  p.pulse_stall_hold_s = 0.10;
  p.pulse_cooldown_s = 0.10;
  return p;
}

PulseCompensator::Input makeStalledInput(double residual_m)
{
  PulseCompensator::Input in;
  in.residual_m = residual_m;
  in.fb_linear = 0.0;
  in.forward_intent = true;
  in.safe_to_creep = true;
  return in;
}

AvoidanceParams makeNarrowParams()
{
  AvoidanceParams params;
  params.narrow_planner.enabled = true;
  params.narrow_planner.auto_detect_enabled = true;
  params.narrow_planner.look_ahead_m = 2.0;
  params.narrow_planner.roi_half_width_m = 1.2;
  params.narrow_planner.bin_size_m = 0.2;
  params.narrow_planner.min_corridor_len_m = 0.4;
  params.narrow_planner.min_side_margin_m = 0.03;
  params.narrow_planner.narrow_width_max_m = 0.60;
  params.narrow_planner.speed_per_clearance = 3.0;
  params.narrow_planner.min_pass_speed_m_s = 0.15;
  return params;
}

void addWallPoint(ObstacleField & field, double x, double y)
{
  ObstaclePrimitive obs;
  obs.x = x;
  obs.y = y;
  obs.radius = 0.01;
  obs.valid = true;
  field.obstacles.push_back(obs);
}

/** 双侧平行墙走廊：x ∈ [0.2, 1.8]，左墙 y=+half_gap，右墙 y=−half_gap */
ObstacleField makeCorridorField(double half_gap)
{
  ObstacleField field;
  for (double x = 0.2; x <= 1.8; x += 0.2) {
    addWallPoint(field, x, half_gap);
    addWallPoint(field, x, -half_gap);
  }
  return field;
}

}  // namespace

// ── P0 PulseCompensator ──

TEST(PulseCompensatorTest, FiresAfterStallHoldWhenResidualRemains)
{
  PulseCompensator comp;
  comp.updateParams(makePulseParams());

  const auto in = makeStalledInput(0.06);
  // 停短未满 hold：不发脉冲
  EXPECT_FALSE(comp.update(in, 0.05));
  // 累计到 hold（0.10 s）后进入 PULSING
  EXPECT_TRUE(comp.update(in, 0.05));
  EXPECT_TRUE(comp.active());
}

TEST(PulseCompensatorTest, NoPulseWhenResidualWithinTolerance)
{
  PulseCompensator comp;
  comp.updateParams(makePulseParams());

  const auto in = makeStalledInput(0.010);  // < stop_tolerance 0.015
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(comp.update(in, 0.05));
  }
}

TEST(PulseCompensatorTest, NoPulseWithoutForwardIntent)
{
  PulseCompensator comp;
  comp.updateParams(makePulseParams());

  auto in = makeStalledInput(0.06);
  in.forward_intent = false;
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(comp.update(in, 0.05));
  }
}

TEST(PulseCompensatorTest, PulseDurationBoundedByOvershootGuard)
{
  PulseCompensator comp;
  comp.updateParams(makePulseParams());

  // 残差 0.023：可用位移 = 0.023 − 0.015 = 0.008 m
  // 护栏时长 = 0.008 / 0.08 = 0.1 s < max_duration 0.25 s
  const auto in = makeStalledInput(0.023);
  EXPECT_FALSE(comp.update(in, 0.05));
  EXPECT_TRUE(comp.update(in, 0.05));   // 进入 PULSING
  EXPECT_TRUE(comp.update(in, 0.05));   // 0.05 s < 0.1 s，仍在脉冲
  EXPECT_FALSE(comp.update(in, 0.06));  // 超过护栏时长 → COOLDOWN
  EXPECT_FALSE(comp.active());
}

TEST(PulseCompensatorTest, StopsImmediatelyWhenResidualReached)
{
  PulseCompensator comp;
  comp.updateParams(makePulseParams());

  auto in = makeStalledInput(0.06);
  EXPECT_FALSE(comp.update(in, 0.05));
  EXPECT_TRUE(comp.update(in, 0.05));

  // 车辆前移后残差达到容差 → 立即停止脉冲并复位
  in.residual_m = 0.012;
  EXPECT_FALSE(comp.update(in, 0.05));
  EXPECT_FALSE(comp.active());
}

TEST(PulseCompensatorTest, DisabledParamsNeverPulse)
{
  PreciseStopParams params = makePulseParams();
  params.enabled = false;
  PulseCompensator comp;
  comp.updateParams(params);

  const auto in = makeStalledInput(0.06);
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(comp.update(in, 0.05));
  }
}

// ── P2 NarrowPassagePlanner ──

TEST(NarrowPassagePlannerTest, DetectsStraightCorridorAndCentersCandidate)
{
  const AvoidanceParams params = makeNarrowParams();
  NarrowPassagePlanner planner(params);

  // 车宽 0.68，gap = 1.10 → 单侧净空 0.21，属窄道（gap − 车宽 = 0.42 ≤ 0.60）
  const ObstacleField field = makeCorridorField(0.55);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;

  const NarrowCorridorPlan plan = planner.plan(field, cmd, 0.68);
  ASSERT_TRUE(plan.valid);
  EXPECT_NEAR(plan.corridor_width_m, 1.10 - 0.02, 0.03);  // 减两侧半径
  EXPECT_GT(plan.corridor_length_m, 0.4);
  // 对称走廊：中线居中，曲率近零，候选直行
  EXPECT_NEAR(plan.curvature_1_m, 0.0, 0.05);
  EXPECT_NEAR(plan.candidate.w, 0.0, 0.05);
  EXPECT_GT(plan.candidate.v, 0.0);
  // 净空限速生效：0.15 + 3.0·(0.20 − 0.03) ≈ 0.66 < 请求 1.0
  EXPECT_LT(plan.candidate.v, cmd.linear.x);
}

TEST(NarrowPassagePlannerTest, OffsetCorridorSteersTowardCenter)
{
  const AvoidanceParams params = makeNarrowParams();
  NarrowPassagePlanner planner(params);

  // 走廊整体向左偏移 0.3 m：左墙 y=0.85，右墙 y=−0.25 → 中线 y=+0.30
  ObstacleField field;
  for (double x = 0.2; x <= 1.8; x += 0.2) {
    addWallPoint(field, x, 0.85);
    addWallPoint(field, x, -0.25);
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.8;

  const NarrowCorridorPlan plan = planner.plan(field, cmd, 0.68);
  ASSERT_TRUE(plan.valid);
  // 中线在左侧 → 起点曲率为正（左转），候选角速度为正
  EXPECT_GT(plan.curvature_1_m, 0.0);
  EXPECT_GT(plan.candidate.w, 0.0);
}

TEST(NarrowPassagePlannerTest, WideGapDoesNotTrigger)
{
  const AvoidanceParams params = makeNarrowParams();
  NarrowPassagePlanner planner(params);

  // gap = 2.0，gap − 车宽 = 1.32 > narrow_width_max 0.60 → 开阔区不干预
  const ObstacleField field = makeCorridorField(1.0);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;

  const NarrowCorridorPlan plan = planner.plan(field, cmd, 0.68);
  EXPECT_FALSE(plan.valid);
}

TEST(NarrowPassagePlannerTest, BlockedCorridorInvalid)
{
  const AvoidanceParams params = makeNarrowParams();
  NarrowPassagePlanner planner(params);

  // 走廊入口即封死：gap < 车宽 + 2·margin
  ObstacleField field = makeCorridorField(0.55);
  addWallPoint(field, 0.3, 0.0);  // 走廊中央障碍（单 bin 内 gap 塌缩）
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;

  const NarrowCorridorPlan plan = planner.plan(field, cmd, 0.68);
  // 中央障碍使首 bin 左壁内缘降到 ~0，走廊在起点即断裂
  EXPECT_FALSE(plan.valid);
}

TEST(NarrowPassagePlannerTest, ReverseOrZeroCommandInvalid)
{
  const AvoidanceParams params = makeNarrowParams();
  NarrowPassagePlanner planner(params);
  const ObstacleField field = makeCorridorField(0.55);

  geometry_msgs::msg::Twist reverse;
  reverse.linear.x = -0.5;
  EXPECT_FALSE(planner.plan(field, reverse, 0.68).valid);

  geometry_msgs::msg::Twist zero;
  EXPECT_FALSE(planner.plan(field, zero, 0.68).valid);
}

TEST(NarrowPassagePlannerTest, SingleWallOnlyNotNarrow)
{
  const AvoidanceParams params = makeNarrowParams();
  NarrowPassagePlanner planner(params);

  // 仅左侧一排障碍，右侧开阔 → 不构成窄道
  ObstacleField field;
  for (double x = 0.2; x <= 1.8; x += 0.2) {
    addWallPoint(field, x, 0.55);
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;

  EXPECT_FALSE(planner.plan(field, cmd, 0.68).valid);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
