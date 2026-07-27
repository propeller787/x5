/**
 * @file test_front_lowpass_stop.cpp
 * @brief 前向近障低通刹停闸单测，见 core/front_lowpass_stop.h
 */
#include <gtest/gtest.h>

#include "safety_node/core/front_lowpass_stop.h"

#include <cmath>
#include <limits>

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

FrontObstacleLowpassStopParams defaultParams()
{
  FrontObstacleLowpassStopParams p;    // 结构体默认值即 yaml 默认值
  p.enabled = true;
  return p;
}

FrontLowpassStopInput makeInput(double clearance_m, double fb_linear, double dt_s = 0.05)
{
  FrontLowpassStopInput in;
  in.forward_clearance_m = clearance_m;
  in.fb_linear_m_s = fb_linear;
  in.command_linear_m_s = fb_linear;
  in.forward_intent = true;    // 默认驾驶员仍在推杆；停稳解锁须显式撤销
  in.dt_s = dt_s;
  return in;
}

FrontLowpassSourceSample makeSourceSample(
  int64_t stamp_ns,
  double clearance_m,
  double age_s = 0.0)
{
  FrontLowpassSourceSample sample;
  sample.frame_valid = true;
  sample.frame_stamp_ns = stamp_ns;
  sample.frame_age_s = age_s;
  sample.clearance_m = clearance_m;
  return sample;
}

/** 把闸推进到锁存态（默认 confirm_frames=2） */
void latch(FrontLowpassStopGate & gate)
{
  gate.update(makeInput(0.16, 0.40));
  ASSERT_TRUE(gate.update(makeInput(0.16, 0.40)).active);
}

}  // namespace

TEST(FrontLowpassStopTest, DisabledNeverActivates)
{
  FrontObstacleLowpassStopParams params = defaultParams();
  params.enabled = false;
  FrontLowpassStopGate gate(params);

  for (int i = 0; i < 20; ++i) {
    const auto result = gate.update(makeInput(0.05, 2.0));
    EXPECT_FALSE(result.active);
    EXPECT_DOUBLE_EQ(result.tau_s, 0.0);
  }
  EXPECT_FALSE(gate.latched());
}

TEST(FrontLowpassStopTest, RequiresConfirmFramesBeforeLatching)
{
  FrontLowpassStopGate gate(defaultParams());

  // 单帧满足不触发（滤体素抖动）
  EXPECT_FALSE(gate.update(makeInput(0.16, 0.40)).active);
  // 中途条件消失 → 确认计数清零，再来一帧仍不触发
  EXPECT_FALSE(gate.update(makeInput(0.50, 0.40)).active);
  EXPECT_FALSE(gate.update(makeInput(0.16, 0.40)).active);
  // 连续两帧满足 → 锁存
  const auto latched = gate.update(makeInput(0.16, 0.40));
  EXPECT_TRUE(latched.active);
  EXPECT_TRUE(latched.just_latched);
  EXPECT_TRUE(gate.latched());
}

TEST(FrontLowpassStopTest, DoesNotTriggerBelowSpeedThreshold)
{
  FrontLowpassStopGate gate(defaultParams());
  // 净空进带但速度未超阈值（MDB 已压住的正常逼近）→ 不接管
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(gate.update(makeInput(0.16, 0.30)).active);
  }
}

TEST(FrontLowpassStopTest, DoesNotTriggerOutsideDistanceBand)
{
  FrontLowpassStopGate gate(defaultParams());
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(gate.update(makeInput(0.17, 2.0)).active);
  }
}

// 核心反振荡用例：低通会把速度压到 trigger_speed_m_s 之下，若拿速度做释放判据，
// 条件一消失 shape() 就把驾驶员推杆速度拉回来，在阈值附近反复启停。
TEST(FrontLowpassStopTest, LatchSurvivesSpeedFallingBelowTrigger)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  for (int i = 0; i < 50; ++i) {
    EXPECT_TRUE(gate.update(makeInput(0.16, 0.0)).active) << "帧 " << i;
  }
  EXPECT_TRUE(gate.latched());
}

TEST(FrontLowpassStopTest, ReleasesOnlyAfterClearanceHysteresisHold)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  // 净空恢复但未达 release_distance_m(0.25) → 保持锁存
  for (int i = 0; i < 20; ++i) {
    EXPECT_TRUE(gate.update(makeInput(0.20, 0.0)).active);
  }
  // 达到释放净空但未满 clear_hold_s(0.15) → 仍保持
  EXPECT_TRUE(gate.update(makeInput(0.30, 0.0, 0.05)).active);
  EXPECT_TRUE(gate.update(makeInput(0.30, 0.0, 0.05)).active);
  // 累计满 0.15 s → 释放
  EXPECT_FALSE(gate.update(makeInput(0.30, 0.0, 0.05)).active);
  EXPECT_FALSE(gate.latched());
}

TEST(FrontLowpassStopTest, ClearHoldRestartsWhenObstacleReappears)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  gate.update(makeInput(0.30, 0.0, 0.05));    // clear 0.05
  gate.update(makeInput(0.30, 0.0, 0.05));    // clear 0.10
  EXPECT_TRUE(gate.update(makeInput(0.16, 0.0, 0.05)).active);    // 障碍回来 → 清零重计
  EXPECT_TRUE(gate.update(makeInput(0.30, 0.0, 0.05)).active);    // clear 0.05
  EXPECT_TRUE(gate.update(makeInput(0.30, 0.0, 0.05)).active);    // clear 0.10
  EXPECT_FALSE(gate.update(makeInput(0.30, 0.0, 0.05)).active);   // clear 0.15 → 释放
}

// 净空 +inf 有两义。实配下 self_filter.x_max_m(1.00) > physicalRect().x_max(0.94)，
// 正前方 6 cm 内的障碍被当自车滤除——低通恰好把车刹到这个距离，障碍随即"消失"。
// 若把 +inf 一律当清空解锁，而 MDB/hard_stop 用同一个 field 也看不见它，车会解锁后重新加速撞上。
TEST(FrontLowpassStopTest, CloseRangeTargetLossHoldsLatch)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);    // 最近有限净空 = 0.16，仍在触发带内

  for (int i = 0; i < 50; ++i) {
    EXPECT_TRUE(gate.update(makeInput(kInf, 0.0)).active) << "帧 " << i;
  }
  EXPECT_TRUE(gate.latched());
}

TEST(FrontLowpassStopTest, FarRangeTargetLossReleases)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  // 障碍先退到触发带之外(0.30 > trigger 0.16)再消失 → 属真清空，走滞回释放
  EXPECT_TRUE(gate.update(makeInput(0.30, 0.0, 0.05)).active);     // clear 0.05
  EXPECT_TRUE(gate.update(makeInput(kInf, 0.0, 0.05)).active);     // clear 0.10
  const auto released = gate.update(makeInput(kInf, 0.0, 0.05));   // clear 0.15
  EXPECT_FALSE(released.active);
  EXPECT_TRUE(released.just_released);
  EXPECT_STREQ(released.release_reason, "净空滞回");
}

// 贴近停靠出路：仅靠净空滞回会把车锁在 stop_buffer 外直到退回 25 cm，
// 充电桩/桌边永远贴不上去。松杆 + 停稳 = 本次快速逼近事件结束。
TEST(FrontLowpassStopTest, ReleasesOnStandstillWhenForwardIntentWithdrawn)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  // 已停稳但仍在推杆 → 保持锁存
  EXPECT_TRUE(gate.update(makeInput(0.16, 0.0)).active);

  FrontLowpassStopInput released = makeInput(0.16, 0.0);
  released.forward_intent = false;
  const auto out = gate.update(released);
  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.just_released);
  EXPECT_STREQ(out.release_reason, "停稳松杆");
  EXPECT_FALSE(gate.latched());
}

TEST(FrontLowpassStopTest, StandstillReleaseRequiresActuallyStopped)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  // 松杆但还在滑行（fb > eps）→ 仍接管，把速度继续刹到零
  FrontLowpassStopInput coasting = makeInput(0.16, 0.20);
  coasting.forward_intent = false;
  EXPECT_TRUE(gate.update(coasting).active);
  EXPECT_TRUE(gate.latched());
}

TEST(FrontLowpassStopTest, SlowReapproachAfterStandstillReleaseDoesNotRetrigger)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);
  FrontLowpassStopInput released = makeInput(0.16, 0.0);
  released.forward_intent = false;
  ASSERT_FALSE(gate.update(released).active);

  // 解锁后从静止慢速贴近：MDB 在 5 cm 处的上限约 0.155 m/s，远低于 trigger 0.35 → 不重触发，
  // 后续精准贴靠交回 MDB / precise_stop。
  for (int i = 0; i < 30; ++i) {
    EXPECT_FALSE(gate.update(makeInput(0.05, 0.155)).active) << "帧 " << i;
  }
}

TEST(FrontLowpassStopTest, EmergencyFrameHandsBackAndUnlatches)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  // 真硬急停（节点层未折算掉）：交回原链路并解锁
  FrontLowpassStopInput in = makeInput(0.16, 0.40);
  in.emergency_brake = true;
  const auto out = gate.update(in);
  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.just_released);
  EXPECT_STREQ(out.release_reason, "急停接管");
  EXPECT_FALSE(gate.latched());
}

TEST(FrontLowpassStopTest, MapVetoYieldHelperAllowsOutsideHardStop)
{
  constexpr double kHardStop = 0.03;
  // MAP_VETO + 走廊 0.19 m：可让位（即使 Footprint min_clearance 已贴零也不再参与判定）
  EXPECT_TRUE(
    mapVetoYieldsToFrontLowpass(true, true, true, 0.19, kHardStop));
  // 开关关：不让位
  EXPECT_FALSE(
    mapVetoYieldsToFrontLowpass(false, true, true, 0.19, kHardStop));
  // 非 map_veto：不让位
  EXPECT_FALSE(
    mapVetoYieldsToFrontLowpass(true, true, false, 0.19, kHardStop));
  // 走廊已进硬停线：不让位
  EXPECT_FALSE(
    mapVetoYieldsToFrontLowpass(true, true, true, 0.02, kHardStop));
  // 走廊仍在硬停外（复现 tilt_on_18：走廊 0.24、侧向已侵入）→ 仍让位
  EXPECT_TRUE(
    mapVetoYieldsToFrontLowpass(true, true, true, 0.24, kHardStop));
}

TEST(FrontLowpassStopTest, MutedMapVetoDoesNotBlockLatch)
{
  // 节点层对可让位的 MAP_VETO 会把 emergency_brake 折成 false，闸应能正常锁存。
  FrontLowpassStopGate gate(defaultParams());
  FrontLowpassStopInput in = makeInput(0.16, 0.40);
  in.emergency_brake = false;
  EXPECT_FALSE(gate.update(in).active);
  EXPECT_TRUE(gate.update(in).active);
}

TEST(FrontLowpassStopTest, ReverseIntentHandsBackAndUnlatches)
{
  FrontLowpassStopGate gate(defaultParams());
  latch(gate);

  // 后退是脱困出路：不接管、立即解锁，不能把驾驶员摁在障碍前
  FrontLowpassStopInput in = makeInput(0.16, 0.40);
  in.reverse_intent = true;
  const auto out = gate.update(in);
  EXPECT_FALSE(out.active);
  EXPECT_TRUE(out.just_released);
  EXPECT_STREQ(out.release_reason, "后退意图");
  EXPECT_FALSE(gate.latched());
}

// 低通滑行距离闭式解 d = τ·v。固定 τ=0.30 时 0.54 m/s 就滑满 16 cm 触底，故 τ 必须随速度收紧。
// 触发带余量 = trigger_distance 0.16 − stop_buffer 0.08 = 0.08。
TEST(FrontLowpassStopTest, TimeConstantTightensWithSpeed)
{
  FrontLowpassStopGate gate(defaultParams());

  // 触发边界 0.36 m/s：0.08/0.36 = 0.222，落在 [0.08, 0.30] 区间内
  gate.update(makeInput(0.16, 0.36));
  EXPECT_NEAR(gate.update(makeInput(0.16, 0.36)).tau_s, 0.08 / 0.36, 1e-9);

  // 1.0 m/s 入场：0.08/1.0 = 0.08，正好到下限
  gate.reset();
  gate.update(makeInput(0.16, 1.0));
  EXPECT_NEAR(gate.update(makeInput(0.16, 1.0)).tau_s, 0.08, 1e-9);

  // 2.0 m/s 入场：0.08/2.0 = 0.04 → 钳到下限 0.08（再急就该交给急停）
  gate.reset();
  gate.update(makeInput(0.16, 2.0));
  EXPECT_NEAR(gate.update(makeInput(0.16, 2.0)).tau_s, 0.08, 1e-9);

  // 锁存后车已被刹到 0.2 m/s：0.08/0.2 = 0.4 → 钳到柔和端 0.30
  gate.reset();
  latch(gate);
  EXPECT_NEAR(gate.update(makeInput(0.16, 0.2)).tau_s, 0.30, 1e-9);
}

TEST(FrontLowpassStopTest, TimeConstantUsesLargerOfFeedbackAndCommand)
{
  FrontLowpassStopGate gate(defaultParams());

  // 反馈落后于命令时按命令定 τ（取大者更保守）：
  // max(fb 0.40, cmd 0.5) = 0.5 → τ = 0.08/0.5 = 0.16（误用 fb 则为 0.2）
  FrontLowpassStopInput in = makeInput(0.16, 0.40);
  in.command_linear_m_s = 0.5;
  gate.update(in);
  EXPECT_NEAR(gate.update(in).tau_s, 0.08 / 0.5, 1e-9);
}

TEST(FrontLowpassStopTest, TimeConstantClampsToMinInsideStopBuffer)
{
  FrontLowpassStopGate gate(defaultParams());

  // 净空已吃进 stop_buffer_m(0.08) → 余量为负，收到最紧
  gate.update(makeInput(0.06, 0.40));
  EXPECT_NEAR(gate.update(makeInput(0.06, 0.40)).tau_s, 0.08, 1e-9);
}

TEST(FrontLowpassTiltTrackerTest, MissingFramePredictsClearanceFromWheelSpeed)
{
  FrontObstacleLowpassStopParams params = defaultParams();
  params.stale_prediction_max_s = 0.20;
  params.prediction_uncertainty_margin_m = 0.01;
  FrontLowpassTiltTracker tracker(params);

  std::array<FrontLowpassSourceSample, 2> samples {};
  samples[0] = makeSourceSample(100, 0.40);
  const auto measured = tracker.update(samples, 0.50, 0.05);
  EXPECT_DOUBLE_EQ(measured.clearance_m, 0.40);
  EXPECT_FALSE(measured.predicted);
  EXPECT_TRUE(measured.valid_for_trigger);

  // 同一时间戳表示下一控制周期没有新帧：0.40 - 0.50*0.10 - 0.01 = 0.34。
  const auto predicted = tracker.update(samples, 0.50, 0.10);
  EXPECT_NEAR(predicted.clearance_m, 0.34, 1e-9);
  EXPECT_TRUE(predicted.predicted);
  EXPECT_NEAR(predicted.observation_age_s, 0.10, 1e-9);
  EXPECT_TRUE(predicted.valid_for_trigger);
}

TEST(FrontLowpassTiltTrackerTest, NewFrameReplacesPreviousPrediction)
{
  FrontObstacleLowpassStopParams params = defaultParams();
  params.prediction_uncertainty_margin_m = 0.01;
  FrontLowpassTiltTracker tracker(params);
  std::array<FrontLowpassSourceSample, 2> samples {};

  samples[0] = makeSourceSample(100, 0.20);
  tracker.update(samples, 0.50, 0.0);
  EXPECT_NEAR(tracker.update(samples, 0.50, 0.10).clearance_m, 0.14, 1e-9);

  // 新时间戳按源整体替换，不能把上一帧预测与新帧叠加。
  samples[0] = makeSourceSample(200, 0.50);
  const auto replaced = tracker.update(samples, 0.50, 0.05);
  EXPECT_DOUBLE_EQ(replaced.clearance_m, 0.50);
  EXPECT_FALSE(replaced.predicted);

  // 新鲜空帧同样替换上一帧，几何层不再保留旧障碍。
  samples[0] = makeSourceSample(300, kInf);
  const auto cleared = tracker.update(samples, 0.50, 0.05);
  EXPECT_FALSE(std::isfinite(cleared.clearance_m));
  EXPECT_EQ(cleared.source_index, -1);
}

TEST(FrontLowpassTiltTrackerTest, SourcesReplaceIndependently)
{
  FrontObstacleLowpassStopParams params = defaultParams();
  params.prediction_uncertainty_margin_m = 0.0;
  FrontLowpassTiltTracker tracker(params);
  std::array<FrontLowpassSourceSample, 2> samples {};
  samples[0] = makeSourceSample(100, 0.30);
  samples[1] = makeSourceSample(100, 0.50);
  tracker.update(samples, 0.20, 0.0);

  // 右源刷新为空不能删除未刷新的左源；左源继续按轮速预测。
  samples[1] = makeSourceSample(200, kInf);
  const auto out = tracker.update(samples, 0.20, 0.10);
  EXPECT_NEAR(out.clearance_m, 0.28, 1e-9);
  EXPECT_EQ(out.source_index, 0);
  EXPECT_TRUE(out.predicted);
}

TEST(FrontLowpassTiltTrackerTest, ExpiredPredictionCannotStartNewLatch)
{
  FrontObstacleLowpassStopParams params = defaultParams();
  params.confirm_frames = 1;
  params.stale_prediction_max_s = 0.10;
  params.prediction_uncertainty_margin_m = 0.0;
  FrontLowpassTiltTracker tracker(params);
  std::array<FrontLowpassSourceSample, 2> samples {};
  samples[0] = makeSourceSample(100, 0.16, 0.20);

  const auto expired = tracker.update(samples, 0.40, 0.0);
  EXPECT_FALSE(expired.valid_for_trigger);

  FrontLowpassStopGate gate(params);
  FrontLowpassStopInput in = makeInput(expired.clearance_m, 0.40);
  in.trigger_observation_valid = expired.valid_for_trigger;
  EXPECT_FALSE(gate.update(in).active);
  EXPECT_FALSE(gate.latched());
}

TEST(FrontLowpassTiltTrackerTest, ExpiredPredictionKeepsExistingLatch)
{
  FrontObstacleLowpassStopParams params = defaultParams();
  params.confirm_frames = 1;
  FrontLowpassStopGate gate(params);

  FrontLowpassStopInput in = makeInput(0.16, 0.40);
  ASSERT_TRUE(gate.update(in).active);
  in.trigger_observation_valid = false;
  EXPECT_TRUE(gate.update(in).active);
  EXPECT_TRUE(gate.latched());
}
