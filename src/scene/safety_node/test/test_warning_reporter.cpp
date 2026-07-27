/**
 * @file test_warning_reporter.cpp
 * @brief 四向碰撞预警状态机单测，见 core/warning_reporter.h
 */
#include <gtest/gtest.h>

#include "safety_node/core/warning_reporter.h"

namespace
{

constexpr int16_t kNone = 0;
constexpr int16_t kDecel = 1;
constexpr int16_t kBrake = 2;
constexpr int16_t kLeaveDecel = 3;
constexpr int16_t kLeaveBrake = 4;

constexpr int16_t kForward = 1;
constexpr int16_t kLeft = 2;
constexpr int16_t kRight = 3;
constexpr int16_t kBackward = 4;

CollisionWarningInput makeInput(double now_s = 0.0)
{
  CollisionWarningInput in;
  in.now_s = now_s;
  return in;
}

/** 空输入推进一拍，用于让心跳计时前进而不改变等级 */
CollisionWarningInput forwardDecel(double now_s)
{
  CollisionWarningInput in = makeInput(now_s);
  in.levels.forward_decel = true;
  return in;
}

CollisionWarningInput forwardBrake(double now_s)
{
  CollisionWarningInput in = makeInput(now_s);
  in.levels.forward_brake = true;
  return in;
}

::testing::AssertionResult hasEvent(
  const std::vector<CollisionWarningEvent> & events, int16_t type, int16_t level)
{
  for (const auto & e : events) {
    if (e.warning_type == type && e.warning_level == level) {
      return ::testing::AssertionSuccess();
    }
  }
  return ::testing::AssertionFailure() << "缺少事件 type=" << type << " level=" << level;
}

}  // namespace

/** 无障碍时不产生任何事件 */
TEST(WarningReporter, IdleEmitsNothing)
{
  CollisionWarningReporter reporter;
  EXPECT_TRUE(reporter.update(makeInput(0.0)).empty());
  EXPECT_TRUE(reporter.update(makeInput(1.0)).empty());
}

/** 进入减速区立即发一条减速 */
TEST(WarningReporter, EntersDecelerationImmediately)
{
  CollisionWarningReporter reporter;
  const auto events = reporter.update(forwardDecel(0.0));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_type, kForward);
  EXPECT_EQ(events[0].warning_level, kDecel);
  EXPECT_EQ(reporter.levelOf(CollisionDirection::FORWARD), kDecel);
}

/** 稳态按 repeat_interval_s 心跳，不是每周期都发 */
TEST(WarningReporter, SteadyStateThrottlesToHeartbeat)
{
  CollisionWarningReporterConfig config;
  config.repeat_interval_s = 1.0;
  CollisionWarningReporter reporter(config);

  ASSERT_EQ(reporter.update(forwardDecel(0.0)).size(), 1U);
  EXPECT_TRUE(reporter.update(forwardDecel(0.2)).empty());
  EXPECT_TRUE(reporter.update(forwardDecel(0.9)).empty());
  // 到点补发同一条
  const auto beat = reporter.update(forwardDecel(1.0));
  ASSERT_EQ(beat.size(), 1U);
  EXPECT_EQ(beat[0].warning_level, kDecel);
  EXPECT_TRUE(reporter.update(forwardDecel(1.5)).empty());
}

/** 障碍消失：先发驶离减速区，再归零且不再重复发 */
TEST(WarningReporter, LeavingDecelerationZoneEmitsLeaveOnce)
{
  CollisionWarningReporter reporter;
  ASSERT_EQ(reporter.update(forwardDecel(0.0)).size(), 1U);

  const auto events = reporter.update(makeInput(0.1));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_type, kForward);
  EXPECT_EQ(events[0].warning_level, kLeaveDecel);
  EXPECT_EQ(reporter.levelOf(CollisionDirection::FORWARD), kNone);

  EXPECT_TRUE(reporter.update(makeInput(2.0)).empty());
}

/** 刹停解除发驶离刹停区（4），而非驶离减速区（3） */
TEST(WarningReporter, LeavingBrakeZoneEmitsLeaveBrake)
{
  CollisionWarningReporter reporter;
  ASSERT_EQ(reporter.update(forwardBrake(0.0)).size(), 1U);

  const auto events = reporter.update(makeInput(0.1));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_level, kLeaveBrake);
}

/** 减速升级到刹停：先补 3 再发 2，跳变不受心跳节流影响（与 scene_node 逐位一致） */
TEST(WarningReporter, DecelToBrakeEmitsLeaveThenBrakeWithoutThrottle)
{
  CollisionWarningReporterConfig config;
  config.repeat_interval_s = 1.0;
  CollisionWarningReporter reporter(config);

  ASSERT_EQ(reporter.update(forwardDecel(0.0)).size(), 1U);
  // 心跳窗口内（0.1 < 1.0）跳变，仍须立即发出
  const auto events = reporter.update(forwardBrake(0.1));
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].warning_level, kLeaveDecel);
  EXPECT_EQ(events[1].warning_level, kBrake);
  EXPECT_EQ(reporter.levelOf(CollisionDirection::FORWARD), kBrake);
}

/** 四个方向各自独立维护状态 */
TEST(WarningReporter, DirectionsAreIndependent)
{
  CollisionWarningReporter reporter;
  CollisionWarningInput in = makeInput(0.0);
  in.levels.forward_decel = true;
  in.levels.left_brake = true;
  in.levels.backward_decel = true;
  in.levels.right_brake = true;

  const auto events = reporter.update(in);
  ASSERT_EQ(events.size(), 4U);
  EXPECT_TRUE(hasEvent(events, kForward, kDecel));
  EXPECT_TRUE(hasEvent(events, kLeft, kBrake));
  EXPECT_TRUE(hasEvent(events, kBackward, kDecel));
  EXPECT_TRUE(hasEvent(events, kRight, kBrake));

  // 仅左侧解除，其余保持
  CollisionWarningInput next = makeInput(0.1);
  next.levels.forward_decel = true;
  next.levels.backward_decel = true;
  next.levels.right_brake = true;
  const auto after = reporter.update(next);
  ASSERT_EQ(after.size(), 1U);
  EXPECT_EQ(after[0].warning_type, kLeft);
  EXPECT_EQ(after[0].warning_level, kLeaveBrake);
}

/** 同方向 brake 与 decel 同时置位时按更严的刹停上报 */
TEST(WarningReporter, BrakeWinsOverDecel)
{
  CollisionWarningReporter reporter;
  CollisionWarningInput in = makeInput(0.0);
  in.levels.forward_decel = true;
  in.levels.forward_brake = true;

  const auto events = reporter.update(in);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_level, kBrake);
}

/** 障碍类急停但 levels 为空：按兜底方向补一条刹停 */
TEST(WarningReporter, EmergencyFallbackUsesFallbackDirection)
{
  CollisionWarningReporter reporter;
  CollisionWarningInput in = makeInput(0.0);
  in.emergency_brake = true;
  in.emergency_reason = EmergencyStopReason::MAP_VETO;
  in.fallback_direction = CollisionDirection::FORWARD;

  const auto events = reporter.update(in);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_type, kForward);
  EXPECT_EQ(events[0].warning_level, kBrake);
}

/** levels.primary 优先于节点层给的兜底方向 */
TEST(WarningReporter, EmergencyFallbackPrefersPrimaryDirection)
{
  CollisionWarningReporter reporter;
  CollisionWarningInput in = makeInput(0.0);
  in.emergency_brake = true;
  in.emergency_reason = EmergencyStopReason::TILT_NEAR_DISTANCE;
  in.levels.primary = CollisionDirection::LEFT;
  in.fallback_direction = CollisionDirection::FORWARD;

  const auto events = reporter.update(in);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_type, kLeft);
}

/** 非障碍类急停（输入故障/抱闸/模式急停）不报成碰撞预警 */
TEST(WarningReporter, NonObstacleEmergencyIsNotReported)
{
  CollisionWarningReporter reporter;
  for (const auto reason :
    {EmergencyStopReason::INPUT_FAULT, EmergencyStopReason::MODE_ESTOP,
      EmergencyStopReason::BRAKE_HOLD})
  {
    CollisionWarningInput in = makeInput(0.0);
    in.emergency_brake = true;
    in.emergency_reason = reason;
    in.fallback_direction = CollisionDirection::FORWARD;
    EXPECT_TRUE(reporter.update(in).empty());
  }
  EXPECT_FALSE(isObstacleEmergencyReason(EmergencyStopReason::INPUT_FAULT));
  EXPECT_TRUE(isObstacleEmergencyReason(EmergencyStopReason::SUDDEN_OBSTACLE));
}

/** 方向未知时不补报（避免伪造方向） */
TEST(WarningReporter, EmergencyWithoutDirectionEmitsNothing)
{
  CollisionWarningReporter reporter;
  CollisionWarningInput in = makeInput(0.0);
  in.emergency_brake = true;
  in.emergency_reason = EmergencyStopReason::MAP_VETO;
  in.fallback_direction = CollisionDirection::NONE;
  EXPECT_TRUE(reporter.update(in).empty());
}

/** 急停兜底可关；关闭后 levels 为空的急停不产生事件 */
TEST(WarningReporter, EmergencyFallbackCanBeDisabled)
{
  CollisionWarningReporterConfig config;
  config.report_emergency_fallback = false;
  CollisionWarningReporter reporter(config);

  CollisionWarningInput in = makeInput(0.0);
  in.emergency_brake = true;
  in.emergency_reason = EmergencyStopReason::MAP_VETO;
  in.fallback_direction = CollisionDirection::FORWARD;
  EXPECT_TRUE(reporter.update(in).empty());
}

/** 安全关闭（suppressed）时挂起的预警收敛出驶离事件，之后静默 */
TEST(WarningReporter, SuppressedFlushesLeaveEvents)
{
  CollisionWarningReporter reporter;
  ASSERT_EQ(reporter.update(forwardBrake(0.0)).size(), 1U);

  CollisionWarningInput in = forwardBrake(0.1);   // 障碍仍在，但安全被关掉
  in.suppressed = true;
  const auto events = reporter.update(in);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_level, kLeaveBrake);

  CollisionWarningInput next = forwardBrake(2.0);
  next.suppressed = true;
  EXPECT_TRUE(reporter.update(next).empty());
}

/** 总开关关闭时恒不产生事件 */
TEST(WarningReporter, DisabledEmitsNothing)
{
  CollisionWarningReporterConfig config;
  config.enabled = false;
  CollisionWarningReporter reporter(config);
  EXPECT_TRUE(reporter.update(forwardBrake(0.0)).empty());
}

/** 时钟回跳时立即重发并重新对时，不会卡死心跳 */
TEST(WarningReporter, ClockJumpBackwardRepublishes)
{
  CollisionWarningReporter reporter;
  ASSERT_EQ(reporter.update(forwardDecel(100.0)).size(), 1U);
  const auto events = reporter.update(forwardDecel(5.0));
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].warning_level, kDecel);
  // 重新对时后恢复正常节流
  EXPECT_TRUE(reporter.update(forwardDecel(5.2)).empty());
}

/** reset() 静默清状态：不产生驶离事件（仅供测试/冷启动） */
TEST(WarningReporter, ResetIsSilent)
{
  CollisionWarningReporter reporter;
  ASSERT_EQ(reporter.update(forwardBrake(0.0)).size(), 1U);
  reporter.reset();
  EXPECT_EQ(reporter.levelOf(CollisionDirection::FORWARD), kNone);
  EXPECT_TRUE(reporter.update(makeInput(0.1)).empty());
}
