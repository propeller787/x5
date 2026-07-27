/**
 * @file test_safety_arbiter.cpp
 * @brief 方案 1 集中仲裁器单测：合成规则 + 单调性 fuzz
 *
 * 单调性是「层可安全叠加」的形式化保证：新增一条仅含 cap/stop 的裁决
 * （无 suggestion / 无 escape_grant）永远不会使输出变宽。
 */
#include <gtest/gtest.h>

#include "safety_node/core/safety_verdict.h"

#include <cmath>
#include <random>

namespace
{

geometry_msgs::msg::Twist makeTwist(double v, double w)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  return twist;
}

LayerVerdict capVerdict(SafetyLayerId id, double forward_cap)
{
  LayerVerdict v;
  v.id = id;
  v.active = true;
  v.forward_cap = forward_cap;
  return v;
}

}  // namespace

TEST(SafetyArbiter, NoVerdictsPassesDesiredThrough)
{
  const auto out = safety_arbiter::arbitrate(makeTwist(1.0, 0.5), {});
  EXPECT_DOUBLE_EQ(out.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(out.twist.angular.z, 0.5);
  EXPECT_FALSE(out.full_stop);
}

TEST(SafetyArbiter, CapsMergeByMin)
{
  std::vector<LayerVerdict> verdicts;
  verdicts.push_back(capVerdict(SafetyLayerId::BRAKING_ENVELOPE, 0.8));
  verdicts.push_back(capVerdict(SafetyLayerId::TURN_CLEARANCE_GUARD, 0.3));
  const auto out = safety_arbiter::arbitrate(makeTwist(1.5, 0.0), verdicts);
  EXPECT_NEAR(out.twist.linear.x, 0.3, 1e-9);
}

TEST(SafetyArbiter, CapOnlyLimitsMatchingDirection)
{
  std::vector<LayerVerdict> verdicts;
  verdicts.push_back(capVerdict(SafetyLayerId::BRAKING_ENVELOPE, 0.3));
  // 倒车命令不受前向 cap 影响
  const auto out = safety_arbiter::arbitrate(makeTwist(-1.0, 0.0), verdicts);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, -1.0);
}

TEST(SafetyArbiter, StopAxesUnionZeroesComponents)
{
  std::vector<LayerVerdict> verdicts;
  LayerVerdict a;
  a.id = SafetyLayerId::DIRECTION_MASK;
  a.active = true;
  a.stop_axes.forward = true;
  verdicts.push_back(a);
  LayerVerdict b;
  b.id = SafetyLayerId::DEGRADATION;
  b.active = true;
  b.stop_axes.left = true;
  verdicts.push_back(b);
  const auto out = safety_arbiter::arbitrate(makeTwist(1.0, 0.7), verdicts);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.twist.angular.z, 0.0);
}

TEST(SafetyArbiter, FullStopZeroesEverything)
{
  std::vector<LayerVerdict> verdicts;
  LayerVerdict stop;
  stop.id = SafetyLayerId::STANDSTILL_GUARD;
  stop.active = true;
  stop.request_full_stop = true;
  stop.reason = EmergencyStopReason::STANDSTILL_GUARD;
  verdicts.push_back(stop);
  const auto out = safety_arbiter::arbitrate(makeTwist(1.0, 0.5), verdicts);
  EXPECT_TRUE(out.full_stop);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(out.twist.angular.z, 0.0);
  EXPECT_EQ(out.reason, EmergencyStopReason::STANDSTILL_GUARD);
  EXPECT_EQ(out.deciding_layer, SafetyLayerId::STANDSTILL_GUARD);
}

TEST(SafetyArbiter, HighestPrioritySuggestionWinsAsBase)
{
  std::vector<LayerVerdict> verdicts;
  LayerVerdict candidate;
  candidate.id = SafetyLayerId::CANDIDATE_SELECT;
  candidate.active = true;
  candidate.has_suggestion = true;
  candidate.suggestion = makeTwist(0.6, 0.1);
  verdicts.push_back(candidate);
  LayerVerdict escape;
  escape.id = SafetyLayerId::RECOVERY_ESCAPE;   // 优先级高于 CANDIDATE_SELECT
  escape.active = true;
  escape.has_suggestion = true;
  escape.suggestion = makeTwist(-0.1, 0.0);
  verdicts.push_back(escape);
  const auto out = safety_arbiter::arbitrate(makeTwist(1.0, 0.5), verdicts);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, -0.1);
}

TEST(SafetyArbiter, EscapeGrantSuppressesLowerPriorityStop)
{
  std::vector<LayerVerdict> verdicts;
  LayerVerdict escape;
  escape.id = SafetyLayerId::RECOVERY_ESCAPE;
  escape.active = true;
  escape.has_suggestion = true;
  escape.escape_grant = true;
  escape.suggestion = makeTwist(-0.1, 0.0);
  verdicts.push_back(escape);
  LayerVerdict gate;
  gate.id = SafetyLayerId::EMERGENCY_GATE;   // 优先级低于 RECOVERY_ESCAPE
  gate.active = true;
  gate.request_full_stop = true;
  verdicts.push_back(gate);
  const auto out = safety_arbiter::arbitrate(makeTwist(0.0, 0.0), verdicts);
  EXPECT_FALSE(out.full_stop);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, -0.1);
}

TEST(SafetyArbiter, CollisionStopNotSuppressedByEscapeGrant)
{
  std::vector<LayerVerdict> verdicts;
  LayerVerdict escape;
  escape.id = SafetyLayerId::RECOVERY_ESCAPE;
  escape.active = true;
  escape.has_suggestion = true;
  escape.escape_grant = true;
  escape.suggestion = makeTwist(-0.1, 0.0);
  verdicts.push_back(escape);
  LayerVerdict gate;
  gate.id = SafetyLayerId::EMERGENCY_GATE;
  gate.active = true;
  gate.request_full_stop = true;
  gate.has_collision_stop = true;   // 真实碰撞级：任何令牌不可豁免
  verdicts.push_back(gate);
  const auto out = safety_arbiter::arbitrate(makeTwist(0.0, 0.0), verdicts);
  EXPECT_TRUE(out.full_stop);
  EXPECT_DOUBLE_EQ(out.twist.linear.x, 0.0);
}

TEST(SafetyArbiter, EscapeGrantDoesNotSuppressHigherPriorityStop)
{
  std::vector<LayerVerdict> verdicts;
  LayerVerdict stop;
  stop.id = SafetyLayerId::STANDSTILL_GUARD;   // 优先级高于 RECOVERY_ESCAPE
  stop.active = true;
  stop.request_full_stop = true;
  verdicts.push_back(stop);
  LayerVerdict escape;
  escape.id = SafetyLayerId::RECOVERY_ESCAPE;
  escape.active = true;
  escape.has_suggestion = true;
  escape.escape_grant = true;
  escape.suggestion = makeTwist(-0.1, 0.0);
  verdicts.push_back(escape);
  const auto out = safety_arbiter::arbitrate(makeTwist(0.0, 0.0), verdicts);
  EXPECT_TRUE(out.full_stop);
}

/**
 * 单调性 fuzz：随机基准 + 随机约束集，再叠加一条随机纯约束裁决
 * （无 suggestion / 无 escape_grant），输出各分量幅值不得增大。
 */
TEST(SafetyArbiter, MonotonicityFuzz)
{
  std::mt19937 rng(20260714U);
  std::uniform_real_distribution<double> speed(-2.5, 2.5);
  std::uniform_real_distribution<double> cap(0.0, 2.0);
  std::uniform_int_distribution<int> coin(0, 1);
  std::uniform_int_distribution<int> layer_pick(
    0, static_cast<int>(SafetyLayerId::COUNT) - 1);

  auto randomConstraint = [&]() {
      LayerVerdict v;
      v.id = static_cast<SafetyLayerId>(layer_pick(rng));
      v.active = true;
      if (coin(rng)) {v.forward_cap = cap(rng);}
      if (coin(rng)) {v.backward_cap = cap(rng);}
      if (coin(rng)) {v.left_cap = cap(rng);}
      if (coin(rng)) {v.right_cap = cap(rng);}
      v.stop_axes.forward = coin(rng) != 0;
      v.stop_axes.backward = coin(rng) != 0;
      v.stop_axes.left = coin(rng) != 0;
      v.stop_axes.right = coin(rng) != 0;
      v.request_full_stop = coin(rng) != 0 && coin(rng) != 0;
      v.has_collision_stop = v.request_full_stop && coin(rng) != 0;
      return v;
    };

  for (int iter = 0; iter < 10000; ++iter) {
    const auto desired = makeTwist(speed(rng), speed(rng));
    std::vector<LayerVerdict> verdicts;
    const int n = layer_pick(rng) % 5;
    for (int i = 0; i < n; ++i) {
      verdicts.push_back(randomConstraint());
    }
    const auto before = safety_arbiter::arbitrate(desired, verdicts);
    verdicts.push_back(randomConstraint());
    const auto after = safety_arbiter::arbitrate(desired, verdicts);
    ASSERT_LE(
      std::abs(after.twist.linear.x),
      std::abs(before.twist.linear.x) + 1e-12) << "iter=" << iter;
    ASSERT_LE(
      std::abs(after.twist.angular.z),
      std::abs(before.twist.angular.z) + 1e-12) << "iter=" << iter;
    // 输出方向不得翻转（约束只能收缩到零，不能反号）
    ASSERT_GE(after.twist.linear.x * before.twist.linear.x, -1e-12);
    ASSERT_GE(after.twist.angular.z * before.twist.angular.z, -1e-12);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
