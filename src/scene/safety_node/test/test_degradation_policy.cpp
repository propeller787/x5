/**
 * @file test_degradation_policy.cpp
 * @brief 方案 4 传感器降级矩阵单测：单源规则 / 多源合成 / 前向覆盖对 / 禁用源
 */
#include <gtest/gtest.h>

#include "safety_node/core/degradation_policy.h"

#include <cmath>

namespace
{

DegradationConfig productionLikeConfig()
{
  DegradationConfig config;
  config.enabled = true;
  config.per_source[kSourceLeft] = {false, false, true, false, 0.8};
  config.per_source[kSourceRight] = {false, false, false, true, 0.8};
  config.per_source[kSourceBack] = {false, true, false, false, 0.0};
  config.per_source[kSourceLeftTilt] = {false, false, false, false, 1.0};
  config.per_source[kSourceRightTilt] = {false, false, false, false, 1.0};
  config.forward_pair_lost = {true, false, false, false, 0.3};
  return config;
}

std::array<bool, kPerceptionSourceCount> allTrue()
{
  std::array<bool, kPerceptionSourceCount> arr;
  arr.fill(true);
  return arr;
}

}  // namespace

TEST(DegradationPolicy, AllFreshIsInactive)
{
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), allTrue(), allTrue());
  EXPECT_FALSE(verdict.active);
  EXPECT_FALSE(std::isfinite(verdict.speed_cap_m_s));
}

TEST(DegradationPolicy, DisabledConfigIsInactive)
{
  auto config = productionLikeConfig();
  config.enabled = false;
  auto fresh = allTrue();
  fresh[kSourceLeft] = false;
  EXPECT_FALSE(degradation::evaluate(config, allTrue(), fresh).active);
}

TEST(DegradationPolicy, LeftLostBlocksLeftTurnAndCapsSpeed)
{
  auto fresh = allTrue();
  fresh[kSourceLeft] = false;
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), allTrue(), fresh);
  EXPECT_TRUE(verdict.active);
  EXPECT_TRUE(verdict.block_left);
  EXPECT_FALSE(verdict.block_right);
  EXPECT_FALSE(verdict.block_forward);
  EXPECT_NEAR(verdict.speed_cap_m_s, 0.8, 1e-9);
  EXPECT_EQ(verdict.lost_sources, "left");
}

TEST(DegradationPolicy, BackLostBlocksReverseWithoutSpeedCap)
{
  auto fresh = allTrue();
  fresh[kSourceBack] = false;
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), allTrue(), fresh);
  EXPECT_TRUE(verdict.block_backward);
  // speed_cap_m_s ≤ 0 表示不限速 → 合成结果保持 +inf
  EXPECT_FALSE(std::isfinite(verdict.speed_cap_m_s));
}

TEST(DegradationPolicy, MultiSourceMergesBlocksAndMinCap)
{
  auto fresh = allTrue();
  fresh[kSourceLeft] = false;
  fresh[kSourceRight] = false;
  fresh[kSourceLeftTilt] = false;
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), allTrue(), fresh);
  EXPECT_TRUE(verdict.block_left);
  EXPECT_TRUE(verdict.block_right);
  EXPECT_NEAR(verdict.speed_cap_m_s, 0.8, 1e-9);   // min(0.8, 0.8, 1.0)
}

TEST(DegradationPolicy, ForwardPairLostBlocksForwardAndCreepCap)
{
  auto fresh = allTrue();
  fresh[kSourceLeftTilt] = false;
  fresh[kSourceRightTilt] = false;
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), allTrue(), fresh);
  EXPECT_TRUE(verdict.block_forward);
  EXPECT_NEAR(verdict.speed_cap_m_s, 0.3, 1e-9);   // min(1.0, 1.0, 0.3)
}

TEST(DegradationPolicy, DisabledSourceDoesNotCountAsLost)
{
  auto enabled = allTrue();
  enabled[kSourceLeft] = false;    // 该源被配置禁用
  auto fresh = allTrue();
  fresh[kSourceLeft] = false;
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), enabled, fresh);
  EXPECT_FALSE(verdict.active);
}

TEST(DegradationPolicy, SinglePairSourceDisabledSkipsPairRule)
{
  auto enabled = allTrue();
  enabled[kSourceRightTilt] = false;   // 覆盖对只剩单侧 → 不触发 pair 规则
  auto fresh = allTrue();
  fresh[kSourceLeftTilt] = false;
  fresh[kSourceRightTilt] = false;
  const auto verdict =
    degradation::evaluate(productionLikeConfig(), enabled, fresh);
  EXPECT_FALSE(verdict.block_forward);
  EXPECT_NEAR(verdict.speed_cap_m_s, 1.0, 1e-9);   // 只剩 left_tilt 单源规则
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
