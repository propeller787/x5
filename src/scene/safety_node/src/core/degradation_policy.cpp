/**
 * @file degradation_policy.cpp
 * @brief 传感器降级矩阵实现：多源失效规则合成
 */
#include "safety_node/core/degradation_policy.h"

#include <algorithm>

namespace degradation
{

const char * sourceName(size_t index)
{
  switch (index) {
    case kSourceLeft: return "left";
    case kSourceRight: return "right";
    case kSourceBack: return "back";
    case kSourceLeftTilt: return "left_tilt";
    case kSourceRightTilt: return "right_tilt";
    default: return "unknown";
  }
}

namespace
{

void mergeRule(DegradationVerdict & verdict, const DegradationRule & rule)
{
  verdict.active = true;
  verdict.block_forward = verdict.block_forward || rule.block_forward;
  verdict.block_backward = verdict.block_backward || rule.block_backward;
  verdict.block_left = verdict.block_left || rule.block_left;
  verdict.block_right = verdict.block_right || rule.block_right;
  if (rule.speed_cap_m_s > 0.0) {
    verdict.speed_cap_m_s = std::min(verdict.speed_cap_m_s, rule.speed_cap_m_s);
  }
}

}  // namespace

DegradationVerdict evaluate(
  const DegradationConfig & config,
  const std::array<bool, kPerceptionSourceCount> & source_enabled,
  const std::array<bool, kPerceptionSourceCount> & source_fresh)
{
  DegradationVerdict verdict;
  if (!config.enabled) {
    return verdict;
  }

  for (size_t i = 0; i < kPerceptionSourceCount; ++i) {
    if (!source_enabled[i] || source_fresh[i]) {
      continue;
    }
    mergeRule(verdict, config.per_source[i]);
    if (!verdict.lost_sources.empty()) {
      verdict.lost_sources += ",";
    }
    verdict.lost_sources += sourceName(i);
  }

  // 前向覆盖对：两路斜雷达同时启用且同时失效 → 加严规则
  const bool pair_enabled =
    source_enabled[kSourceLeftTilt] && source_enabled[kSourceRightTilt];
  const bool pair_lost =
    pair_enabled &&
    !source_fresh[kSourceLeftTilt] && !source_fresh[kSourceRightTilt];
  if (pair_lost) {
    mergeRule(verdict, config.forward_pair_lost);
  }

  return verdict;
}

}  // namespace degradation
