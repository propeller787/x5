/**
 * @file degradation_policy.h
 * @brief 传感器降级矩阵（见 PRODUCTION_HARDENING_PLAN.md 方案 4）
 *
 * 现状问题：min_fresh_horizontal_sources=1 只提供「全瞎才停」的全局兜底，
 * 单路雷达失效时其覆盖侧的盲区被当作自由空间，车辆仍全速全向运行。
 *
 * 本策略把「哪路雷达失效 → 封哪个运动轴 + 限速几档」显式配置化：
 *   - 每源一条 DegradationRule（封锁轴集合 + 全局限速档）；
 *   - forward_pair_lost：两路斜雷达（前向覆盖对）同时失效的加严规则；
 *   - 多源同时失效：封锁轴取并集，限速取最小值。
 *
 * 生效方式：verdict 在 evaluate() 入口对 eval_cmd 做与 direction_mask
 * 同款的分轴裁剪 + 限速钳制，复用已验证的封锁语义，不新开裁剪路径。
 * 全局 min_fresh_* / INPUT_FAULT 判据保留为最终兜底（全瞎必停）。
 *
 * 已知边界（文档化）：recovery / pinned escape 使用驾驶员原始命令评估脱困，
 * 不受降级封锁——脱困本身是低速受控且经 Footprint/Map 复验的运动。
 *
 * 纯查表逻辑、无 ROS 依赖，便于单元测试。
 */
#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>

/** 感知源索引，与 PerceptionUtils::sourceId 一致 */
enum PerceptionSourceIndex : size_t
{
  kSourceLeft = 0,
  kSourceRight = 1,
  kSourceBack = 2,
  kSourceLeftTilt = 3,
  kSourceRightTilt = 4,
  kPerceptionSourceCount = 5,
};

/** 单源失效时的动作 */
struct DegradationRule
{
  bool block_forward = false;
  bool block_backward = false;
  bool block_left = false;      ///< 封锁 +w（左转）
  bool block_right = false;     ///< 封锁 −w（右转）
  /** 全局线速度上限 (m/s)；≤0 表示不限速 */
  double speed_cap_m_s = 0.0;
};

struct DegradationConfig
{
  bool enabled = false;
  /** 按源索引的失效规则（left/right/back/left_tilt/right_tilt） */
  std::array<DegradationRule, kPerceptionSourceCount> per_source {};
  /** 前向覆盖对（两路斜雷达）同时失效的加严规则 */
  DegradationRule forward_pair_lost;
};

/** 单周期降级裁决 */
struct DegradationVerdict
{
  bool active = false;
  bool block_forward = false;
  bool block_backward = false;
  bool block_left = false;
  bool block_right = false;
  /** 合成限速 (m/s)；+inf = 不限速 */
  double speed_cap_m_s = std::numeric_limits<double>::infinity();
  /** 失效源名列表（逗号分隔），供日志/健康摘要 */
  std::string lost_sources;
};

namespace degradation
{

const char * sourceName(size_t index);

/**
 * @param config          降级配置
 * @param source_enabled  各源是否被配置启用（未启用的源不算失效）
 * @param source_fresh    各源当前是否新鲜（消息 + TF 均在时限内）
 */
DegradationVerdict evaluate(
  const DegradationConfig & config,
  const std::array<bool, kPerceptionSourceCount> & source_enabled,
  const std::array<bool, kPerceptionSourceCount> & source_fresh);

}  // namespace degradation
