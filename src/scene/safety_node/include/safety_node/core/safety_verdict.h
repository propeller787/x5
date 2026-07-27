/**
 * @file safety_verdict.h
 * @brief 统一安全裁决类型 + 集中仲裁器（见 PRODUCTION_HARDENING_PLAN.md 方案 1）
 *
 * 目标形态：evaluate() 各安全层不再原地修改命令，而是各自产出 LayerVerdict，
 * 由 arbitrate() 按唯一的优先级表合成最终输出。迁移分步进行：
 *   步骤 A（当前）：影子仲裁——各层照旧原地生效，同时旁路记录 verdict，
 *   末尾用 arbitrate() 复算并与真实输出比对，不一致计入日志/健康摘要。
 *   步骤 B/C/D：逐层切换为「verdict 即真源」，每步以 bag 回归守门（见方案文档）。
 *
 * 仲裁规则（与单测 test_safety_arbiter.cpp 绑定一致）：
 *   1. 基准命令 = 优先级最高的活跃 suggestion（无则用驾驶员期望）；
 *   2. 逃逸令牌（escape_grant）豁免优先级低于其发放层的 stop/full_stop，
 *      但不可豁免 has_collision_stop（真实碰撞全向停不可绕过）；
 *   3. 分向限速 cap 按 min 合成；stop_axes 按并集清零对应分量；
 *   4. request_full_stop（未被豁免）→ 整帧零速；
 *   5. 单调性：新增一条仅含 cap/stop 的裁决（无 suggestion / 无 escape_grant）
 *      永远不会使输出变宽——这是「层可安全叠加」的形式化保证。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <geometry_msgs/msg/twist.hpp>
#include <cstdint>
#include <limits>
#include <vector>

/**
 * 安全层标识。枚举序即优先级：数值越小优先级越高。
 * 优先级同时决定：suggestion 基准的选取顺序、逃逸令牌的豁免边界。
 */
enum class SafetyLayerId : uint8_t
{
  INPUT_FAULT = 0,       ///< 输入故障（节点层，最高优先级）
  MODE_ESTOP,            ///< ESTOP 模式
  DEGRADATION,           ///< 传感器降级矩阵（方案 4）
  STANDSTILL_GUARD,      ///< P0-B 停稳看护
  SUDDEN_OBSTACLE,       ///< L0-EB 突发障碍紧急制动
  MAP_VETO,              ///< LocalMap 硬约束否决
  RECOVERY_ESCAPE,       ///< 脱困（recovery / directional / pinned），持逃逸令牌
  DIRECTION_MASK,        ///< L1 方向解耦掩码（分轴封锁）
  FORWARD_STOP_HOLD,     ///< MDB 停车后前向保持
  PULSE_COMP,            ///< P0 终端脉冲补偿（受控微动作 suggestion）
  CANDIDATE_SELECT,      ///< 候选选速结果（含共享控制融合与加减速限幅）
  BRAKING_ENVELOPE,      ///< 制动包络限速
  TURN_CLEARANCE_GUARD,  ///< 转弯净空护栏限速
  TURN_ENVELOPE,         ///< 转弯合成速度包络
  EMERGENCY_GATE,        ///< EmergencyBrake 最终硬闸
  COUNT,
};

/** 分轴停车掩码（true = 该方向分量强制清零） */
struct VerdictAxisMask
{
  bool forward = false;
  bool backward = false;
  bool left = false;      ///< +w（左转）
  bool right = false;     ///< −w（右转）

  bool any() const {return forward || backward || left || right;}
};

/** 单层单周期裁决 */
struct LayerVerdict
{
  SafetyLayerId id = SafetyLayerId::COUNT;
  bool active = false;

  /** 分向速度上限（绝对值，m/s 或 rad/s）；+inf = 不限制 */
  double forward_cap = std::numeric_limits<double>::infinity();
  double backward_cap = std::numeric_limits<double>::infinity();
  double left_cap = std::numeric_limits<double>::infinity();
  double right_cap = std::numeric_limits<double>::infinity();

  /** 分轴强制清零 */
  VerdictAxisMask stop_axes;
  /** 整帧零速请求 */
  bool request_full_stop = false;
  /**
   * 真实碰撞级停车：不可被任何逃逸令牌豁免
   * （对应现行为「has_collision / 全向 emergency 仍整帧停，不放宽」）。
   */
  bool has_collision_stop = false;

  /** 该层给出的基准命令建议（候选选速 / 脱困命令 / 脉冲微动作） */
  bool has_suggestion = false;
  geometry_msgs::msg::Twist suggestion;
  /**
   * 逃逸令牌：suggestion 是经过安全复验的受控逃逸，
   * 豁免优先级低于本层的 stop（如 EmergencyBrake 硬闸），
   * 使脱困命令不被下游硬停重新钉零。
   */
  bool escape_grant = false;

  EmergencyStopReason reason = EmergencyStopReason::NONE;
  bool bypass_smoothing = false;
};

/** 仲裁结果 */
struct ArbitrationResult
{
  geometry_msgs::msg::Twist twist;
  bool full_stop = false;
  EmergencyStopReason reason = EmergencyStopReason::NONE;
  SafetyLayerId deciding_layer = SafetyLayerId::COUNT;
  bool escape_granted = false;
  bool bypass_smoothing = false;
};

namespace safety_arbiter
{

/** 层优先级：数值越小越高（当前直接取枚举序数，集中一处便于调整） */
int layerPriority(SafetyLayerId id);

/** 层名（日志 / 健康摘要用） */
const char * layerName(SafetyLayerId id);

/**
 * 集中仲裁：按文件头注释的 5 条规则，从 desired + 各层裁决合成最终命令。
 * 纯函数、无状态，可独立 fuzz（见 test_safety_arbiter.cpp 单调性测试）。
 */
ArbitrationResult arbitrate(
  const geometry_msgs::msg::Twist & desired,
  const std::vector<LayerVerdict> & verdicts);

}  // namespace safety_arbiter
