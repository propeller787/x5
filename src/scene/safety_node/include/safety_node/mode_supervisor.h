/**
 * @file mode_supervisor.h
 * @brief 安全模式状态机与各模式下的子系统剖面
 *
 * 模式与子系统开关对应 HYBRID_FOOTPRINT_LOCALMAP.md §6.2：
 *   PASS_THROUGH → 关闭 Footprint / LocalMap / Map veto
 *   SAFE_ASSIST  → 全开，标准 lethal 阈值
 *   NARROW       -> configured low-clearance geometry and map inflation
 *   ESTOP        → 强制急停逻辑
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <string>

/** 某一 SafetyMode 下的运行时剖面（阈值 + 子系统开关） */
struct ModeProfile
{
  bool avoidance_overrides_enabled = false;
  double footprint_side_margin_m = 0.0;
  double obstacle_radius_m = 0.0;
  double hard_stop_distance_m = 0.0;
  double hard_stop_release_distance_m = 0.0;
  bool obstacle_field_enabled = true;        ///< 是否构建 ObstacleField
  bool footprint_selector_enabled = true;    ///< 是否做 Footprint 选速
  bool local_map_update_enabled = true;      ///< 是否更新 LocalMap
  bool map_constraint_enabled = true;        ///< 是否启用 Map veto
  bool emergency_stop_enabled = true;        ///< 是否启用急停兜底
  double lethal_threshold = 0.65;            ///< 当前模式 lethal 判定阈值
  double inflation_radius_m = 0.15;          ///< 预留：模式化膨胀（V2）
};

class ModeSupervisor {
public:
  explicit ModeSupervisor(
    SafetyMode initial_mode,
    const NarrowPassageConfig & narrow_passage = NarrowPassageConfig {});

  SafetyMode mode() const {return mode_;}

  /** 根据当前 mode_ 返回子系统剖面 */
  ModeProfile profile() const;

  bool setMode(SafetyMode mode);

  bool setModeFromString(const std::string & name);

  void setNarrowPassageConfig(const NarrowPassageConfig & config);

  bool onModeChanged(SafetyMode previous, SafetyMode current) const;

  static std::string modeToString(SafetyMode mode);

private:
  SafetyMode mode_;
  NarrowPassageConfig narrow_passage_;
};

AvoidanceParams applyModeProfile(
  const AvoidanceParams & base,
  const ModeProfile & profile);
