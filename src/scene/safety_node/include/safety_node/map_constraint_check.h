/**
 * @file map_constraint_check.h
 * @brief 轨迹沿 RollingLocalMap 的 lethal 硬约束查询
 *
 * 对 Footprint 已选中的候选轨迹，沿离散 pose 采样：
 * - check_corners=true：检查膨胀 footprint 四角
 * - check_corners=false：仅检查 pose 中心点
 *
 * 任一采样点 cost >= lethal_threshold 则判定 lethal，触发 Map veto。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/vehicle_footprint.h"

class MapConstraintCheck {
public:
  MapConstraintCheck(const AvoidanceParams & fp_params, const MapConstraintConfig & config);

  void updateFootprintParams(const AvoidanceParams & fp_params);
  void updateConfig(const MapConstraintConfig & config);

  MapConstraintResult queryTrajectory(
    const RollingLocalMapGrid & map,
    const std::vector < PredictedPose > & poses,
    double lethal_threshold,
    bool check_corners) const;

  /**
   * @brief 判断轨迹是否侵入 lethal 区
   * @param exemptions L1 近障豁免圆（可空）：lethal 采样点落在任一圆内则忽略。
   *        近障已由 Footprint 威胁谓词判定「该候选不将其推向物理接触」，
   *        其写入地图的占用/膨胀栅格不应再次否决同一候选（方向解耦，见 direction_mask.h）。
   *        圆外的新 lethal 栅格仍正常否决。
   * @return true 表示 lethal（应否决该候选）
   */
  bool trajectoryLethal(
    const RollingLocalMapGrid & map,
    const std::vector < PredictedPose > & poses,
    double lethal_threshold,
    bool check_corners,
    MapConstraintResult & out,
    const std::vector < MapExemptCircle > * exemptions = nullptr) const;

  /**
   * @brief 判断轨迹是否「不进入起始姿势未占用的新 lethal 栅格」（不恶化，用于原地转向豁免）
   *
   * 以起始位姿 footprint 覆盖的 lethal 栅格为豁免集：后续任一位姿只要踏入不在该集合内的
   * 新 lethal 栅格即返回 false（恶化）。与 trajectoryEscapesInitialLethal 的区别——本函数
   * 不要求清空/退离/进展，仅要求「不进新 lethal」，故允许在已 veto（起始已 lethal）态下
   * 维持原覆盖地转向脱困。起始集为空（起始不 lethal）时退化为「全程不得触任何 lethal」。
   * footprint 采样越出地图边界视为不安全（返回 false，保守）。
   * @return true 表示安全（不恶化）；false 表示踏入新 lethal 或越界。
   */
  bool trajectoryNoNewLethal(
    const RollingLocalMapGrid & map,
    const std::vector < PredictedPose > & poses,
    double lethal_threshold,
    bool check_corners,
    MapConstraintResult & out) const;

  /**
   * @brief 判断轨迹是否只是在远离当前已占用的地图栅格
   *
   * 初始 footprint 必须已命中 lethal；后续不得进入任何新的 lethal 栅格，
   * 并且必须在 max_escape_time_s 内清除全部初始占用，清除后不得再次进入。
   */
  bool trajectoryEscapesInitialLethal(
    const RollingLocalMapGrid & map,
    const std::vector < PredictedPose > & poses,
    double lethal_threshold,
    bool check_corners,
    double max_escape_time_s,
    MapConstraintResult & out) const;

private:
  AvoidanceParams fp_params_;
  MapConstraintConfig config_;
  VehicleFootprint footprint_;
};
