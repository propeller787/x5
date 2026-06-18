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
   * @return true 表示 lethal（应否决该候选）
   */
  bool trajectoryLethal(
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
