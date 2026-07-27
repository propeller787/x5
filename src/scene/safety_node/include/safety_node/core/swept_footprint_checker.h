/**
 * @file swept_footprint_checker.h
 * @brief Footprint 扫掠碰撞检测
 *
 * 对轨迹上每个 PredictedPose，将 ObstacleField 中的 primitive 变换到该 pose
 * 下的车体坐标系，与膨胀矩形做碰撞/净空检测，得到 ttc 与 min_distance。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/vehicle_footprint.h"

class SweptFootprintChecker {
public:
  SweptFootprintChecker(const AvoidanceParams & params, VehicleFootprint & footprint);

  void updateParams(const AvoidanceParams & params);

  CollisionCheckResult checkTrajectory(
    const TrajectoryRollout & trajectory,
    const ObstacleField & field) const;

private:
  AvoidanceParams params_;
  VehicleFootprint & footprint_;
};
