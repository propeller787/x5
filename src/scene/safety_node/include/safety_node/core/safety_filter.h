/**
 * @file safety_filter.h
 * @brief Footprint 选速：选择最接近手柄意图的安全候选
 *
 * 在所有候选轨迹完成扫掠碰撞后：
 * 1. 优先选无碰撞且 command_error 最小者
 * 2. 若全碰撞，选 ttc 最大 / 速度最小者降级
 * 3. 应用周期内加减速限制与绝对限速
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/swept_footprint_checker.h"

#include <geometry_msgs/msg/twist.hpp>
#include <vector>

class SafetyFilter {
public:
  SafetyFilter(const AvoidanceParams & params, SweptFootprintChecker & checker);

  void updateParams(const AvoidanceParams & params);

  SafetyFilterResult selectSafeVelocity(
    const geometry_msgs::msg::Twist & cmd,
    const std::vector < TrajectoryRollout > & rollouts,
    const ObstacleField & field,
    double prev_linear_x,
    double prev_angular_z,
    double dt_s) const;

private:
  double commandErrorCost(
    const VelocityCandidate & cand,
    const geometry_msgs::msg::Twist & cmd) const;

  AvoidanceParams params_;
  SweptFootprintChecker & checker_;
};
