/**
 * @file emergency_brake.h
 * @brief 净空/物理制动能力不足时的硬急停兜底
 *
 * 默认按硬停净空和制动距离判定；TTC 仅在兼容开关启用时参与。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <geometry_msgs/msg/twist.hpp>
#include <limits>

/** 停驻后按手柄方向评估净空，避免前方近障阻断安全后退/转向 */
struct EmergencyBrakeContext
{
  bool vehicle_stopped = false;
  bool direction_command_active = false;
  bool directional_clearance_valid = false;
  bool controlled_escape_active = false;
  double directional_clearance = std::numeric_limits < double > ::infinity();
};

class EmergencyBrake {
public:
  explicit EmergencyBrake(const AvoidanceParams & params);

  void updateParams(const AvoidanceParams & params);

  /** 根据聚合风险判断是否必须刹停 */
  bool shouldEmergencyBrake(
    const CollisionRisk & risk,
    const EmergencyBrakeContext * context = nullptr) const;
  bool update(
    const CollisionRisk & risk,
    double dt_s,
    const EmergencyBrakeContext * context = nullptr);
  void latch();
  void reset();
  bool latched() const;

  /** 返回零 Twist */
  geometry_msgs::msg::Twist applyEmergencyBrake() const;

private:
  bool releaseConditionsMet(
    const CollisionRisk & risk,
    const EmergencyBrakeContext * context = nullptr) const;
  double effectiveClearance(
    const CollisionRisk & risk,
    const EmergencyBrakeContext * context) const;

  AvoidanceParams params_;
  bool latched_ = false;
  double clear_time_s_ = 0.0;
};
