/**
 * @file sudden_obstacle_detector.h
 * @brief L0-EB 突发障碍动态紧急制动检测器
 *
 * 每周期按"所需减速度" a_req = v² / (2·d_eff) 判定是否为突发障碍：
 *   d_eff = 前向净空 − v·t_react − d_safe
 * a_req 超过舒适减速度即视为突发/切入（MDB 没能提前压住），触发紧急制动，
 * 由调用方把减速度抬到 emergency_decel 并绕过正常 max_decel 限幅。
 *
 * 纯几何 / 状态机，无 ROS 依赖，便于单元测试。见 SUDDEN_BRAKE_PLAN.md。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

/** 单周期检测结果 */
struct SuddenObstacleResult
{
  /** 本周期是否处于紧急制动（含锁存保持期间） */
  bool trigger = false;
  /** 所需减速度是否超过底盘紧急减速能力（物理停不住，碰撞风险） */
  bool saturated = false;
  /** 低通后的所需减速度 a_req (m/s²) */
  double a_req = 0.0;
};

class SuddenObstacleDetector
{
public:
  explicit SuddenObstacleDetector(const SuddenObstacleParams & params);

  void updateParams(const SuddenObstacleParams & params);

  /**
   * @param forward_clearance_m 前向走廊内到物理车体的最近净空 (m)，无障碍时传 +inf
   * @param forward_speed       当前前向速度 (m/s)，倒车/静止传 ≤0
   * @param d_safe_m            停在障碍前保留的几何余量 (m)（hard_stop + buffer）
   * @param dt_s                控制周期 (s)
   */
  SuddenObstacleResult update(
    double forward_clearance_m,
    double forward_speed,
    double d_safe_m,
    double dt_s);

  void reset();
  bool latched() const {return latched_;}

private:
  SuddenObstacleParams params_;
  double a_req_filtered_ = 0.0;
  int confirm_count_ = 0;
  bool latched_ = false;
  double clear_time_s_ = 0.0;
};
