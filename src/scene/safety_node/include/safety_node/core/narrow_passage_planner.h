/**
 * @file narrow_passage_planner.h
 * @brief P2 窄道通过规划：走廊提取 + 三次 Bézier 中线导引
 *
 * 纯几何模块，无 ROS 依赖（仅共享类型）。流程：
 *   1. 前向 ROI 内障碍按 y 正负分左右壁，沿 x 分 bin 取各 bin 内壁最近内缘；
 *   2. 从车头向前累计连续可行段（gap ≥ 车宽 + 2·min_side_margin），
 *      要求长度 ≥ min_corridor_len 且至少一个 bin 两侧都有墙（排除开阔区）；
 *   3. 走廊中线出口点 + 出口切线定三次 Bézier（P0=车体原点、入口航向 0），
 *      取起点曲率 κ0 → 居中候选 w = v·κ0；
 *   4. 线速度按最小侧向净空平滑限速（clearance-scaled），保底 min_pass_speed。
 *
 * 输出候选注入 AvoidanceController 候选集后，仍需通过 Footprint 扫掠与
 * Map veto 的全部安全检查——本模块只提供导引，不放宽任何安全判据。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <geometry_msgs/msg/twist.hpp>

/** 单周期窄道规划结果 */
struct NarrowCorridorPlan
{
  bool valid = false;
  /** 走廊最小间隙宽度 (m)，仅统计有墙的 bin */
  double corridor_width_m = std::numeric_limits < double > ::infinity();
  /** 连续可行走廊长度 (m) */
  double corridor_length_m = 0.0;
  /** 车体居中时的单侧侧向净空 (m) = (gap − 车宽)/2 */
  double lateral_clearance_m = 0.0;
  /** Bézier 中线在车体处的曲率 (1/m)，左转为正 */
  double curvature_1_m = 0.0;
  /** 侧向净空反算的线速度上限 (m/s) */
  double speed_cap_m_s = std::numeric_limits < double > ::infinity();
  /** 居中导引候选（注入候选集） */
  VelocityCandidate candidate;
};

class NarrowPassagePlanner {
public:
  explicit NarrowPassagePlanner(const AvoidanceParams & params);

  void updateParams(const AvoidanceParams & params);

  /**
   * 单周期窄道提取与导引候选生成。
   * @param field           当前水平障碍场（base_link）
   * @param cmd             驾驶员期望 Twist（仅前向命令时规划）
   * @param vehicle_width_m 物理车体最大宽度 (m)
   * @return 未检出有效窄道时 valid=false
   */
  NarrowCorridorPlan plan(
    const ObstacleField & field,
    const geometry_msgs::msg::Twist & cmd,
    double vehicle_width_m) const;

private:
  AvoidanceParams params_;
};
