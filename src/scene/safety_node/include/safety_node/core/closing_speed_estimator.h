/**
 * @file closing_speed_estimator.h
 * @brief P0-A' 动态障碍逼近速度估计（见 PRODUCTION_HARDENING_PLAN.md 方案 5）
 *
 * 由前向走廊最近物理净空的逐周期差分，估计「障碍自身的逼近速度」
 * （已扣除自车前进分量）。输出接入两处：
 *   1. 制动包络：动态预留距离 = max(参数下限, 估计值) × 停车总时间，
 *      替代原先固定常数的 braking_obstacle_closing_reserve_m_s；
 *   2. L0-EB：a_req 用相对速度 v_rel = v_self + closing_rate 计算，
 *      对向切入时触发显著提前。
 *
 * 鲁棒性设计（每条对应一个单测）：
 *   - 只输出逼近分量（≥0），远离视为 0；输出钳制到 max_rate_m_s；
 *   - 须连续 confirm_frames 帧观测到逼近才输出非零，滤单帧体素抖动；
 *   - 净空增大突变（> jump_reset_m，典型为漏检恢复/障碍离开）→ 复位估计；
 *   - 任一侧净空非有限（无障碍/首帧）→ 复位，不外推。
 *
 * 明确不做：多目标跟踪/ID 关联（2D 稀疏点下投入产出比不成立）。
 * 能力边界：横穿行人在进入前向走廊之前不可预判——见 README 能力边界章节。
 *
 * 纯算法、无 ROS 依赖，便于单元测试。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

class ClosingSpeedEstimator
{
public:
  explicit ClosingSpeedEstimator(const ClosingSpeedParams & params);

  void updateParams(const ClosingSpeedParams & params);

  /**
   * @param clearance_m         本周期前向走廊最近物理净空 (m)；无障碍传 +inf
   * @param ego_forward_speed   自车前向速度 (m/s)，取 max(0, fb_linear)
   * @param dt_s                控制周期 (s)
   * @return 障碍自身逼近速度估计 (m/s)，≥0；未确认/复位时为 0
   */
  double update(double clearance_m, double ego_forward_speed, double dt_s);

  /** 当前估计值（不推进状态） */
  double rate() const {return confirmed_ ? filtered_rate_m_s_ : 0.0;}

  void reset();

private:
  ClosingSpeedParams params_;
  double prev_clearance_m_ = 0.0;
  bool has_prev_ = false;
  double filtered_rate_m_s_ = 0.0;
  int confirm_count_ = 0;
  bool confirmed_ = false;
};
