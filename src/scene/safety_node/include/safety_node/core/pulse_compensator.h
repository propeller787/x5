/**
 * @file pulse_compensator.h
 * @brief P0 终端脉冲补偿：MDB 停短（底盘死区/静摩擦）时的有界前向微脉冲
 *
 * 近距指数速度律的终端速度会低于底盘可执行最小速度，导致车辆停在
 * hard-stop 目标线之外（停短）。本模块检测「有前向意图 + 反馈停住 +
 * 残差未达容差」的停短状态，发出速度略高于死区的前向微脉冲；
 * 单脉冲位移由过冲护栏约束（creep·t ≤ 残差 − 容差），从物理上保证
 * 不会因补偿越线。一次不到位则冷却后再补，逐步收敛到目标线。
 *
 * 本模块只产生「是否应输出 creep」的建议；上游 AvoidanceController
 * 在无急停/脱困/at_min_stop 的前提下才会采纳，采纳后的输出仍要经过
 * 最终全量安全复查与 EmergencyBrake 硬闸。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

class PulseCompensator {
public:
  /** 单周期输入快照 */
  struct Input
  {
    /** 剩余到 hard-stop 目标线的距离 (m)，即 braking_effective_remaining */
    double residual_m = std::numeric_limits < double > ::infinity();
    /** 轮速反馈线速度 (m/s) */
    double fb_linear = 0.0;
    /** 驾驶员是否仍保持前向意图 */
    bool forward_intent = false;
    /** 主链路是否允许微动（无急停/脱困/侵入/at_min_stop） */
    bool safe_to_creep = false;
  };

  void updateParams(const PreciseStopParams & params);

  /** 单周期更新；返回 true 时本周期应输出 pulse_creep_speed_m_s 前向速度 */
  bool update(const Input & in, double dt_s);

  void reset();

  bool active() const;

private:
  enum class State
  {
    IDLE,        ///< 监测停短
    PULSING,     ///< 输出微脉冲
    COOLDOWN,    ///< 脉冲后冷却，等待反馈稳定
  };

  PreciseStopParams params_;
  State state_ = State::IDLE;
  double stall_time_s_ = 0.0;
  double pulse_time_s_ = 0.0;
  double cooldown_time_s_ = 0.0;
};
