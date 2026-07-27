/**
 * @file pulse_compensator.cpp
 * @brief P0 终端脉冲补偿状态机实现
 */
#include "safety_node/core/pulse_compensator.h"

#include <algorithm>
#include <cmath>

void PulseCompensator::updateParams(const PreciseStopParams & params)
{
  params_ = params;
  reset();
}

void PulseCompensator::reset()
{
  state_ = State::IDLE;
  stall_time_s_ = 0.0;
  pulse_time_s_ = 0.0;
  cooldown_time_s_ = 0.0;
}

bool PulseCompensator::active() const
{
  return state_ == State::PULSING;
}

bool PulseCompensator::update(const Input & in, double dt_s)
{
  const double dt = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;

  // 前置条件不满足即回到 IDLE：残差已达容差（精准到位）、无前向意图、
  // 主链路禁止微动、或残差不可信时都不发脉冲。
  if (!params_.enabled || !params_.pulse_enabled ||
    !in.safe_to_creep || !in.forward_intent ||
    !std::isfinite(in.residual_m) ||
    in.residual_m <= params_.stop_tolerance_m)
  {
    reset();
    return false;
  }

  const bool stalled =
    std::abs(in.fb_linear) <= std::max(0.0, params_.pulse_stall_speed_eps_m_s);

  switch (state_) {
    case State::IDLE:
      if (stalled) {
        stall_time_s_ += dt;
      } else {
        stall_time_s_ = 0.0;
      }
      if (stall_time_s_ + 1e-9 >= std::max(0.0, params_.pulse_stall_hold_s)) {
        state_ = State::PULSING;
        pulse_time_s_ = 0.0;
      }
      break;

    case State::PULSING: {
      pulse_time_s_ += dt;
      // 过冲护栏：单脉冲位移不得越过剩余距离（残差每周期实测更新，
      // 车辆一旦前移残差缩小，允许时长同步收紧）。
      const double creep = std::max(1e-3, params_.pulse_creep_speed_m_s);
      const double bounded_duration_s = std::min(
        std::max(0.0, params_.pulse_max_duration_s),
        (in.residual_m - params_.stop_tolerance_m) / creep);
      if (pulse_time_s_ + 1e-9 >= bounded_duration_s) {
        state_ = State::COOLDOWN;
        cooldown_time_s_ = 0.0;
      }
      break;
    }

    case State::COOLDOWN:
      cooldown_time_s_ += dt;
      if (cooldown_time_s_ + 1e-9 >= std::max(0.0, params_.pulse_cooldown_s)) {
        state_ = State::IDLE;
        stall_time_s_ = 0.0;
      }
      break;
  }

  return state_ == State::PULSING;
}
