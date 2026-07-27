/**
 * @file front_lowpass_stop.cpp
 * @brief 前向近障低通刹停闸实现，见 front_lowpass_stop.h
 */
#include "safety_node/core/front_lowpass_stop.h"

#include <algorithm>
#include <cmath>

FrontLowpassTiltTracker::FrontLowpassTiltTracker(
  const FrontObstacleLowpassStopParams & params)
: params_(params)
{
}

void FrontLowpassTiltTracker::updateParams(const FrontObstacleLowpassStopParams & params)
{
  params_ = params;
  reset();
}

void FrontLowpassTiltTracker::reset()
{
  sources_ = {};
}

FrontLowpassClearanceEstimate FrontLowpassTiltTracker::update(
  const std::array<FrontLowpassSourceSample, 2> & samples,
  double fb_linear_m_s,
  double dt_s)
{
  FrontLowpassClearanceEstimate result;
  const double speed = std::isfinite(fb_linear_m_s) ?
    std::max(0.0, fb_linear_m_s) : 0.0;
  const double dt = std::isfinite(dt_s) ? std::max(0.0, dt_s) : 0.0;
  const double prediction_limit = std::max(0.0, params_.stale_prediction_max_s);
  const double uncertainty_margin =
    std::max(0.0, params_.prediction_uncertainty_margin_m);

  for (size_t i = 0; i < sources_.size(); ++i) {
    auto & state = sources_[i];
    const auto & sample = samples[i];
    const bool new_frame =
      sample.frame_valid && sample.frame_stamp_ns > 0 &&
      (!state.has_frame || sample.frame_stamp_ns != state.frame_stamp_ns);

    if (new_frame) {
      // 同一源的新完整帧整体替换历史，包括 +inf（新帧明确无前向障碍）。
      state.has_frame = true;
      state.frame_stamp_ns = sample.frame_stamp_ns;
      state.measured_clearance_m = sample.clearance_m;
      state.frame_age_s = std::isfinite(sample.frame_age_s) ?
        std::max(0.0, sample.frame_age_s) : 0.0;
      state.forward_travel_m = params_.stale_prediction_enabled ?
        speed * state.frame_age_s : 0.0;
    } else if (state.has_frame) {
      // 控制频率通常高于雷达帧率；同一帧期间用轮速反馈积分自车前移量。
      double next_age = state.frame_age_s + dt;
      if (sample.frame_valid && std::isfinite(sample.frame_age_s)) {
        next_age = std::max(next_age, std::max(0.0, sample.frame_age_s));
      }
      const double age_step = std::max(0.0, next_age - state.frame_age_s);
      state.frame_age_s = next_age;
      if (params_.stale_prediction_enabled) {
        state.forward_travel_m += speed * age_step;
      }
    }

    if (!state.has_frame || !std::isfinite(state.measured_clearance_m)) {
      continue;
    }

    double effective_clearance = state.measured_clearance_m;
    const bool predicted =
      params_.stale_prediction_enabled && state.frame_age_s > 1e-6;
    if (predicted) {
      effective_clearance = std::max(
        0.0,
        effective_clearance - state.forward_travel_m - uncertainty_margin);
    }

    if (effective_clearance < result.clearance_m) {
      result.clearance_m = effective_clearance;
      result.predicted = predicted;
      result.observation_age_s = state.frame_age_s;
      result.valid_for_trigger = state.frame_age_s <= prediction_limit + 1e-9;
      result.source_index = static_cast<int>(i);
    }
  }

  return result;
}

FrontLowpassStopGate::FrontLowpassStopGate(const FrontObstacleLowpassStopParams & params)
: params_(params)
{
}

void FrontLowpassStopGate::updateParams(const FrontObstacleLowpassStopParams & params)
{
  params_ = params;
  reset();
}

void FrontLowpassStopGate::reset()
{
  latched_ = false;
  confirm_count_ = 0;
  clear_time_s_ = 0.0;
  last_finite_clearance_m_ = 0.0;
}

bool mapVetoYieldsToFrontLowpass(
  bool override_enabled,
  bool emergency_brake,
  bool is_map_veto_reason,
  double forward_corridor_clearance_m,
  double hard_stop_distance_m)
{
  if (!override_enabled || !emergency_brake || !is_map_veto_reason) {
    return false;
  }
  const double hard_stop = std::max(0.0, hard_stop_distance_m);
  // 只看前向走廊：与低通闸 trigger 同源。全向 min_clearance 不参与——侧向贴障时
  // Footprint 净空可早于走廊触硬停线，会把「盖过 map_veto」永久关掉。
  const bool corridor_inside_hard_stop =
    std::isfinite(forward_corridor_clearance_m) &&
    forward_corridor_clearance_m <= hard_stop;
  return !corridor_inside_hard_stop;
}

double FrontLowpassStopGate::timeConstant(
  double forward_clearance_m,
  double speed_m_s) const
{
  const double tau_max = std::max(1e-3, params_.filter_time_constant_s);
  const double tau_min = std::clamp(params_.min_time_constant_s, 1e-3, tau_max);
  if (!std::isfinite(forward_clearance_m)) {
    return tau_max;    // 走廊已清空（释放滞回期内），按最柔和收敛
  }
  const double margin = forward_clearance_m - std::max(0.0, params_.stop_buffer_m);
  if (margin <= 0.0) {
    return tau_min;    // 已吃进预留余量，收到最紧
  }
  const double speed = std::max(std::abs(speed_m_s), 1e-3);
  // 低通滑行距离闭式解 d = τ·v ⇒ τ = 剩余余量 / 当前速度
  return std::clamp(margin / speed, tau_min, tau_max);
}

FrontLowpassStopResult FrontLowpassStopGate::update(const FrontLowpassStopInput & in)
{
  FrontLowpassStopResult result;
  if (!params_.enabled) {
    reset();
    return result;
  }

  // 真硬急停（节点层已把「可柔停的 MAP_VETO」折算掉）与后退意图：立即解锁放行。
  // 注意 forward_clearance_m 在各条急停 early-return 上保持 +inf，与此处门控互为冗余。
  if (in.emergency_brake || in.reverse_intent) {
    const bool was_latched = latched_;
    reset();
    if (was_latched) {
      result.just_released = true;
      result.release_reason = in.emergency_brake ? "急停接管" : "后退意图";
    }
    return result;
  }

  const double valid_dt_s = std::isfinite(in.dt_s) ? std::max(0.0, in.dt_s) : 0.0;
  const bool in_band =
    std::isfinite(in.forward_clearance_m) &&
    in.forward_clearance_m <= params_.trigger_distance_m;
  // 逐周期记住最近的有限观测，供释放路径 2 判别 +inf 的两种含义。
  if (std::isfinite(in.forward_clearance_m)) {
    last_finite_clearance_m_ = in.forward_clearance_m;
  }

  if (!latched_) {
    if (in.trigger_observation_valid &&
      in_band && in.fb_linear_m_s > params_.trigger_speed_m_s)
    {
      if (++confirm_count_ >= std::max(1, params_.confirm_frames)) {
        latched_ = true;
        clear_time_s_ = 0.0;
        result.just_latched = true;
      }
    } else {
      confirm_count_ = 0;
    }
    if (!latched_) {
      return result;
    }
  } else {
    // 释放路径 1：已停稳且驾驶员撤销了前进意图 —— 本次「快速逼近」事件结束。
    // 少了这条，车会被锁在 stop_buffer 外直到退回 release_distance_m，贴近停靠做不到。
    // 从静止再推杆时 MDB 在该净空下的上限已远低于 trigger_speed_m_s，不会立刻重新触发。
    const double standstill_eps =
      std::max(0.0, params_.standstill_release_speed_eps_m_s);
    if (!in.forward_intent && std::abs(in.fb_linear_m_s) <= standstill_eps) {
      reset();
      result.just_released = true;
      result.release_reason = "停稳松杆";
      return result;
    }

    // 释放路径 2：净空滞回。只看净空不看速度——速度条件会被低通自己压没，
    // 拿它做释放判据必然在 trigger_speed_m_s 附近反复启停。
    //
    // +inf 有两种含义，必须区分，否则会把车送去撞：
    //   (a) 走廊真的空了（障碍移开 / 驾驶员转向让开）；
    //   (b) 障碍进了自车过滤盲区，或本帧感知丢点。
    // (b) 是实配下的常态：physicalRect().x_max=0.94 而 perception.self_filter.x_max_m=1.00，
    // 正前方 6 cm 以内的障碍被当自车滤除；low-pass 恰好把车刹到这个距离，障碍随即消失。
    // 若把 +inf 一律当作 (a) 解锁，而 local_map.rebuild_each_frame=true 无跨帧记忆、
    // MDB/hard_stop 用的同一个 field 也看不见该障碍 —— 车会解锁后重新加速撞上去。
    // 判别：只有「障碍在触发带之外就已消失」才算真清空；带内失去目标一律保守视为它还在，
    // 由停稳松杆（路径 1）或后退给出出路。
    const bool cleared = std::isfinite(in.forward_clearance_m) ?
      in.forward_clearance_m >= params_.release_distance_m :
      last_finite_clearance_m_ > params_.trigger_distance_m;
    if (cleared) {
      clear_time_s_ += valid_dt_s;
      if (clear_time_s_ >= std::max(0.0, params_.clear_hold_s)) {
        reset();
        result.just_released = true;
        result.release_reason = "净空滞回";
        return result;
      }
    } else {
      clear_time_s_ = 0.0;
    }
  }

  result.active = true;
  result.tau_s = timeConstant(
    in.forward_clearance_m,
    std::max(in.fb_linear_m_s, in.command_linear_m_s));
  return result;
}
