/**
 * @file front_tilt_speed_cap.cpp
 * @brief 斜雷达前向近障软限速闸实现，见 front_tilt_speed_cap.h
 */
#include "safety_node/core/front_tilt_speed_cap.h"

#include <algorithm>
#include <cmath>

FrontTiltSpeedCapGate::FrontTiltSpeedCapGate(const FrontTiltSpeedCapParams & params)
: params_(params)
{
}

void FrontTiltSpeedCapGate::updateParams(const FrontTiltSpeedCapParams & params)
{
  params_ = params;
  reset();
}

void FrontTiltSpeedCapGate::reset()
{
  latched_ = false;
  confirm_count_ = 0;
  clear_time_s_ = 0.0;
}

FrontTiltSpeedCapResult FrontTiltSpeedCapGate::update(const FrontTiltSpeedCapInput & in)
{
  FrontTiltSpeedCapResult result;
  if (!params_.enabled) {
    reset();
    return result;    // active=false, cap=+inf
  }

  const double valid_dt_s = std::isfinite(in.dt_s) ? std::max(0.0, in.dt_s) : 0.0;
  // 在触发带内 = 有限净空且 ≤ trigger_distance_m。+inf（走廊内无障碍）不在带内。
  const bool in_band =
    std::isfinite(in.forward_clearance_m) &&
    in.forward_clearance_m <= params_.trigger_distance_m;

  if (!latched_) {
    if (in_band) {
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
    // 滞回释放：净空恢复到 release_distance_m 以上（含 +inf = 走廊清空）持续 clear_hold_s。
    // trigger < release 的滞回带避免在边界反复升降速。
    const bool cleared =
      !std::isfinite(in.forward_clearance_m) ||
      in.forward_clearance_m >= params_.release_distance_m;
    if (cleared) {
      clear_time_s_ += valid_dt_s;
      if (clear_time_s_ >= std::max(0.0, params_.clear_hold_s)) {
        reset();
        result.just_released = true;
        return result;
      }
    } else {
      clear_time_s_ = 0.0;
    }
  }

  result.active = true;
  result.speed_cap_m_s = std::max(0.0, params_.speed_cap_m_s);
  return result;
}
