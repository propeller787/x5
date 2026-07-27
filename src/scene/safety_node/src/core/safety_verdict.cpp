/**
 * @file safety_verdict.cpp
 * @brief 集中仲裁器实现（规则见 safety_verdict.h 头注释，与单测绑定一致）
 */
#include "safety_node/core/safety_verdict.h"

#include <algorithm>
#include <cmath>

namespace safety_arbiter
{

int layerPriority(SafetyLayerId id)
{
  return static_cast<int>(id);
}

const char * layerName(SafetyLayerId id)
{
  switch (id) {
    case SafetyLayerId::INPUT_FAULT: return "input_fault";
    case SafetyLayerId::MODE_ESTOP: return "mode_estop";
    case SafetyLayerId::DEGRADATION: return "degradation";
    case SafetyLayerId::STANDSTILL_GUARD: return "standstill_guard";
    case SafetyLayerId::SUDDEN_OBSTACLE: return "sudden_obstacle";
    case SafetyLayerId::MAP_VETO: return "map_veto";
    case SafetyLayerId::RECOVERY_ESCAPE: return "recovery_escape";
    case SafetyLayerId::DIRECTION_MASK: return "direction_mask";
    case SafetyLayerId::FORWARD_STOP_HOLD: return "forward_stop_hold";
    case SafetyLayerId::PULSE_COMP: return "pulse_comp";
    case SafetyLayerId::CANDIDATE_SELECT: return "candidate_select";
    case SafetyLayerId::BRAKING_ENVELOPE: return "braking_envelope";
    case SafetyLayerId::TURN_CLEARANCE_GUARD: return "turn_clearance_guard";
    case SafetyLayerId::TURN_ENVELOPE: return "turn_envelope";
    case SafetyLayerId::EMERGENCY_GATE: return "emergency_gate";
    case SafetyLayerId::COUNT: break;
  }
  return "unknown";
}

ArbitrationResult arbitrate(
  const geometry_msgs::msg::Twist & desired,
  const std::vector<LayerVerdict> & verdicts)
{
  ArbitrationResult out;
  out.twist = desired;

  // ── 规则 1：基准命令 = 优先级最高的活跃 suggestion ──
  int best_suggestion_priority = std::numeric_limits<int>::max();
  for (const auto & v : verdicts) {
    if (!v.active || !v.has_suggestion) {
      continue;
    }
    const int prio = layerPriority(v.id);
    if (prio < best_suggestion_priority) {
      best_suggestion_priority = prio;
      out.twist = v.suggestion;
      out.deciding_layer = v.id;
    }
  }

  // ── 规则 2：最高优先级的逃逸令牌决定豁免边界 ──
  int escape_priority = std::numeric_limits<int>::max();
  for (const auto & v : verdicts) {
    if (v.active && v.has_suggestion && v.escape_grant) {
      escape_priority = std::min(escape_priority, layerPriority(v.id));
    }
  }
  out.escape_granted = escape_priority < std::numeric_limits<int>::max();

  const auto suppressed = [&](const LayerVerdict & v) {
      // 逃逸令牌只豁免优先级低于发放层的约束；碰撞级停车永不豁免。
      return out.escape_granted &&
             layerPriority(v.id) > escape_priority &&
             !v.has_collision_stop;
    };

  // ── 规则 3：分向 cap 按 min 合成，stop_axes 按并集清零 ──
  for (const auto & v : verdicts) {
    if (!v.active || suppressed(v)) {
      continue;
    }
    double & lin = out.twist.linear.x;
    double & ang = out.twist.angular.z;
    if (lin > 0.0 && std::isfinite(v.forward_cap)) {
      lin = std::min(lin, std::max(0.0, v.forward_cap));
    }
    if (lin < 0.0 && std::isfinite(v.backward_cap)) {
      lin = std::max(lin, -std::max(0.0, v.backward_cap));
    }
    if (ang > 0.0 && std::isfinite(v.left_cap)) {
      ang = std::min(ang, std::max(0.0, v.left_cap));
    }
    if (ang < 0.0 && std::isfinite(v.right_cap)) {
      ang = std::max(ang, -std::max(0.0, v.right_cap));
    }
  }
  for (const auto & v : verdicts) {
    if (!v.active || suppressed(v)) {
      continue;
    }
    if (v.stop_axes.forward && out.twist.linear.x > 0.0) {
      out.twist.linear.x = 0.0;
    }
    if (v.stop_axes.backward && out.twist.linear.x < 0.0) {
      out.twist.linear.x = 0.0;
    }
    if (v.stop_axes.left && out.twist.angular.z > 0.0) {
      out.twist.angular.z = 0.0;
    }
    if (v.stop_axes.right && out.twist.angular.z < 0.0) {
      out.twist.angular.z = 0.0;
    }
  }

  // ── 规则 4：整帧零速请求（未被豁免） ──
  int stop_priority = std::numeric_limits<int>::max();
  for (const auto & v : verdicts) {
    if (!v.active || !v.request_full_stop || suppressed(v)) {
      continue;
    }
    out.full_stop = true;
    const int prio = layerPriority(v.id);
    if (prio < stop_priority) {
      stop_priority = prio;
      out.deciding_layer = v.id;
      out.reason = v.reason;
      out.bypass_smoothing = v.bypass_smoothing;
    }
  }
  if (out.full_stop) {
    out.twist = geometry_msgs::msg::Twist{};
  }

  return out;
}

}  // namespace safety_arbiter
