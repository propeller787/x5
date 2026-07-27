/**
 * @file mode_supervisor.cpp
 * @brief 安全模式 → ModeProfile 映射
 */
#include "safety_node/mode_supervisor.h"

#include <algorithm>

ModeSupervisor::ModeSupervisor(
  SafetyMode initial_mode,
  const NarrowPassageConfig & narrow_passage)
: mode_(initial_mode)
  , narrow_passage_(narrow_passage)
{
}

ModeProfile ModeSupervisor::profile() const
{
  ModeProfile p;
  switch (mode_) {
    case SafetyMode::PASS_THROUGH:
      // 透传：关闭全部避障子系统，仅保留 OutputShaper
      p.obstacle_field_enabled = false;
      p.footprint_selector_enabled = false;
      p.local_map_update_enabled = false;
      p.map_constraint_enabled = false;
      p.emergency_stop_enabled = false;
      break;
    case SafetyMode::CRUISE:
      p.lethal_threshold = 0.65;
      p.inflation_radius_m = 0.15;
      break;
    case SafetyMode::NARROW:
      p.avoidance_overrides_enabled = true;
      p.footprint_side_margin_m = narrow_passage_.footprint_side_margin_m;
      p.obstacle_radius_m = narrow_passage_.obstacle_radius_m;
      p.hard_stop_distance_m = narrow_passage_.hard_stop_distance_m;
      p.hard_stop_release_distance_m =
        narrow_passage_.hard_stop_release_distance_m;
      p.inflation_radius_m =
        narrow_passage_.local_map_inflation_radius_m;
      // Narrow mode uses an explicit low-clearance geometry profile.
      p.lethal_threshold = narrow_passage_.lethal_threshold;
      break;
    case SafetyMode::ESTOP:
      p.obstacle_field_enabled = false;
      p.footprint_selector_enabled = false;
      p.local_map_update_enabled = false;
      p.map_constraint_enabled = true;
      p.emergency_stop_enabled = true;
      break;
    case SafetyMode::SAFE_ASSIST:
    default:
      p.lethal_threshold = 0.65;
      p.inflation_radius_m = 0.15;
      break;
  }
  return p;
}

bool ModeSupervisor::setMode(SafetyMode mode)
{
  if (mode_ == mode) {
    return false;
  }
  mode_ = mode;
  return true;
}

bool ModeSupervisor::setModeFromString(const std::string & name)
{
  return setMode(parseSafetyMode(name));
}

void ModeSupervisor::setNarrowPassageConfig(const NarrowPassageConfig & config)
{
  narrow_passage_ = config;
}

bool ModeSupervisor::onModeChanged(SafetyMode previous, SafetyMode current) const
{
  return previous != current;
}

std::string ModeSupervisor::modeToString(SafetyMode mode)
{
  switch (mode) {
    case SafetyMode::PASS_THROUGH:
      return "pass_through";
    case SafetyMode::CRUISE:
      return "cruise";
    case SafetyMode::NARROW:
      return "narrow";
    case SafetyMode::ESTOP:
      return "estop";
    case SafetyMode::SAFE_ASSIST:
    default:
      return "safe_assist";
  }
}

AvoidanceParams applyModeProfile(
  const AvoidanceParams & base,
  const ModeProfile & profile)
{
  AvoidanceParams active = base;
  if (!profile.avoidance_overrides_enabled) {
    return active;
  }

  active.footprint_side_margin = std::max(0.0, profile.footprint_side_margin_m);
  active.footprint_obstacle_radius_default =
    std::max(0.0, profile.obstacle_radius_m);
  active.hard_stop_distance_m = std::max(0.0, profile.hard_stop_distance_m);
  active.hard_stop_release_distance_m =
    std::max(
    active.hard_stop_distance_m,
    std::max(0.0, profile.hard_stop_release_distance_m));
  return active;
}
