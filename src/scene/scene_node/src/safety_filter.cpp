#include "safety_filter.h"

#include <cmath>
#include <limits>

namespace scene_node::avoidance
{
namespace
{

double commandError(
  const MotionCommand & desired,
  const MotionCommand & candidate,
  const SafetyFilterParams & params)
{
  return std::abs(candidate.linear_x - desired.linear_x) +
         params.angular_weight * std::abs(candidate.angular_z - desired.angular_z);
}

}  // namespace

SafetyFilterResult SafetyFilter::select(
  const MotionCommand & desired,
  const std::vector<CandidateTrajectory> & trajectories,
  const std::vector<CollisionRisk> & risks,
  const SafetyFilterParams & params) const
{
  SafetyFilterResult result;
  result.selected_command = desired;
  result.trajectory_count = trajectories.size();

  if (trajectories.empty() || risks.empty()) {
    result.emergency_brake = true;
    result.selected_command = MotionCommand{0.0, 0.0};
    result.risk.level = RiskLevel::BRAKING;
    result.risk.has_collision = true;
    result.risk.time_to_collision = 0.0;
    return result;
  }

  std::size_t best_clear_index = trajectories.size();
  double best_clear_score = std::numeric_limits<double>::infinity();

  std::size_t best_ttc_index = 0;
  double best_ttc = -1.0;

  for (std::size_t i = 0; i < trajectories.size() && i < risks.size(); ++i) {
    const auto & risk = risks[i];
    if (!risk.has_collision) {
      const double score = commandError(desired, trajectories[i].command, params);
      if (score < best_clear_score) {
        best_clear_score = score;
        best_clear_index = i;
      }
    } else if (risk.time_to_collision > best_ttc) {
      best_ttc = risk.time_to_collision;
      best_ttc_index = i;
    }
  }

  if (best_clear_index < trajectories.size()) {
    result.selected_command = trajectories[best_clear_index].command;
    result.risk = risks[best_clear_index];
    return result;
  }

  result.selected_command = trajectories[best_ttc_index].command;
  result.risk = risks[best_ttc_index];
  result.emergency_brake = result.risk.time_to_collision <= params.emergency_ttc_s ||
    result.risk.level == RiskLevel::BRAKING;

  if (result.emergency_brake) {
    result.selected_command = MotionCommand{0.0, 0.0};
    result.risk.level = RiskLevel::BRAKING;
  }

  return result;
}

}  // namespace scene_node::avoidance
