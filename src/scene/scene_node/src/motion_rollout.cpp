#include "motion_rollout.h"

#include <algorithm>
#include <cmath>

namespace scene_node::avoidance
{
namespace
{

bool nearlyEqual(double a, double b)
{
  return std::abs(a - b) < 1e-6;
}

bool containsCommand(const std::vector<MotionCommand> & commands, const MotionCommand & command)
{
  return std::any_of(
    commands.begin(), commands.end(), [&](const MotionCommand & existing) {
      return nearlyEqual(existing.linear_x, command.linear_x) &&
      nearlyEqual(existing.angular_z, command.angular_z);
    });
}

void addUnique(std::vector<MotionCommand> & commands, const MotionCommand & command)
{
  if (!containsCommand(commands, command)) {
    commands.push_back(command);
  }
}

}  // namespace

std::vector<CandidateTrajectory> MotionRollout::generate(
  const MotionCommand & command,
  const RolloutParams & params) const
{
  std::vector<MotionCommand> commands;
  commands.reserve(static_cast<std::size_t>(params.speed_samples + params.angular_samples + 2));

  addUnique(commands, command);

  const bool has_linear = std::abs(command.linear_x) > params.linear_deadband;
  const bool has_angular = std::abs(command.angular_z) > params.angular_deadband;

  if (has_linear) {
    const int samples = std::max(2, params.speed_samples);
    for (int i = 1; i < samples; ++i) {
      const double ratio = 1.0 - static_cast<double>(i) / static_cast<double>(samples - 1);
      addUnique(commands, MotionCommand{command.linear_x * ratio, command.angular_z});
    }
  }

  if (has_angular) {
    const int samples = std::max(2, params.angular_samples);
    for (int i = 0; i < samples; ++i) {
      const double ratio = 1.0 - static_cast<double>(i) / static_cast<double>(samples - 1);
      addUnique(commands, MotionCommand{0.0, command.angular_z * ratio});
    }
  }

  addUnique(commands, MotionCommand{0.0, 0.0});

  std::vector<CandidateTrajectory> trajectories;
  trajectories.reserve(commands.size());
  for (const auto & candidate : commands) {
    trajectories.push_back(buildTrajectory(candidate, params));
  }
  return trajectories;
}

CandidateTrajectory MotionRollout::buildTrajectory(
  const MotionCommand & command,
  const RolloutParams & params) const
{
  CandidateTrajectory trajectory;
  trajectory.command = command;

  const double horizon = std::max(0.0, params.horizon_s);
  const double dt = std::max(0.02, params.dt_s);
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon / dt)));
  trajectory.poses.reserve(static_cast<std::size_t>(steps + 1));

  const double v = command.linear_x;
  const double w = command.angular_z;
  constexpr double kAngularEps = 1e-5;

  for (int i = 0; i <= steps; ++i) {
    const double t = std::min(horizon, static_cast<double>(i) * dt);
    PredictedPose pose;
    pose.t = t;

    if (std::abs(w) < kAngularEps) {
      pose.x = v * t;
      pose.y = 0.0;
      pose.yaw = 0.0;
    } else {
      const double yaw = w * t;
      const double radius = v / w;
      pose.x = radius * std::sin(yaw);
      pose.y = radius * (1.0 - std::cos(yaw));
      pose.yaw = yaw;
    }

    trajectory.poses.push_back(pose);
  }

  return trajectory;
}

}  // namespace scene_node::avoidance
