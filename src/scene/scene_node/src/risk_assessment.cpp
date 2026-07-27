#include "risk_assessment.h"

namespace scene_node::avoidance
{

SafetyFilterResult RiskAssessment::evaluate(
  const MotionCommand & desired,
  const VehicleFeedback & /*feedback*/,
  const std::map<std::string, std::vector<ClusterInfo>> & radar_clusters,
  const std::map<std::string, rclcpp::Time> & radar_times,
  const rclcpp::Time & now,
  const RiskAssessmentParams & params) const
{
  const ObstacleField field = obstacle_builder_.buildFromClusters(
    radar_clusters, radar_times, now, params.obstacle);

  const auto trajectories = motion_rollout_.generate(desired, params.rollout);

  std::vector<CollisionRisk> risks;
  risks.reserve(trajectories.size());
  for (const auto & trajectory : trajectories) {
    risks.push_back(
      swept_checker_.evaluate(
        trajectory, field, params.footprint, params.margins, params.risk));
  }

  auto result = safety_filter_.select(desired, trajectories, risks, params.filter);
  result.obstacle_count = field.obstacles.size();
  result.trajectory_count = trajectories.size();
  return result;
}

}  // namespace scene_node::avoidance
