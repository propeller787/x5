#pragma once

#include "avoidance_types.h"
#include "motion_rollout.h"
#include "obstacle_field.h"
#include "safety_filter.h"
#include "swept_footprint_checker.h"
#include <map>

namespace scene_node::avoidance {

  class RiskAssessment {
public:
    SafetyFilterResult evaluate(
      const MotionCommand & desired,
      const VehicleFeedback & feedback,
      const std::map < std::string, std::vector < ClusterInfo >> & radar_clusters,
      const std::map < std::string, rclcpp::Time > & radar_times,
      const rclcpp::Time & now,
      const RiskAssessmentParams & params) const;

private:
    ObstacleFieldBuilder obstacle_builder_;
    MotionRollout motion_rollout_;
    SweptFootprintChecker swept_checker_;
    SafetyFilter safety_filter_;
  };

}  // namespace scene_node::avoidance
