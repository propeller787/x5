#pragma once

#include "avoidance_types.h"

namespace scene_node::avoidance {

  class SafetyFilter {
public:
    SafetyFilterResult select(
      const MotionCommand & desired,
      const std::vector < CandidateTrajectory > & trajectories,
      const std::vector < CollisionRisk > & risks,
      const SafetyFilterParams & params) const;
  };

}  // namespace scene_node::avoidance
