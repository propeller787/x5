#pragma once

#include "avoidance_types.h"

namespace scene_node::avoidance {

  class MotionRollout {
public:
    std::vector < CandidateTrajectory > generate(
      const MotionCommand & command,
      const RolloutParams & params) const;

private:
    CandidateTrajectory buildTrajectory(
      const MotionCommand & command,
      const RolloutParams & params) const;
  };

}  // namespace scene_node::avoidance
