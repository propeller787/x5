#pragma once

#include "avoidance_types.h"

namespace scene_node::avoidance {

  class SweptFootprintChecker {
public:
    CollisionRisk evaluate(
      const CandidateTrajectory & trajectory,
      const ObstacleField & field,
      const FootprintBox & footprint,
      const FootprintMargins & margins,
      const RiskParams & params) const;
  };

}  // namespace scene_node::avoidance
