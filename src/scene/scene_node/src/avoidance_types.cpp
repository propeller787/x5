#include "avoidance_types.h"

namespace scene_node::avoidance
{

const char * riskLevelName(RiskLevel level)
{
  switch (level) {
    case RiskLevel::CLEAR:
      return "CLEAR";
    case RiskLevel::WARNING:
      return "WARNING";
    case RiskLevel::LIMITING:
      return "LIMITING";
    case RiskLevel::BRAKING:
      return "BRAKING";
  }
  return "UNKNOWN";
}

}  // namespace scene_node::avoidance
