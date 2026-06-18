#include "safety_node/perception_utils.h"

#include <cmath>

bool PerceptionUtils::validRange(float range, float range_min, float range_max)
{
  return std::isfinite(range) && range >= range_min && range <= range_max;
}

std::pair<double, double> PerceptionUtils::polarToCartesian(double range, double angle)
{
  return {range * std::cos(angle), range * std::sin(angle)};
}

bool PerceptionUtils::inRoi(double x, double y)
{
  return x >= -2.5 && x <= 5.0 && y >= -1.8 && y <= 1.8;
}

bool PerceptionUtils::validTiltZ(double z, double threshold)
{
  return std::isfinite(z) && (threshold <= 0.0 || std::abs(z) <= threshold);
}

uint8_t PerceptionUtils::sourceId(const std::string & source_name)
{
  if (source_name == "right") {
    return 1;
  }
  if (source_name == "back") {
    return 2;
  }
  if (source_name == "left_tilt") {
    return 3;
  }
  if (source_name == "right_tilt") {
    return 4;
  }
  return 0;
}

std::array<uint8_t, 3> PerceptionUtils::debugColor(uint8_t source_id)
{
  switch (source_id) {
    case 1:
      return {80, 255, 80};
    case 2:
      return {255, 160, 40};
    case 3:
      return {80, 160, 255};
    case 4:
      return {255, 80, 220};
    case 0:
    default:
      return {40, 240, 240};
  }
}
