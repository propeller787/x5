#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>

class PerceptionUtils {
public:
  static bool validRange(float range, float range_min, float range_max);
  static std::pair < double, double > polarToCartesian(double range, double angle);
  static bool inRoi(double x, double y);
  static bool validTiltZ(double z, double threshold);
  static uint8_t sourceId(const std::string & source_name);
  static std::array < uint8_t, 3 > debugColor(uint8_t source_id);
};
