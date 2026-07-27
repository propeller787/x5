#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>

class PerceptionUtils {
public:
  static bool validRange(float range, float range_min, float range_max);
  static std::pair < double, double > polarToCartesian(double range, double angle);
  /** ROI 闸口：检查点是否在感知感兴趣区域内。
   *  边界参数由调用方传入（从 ROS 参数加载），替代旧版硬编码 [-2.5, 5.0] × [-1.8, 1.8]。 */
  static bool inRoi(
    double x, double y,
    double x_min, double x_max,
    double y_min, double y_max);
  static bool validTiltZ(double z, double threshold);
  static uint8_t sourceId(const std::string & source_name);
  static std::array < uint8_t, 3 > debugColor(uint8_t source_id);
};
