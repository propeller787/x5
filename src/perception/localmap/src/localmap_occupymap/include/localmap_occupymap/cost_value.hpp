// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALMAP_OCCUPYMAP__COST_VALUE_HPP_
#define LOCALMAP_OCCUPYMAP__COST_VALUE_HPP_

#include <cstdint>

namespace localmap_occupymap
{

// Nav2 costmap_2d cost constants (nav2_costmap_2d/cost_values.hpp).
inline constexpr std::uint8_t kCostFree = 0;
inline constexpr std::uint8_t kCostInscribed = 253;
inline constexpr std::uint8_t kCostLethal = 254;
inline constexpr std::uint8_t kCostUnknown = 255;

// Nav2 Costmap2DPublisher cost_translation_table_ (0-255 -> -1..100).
inline int8_t costToOccupancyValue(std::uint8_t cost)
{
  if (cost == kCostFree) {
    return 0;
  }
  if (cost == kCostInscribed) {
    return 99;
  }
  if (cost == kCostLethal) {
    return 100;
  }
  if (cost == kCostUnknown) {
    return -1;
  }
  return static_cast<int8_t>(1 + (97 * (cost - 1)) / 251);
}

}  // namespace localmap_occupymap

#endif  // LOCALMAP_OCCUPYMAP__COST_VALUE_HPP_
