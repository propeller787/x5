/**
 * @file obstacle_field.h
 * @brief 点云 → 轻量 ObstacleField（Footprint 快路径）
 *
 * 将 base_link 点云体素化（默认 12cm），每个 occupied 体素生成一个
 * ObstaclePrimitive，供 SweptFootprintChecker 做圆形近似碰撞。
 * 按距车体距离裁剪至 max_obstacle_primitives。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <tuple>
#include <vector>

class ObstacleFieldBuilder {
public:
  static ObstacleField buildFromPoints(
    const std::vector < std::tuple < double, double, double >> & points,
    const rclcpp::Time & stamp,
    const AvoidanceParams & params,
    double voxel_size_m = 0.12);

  static ObstacleField buildFromPoints(
    const std::vector < ObstaclePoint > & points,
    const rclcpp::Time & stamp,
    const AvoidanceParams & params,
    double voxel_size_m,
    size_t max_primitives);
};
