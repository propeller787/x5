#ifndef WHEELCHAIR_OBSTACLE__REBUILD_OBSTACLE_LAYER_HPP_
#define WHEELCHAIR_OBSTACLE__REBUILD_OBSTACLE_LAYER_HPP_

#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace wheelchair_obstacle
{

// ObstacleLayer 的“每周期整图重建”变体（方案 B）。
//
// 原生 ObstacleLayer 会跨周期累积标记，旧障碍只能靠“传感器原点->观测点”的射线清除，
// 因此 ROI 之外或无返回的方位一旦标记就残留（障碍离开后不刷新）。本层在每个更新周期
// 先把本层内部栅格整体清空，再用当前帧点云重建，从而保证障碍物离开后立即消失。
//
// 代价：丢失时间记忆，单帧漏检会导致障碍瞬时闪烁——这是方案 B 的固有取舍。
class RebuildObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
};

}  // namespace wheelchair_obstacle

#endif  // WHEELCHAIR_OBSTACLE__REBUILD_OBSTACLE_LAYER_HPP_
