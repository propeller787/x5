#include "wheelchair_obstacle/rebuild_obstacle_layer.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace wheelchair_obstacle
{

void RebuildObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // 1) 清空本层内部栅格：抹掉上一周期累积的所有标记（方案 B 的核心动作）。
  //    对 6x6m@0.05 的窗口约 1.4 万格，是一次 memset，耗时可忽略。
  resetMaps();

  // 2) 用当前帧点云重新标记 + 射线清除，完全沿用 ObstacleLayer 既有逻辑。
  ObstacleLayer::updateBounds(
    robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  // 3) 把整个滚动窗口纳入本次更新边界。父类只把观测点附近的小范围写回 master
  //    代价地图；若不扩到全窗口，被我们清空（现为 free）的旧格子就无法覆盖 master
  //    中残留的 lethal 标记，方案 B 的“整图重建”便不生效。LayeredCostmap 会在
  //    updateCosts 前按此边界把 master 复位为默认值，从而实现真正的整图重建。
  *min_x = std::min(*min_x, origin_x_);
  *min_y = std::min(*min_y, origin_y_);
  *max_x = std::max(*max_x, origin_x_ + size_x_ * resolution_);
  *max_y = std::max(*max_y, origin_y_ + size_y_ * resolution_);
}

}  // namespace wheelchair_obstacle

PLUGINLIB_EXPORT_CLASS(wheelchair_obstacle::RebuildObstacleLayer, nav2_costmap_2d::Layer)
