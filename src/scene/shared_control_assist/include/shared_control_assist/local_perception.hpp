#pragma once

#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

#include "shared_control_assist/types.hpp"

namespace shared_control_assist {

// 局部感知模块。
//
// 输入：
// - 已经转换到 base_frame 下的二维障碍点 points。
// - points 通常由三路 LaserScan 经过 TF 合并得到。
//
// 输出：
// - ObstacleDistance:
//   用于最终安全裁剪，包括前/后/左/右最近距离、四角距离、停车标志。
// - nav_msgs::msg::OccupancyGrid:
//   用于 RViz 可视化和现场调试，当前不参与控制决策。
//
// 设计边界：
// - 这是局部低速辅助，不做全局地图、不做 SLAM、不做路径规划。
// - 当前栅格没有时间衰减缓存；每次调用都基于本周期新鲜点重建。
// - 车体 footprint 过滤采用矩形近似，后续可升级为多边形 footprint。
class LocalPerception {
public:
    explicit LocalPerception(const AssistParams& params);

    // 计算各方向障碍距离和停车标志。
    //
    // 典型用途：
    // - v > 0 时检查 front_min/front_stop。
    // - v < 0 时检查 rear_min/rear_stop。
    // - omega > 0 左转时检查 left_min/front_left_min。
    // - omega < 0 右转时检查 right_min/front_right_min。
    //
    // 返回值中的距离若为 infinity，表示该方向未观测到有效障碍。
    ObstacleDistance computeObstacleDistance(const std::vector<Point2D>& points) const;

    // 构建 base_frame 下的局部占据栅格。
    //
    // 栅格范围和分辨率来自 AssistParams:
    // - grid_x_min/grid_x_max
    // - grid_y_min/grid_y_max
    // - grid_resolution
    //
    // 栅格值：
    // - 0   : 当前未占据。
    // - 100 : 障碍或膨胀后的障碍。
    //
    // 注意：
    // - 未使用 -1 表示 unknown，因为该栅格主要用于局部调试显示。
    // - obstacle inflation 只影响栅格显示，不直接修改安全距离计算。
    nav_msgs::msg::OccupancyGrid buildLocalGrid(
        const std::vector<Point2D>& points,
        const rclcpp::Time& stamp) const;

private:
    // 持有参数引用，避免复制大量配置；调用方需保证 params 生命周期
    // 长于 LocalPerception。
    const AssistParams& params_;
};

}  // namespace shared_control_assist
