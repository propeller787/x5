/**
 * @file path_tracker.h
 * @brief 路径跟踪器 — Pure Pursuit 算法
 *
 * 无状态计算类，所有方法均为 const，线程安全。
 */
#pragma once

#include <utility>
#include "center_line_calculator.h"

namespace scene_node {

/// 路径跟踪参数
struct PathTrackerParams {
    double base_lookahead  = 0.3;  ///< 基础前瞻距离 (m)
    double lookahead_gain  = 0.5;  ///< 速度比例前瞻增益 (s)
    double min_lookahead   = 0.2;  ///< 最小前瞻距离 (m)
    double max_lookahead   = 1.5;  ///< 最大前瞻距离 (m)
    double max_angular     = 0.4;  ///< 最大角速度修正 (rad/s)
    double max_curvature   = 2.0;  ///< 最大曲率 (m⁻¹)
};

/// 路径跟踪输出
struct TrackingResult {
    double angular_correction = 0.0; ///< 角速度修正 (rad/s)
    double lateral_error      = 0.0; ///< 横向偏差 (m)
    double heading_error      = 0.0; ///< 航向偏差 (rad)
    double lookahead_x        = 0.0; ///< 前瞻点 x (m)
    double lookahead_y        = 0.0; ///< 前瞻点 y (m)
    int    nearest_index      = 0;   ///< 最近路径点索引
    bool   is_valid           = false;
};

class PathTracker {
public:
    /// Pure Pursuit 路径跟踪
    /// @param path          中心线路径
    /// @param user_linear_x 用户前进速度 (m/s)
    /// @param bend_index    弯道指标（用于自适应前瞻）
    TrackingResult track(
        const CenterPath& path,
        double user_linear_x,
        double bend_index = 0.0) const;

    void setParams(const PathTrackerParams& params) { params_ = params; }
    PathTrackerParams getParams() const { return params_; }

private:
    /// 查找距车辆（原点）最近的路径点索引
    int findNearestPoint(const CenterPath& path) const;

    /// 沿路径搜索前瞻点，返回 {lx, ly}
    std::pair<double, double> findLookaheadPoint(
        const CenterPath& path,
        int nearest_idx,
        double lookahead_dist) const;

    /// Pure Pursuit 曲率 = 2 * ly / L²
    double computeCurvature(double lx, double ly) const;

    PathTrackerParams params_;
};

} // namespace scene_node
