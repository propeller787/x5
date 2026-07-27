/**
 * @file path_tracker.cpp
 * @brief Pure Pursuit 路径跟踪器实现
 */
#include "path_tracker.h"

#include <algorithm>
#include <cmath>

namespace scene_node {

TrackingResult PathTracker::track(
    const CenterPath& path,
    double user_linear_x,
    double bend_index) const
{
    TrackingResult result;

    if (!path.is_valid || path.points.size() < 2) {
        return result;
    }

    // Step 1: 找最近路径点（车辆在 base_link 原点）
    int nearest = findNearestPoint(path);
    result.nearest_index = nearest;
    result.lateral_error  = path.points[nearest].y;   // 正值 = 路径在左侧
    result.heading_error  = path.points[nearest].heading;

    // Step 2: 计算前瞻距离（速度自适应 + 弯道缩短）
    double lookahead = params_.base_lookahead +
                       std::abs(user_linear_x) * params_.lookahead_gain;

    if (bend_index > 0.3) {
        double bend_factor = 1.0 - 0.3 * std::min(bend_index / 1.0, 1.0);
        lookahead *= bend_factor;
    }
    lookahead = std::clamp(lookahead, params_.min_lookahead, params_.max_lookahead);

    // Step 3: 找前瞻点
    auto [lx, ly] = findLookaheadPoint(path, nearest, lookahead);
    result.lookahead_x = lx;
    result.lookahead_y = ly;

    // Step 4: Pure Pursuit 曲率
    double curvature = computeCurvature(lx, ly);
    curvature = std::clamp(curvature, -params_.max_curvature, params_.max_curvature);

    // Step 5: 角速度 = 线速度 × 曲率（低速时抑制修正）
    if (std::abs(user_linear_x) < 0.05) {
        result.angular_correction = 0.0;
    } else {
        double angular = user_linear_x * curvature;
        result.angular_correction = std::clamp(angular, -params_.max_angular, params_.max_angular);
    }

    result.is_valid = true;
    return result;
}

int PathTracker::findNearestPoint(const CenterPath& path) const {
    int    nearest  = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path.points.size(); ++i) {
        double dx   = path.points[i].x; // vehicle at origin
        double dy   = path.points[i].y;
        double dist = dx * dx + dy * dy;
        if (dist < min_dist) {
            min_dist = dist;
            nearest  = static_cast<int>(i);
        }
    }
    return nearest;
}

std::pair<double, double> PathTracker::findLookaheadPoint(
    const CenterPath& path,
    int nearest_idx,
    double lookahead_dist) const
{
    double accumulated = 0.0;

    for (size_t i = static_cast<size_t>(nearest_idx);
         i + 1 < path.points.size(); ++i)
    {
        double dx      = path.points[i + 1].x - path.points[i].x;
        double dy      = path.points[i + 1].y - path.points[i].y;
        double seg_len = std::sqrt(dx * dx + dy * dy);
        accumulated   += seg_len;

        if (accumulated >= lookahead_dist) {
            double overshoot = accumulated - lookahead_dist;
            double ratio     = (seg_len > 1e-6) ? (1.0 - overshoot / seg_len) : 0.0;
            return {
                path.points[i].x + ratio * dx,
                path.points[i].y + ratio * dy
            };
        }
    }

    // 路径不够长，返回最后一点
    return {path.points.back().x, path.points.back().y};
}

double PathTracker::computeCurvature(double lx, double ly) const {
    double L_sq = lx * lx + ly * ly;
    if (L_sq < 1e-6) return 0.0;
    // Pure Pursuit: curvature = 2 * sin(alpha) / L = 2 * ly / L²
    return 2.0 * ly / L_sq;
}

} // namespace scene_node
