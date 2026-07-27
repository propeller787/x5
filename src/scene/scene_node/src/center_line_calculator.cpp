/**
 * @file center_line_calculator.cpp
 * @brief 中心线计算器实现
 */
#include "center_line_calculator.h"

#include <algorithm>
#include <cmath>

namespace scene_node {

CenterPath CenterLineCalculator::calculate(
    const WallLine& left_wall,
    const WallLine& right_wall) const
{
    CenterPath path;

    if (!left_wall.is_valid || !right_wall.is_valid) {
        return path;
    }

    // 确定两侧重叠的 x 范围
    double x_start = std::max(left_wall.getXStart(), right_wall.getXStart());
    double x_end   = std::min(left_wall.getXEnd(),   right_wall.getXEnd());

    if (x_end - x_start < params_.min_path_length) {
        return path; // 路径太短
    }

    int sample_count = std::max(2, params_.sample_count);
    double step = (x_end - x_start) / (sample_count - 1);
    path.min_width = 1e6;
    path.max_width = 0.0;

    for (int i = 0; i < sample_count; ++i) {
        double x       = x_start + i * step;
        double y_left  = left_wall.evaluate(x);
        double y_right = right_wall.evaluate(x);

        CenterPoint cp;
        cp.x     = x;
        cp.y     = (y_left + y_right) / 2.0;
        cp.width = std::abs(y_left - y_right);

        // 通道宽度安全检查
        double clearance = cp.width - params_.robot_width;
        if (clearance < params_.min_clearance * 2.0) {
            // 该处不可通行，截断路径
            if (path.points.empty()) {
                return path; // 起点就不可通行
            }
            break;
        }

        path.min_width = std::min(path.min_width, cp.width);
        path.max_width = std::max(path.max_width, cp.width);
        path.points.push_back(cp);
    }

    if (path.points.empty()) {
        return path;
    }

    // 计算各点朝向
    for (size_t i = 0; i < path.points.size(); ++i) {
        if (i + 1 < path.points.size()) {
            double dx = path.points[i + 1].x - path.points[i].x;
            double dy = path.points[i + 1].y - path.points[i].y;
            path.points[i].heading = std::atan2(dy, dx);
        } else {
            path.points[i].heading = (i > 0) ? path.points[i - 1].heading : 0.0;
        }
    }

    // 计算路径总长度
    path.length = 0.0;
    for (size_t i = 1; i < path.points.size(); ++i) {
        double dx = path.points[i].x - path.points[i - 1].x;
        double dy = path.points[i].y - path.points[i - 1].y;
        path.length += std::sqrt(dx * dx + dy * dy);
    }

    path.bend_index = calculateBendIndex(left_wall, right_wall);
    path.is_valid   = path.points.size() >= 3;

    return path;
}

double CenterLineCalculator::calculateBendIndex(
    const WallLine& left_wall,
    const WallLine& right_wall) const
{
    if (left_wall.segments.empty() || right_wall.segments.empty()) {
        return 0.0;
    }
    double kl = left_wall.segments.front().k;
    double kr = right_wall.segments.front().k;
    return std::abs(kl - kr) + std::abs(kl) + std::abs(kr);
}

} // namespace scene_node
