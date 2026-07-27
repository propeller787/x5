/**
 * @file wall_detector.cpp
 * @brief 墙线检测器实现
 */
#include "wall_detector.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace scene_node {

// ─────────────────────── WallLine 辅助方法 ───────────────────────

double WallLine::evaluate(double x) const {
    if (segments.empty()) return 0.0;
    if (segments.size() == 1) return segments[0].evaluate(x);

    // 两段：根据 x 落在哪段选择
    if (x <= segments[0].x_end) {
        return segments[0].evaluate(x);
    }
    return segments[1].evaluate(x);
}

double WallLine::getXStart() const {
    if (segments.empty()) return 0.0;
    return segments.front().x_start;
}

double WallLine::getXEnd() const {
    if (segments.empty()) return 0.0;
    return segments.back().x_end;
}

// ─────────────────────── WallDetector ───────────────────────

WallDetector::WallDetector(rclcpp::Logger logger)
    : logger_(logger) {}

WallLine WallDetector::detectWall(
    const std::vector<ClusterInfo>& clusters,
    bool is_left) const
{
    WallLine result;

    // Step 1: 提取边界点
    auto raw_points = extractBoundaryPoints(clusters, is_left);
    if (static_cast<int>(raw_points.size()) < params_.min_points) {
        return result;
    }

    // Step 2: 过滤孤立点
    auto points = filterOutliers(raw_points);
    if (static_cast<int>(points.size()) < params_.min_points) {
        return result;
    }

    // Step 3: 整体拟合
    auto segment = fitLine(points);

    if (segment.r_squared >= params_.r_squared_threshold) {
        // 拟合质量良好，单段即可
        result.segments.push_back(segment);
        result.is_valid          = true;
        result.overall_r_squared = segment.r_squared;

    } else if (segment.r_squared >= params_.r_squared_fallback &&
               static_cast<int>(points.size()) >= 6) {
        // 尝试分段拟合
        auto segs = fitSegmented(points);
        bool all_valid = !segs.empty();
        for (const auto& s : segs) {
            if (s.r_squared < params_.segment_r_squared_threshold) {
                all_valid = false;
                break;
            }
        }
        if (all_valid) {
            result.segments = segs;
            result.is_valid = true;
            // 加权平均 R²
            double total_pts = 0.0, weighted_r2 = 0.0;
            for (const auto& s : segs) {
                weighted_r2 += s.r_squared * s.point_count;
                total_pts   += s.point_count;
            }
            result.overall_r_squared = (total_pts > 0) ? weighted_r2 / total_pts : 0.0;
        } else {
            // 分段失败，使用低质量整体拟合
            result.segments.push_back(segment);
            result.is_valid          = true;
            result.overall_r_squared = segment.r_squared;
        }
    } else {
        // 质量太低，不接受
        return result;
    }

    // 计算平均距离（|y| 均值）
    double sum_dist = 0.0;
    for (const auto& p : points) {
        sum_dist += std::abs(p.second);
    }
    result.average_distance = sum_dist / static_cast<double>(points.size());

    return result;
}

std::vector<std::pair<double, double>> WallDetector::extractBoundaryPoints(
    const std::vector<ClusterInfo>& clusters,
    bool is_left) const
{
    std::vector<std::pair<double, double>> points;

    for (const auto& c : clusters) {
        // 只取车前方、检测范围内的聚类
        if (c.center_x < 0.0 || c.center_x > params_.detect_range) {
            continue;
        }

        // 内侧边界：左侧取 min_y，右侧取 max_y
        double boundary_y = is_left ? c.min_y : c.max_y;
        points.emplace_back(c.center_x, boundary_y);

        // 若聚类较大，补充首尾端点提高拟合精度
        if (c.point_count > 5 && c.depth > 0.2) {
            points.emplace_back(c.min_x, boundary_y);
            points.emplace_back(c.max_x, boundary_y);
        }
    }

    // 按 x 升序排序
    std::sort(points.begin(), points.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    return points;
}

std::vector<std::pair<double, double>> WallDetector::filterOutliers(
    const std::vector<std::pair<double, double>>& points) const
{
    if (points.size() <= 2) return points;

    std::vector<std::pair<double, double>> result;
    result.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        double min_dist = std::numeric_limits<double>::max();
        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;
            double dx = points[i].first  - points[j].first;
            double dy = points[i].second - points[j].second;
            double d  = std::sqrt(dx * dx + dy * dy);
            min_dist = std::min(min_dist, d);
        }
        if (min_dist <= params_.outlier_distance) {
            result.push_back(points[i]);
        }
    }

    return result.empty() ? points : result;
}

WallSegment WallDetector::fitLine(
    const std::vector<std::pair<double, double>>& points) const
{
    WallSegment seg;
    const int n = static_cast<int>(points.size());
    if (n < 2) return seg;

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (const auto& [x, y] : points) {
        sx  += x;
        sy  += y;
        sxx += x * x;
        sxy += x * y;
    }

    double denom = n * sxx - sx * sx;
    seg.point_count = n;
    seg.x_start     = points.front().first;
    seg.x_end       = points.back().first;

    if (std::abs(denom) < 1e-10) {
        // 所有 x 相同，退化
        seg.k = 0.0;
        seg.b = sy / n;
        seg.r_squared = 0.0;
        return seg;
    }

    seg.k = (n * sxy - sx * sy) / denom;
    seg.b = (sy - seg.k * sx) / n;

    // 计算 R²
    double y_mean = sy / n;
    double ss_tot = 0.0, ss_res = 0.0;
    for (const auto& [x, y] : points) {
        ss_tot += (y - y_mean) * (y - y_mean);
        double y_pred = seg.k * x + seg.b;
        ss_res += (y - y_pred) * (y - y_pred);
    }
    seg.r_squared = (ss_tot > 1e-10) ? (1.0 - ss_res / ss_tot) : 0.0;

    return seg;
}

std::vector<WallSegment> WallDetector::fitSegmented(
    const std::vector<std::pair<double, double>>& points) const
{
    if (static_cast<int>(points.size()) < 6) return {};

    size_t mid = points.size() / 2;
    std::vector<std::pair<double, double>> first_half(points.begin(), points.begin() + mid);
    std::vector<std::pair<double, double>> second_half(points.begin() + mid, points.end());

    auto seg1 = fitLine(first_half);
    auto seg2 = fitLine(second_half);

    // 连接处连续性修正（差距 > 0.1m 时折中）
    double x_mid       = points[mid].first;
    double y1_at_mid   = seg1.k * x_mid + seg1.b;
    double y2_at_mid   = seg2.k * x_mid + seg2.b;
    double gap         = y2_at_mid - y1_at_mid;
    if (std::abs(gap) > 0.1) {
        seg2.b -= gap * 0.5;
    }

    return {seg1, seg2};
}

} // namespace scene_node
