/**
 * @file wall_detector.h
 * @brief 墙线检测器 — 从聚类点云拟合左/右墙线
 *
 * 无状态计算类，所有方法均为 const，线程安全。
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <utility>
#include "clustering.h"

namespace scene_node {

/// 墙壁线段（y = kx + b，base_link 坐标系）
struct WallSegment {
    double k = 0.0;         ///< 斜率
    double b = 0.0;         ///< 截距
    double x_start = 0.0;   ///< 起始 x (m)
    double x_end   = 0.0;   ///< 终止 x (m)
    double r_squared = 0.0; ///< 拟合质量 R²
    int    point_count = 0; ///< 参与拟合的点数

    /// 在给定 x 处求 y
    double evaluate(double x) const { return k * x + b; }
};

/// 完整墙线（1 段或 2 段分段线性）
struct WallLine {
    std::vector<WallSegment> segments;
    bool   is_valid           = false;
    double overall_r_squared  = 0.0;
    double average_distance   = 0.0; ///< 平均到车体中线距离 (m)

    /// 在给定 x 处求 y（自动选择对应线段）
    double evaluate(double x) const;

    /// 有效 x 范围
    double getXStart() const;
    double getXEnd()   const;
};

/// 墙线检测参数
struct WallDetectorParams {
    double detect_range                = 3.0;  ///< 检测前方范围 (m)
    int    min_points                  = 3;    ///< 最小拟合点数
    double r_squared_threshold         = 0.7;  ///< 良好拟合阈值
    double r_squared_fallback          = 0.5;  ///< 退化阈值
    double segment_r_squared_threshold = 0.6;  ///< 分段拟合阈值
    double outlier_distance            = 0.5;  ///< 孤立点距离阈值 (m)
};

class WallDetector {
public:
    explicit WallDetector(rclcpp::Logger logger);

    /// 从聚类数据检测墙线
    /// @param clusters 某一侧聚类列表
    /// @param is_left  是否为左侧
    WallLine detectWall(
        const std::vector<ClusterInfo>& clusters,
        bool is_left) const;

    void setParams(const WallDetectorParams& params) { params_ = params; }
    WallDetectorParams getParams() const { return params_; }

private:
    /// 从聚类提取内侧边界点，按 x 排序
    std::vector<std::pair<double, double>> extractBoundaryPoints(
        const std::vector<ClusterInfo>& clusters,
        bool is_left) const;

    /// 过滤孤立点（与最近邻距离 > outlier_distance）
    std::vector<std::pair<double, double>> filterOutliers(
        const std::vector<std::pair<double, double>>& points) const;

    /// 最小二乘线性拟合，返回 WallSegment
    WallSegment fitLine(
        const std::vector<std::pair<double, double>>& points) const;

    /// 分段拟合（等分两段）
    std::vector<WallSegment> fitSegmented(
        const std::vector<std::pair<double, double>>& points) const;

    rclcpp::Logger     logger_;
    WallDetectorParams params_;
};

} // namespace scene_node
