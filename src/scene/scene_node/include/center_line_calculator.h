/**
 * @file center_line_calculator.h
 * @brief 中心线计算器 — 从两条墙线计算通道中心路径
 *
 * 无状态计算类，所有方法均为 const，线程安全。
 */
#pragma once

#include <vector>
#include "wall_detector.h"

namespace scene_node {

/// 中心线路径点
struct CenterPoint {
    double x       = 0.0; ///< base_link x (m)
    double y       = 0.0; ///< 中心线 y (m)
    double width   = 0.0; ///< 该处通道宽度 (m)
    double heading = 0.0; ///< 中心线朝向 (rad)
};

/// 中心线路径
struct CenterPath {
    std::vector<CenterPoint> points;
    bool   is_valid    = false;
    double min_width   = 0.0;
    double max_width   = 0.0;
    double length      = 0.0;
    double bend_index  = 0.0; ///< 弯道指标，越大越弯
};

/// 中心线计算参数
struct CenterLineParams {
    int    sample_count    = 20;    ///< 采样点数
    double min_path_length = 0.3;   ///< 最小路径长度 (m)
    double robot_width     = 0.70;  ///< 机器人宽度 (m)
    double min_clearance   = 0.05;  ///< 最小单侧通行间隙 (m)
};

class CenterLineCalculator {
public:
    /// 从两条墙线计算中心线路径
    CenterPath calculate(
        const WallLine& left_wall,
        const WallLine& right_wall) const;

    /// 计算弯道指标（越大越弯）
    double calculateBendIndex(
        const WallLine& left_wall,
        const WallLine& right_wall) const;

    void setParams(const CenterLineParams& params) { params_ = params; }
    CenterLineParams getParams() const { return params_; }

private:
    CenterLineParams params_;
};

} // namespace scene_node
