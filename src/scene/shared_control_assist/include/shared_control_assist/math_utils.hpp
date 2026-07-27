#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/time.hpp>

#include "shared_control_assist/types.hpp"

namespace shared_control_assist::math {

// 常用数学常量。
// kInf 用于表示“当前方向没有观测到障碍”，后续安全裁剪会用 isfinite()
// 判断是否真的需要限速。
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// 将 value 限制在 [low, high] 范围内。
// 用途：
// - 限制线速度/角速度最大值。
// - 限制置信度、比例系数到 [0, 1]。
// 注意：调用者应保证 low <= high。
inline double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

// 对称限幅：将 value 限制到 [-max_abs, max_abs]。
// 适用于角速度这类正负方向都允许、但绝对值不能过大的控制量。
// max_abs 即使传入负数，也会取 abs 后使用。
inline double clampAbs(double value, double max_abs)
{
    return clamp(value, -std::abs(max_abs), std::abs(max_abs));
}

// 角度归一化到 [-pi, pi]。
// 后续如果 DoorDetector 增加真实门框/墙线拟合，门方向误差可以用它规整。
// 当前 MVP 的门洞 yaw 暂时为 0，但保留该工具便于扩展。
inline double normalizeAngle(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

// 一阶斜率限制。
// target   : 本周期想输出的目标值。
// previous : 上周期实际输出值。
// max_delta: 本周期允许变化的最大幅度。
//
// 用途：
// - 对 /nav/control 的 linear.x 和 angular.z 做加速度限制。
// - 避免摇杆或安全裁剪突变导致轮椅顿挫。
// 注意：急停时不应使用该函数慢慢降到 0，而应直接输出 0。
inline double rateLimit(double target, double previous, double max_delta)
{
    const double delta = target - previous;
    if (std::abs(delta) <= max_delta) {
        return target;
    }
    return previous + std::copysign(max_delta, delta);
}

// 判断两个标量是否在 tolerance 内足够接近。
// 用途：
// - DoorDetector 判断连续帧候选是否稳定。
// - 宽度、中心偏移、角度变化小于阈值时才增加 stable_count。
inline bool closeEnough(double a, double b, double tolerance)
{
    return std::abs(a - b) <= tolerance;
}

// 若 value 是有限数，则返回 value；否则返回 fallback。
// 用途：
// - 调试 JSON 中不直接输出 infinity。
// - 控制器中遇到无观测间隙时给一个保守/中性默认值。
inline double finiteOr(double value, double fallback)
{
    return std::isfinite(value) ? value : fallback;
}

// 由车辆参数计算车体半宽。
// 所有左右距离、门洞间隙、安全区宽度都以该值为基础。
inline double halfWidth(const AssistParams& params)
{
    return 0.5 * params.vehicle_width;
}

// 判断 base_frame 下的点是否落在车体 footprint 内。
// 这些点通常来自雷达扫到车体自身、脚踏或固定结构，应该从障碍点里剔除。
//
// 判断区域：
// - x ∈ [rear_edge - padding, front_edge + padding]
// - |y| <= half_width + padding
//
// 当前使用矩形 footprint。后续如需更准确，可替换为多边形点内判断。
inline bool isInsideFootprint(const AssistParams& params, double x, double y)
{
    return x >= params.rear_edge - params.footprint_padding &&
        x <= params.front_edge + params.footprint_padding &&
        std::abs(y) <= halfWidth(params) + params.footprint_padding;
}

// 距离限速曲线。
//
// 输入：
// - distance      : 当前方向最近障碍距离。
// - stop_distance : 小于等于该距离时输出比例为 0。
// - slow_distance : 大于等于该距离时输出比例为 1。
//
// 输出：
// - [0, 1] 的速度缩放比例。
//
// 曲线含义：
// - distance <= stop_distance  : 停车。
// - distance >= slow_distance  : 不限速。
// - 中间区间                 : 线性降速。
//
// 这比硬切换更平滑，适合低速载人设备。
inline double distanceLimitScale(double distance, double stop_distance, double slow_distance)
{
    if (!std::isfinite(distance)) {
        return 1.0;
    }
    if (distance <= stop_distance) {
        return 0.0;
    }
    if (distance >= slow_distance) {
        return 1.0;
    }
    return clamp((distance - stop_distance) / std::max(0.01, slow_distance - stop_distance), 0.0, 1.0);
}

// 计算时间戳距离当前时刻的年龄，单位 second。
// 若 stamp 为空时间戳，返回 infinity，等价于“无限久未更新”。
//
// 用途：
// - 判断摇杆命令是否超时。
// - 判断雷达帧是否新鲜。
// - 判断门候选短时保持是否过期。
inline double timeAgeSeconds(const rclcpp::Time& now, const rclcpp::Time& stamp)
{
    if (stamp.nanoseconds() == 0) {
        return kInf;
    }
    const double age = (now - stamp).seconds();
    return std::max(0.0, age);
}

}  // namespace shared_control_assist::math
