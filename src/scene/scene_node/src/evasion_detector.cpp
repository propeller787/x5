/**
 * @file evasion_detector.cpp
 * @author likang (likang@freego.com)
 * @brief 操作者避让意图检测器实现
 * @version 0.1
 * @date 2026-04-06
 * @copyright Copyright (c) 2026 Freego Technology Co., Ltd.
 */

#include "evasion_detector.h"
#include <cmath>
#include <algorithm>

EvasionDetector::EvasionDetector(rclcpp::Logger logger)
    : logger_(logger)
{
}

void EvasionDetector::updateParams(const EvasionDetectorParams& params)
{
    params_ = params;
    RCLCPP_INFO(logger_, "[EvasionDetector] 参数更新: enable=%d, angular_threshold=%.3f, "
                "angular_full=%.3f, min_factor=%.2f, safe_dist=%.3f",
                params_.enable, params_.angular_threshold,
                params_.angular_full_evasion, params_.min_factor,
                params_.safe_distance_override);
}

double EvasionDetector::computeEvasionFactor(double feedback_angular_z,
                                              double feedback_linear_x,
                                              double forward_distance) const
{
    // 关闭时不衰减
    if (!params_.enable) {
        return 1.0;
    }

    // 轮速反馈无前进运动，不衰减（与原先摇杆「无前进意图」语义对应）
    if (feedback_linear_x <= 0.0) {
        return 1.0;
    }

    // 固定停车距离内不衰减（安全底线）
    if (forward_distance <= params_.safe_distance_override) {
        return 1.0;
    }

    const double abs_angular = std::abs(feedback_angular_z);

    // 角速度未达到避让阈值，不衰减
    if (abs_angular < params_.angular_threshold) {
        return 1.0;
    }

    // 防止除零
    const double range = params_.angular_full_evasion - params_.angular_threshold;
    if (range <= 1e-6) {
        return params_.min_factor;
    }

    // 线性插值: angular_threshold → 1.0, angular_full_evasion → min_factor
    const double t = std::clamp((abs_angular - params_.angular_threshold) / range, 0.0, 1.0);
    const double factor = 1.0 - t * (1.0 - params_.min_factor);

    RCLCPP_DEBUG(logger_, "[EvasionDetector] omega_fb=%.3f, v_fb=%.3f, dist=%.3f, factor=%.3f",
                 feedback_angular_z, feedback_linear_x, forward_distance, factor);

    return factor;
}

double EvasionDetector::applyEvasionRelax(double original_limit, double max_speed_limit, double factor) const
{
    if (!params_.enable || factor >= 1.0) {
        return original_limit;
    }

    // factor=1.0 → 保持 original_limit（全干预）
    // factor=min_factor → original_limit + (1-min_factor)*(max-original)（大幅放松）
    const double relaxed = original_limit + (1.0 - factor) * (max_speed_limit - original_limit);
    return std::min(relaxed, max_speed_limit);
}
