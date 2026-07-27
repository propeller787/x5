#include "intent_avoidance/control_generator.h"

#include <algorithm>
#include <cmath>

namespace intent_avoidance {

ControlGenerator::ControlGenerator(const Config& config)
    : config_(config)
{
}

geometry_msgs::msg::Twist ControlGenerator::generate(
    const AvoidanceDecision& decision,
    const UserIntent& /*user_intent*/,
    const SpeedInfo& /*actual_speed*/)
{
    geometry_msgs::msg::Twist output;

    double target_vx = decision.target_linear_x;
    double target_wz = decision.target_angular_z;

    // 限幅
    target_vx = std::max(target_vx, 0.0);  // 不后退
    target_wz = std::clamp(target_wz, -config_.max_angular_guidance, config_.max_angular_guidance);

    // 加速度限制 (线速度)
    double dt = config_.cycle_time;
    double max_delta = config_.max_linear_accel * dt;
    double vx_delta = target_vx - prev_output_vx_;
    vx_delta = std::clamp(vx_delta, -max_delta, max_delta);
    double raw_vx = prev_output_vx_ + vx_delta;

    // 强制刹车时的减速：允许更快的减速
    if (decision.braking) {
        // 紧急减速：最大减速度
        double brake_delta = config_.max_linear_accel * 2.0 * dt;
        raw_vx = prev_output_vx_ - brake_delta;
        raw_vx = std::max(raw_vx, 0.0);
    }

    // 渐停保护：目标为0且实际速度很小 → 直接置零
    if (target_vx < 0.001 && raw_vx < 0.02) {
        raw_vx = 0.0;
    }

    // 一阶低通滤波
    double alpha = config_.output_smooth_alpha;
    double smooth_vx = alpha * raw_vx + (1.0 - alpha) * prev_output_vx_;
    double smooth_wz = alpha * target_wz + (1.0 - alpha) * prev_output_wz_;

    // 更新状态
    prev_output_vx_ = smooth_vx;
    prev_output_wz_ = smooth_wz;

    output.linear.x = smooth_vx;
    output.linear.y = 0.0;
    output.linear.z = 0.0;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = smooth_wz;

    return output;
}

void ControlGenerator::reset()
{
    prev_output_vx_ = 0.0;
    prev_output_wz_ = 0.0;
    recovery_ramp_ = 0.0;
}

}  // namespace intent_avoidance
