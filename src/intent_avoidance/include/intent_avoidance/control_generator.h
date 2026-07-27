#pragma once

#include "intent_avoidance/types.h"

#include <geometry_msgs/msg/twist.hpp>

namespace intent_avoidance {

// 决策 → Twist输出 + 平滑滤波
class ControlGenerator {
public:
    struct Config {
        double max_angular_guidance = 0.5;    // rad/s
        double max_linear_accel = 0.8;        // m/s²
        double output_smooth_alpha = 0.3;     // 低通系数 (0,1]
        double speed_recovery_time = 1.5;     // 速度恢复时间(s)
        double cycle_time = 0.1;              // 控制周期(s)，默认10Hz
    };

    explicit ControlGenerator(const Config& config);

    // 生成输出Twist，含平滑和限幅
    geometry_msgs::msg::Twist generate(
        const AvoidanceDecision& decision,
        const UserIntent& user_intent,
        const SpeedInfo& actual_speed);

    // 重置平滑状态（模式切换时调用）
    void reset();

private:
    Config config_;
    double prev_output_vx_ = 0.0;
    double prev_output_wz_ = 0.0;
    double recovery_ramp_ = 0.0;  // 恢复阶段速度增量
};

}  // namespace intent_avoidance
