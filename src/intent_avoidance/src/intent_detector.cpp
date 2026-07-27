#include "intent_avoidance/intent_detector.h"

#include <cmath>
#include <algorithm>

namespace intent_avoidance {

IntentDetector::IntentDetector(const Config& config)
    : config_(config)
{
}

UserIntent IntentDetector::detect(const interface::msg::Joystick& msg)
{
    UserIntent intent;

    intent.rocker_x = msg.rocker_x;
    intent.rocker_y = msg.rocker_y;

    double x = static_cast<double>(msg.rocker_x);
    double y = static_cast<double>(msg.rocker_y);

    // 意图方向角：atan2(y, x)，前推x正，右拨y正
    intent.angle_rad = std::atan2(y, x);
    intent.angle_deg = intent.angle_rad * 180.0 / M_PI;

    // 意图强度
    intent.strength = std::hypot(x, y) / 100.0;
    intent.strength = std::clamp(intent.strength, 0.0, 1.0);

    // 死区判定
    int16_t dz = config_.joystick_deadzone;
    intent.has_forward_intent = (msg.rocker_x > dz);
    intent.has_steering_intent = (std::abs(msg.rocker_y) > dz);
    intent.is_neutral = (std::abs(msg.rocker_x) <= dz) && (std::abs(msg.rocker_y) <= dz);

    return intent;
}

}  // namespace intent_avoidance
