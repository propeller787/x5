#pragma once

#include "intent_avoidance/types.h"

#include <interface/msg/joystick.hpp>

namespace intent_avoidance {

// 摇杆 → 用户意图
class IntentDetector {
public:
    struct Config {
        int16_t joystick_deadzone = 15;  // 摇杆死区阈值
    };

    explicit IntentDetector(const Config& config);

    // 从 Joystick 消息解析用户意图
    UserIntent detect(const interface::msg::Joystick& msg);

private:
    Config config_;
};

}  // namespace intent_avoidance
