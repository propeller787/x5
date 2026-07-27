/**
 * @file tft_audio.cpp
 */

#include "tft_audio.h"
#include "tft_user_config.h"
#include <utils/topic.h>
#include <utils/warning_type.h>

namespace tft_service {

void TftAudioPublisher::init(rclcpp::Node* node) {
    if (!node) return;
    pub_ = node->create_publisher<std_msgs::msg::Int32>(audioTopic, 10);
}

void TftAudioPublisher::publishCode(int32_t code) {
    if (!pub_) return;
    if (TftUserConfig::instance().getSoundOn() == 0) return;
    std_msgs::msg::Int32 m;
    m.data = code;
    pub_->publish(m);
}

void TftAudioPublisher::playPromptTone(prompt_tone_type tone) {
    publishCode(static_cast<int32_t>(tone));
}

void TftAudioPublisher::playTtsCode(tts_code_type code) {
    publishCode(static_cast<int32_t>(code));
}

void TftAudioPublisher::onCruiseEndReason(CruiseEndAudioReason reason) {
    switch (reason) {
        case CruiseEndAudioReason::ObstacleRelated:
            playPromptTone(PROMPT_TONE_TYPE_OBSTACLE);
            break;
        case CruiseEndAudioReason::NoForwardSpeed:
            playPromptTone(PROMPT_TONE_TYPE_EXIT);
            break;
        case CruiseEndAudioReason::CruiseActivateSuccess:
            playPromptTone(PROMPT_TONE_TYPE_ACTIVATION);
            break;
        case CruiseEndAudioReason::CruiseOpenSuccess:
            playTtsCode(TTS_CODE_TYPE_CRUISE);
            break;
        default:
            break;
    }
}

void TftAudioPublisher::onRechargeObstacleBlocking() {
    playTtsCode(TTS_CODE_TYPE_REMOVE_OBSTACLES);
}

void TftAudioPublisher::onCollisionWarningLevel(CollisionAudioTrack track, int warning_level) {
    const int idx = static_cast<int>(track);
    if (idx < 0 || idx > 3) return;
    const int prev = last_collision_level_[idx];
    last_collision_level_[idx] = warning_level;
    const bool in_zone =
        (warning_level == static_cast<int>(WarningLevel::DECELERATION) ||
         warning_level == static_cast<int>(WarningLevel::BRAKE));
    const bool was_in_zone =
        (prev == static_cast<int>(WarningLevel::DECELERATION) ||
         prev == static_cast<int>(WarningLevel::BRAKE));
    if (in_zone) {
        playPromptTone(PROMPT_TONE_TYPE_OBSTACLE);
    }
}

}  // namespace tft_service
