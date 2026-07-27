/**
 * @file tft_audio.h
 * @brief 向 /vlago/audio/play 发布音频指令（与 control/define.h 中 tts_code_type、prompt_tone_type 一致）。
 */

#pragma once

#include "control/define.h"
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace tft_service {

/** scene_node 发布到 /vlago/cruise/status 的结束原因 */
enum class CruiseEndAudioReason : int32_t {
    NoForwardSpeed = 1,
    ObstacleRelated = 2,
    CruiseActivateSuccess = 3,
    CruiseOpenSuccess = 4,
};

/** 与 CollisionWarning 四向回调对应的声道下标 */
enum class CollisionAudioTrack : int8_t {
    Forward = 0,
    Backward = 1,
    Left = 2,
    Right = 3,
};

/** CarInfo.getcharging / 充电状态回调：0 未充 1 线充 2 基站充 */
enum class ChargingConnectStatus : int32_t {
    NotCharging = 0,
    LineCharging = 1,
    BaseStationCharging = 2,
};

/** RechargeStatus.status：与需求「障碍物阻挡」播报对应 */
enum class RechargeStatusForAudio : int16_t {
    ObstacleBlocking = 4,
};

class TftAudioPublisher {
public:
    void init(rclcpp::Node* node);

    void playPromptTone(prompt_tone_type tone);

    /** TTS 播报，码值见 tts_code_type（0x80+） */
    void playTtsCode(tts_code_type code);

    void onCruiseEndReason(CruiseEndAudioReason reason);

    void onRechargeObstacleBlocking();

    void onCollisionWarningLevel(CollisionAudioTrack track, int warning_level);

private:
    void publishCode(int32_t code);

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    int last_collision_level_[4]{};
};

}  // namespace tft_service
