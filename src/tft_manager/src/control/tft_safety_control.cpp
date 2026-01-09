#pragma once
// 安全预警
#include "tft_safety_control.h"

using namespace tft_service::control;

SafetyControl::SafetyControl() : Control("SafetyControl") {
    sub_auxiliary_alarm_ =
        global_->node_->create_subscription<interface::msg::AuxiliaryAlarm>(auxiliaryAlarmTopic, 10, std::bind(&SafetyControl::handleAuxiliaryAlarm, this, _1));
    sub_foot_alarm_ = global_->node_->create_subscription<std_msgs::msg::Int32>(footLimiterTopic, 10, std::bind(&SafetyControl::handleFootAlarm, this, _1));
}
SafetyControl::~SafetyControl() {
    // Destructor implementation
}

void SafetyControl::handleAuxiliaryAlarm(const interface::msg::AuxiliaryAlarm msg) {
    if (msg.alarm_level == 3) {
        // 抑制：3 秒内重复预警忽略
        const auto now = std::chrono::steady_clock::now();
        if (last_aux_alarm_time_.time_since_epoch().count() != 0) {
            const auto since_last = now - last_aux_alarm_time_;
            if (since_last < std::chrono::seconds(3)) {
                return;
            }
        }
        last_aux_alarm_time_ = now;
        
        // global_->setType(tft_service::TFT_TYPE::OBSTACLE_AVOIDANCE);
        //  发送音频   发送AUDIO_TYPE_OBSTACLE_AVOIDANCE_21

        std_msgs::msg::Int32 audio_msg;
        audio_msg.data = AUDIO_TYPE_OBSTACLE_AVOIDANCE_21;
        audio_pub_->publish(audio_msg);
        //  发送预警界面
        global_->sendPageId(OBSTACLE_PASSAGE_PAGE_ID);
        global_->sendPageHeader();
        // 注意：sleep(1) 已移除，如果需要延迟恢复标准模式，应使用异步定时器
        //  global_->setType(tft_service::TFT_TYPE::STANDARD);
    }
}

void SafetyControl::handleFootAlarm(const std_msgs::msg::Int32 msg) {
    if (global_->getType() == tft_service::TFT_TYPE::HEALTH) {
        return; // 仅在标准和避障模式下处理脚部报
    }
    if (msg.data == foot_alarm_level_) {
        return; // 状态未改变，不需要处理
    }
    foot_alarm_level_ = msg.data;
    switch (msg.data) {
    case 0: {
        global_->sendPageId(SEAT_POSITION_PAGE_ID);
        global_->setType(tft_service::TFT_TYPE::STANDARD);
        break;
    }
    case 1: {
        global_->sendPageId(LEFT_FOOT_PAGE_ID);
        global_->setType(tft_service::TFT_TYPE::OBSTACLE_AVOIDANCE);
        break;
    }
    case 2: {
        global_->sendPageId(RIGHT_FOOT_PAGE_ID);
        global_->setType(tft_service::TFT_TYPE::OBSTACLE_AVOIDANCE);
        break;
    }
    case 3: {
        global_->sendPageId(LEFT_RIGHT_FOOT_PAGE_ID);
        global_->setType(tft_service::TFT_TYPE::OBSTACLE_AVOIDANCE);
        break;
    }
    default:
        // 其他情况
        break;
    }
    // 发送音频   发送AUDIO_TYPE_SAFETY_SEAT_DETECTION_20
    std_msgs::msg::Int32 audio_msg;
    audio_msg.data = AUDIO_TYPE_SAFETY_SEAT_DETECTION_20;
    audio_pub_->publish(audio_msg);
    global_->sendPageHeader();
}
