#include "tft_alarm_control.h"

using namespace tft_service::control;

AlarmControl::AlarmControl() : Control("AlarmControl") {
    auxiliary_alarm_sub_back_ = global_->node_->create_subscription<interface::msg::AuxiliaryAlarm>(
        auxiliaryAlarmBackTopic, 1, std::bind(&AlarmControl::alarmCallback, this, std::placeholders::_1));
}
AlarmControl::~AlarmControl() {
    // Destructor implementation
}

void AlarmControl::alarmCallback(const interface::msg::AuxiliaryAlarm::SharedPtr msg) {
    if (index_ == 0 || index_ % 60 == 0) {

        std_msgs::msg::Int32 audio_msg;
        audio_msg.data = AUDIO_TYPE_ALARM_AUDIO_22;
        audio_pub_->publish(audio_msg);
    }
    if (index_++ == 60) {
        index_ = 0;
    }
}
