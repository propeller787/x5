#include "tft_cruise_control.h"
#include <chrono>

using namespace tft_service::control;

CruiseControl::CruiseControl()
    : Control("CruiseControl")
{
    using namespace std::placeholders;
    // mode_status_ptr_ = global_->node_->create_subscription<interface::msg::ReportMode>(
    //     modeStatusTopic, 2,
    //     std::bind(&CruiseControl::handleReportMode, this, _1));
    cruise_alarm_sub_ = global_->node_->create_subscription<std_msgs::msg::Int32>(
        cursorAlarmTopic, 2,
        std::bind(&CruiseControl::handleCruiseAlarm, this, _1));
    mode_status_str_ptr_ = global_->node_->create_subscription<std_msgs::msg::String>(
        sceneServiceModeTopic, 2,
        std::bind(&CruiseControl::handleModeStatusStr, this, _1));
}

CruiseControl::~CruiseControl()
{
    // Destructor implementation
}

void CruiseControl::handleReportMode(const interface::msg::ReportMode msg)
{
    for (const auto& mode : msg.mode_list) {
        if (mode.name == "cruise") {
            if (mode.status) {
                global_->setType(tft_service::TFT_TYPE::CRUISE);
                if (global_->getSpeedkmh() == 0.0) {
                    // 发送巡航界面 1
                } else {
                    // 发送巡航界面 2 速度
                }
            }
            if (!mode.status && global_->getType() == tft_service::TFT_TYPE::CRUISE) {
                global_->setType(tft_service::TFT_TYPE::STANDARD);
            }
            // 更新巡航状态
            cruise_status_ = mode.status;
        }
        if (mode.name == "auxiliary") {
            if (mode.status) {
                std::cout << "安全功能已启用" << std::endl;
                global_->sendAddressData(OBSTACLE_AVOIDANCE_FRAME_ID, 2); // 发送波浪
            } else {
                std::cout << "安全功能已禁用" << std::endl;
                global_->sendAddressData(OBSTACLE_AVOIDANCE_FRAME_ID, 1); // 发送波浪
            }
        }
    }
}

void CruiseControl::handleCruiseAlarm(const std_msgs::msg::Int32 msg)
{
    if (global_->getType() == tft_service::TFT_TYPE::CRUISE
        && global_->getSpeedkmh() == 0.0) {
        // 发送预警界面
    }
}

void CruiseControl::handleModeStatusStr(const std_msgs::msg::String msg)
{
    auto mode = msg.data;
    if (mode == "auxiliary") {
        global_->sendAddressData(OBSTACLE_AVOIDANCE_FRAME_ID, 2); // 发送波浪
    }
    if (mode != "auxiliary") {
        global_->sendAddressData(OBSTACLE_AVOIDANCE_FRAME_ID, 1);
    }
}
