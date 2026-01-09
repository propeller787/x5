#include "tft_charger_control.h"

using namespace tft_service::control;

ChargerControl::ChargerControl()
    : Control("ChargerControl")
{
    charger_sta_ptr_ = global_->node_->create_subscription<interface::msg::RechargeStatus>(
        rechargeStatusTopic, 10,
        std::bind(&ChargerControl::handleChargerStatus, this, _1));
    // Constructor implementation
}

ChargerControl::~ChargerControl()
{
    // Destructor implementation
}

void ChargerControl::handleChargerStatus(const interface::msg::RechargeStatus msg)
{
    switch (msg.status) {
    case 1: // 搜索充电站
        global_->setType(tft_service::TFT_TYPE::AUTOMATIC_RETRACTION);
        /* code */
        break;
    case 2: // 未找到充电站
        /* code */
        break;
    case 3: // 找到充电站并前往
        /* code */
        break;
    case 4: // 检测到障碍物
        /* code */
        break;
    case 5: // 开始充电
        /* code */
        global_->setType(tft_service::TFT_TYPE::STANDARD);
        break;
    case 6: // 停止充电
        /* code */
        break;
    default:
        break;
    }
}