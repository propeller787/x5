#pragma once
// 回充

#include "control.h"
#include <interface/msg/recharge_status.hpp>

namespace tft_service::control {

class ChargerControl : public Control {
public:
    ChargerControl();
    ~ChargerControl();

private:
    void handleChargerStatus(const interface::msg::RechargeStatus msg);

private:
    rclcpp::Subscription<interface::msg::RechargeStatus>::SharedPtr charger_sta_ptr_ { nullptr };
};

}