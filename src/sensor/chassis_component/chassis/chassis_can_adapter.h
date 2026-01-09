#pragma once
#include "../../global.h"
#include "../can_adapter.h"
#include "../can_transreceiver.h"
#include "../chassis/chassis_can_adapter.h"
#include "../handle/handle_can_adapter.h"
namespace sensor::chassis {

#define CHASSIS_1_FRAME_ID (0x21400u)
#define CHASSIS_2_FRAME_ID (0x28400u)

class ChassisCanAdapter : public CanAdapter {
public:
    explicit ChassisCanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs);

    virtual void onReceiveCanMessage(const std::string& dev, int fid, muduo::net::Buffer& buffer) override;
    virtual void onReceiveGnssMessage(const interface::msg::Gnss::SharedPtr msg) override;
    virtual void onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) override;
    virtual void sendCarInfo() override;
    virtual void sendBattery() override;
    virtual void onRechargeStatusMessage(const interface::msg::RechargeStatus::SharedPtr msg) override;
    virtual void onRecieveSpeedbumpMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg) override;
    virtual void onRecievePaDaoMessage(const std::shared_ptr<std_msgs::msg::Int32> msg) override;

private:
    // 底盘速度
    void onReceive214Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive284Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    float carCrtlGetMotorSpeed(uint8_t high, uint8_t low);

private:
    using CanMessageUnpackFunction = std::function<void(const std::string& dev, int fid, muduo::net::Buffer& buffer)>;
    std::map<int, CanMessageUnpackFunction> message_unpack_table_;
    interface::msg::Control::SharedPtr control_info_;
    uint16_t electricquantity_ = 0; // 当前电量
    uint16_t remainingmileage_ = 0; // 剩余里程
};

}