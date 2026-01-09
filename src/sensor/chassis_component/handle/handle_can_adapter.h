#pragma once
#include "../../global.h"
#include "../can_adapter.h"
#include "../can_transreceiver.h"
#include "../dbc/odrive.h"
#include <muduo/net/EventLoop.h>
#include <thread>
#include <can_msgs/msg/frame.hpp>
namespace sensor::chassis {
#define HANDLE_FRAME_ID (0x801400u)
#define ZQZY 1
class HandleCanAdapter : public CanAdapter {
public:
    explicit HandleCanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs);
    virtual void onReceiveCanMessage(const std::string& dev, int fid, muduo::net::Buffer& buffer) override;
    virtual void onReceiveGnssMessage(const interface::msg::Gnss::SharedPtr msg) override;
    virtual void onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) override;
    virtual void onRechargeStatusMessage(const interface::msg::RechargeStatus::SharedPtr msg) override;
    virtual void onRecieveSpeedbumpMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg) override;
    virtual void onRecievePaDaoMessage(const std::shared_ptr<std_msgs::msg::Int32> msg) override;
    virtual void sendCarInfo() override;
    virtual void sendBattery() override;

private:
    // 摇杆
    void onReceive8014Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive220Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive217Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    // void onReceive219Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive21aInfo(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive21cInfo(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive225Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive222Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive224Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive201Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive301Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive226Info(const std::string&, int fid, muduo::net::Buffer& buffer);

    void onReceive101Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive100Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    //错误描述信息
    void onReceive270Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive271Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive272Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive27FInfo(const std::string&, int fid, muduo::net::Buffer& buffer);

    void onReceive280Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive281Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive282Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void send90108();

private:
    using CanMessageUnpackFunction = std::function<void(const std::string& dev, int fid, muduo::net::Buffer& buffer)>;
    std::map<int, CanMessageUnpackFunction> message_unpack_table_;
    interface::msg::Control control_info_;
    int i = 0;
    muduo::Timestamp last_backup_car_time_;
    bool is_backup_car_ = false;
    bool backup_car_ = true;
    interface::msg::CarInfo car_info_ {}; // 使用花括号进行值初始化
    size_t speep_dump_count_ = 0;
    size_t pa_dao_type_ = 0;
    bool speedbump_flag_ { false };
    bool podao_flag_ { false };
    bool uppd_flag_ { false };
    // 有速度时为true 原子操作
    std::atomic_bool speed_status_ { true };
    
    // CAN数据发布器 - 用于实时发布CAN数据到话题供recorder订阅
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;
    
    // 发布CAN数据到话题
    void publishCanFrame(const std::string& dev, int fid, muduo::net::Buffer& buffer);
};

}