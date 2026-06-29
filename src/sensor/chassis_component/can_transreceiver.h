#pragma once

#include "sensor_utils.hpp"
#include "muduo/net/Buffer.h"
#include "muduo/net/Channel.h"
#include "muduo/net/EventLoop.h"

#include <rclcpp/rclcpp.hpp>

#include <functional>

namespace sensor::chassis {

using CanReceiveCallback = std::function<void(int frameid, muduo::net::Buffer&)>;

class CanTransreceiver {
public:
    CanTransreceiver(std::string dev, muduo::net::EventLoop* loop);
    ~CanTransreceiver();
    void start();
    /** 暂停 CAN 接收（用于 OTA 升级等场景） */
    void stop();
    /** 恢复 CAN 接收 */
    void resume();

    void filter();
    void filter(std::map<uint16_t, unsigned int> canIdMap);
    void send(int frameid, muduo::net::Buffer& buf);
    void sendLoog(int frameid);
    void setReceiveCallback(CanReceiveCallback cb);

private:
    void onDataReceived();

private:
    const std::string can_dev_;
    muduo::net::EventLoop* loop_;
    rclcpp::Logger logger_;
    std::unique_ptr<muduo::net::Channel> chnl_;
    sensor::utils::Signal<void(int frameid, muduo::net::Buffer&)> signal_;
    std::set<sensor::utils::Slot> slots_;
    // 错误状态跟踪：记录当前处于错误状态的错误码（0表示无错误）
    uint32_t current_error_code_ = 0;
};

}