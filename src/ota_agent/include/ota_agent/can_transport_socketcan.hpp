#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <rclcpp/rclcpp.hpp>

#include "ota_agent/can_transport.hpp"

namespace ota_agent {

class SocketCanTransport : public ICanTransport {
public:
    SocketCanTransport(const std::string& ifname, rclcpp::Logger logger);
    ~SocketCanTransport() override;
    bool send(const CanFrame& frame, std::string& err) override;
    void set_receive_handler(std::function<void(const CanFrame&)> cb) override;
    void close() override;
private:
    void rx_loop();

    int sock_ = -1;
    uint32_t can_id_mask_ = 0x1FFFFFFF; // permit std+ext
    std::string ifname_;
    rclcpp::Logger logger_;
    std::function<void(const CanFrame&)> rx_cb_;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};
};

} // namespace ota_agent
