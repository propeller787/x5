#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include "ota_agent/can_transport.hpp"

namespace ota_agent {

class MockCanTransport : public ICanTransport {
public:
    explicit MockCanTransport(rclcpp::Logger logger,
                              std::string outcome = "success",
                              int step_delay_ms = 100,
                              int fail_after_step = 2)
        : logger_(logger), outcome_(std::move(outcome)), step_delay_ms_(step_delay_ms), fail_after_step_(fail_after_step) {}

    bool send(const CanFrame& frame, std::string& err) override;
    void set_receive_handler(std::function<void(const CanFrame&)> cb) override { rx_cb_ = std::move(cb); }
    void set_receive_handler(std::function<void(uint32_t, const std::vector<uint8_t>&)> cb) override { 
        auto cb_wrapper = [cb](const CanFrame& frame) {
            cb(frame.id, frame.data);
        };
        rx_cb_ = std::move(cb_wrapper); 
    }
    void close() override { rx_cb_ = nullptr; }
private:
    rclcpp::Logger logger_;
    std::function<void(const CanFrame&)> rx_cb_;
    std::string outcome_; // success | failure | timeout
    int step_delay_ms_ = 100;
    int fail_after_step_ = 2;
    int step_counter_ = 0;
};

} // namespace ota_agent
