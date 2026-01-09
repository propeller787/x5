#include "ota_agent/can_transport_mock.hpp"

#include <thread>

namespace ota_agent {

bool MockCanTransport::send(const CanFrame& frame, std::string& err) {
    if (frame.data.empty()) {
        err = "empty frame";
        return false;
    }
    RCLCPP_DEBUG(logger_, "[CAN-MOCK] send id=0x%X dlc=%u len=%zu", frame.id, frame.dlc, frame.data.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, step_delay_ms_)));

    if (!rx_cb_) return true;

    if (outcome_ == "timeout") {
        err = "mock timeout";
        return false;
    }

    ++step_counter_;
    if (outcome_ == "failure" && step_counter_ >= std::max(1, fail_after_step_)) {
        CanFrame f = frame; f.data = {'F'}; f.dlc = 1; rx_cb_(f);
        return true;
    }

    if (step_counter_ == 1) {
        CanFrame p = frame; p.data = {'P', 30}; p.dlc = 2; rx_cb_(p);
        return true;
    } else if (step_counter_ == 2) {
        CanFrame p2 = frame; p2.data = {'P', 70}; p2.dlc = 2; rx_cb_(p2);
        return true;
    }

    CanFrame ok = frame; ok.data = {'S'}; ok.dlc = 1; rx_cb_(ok);
    return true;
}

} // namespace ota_agent
