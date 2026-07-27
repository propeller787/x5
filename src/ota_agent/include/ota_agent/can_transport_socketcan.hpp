#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ota_agent/can_transport.hpp"

namespace ota_agent {

namespace Constants {
    constexpr uint32_t DEFAULT_DRIVER_RX_ID = 0x701;
    constexpr uint32_t DEFAULT_DRIVER_TX_ID = 0x709;
    constexpr size_t ISO_TP_SINGLE_FRAME_MAX = 7;
    constexpr size_t ISO_TP_FIRST_FRAME_PAYLOAD = 6;
    constexpr size_t ISO_TP_CONTINUOUS_FRAME_PAYLOAD = 7;
    constexpr size_t CAN_FRAME_MAX_DLC = 8;
    constexpr uint32_t ISO_TP_MAX_LEN = 0xFFF;
    constexpr int CAN_SEND_MAX_RETRIES = 5;
    constexpr int CAN_SEND_RETRY_DELAY_MS = 1;
    constexpr int CAN_RX_LOOP_DELAY_MS = 5;
    constexpr int CAN_STABILIZE_DELAY_MS = 100;
    constexpr int DEFAULT_TIMEOUT_MS = 10000;
    constexpr int FLOW_CONTROL_TIMEOUT_MS = 10000;
    constexpr int FLOW_CONTROL_WAIT_MS = 1000;
    

    // UDS服务ID
    constexpr uint8_t SID_REQUEST_DOWNLOAD = 0x34;
    constexpr uint8_t SID_TRANSFER_DATA = 0x36;
    constexpr uint8_t SID_TRANSFER_EXIT = 0x37;
    constexpr uint8_t SID_ROUTINE_CONTROL = 0x31;
    constexpr uint8_t SID_ERASE_MEMORY = 0x31;

    // 正响应SID
    constexpr uint8_t SID_POS_RESPONSE_OFFSET = 0x40;
    constexpr uint8_t SID_POS_RESPONSE_DOWNLOAD = 0x74;
    constexpr uint8_t SID_POS_RESPONSE_ROUTINE = 0x71;
    constexpr uint8_t SID_POS_RESPONSE_TRANSFER_DATA = 0x76;
    constexpr uint8_t SID_POS_RESPONSE_EXIT = 0x77;

    // 流控帧
    constexpr uint8_t PCI_FLOW_CONTROL = 0x30;

    // HEX文件相关
    constexpr uint32_t HEX_INVALID_ADDRESS = 0xFFFFFFFF;
    constexpr uint8_t HEX_FILL_BYTE = 0xFF;
};

class SocketCanTransport : public ICanTransport {
public:
    SocketCanTransport(const std::string& ifname, rclcpp::Logger logger);
    ~SocketCanTransport() override;
    bool send(const CanFrame& frame, std::string& err) override;
    void set_receive_handler(std::function<void(const CanFrame&)> cb) override;
    void set_receive_handler(std::function<void(uint32_t, const std::vector<uint8_t>&)> cb);
    void close() override;
private:
    void rx_loop();

    int sock_ = -1;
    uint32_t can_id_mask_ = 0x1FFFFFFF; // permit std+ext
    std::string ifname_;
    rclcpp::Logger logger_;
    std::function<void(const CanFrame&)> rx_cb_;
    std::shared_ptr<std::function<void(uint32_t, const std::vector<uint8_t>&)>> rx_cb_wrapper_;
    std::mutex rx_cb_mutex_;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};
};

} // namespace ota_agent
