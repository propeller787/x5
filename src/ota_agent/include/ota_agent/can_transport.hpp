#pragma once

#include <cstdint>
#include <vector>
#include <functional>

namespace ota_agent {

struct CanFrame {
    uint32_t id = 0;
    std::vector<uint8_t> data;
    uint8_t dlc = 0;
    bool is_extended = false;
};

class ICanTransport {
public:
    virtual ~ICanTransport() = default;
    virtual bool send(const CanFrame& frame, std::string& err) = 0;
    virtual void set_receive_handler(std::function<void(const CanFrame&)> cb) = 0;
    virtual void close() = 0; // 主动关闭底层传输资源
};

} // namespace ota_agent
