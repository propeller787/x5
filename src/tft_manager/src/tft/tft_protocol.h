#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <utils/serial_port.h>

namespace tft_service {

struct DcBusFrame {
    uint16_t header = 0xAA55;
    uint8_t addr = 0;
    uint8_t len = 0;
    uint8_t func = 0; // 功能码 F1写寄存器 F2读寄存器
    uint16_t reg = 0; // 寄存器地址
    std::vector<uint8_t> payload; // 数据
    uint16_t crc = 0xCCCC;
    bool crc_enabled = true;
};

class ProtocolCodec {
public:
    virtual ~ProtocolCodec() = default;
    virtual std::vector<uint8_t> encodeWrite(uint8_t addr, uint16_t reg, uint16_t value) = 0;
    virtual std::vector<uint8_t> encodeWriteFloat(uint8_t addr, uint16_t reg, float value) = 0;
    virtual std::vector<uint8_t> encodeWritePayload(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload) = 0;
    virtual std::vector<uint8_t> encodeRead(uint8_t addr, uint16_t reg, uint8_t count) = 0;
    virtual std::vector<DcBusFrame> tryDecode(std::vector<uint8_t>& buffer) = 0;
};

class DcBusCodec : public ProtocolCodec {
public:
    explicit DcBusCodec(bool crc_enabled = true, uint16_t header = 0xAA55);

    std::vector<uint8_t> encodeWrite(uint8_t addr, uint16_t reg, uint16_t value) override;
    std::vector<uint8_t> encodeWriteFloat(uint8_t addr, uint16_t reg, float value) override;
    std::vector<uint8_t> encodeWritePayload(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload) override;
    std::vector<uint8_t> encodeRead(uint8_t addr, uint16_t reg, uint8_t count) override;
    std::vector<DcBusFrame> tryDecode(std::vector<uint8_t>& buffer) override;

private:
    uint16_t calcCrc(const uint8_t* data, size_t len) const;
    std::vector<uint8_t> pack(uint8_t addr, std::vector<uint8_t>&& body) const;

private:
    bool crc_enabled_ = true;
    uint16_t header_ = 0xAA55;
};

class SerialSession {
public:
    using FrameHandler = std::function<void(const DcBusFrame&)>;

    SerialSession(std::shared_ptr<utils::SerialPort> port,
                  std::shared_ptr<ProtocolCodec> codec,
                  FrameHandler handler);
    ~SerialSession();

    void start();
    void stop();

    void sendRaw(const std::vector<uint8_t>& bytes);

    std::shared_ptr<ProtocolCodec> codec() const { return codec_; }

private:
    void readLoop();

private:
    std::shared_ptr<utils::SerialPort> port_;
    std::shared_ptr<ProtocolCodec> codec_;
    FrameHandler handler_;
    std::atomic<bool> running_{false};
    std::thread read_thread_;
    std::mutex write_mutex_;
    std::vector<uint8_t> recv_buffer_;
};

} // namespace tft_service
