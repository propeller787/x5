#include "tft_protocol.h"

#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

namespace tft_service {
DcBusCodec::DcBusCodec(bool crc_enabled, uint16_t header)
    : crc_enabled_(crc_enabled), header_(header) {}

uint16_t DcBusCodec::calcCrc(const uint8_t* data, size_t len) const {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            uint16_t carry = crc & 0x0001;
            crc >>= 1;
            if (carry == 1) {
                crc ^= 0xA001;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> DcBusCodec::pack(uint8_t addr, std::vector<uint8_t>&& body) const {
    std::vector<uint8_t> frame;
    const uint8_t len = static_cast<uint8_t>(body.size() + 2); // body + CRC
    frame.reserve(4 + len);
    frame.push_back(static_cast<uint8_t>(header_ >> 8));
    frame.push_back(static_cast<uint8_t>(header_ & 0xFF));
    frame.push_back(addr);
    frame.push_back(len);
    frame.insert(frame.end(), body.begin(), body.end());

    uint16_t crc = 0xCCCC;
    if (crc_enabled_) {
        // CRC covers: addr + len + body (without CRC)
        crc = calcCrc(frame.data() + 2, 2 + body.size());
    }
    // CRC 低字节在前
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

std::vector<uint8_t> DcBusCodec::encodeWrite(uint8_t addr, uint16_t reg, uint16_t value) {
    std::vector<uint8_t> body;
    body.reserve(5);
    body.push_back(0xF1);
    body.push_back(static_cast<uint8_t>(reg >> 8));
    body.push_back(static_cast<uint8_t>(reg & 0xFF));
    body.push_back(static_cast<uint8_t>(value >> 8));
    body.push_back(static_cast<uint8_t>(value & 0xFF));
    return pack(addr, std::move(body));
}
//float 1.2 的 IEEE 754 十六进制：0x3F99999A
std::vector<uint8_t> DcBusCodec::encodeWriteFloat(uint8_t addr, uint16_t reg, float value) {
    std::vector<uint8_t> body;
    body.reserve(7);
    body.push_back(0xF1);
    // 浮点数转换为16进制
    body.push_back(static_cast<uint8_t>(reg >> 8));
    body.push_back(static_cast<uint8_t>(reg & 0xFF));
    uint32_t value32;
    std::memcpy(&value32, &value, sizeof(value32));
    body.push_back(static_cast<uint8_t>(value32 >> 24) & 0xFF); // 第1字节（大端）
    body.push_back(static_cast<uint8_t>((value32 >> 16) & 0xFF)); // 第2字节
    body.push_back(static_cast<uint8_t>((value32 >> 8)  & 0xFF)); // 第3字节
    body.push_back(static_cast<uint8_t>(value32        & 0xFF)); // 第4字节
    return pack(addr, std::move(body));
}
std::vector<uint8_t> DcBusCodec::encodeWritePayload(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> body;
    body.reserve(3 + payload.size());
    body.push_back(0xF1);
    body.push_back(static_cast<uint8_t>(reg >> 8));
    body.push_back(static_cast<uint8_t>(reg & 0xFF));
    body.insert(body.end(), payload.begin(), payload.end());
    return pack(addr, std::move(body));
}

std::vector<uint8_t> DcBusCodec::encodeRead(uint8_t addr, uint16_t reg, uint8_t count) {
    std::vector<uint8_t> body;
    body.reserve(4);
    body.push_back(0xF2); // 读寄存器指令（协议定义）
    body.push_back(static_cast<uint8_t>(reg >> 8));
    body.push_back(static_cast<uint8_t>(reg & 0xFF));
    body.push_back(count);
    return pack(addr, std::move(body));
}

std::vector<DcBusFrame> DcBusCodec::tryDecode(std::vector<uint8_t>& buffer) {
    std::vector<DcBusFrame> frames;
    size_t idx = 0;

    while (buffer.size() - idx >= 7) { // minimal frame size
        if (!(buffer[idx] == static_cast<uint8_t>(header_ >> 8) && buffer[idx + 1] == static_cast<uint8_t>(header_ & 0xFF))) {
            ++idx;
            continue;
        }

        const uint8_t addr = buffer[idx + 2];
        const uint8_t len = buffer[idx + 3];
        const size_t frame_len = 4 + len; // header(2)+addr(1)+len(1)+len bytes
        if (buffer.size() - idx < frame_len) {
            break; // incomplete
        }

        const size_t body_no_crc_len = len >= 2 ? static_cast<size_t>(len - 2) : 0;
        if (body_no_crc_len == 0) {
            ++idx;
            continue;
        }

        const size_t crc_pos = idx + 4 + body_no_crc_len;
        // CRC 低字节在前
        const uint16_t crc_rx = static_cast<uint16_t>(buffer[crc_pos] | (buffer[crc_pos + 1] << 8));

        if (crc_enabled_) {
            const uint16_t crc_calc = calcCrc(buffer.data() + idx + 2, 2 + body_no_crc_len);
            if (crc_calc != crc_rx) {
                RCLCPP_WARN(rclcpp::get_logger("DcBusCodec"), "CRC mismatch, drop frame");
                ++idx;
                continue;
            }
        }

        const uint8_t func = buffer[idx + 4];
        DcBusFrame frame;
        frame.header = header_;
        frame.addr = addr;
        frame.len = len;
        frame.func = func;
        frame.crc = crc_rx;
        frame.crc_enabled = crc_enabled_;

        if (body_no_crc_len >= 3) {
            frame.reg = static_cast<uint16_t>(buffer[idx + 5] << 8 | buffer[idx + 6]);
            const size_t payload_len = body_no_crc_len - 3;
            if (payload_len > 0) {
                frame.payload.assign(buffer.begin() + idx + 7, buffer.begin() + idx + 7 + payload_len);
            }
        }

        frames.push_back(std::move(frame));
        idx += frame_len;
    }

    if (idx > 0) {
        buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(idx));
    }

    return frames;
}

SerialSession::SerialSession(std::shared_ptr<utils::SerialPort> port,
                             std::shared_ptr<ProtocolCodec> codec,
                             FrameHandler handler)
    : port_(std::move(port)), codec_(std::move(codec)), handler_(std::move(handler)) {}

SerialSession::~SerialSession() {
    stop();
}

void SerialSession::start() {
    if (running_.exchange(true)) {
        return;
    }
    read_thread_ = std::thread(&SerialSession::readLoop, this);
}

void SerialSession::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void SerialSession::sendRaw(const std::vector<uint8_t>& bytes) {
    if (!port_) {
        return;
    }
    std::lock_guard<std::mutex> lock(write_mutex_);
    port_->write(reinterpret_cast<char*>(const_cast<uint8_t*>(bytes.data())), static_cast<int>(bytes.size()));
}

void SerialSession::readLoop() {
    constexpr auto kSleep = std::chrono::milliseconds(5);
    std::vector<uint8_t> tmp(256);
    while (running_.load()) {
        int n = port_ ? port_->read(reinterpret_cast<char*>(tmp.data()), static_cast<int>(tmp.size())) : -1;
        if (n > 0) {
            recv_buffer_.insert(recv_buffer_.end(), tmp.begin(), tmp.begin() + n);
            if (codec_) {
                auto frames = codec_->tryDecode(recv_buffer_);
                for (const auto& f : frames) {
                    if (handler_) {
                        handler_(f);
                    }
                }
            }
        } else {
            std::this_thread::sleep_for(kSleep);
        }
    }
}

} // namespace tft_service
