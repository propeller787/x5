#include "tft_module.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <utility>
using namespace std;

namespace tft_service {

std::string bytesToHexStringTft(const char *data, size_t length) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; ++i) {
        ss << std::setw(2) << static_cast<int>(static_cast<unsigned char>(data[i]));
        if (i < length - 1) {
            ss << " ";
        }
    }
    return ss.str();
}

bool TftModule::serial_data_analyse(TFT_PACK& pack)
{
    (void)pack;
    char once_data[REC_BUFF_LEN];
    memset(once_data, 0, REC_BUFF_LEN * sizeof(char));
    _serial_port_ptr->read(once_data, REC_BUFF_LEN);
    return true;
}
void TftModule::setStart(bool status)
{
    i_start_ = status;
}

void TftModule::thread_read_serial_data()
{
    // 优化：使用更短的轮询间隔（10ms），提高响应速度
    const auto poll_interval = std::chrono::milliseconds(10);
    
    while (true) {
        std::this_thread::sleep_for(poll_interval);
        
        // 检查连接状态
        if (connection_state_ != ConnectionState::CONNECTED) {
            // 如果连接失败，尝试重新初始化
            if (connection_state_ == ConnectionState::ERROR) {
                RCLCPP_WARN(rclcpp::get_logger("TftModule"), "尝试重新初始化串口...");
                initializeWithRetry(1); // 只重试1次，避免频繁重试
            }
            continue;
        }
        
        if (!i_start_) {
            continue;
        }
        
        TFT_PACK pack{};
        bool ret = serial_data_analyse(pack);
        if (ret) {
            // TODO: 处理接收到的数据包
        }
    }
}

TftModule::TftModule(std::string device_address, bool crc_enabled, uint16_t frame_header, uint8_t station_addr)
    : device_address_(device_address)
{
    _total_len = 0;
    connection_state_ = ConnectionState::DISCONNECTED;
    _index = 1;
    crc_enabled_ = crc_enabled;
    frame_header_ = frame_header;
    station_addr_ = station_addr;
    
    // 初始化串口，带重试机制
    initializeWithRetry(3);
}

bool TftModule::initializeWithRetry(int max_retries)
{
    OpenOptions gps_serial_port_option;
    gps_serial_port_option.auto_open = false;
    gps_serial_port_option.baud_rate = BR115200;
    gps_serial_port_option.data_bits = DATA_BITS_8;
    gps_serial_port_option.parity = PARITY_NONE;
    gps_serial_port_option.stop_bits = STOP_BITS_1;
    gps_serial_port_option.vmin = 0;
    gps_serial_port_option.vtime = 50;
    gps_serial_port_option.xany = false;
    gps_serial_port_option.xoff = false;
    gps_serial_port_option.xon = false;

    connection_state_ = ConnectionState::CONNECTING;
    
    for (int i = 0; i < max_retries; ++i) {
        if (!_serial_port_ptr) {
            _serial_port_ptr = std::make_shared<SerialPort>(device_address_, gps_serial_port_option);
        }
        
        bool ret = _serial_port_ptr->start();
        if (ret) {
            connection_state_ = ConnectionState::CONNECTED;
            RCLCPP_INFO(rclcpp::get_logger("TftModule"), "串口初始化成功: %s (尝试 %d/%d)", 
                       device_address_.c_str(), i + 1, max_retries);
            setupProtocolSession();
            return true;
        }
        
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "串口初始化失败: %s (尝试 %d/%d)", 
                   device_address_.c_str(), i + 1, max_retries);
        
        if (i < max_retries - 1) {
            // 等待1秒后重试
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    connection_state_ = ConnectionState::ERROR;
    RCLCPP_ERROR(rclcpp::get_logger("TftModule"), "串口初始化失败，已重试 %d 次: %s", 
                max_retries, device_address_.c_str());
    return false;
}
void TftModule::startWrite(const char* buffer, int buf_len)
{
    if (connection_state_ != ConnectionState::CONNECTED || !_serial_port_ptr) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "串口未连接，无法发送数据");
        return;
    }
    
    if (session_) {
        std::vector<uint8_t> data(buffer, buffer + buf_len);
        session_->sendRaw(data);
        return;
    }

    _serial_port_ptr->write(const_cast<char*>(buffer), buf_len);
}
TftModule::~TftModule()
{
    stopProtocolSession();
}

bool TftModule::sendDcBusWrite(uint8_t addr, uint16_t reg, uint16_t value)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_ || !codec_) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "DCBUS未初始化，无法发送写指令");
        return false;
    }
    auto bytes = codec_->encodeWrite(addr, reg, value);
    session_->sendRaw(bytes);
    // 打印bytes的十六进制
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "页面--发送DCBUS写指令: addr=0x%02X, reg=0x%04X, value=0x%04X", addr, reg, value);
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "页面--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(bytes.data()), bytes.size()).c_str());
    return true;
}

bool TftModule::sendDcBusWriteAndVerify(uint8_t addr, uint16_t reg, uint16_t value,
                                        std::chrono::milliseconds timeout)
{
    {
        std::lock_guard<std::mutex> vlk(verify_mtx_);
        if (verify_pending_.active) {
            RCLCPP_WARN(rclcpp::get_logger("TftModule"),
                        "sendDcBusWriteAndVerify: 上一笔读回校验未完成，忽略本次");
            return false;
        }
        verify_pending_.active = true;
        verify_pending_.finished = false;
        verify_pending_.success = false;
        verify_pending_.addr = addr;
        verify_pending_.reg = reg;
        verify_pending_.expected = value;
        verify_pending_.last_read = 0;
    }

    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (!session_ || !codec_) {
            std::lock_guard<std::mutex> vlk(verify_mtx_);
            verify_pending_.active = false;
            RCLCPP_WARN(rclcpp::get_logger("TftModule"), "DCBUS未初始化，无法写后校验");
            return false;
        }
        auto write_bytes = codec_->encodeWrite(addr, reg, value);
        auto read_bytes = codec_->encodeRead(addr, reg, 1);
        session_->sendRaw(write_bytes);
        session_->sendRaw(read_bytes);
        RCLCPP_INFO(rclcpp::get_logger("TftModule"),
                    "DCBUS写后校验: 写 reg=0x%04X value=0x%04X, 已发读请求 %s",
                    reg, value,
                    bytesToHexStringTft(reinterpret_cast<const char*>(read_bytes.data()), read_bytes.size()).c_str());
    }

    std::unique_lock<std::mutex> vlk(verify_mtx_);
    if (!verify_cv_.wait_for(vlk, timeout, [this] { return verify_pending_.finished; })) {
        verify_pending_.active = false;
        verify_pending_.finished = false;
        RCLCPP_WARN(rclcpp::get_logger("TftModule"),
                    "DCBUS写后校验超时: addr=0x%02X reg=0x%04X 期望=0x%04X",
                    addr, reg, value);
        return false;
    }
    const bool ok = verify_pending_.success;
    if (!ok) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"),
                    "DCBUS写后校验值不一致: reg=0x%04X 期望=0x%04X 读回=0x%04X",
                    reg, value, verify_pending_.last_read);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("TftModule"),
                    "DCBUS写后校验成功: reg=0x%04X value=0x%04X", reg, value);
    }
    verify_pending_.active = false;
    verify_pending_.finished = false;
    return ok;
}

bool TftModule::sendDcBusFloatWrite(uint8_t addr, uint16_t reg, float value)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_ || !codec_) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "DCBUS未初始化，无法发送写指令");
        return false;
    }
    auto bytes = codec_->encodeWriteFloat(addr, reg, value);
    session_->sendRaw(bytes);
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "速度--发送DCBUS写指令: addr=0x%02X, reg=0x%04X, value=%.2f", addr, reg, value);
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "速度--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(bytes.data()), bytes.size()).c_str());
    return true;
}

bool TftModule::sendDcBusTextWrite(uint8_t addr, uint16_t reg, const std::string& text)
{
    // 将 UTF8 字符串转换为字节向量（UTF8 字符串本身就是字节序列）
    std::vector<uint8_t> payload;
    payload.reserve(text.size() + 3);  // 预留空间：字符串 + 3个结束字节
    
    // 将字符串的每个字符（UTF8 字节）添加到 payload
    // 例如 "广州大彩HMI" -> E5 B9 BF E5 B7 9E E5 A4 A7 E5 BD A9 48 4D 49
    for (unsigned char byte : text) {
        payload.push_back(static_cast<uint8_t>(byte));
    }
    payload.push_back(0x00);
    sendDcBusWritePayload(addr, reg, payload);
    // 打印调试信息（UTF8 转 16 进制）
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "文本--发送DCBUS写指令: %s", text.c_str());
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "文本--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(payload.data()), payload.size()).c_str());
    return true;
}

bool TftModule::sendDcBusWritePayload(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_ || !codec_) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "DCBUS未初始化，无法发送写指令");
        return false;
    }
    auto bytes = codec_->encodeWritePayload(addr, reg, payload);
    session_->sendRaw(bytes);
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "文本--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(bytes.data()), bytes.size()).c_str());
    return true;
}

bool TftModule::sendDcBusLaneWarningWrite(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_ || !codec_) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "DCBUS未初始化，无法发送写指令");
        return false;
    }
    auto bytes = codec_->encodeWritePayload(addr, reg, payload);
    session_->sendRaw(bytes);
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "车道警告线--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(bytes.data()), bytes.size()).c_str());
    return true;
}

bool TftModule::sendDcBusCurveFitWrite(uint8_t addr, uint16_t reg, uint16_t rgb565,
                                       const std::vector<std::pair<uint16_t, uint16_t>>& points)
{
    constexpr uint16_t kCurveFitCmd = 0x0020;
    constexpr std::size_t kMaxPoints = 32;

    if (points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "sendDcBusCurveFitWrite: 点数为 0，忽略");
        return false;
    }
    if (points.size() > kMaxPoints) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "sendDcBusCurveFitWrite: 点数 %zu 超过上限 %zu，忽略",
                    points.size(), kMaxPoints);
        return false;
    }
    std::vector<std::pair<uint16_t, uint16_t>> left_points, right_points;
    // 左侧6个点
    for (int i = 0; i < 6; i ++) {
        left_points.emplace_back(points[i]);
    }
    // 右侧6个点
    for (int i = 6; i < 12; i ++) {
        right_points.emplace_back(points[i]);
    }

    auto payload = [rgb565](const std::vector<std::pair<uint16_t, uint16_t>>& points){
        std::vector<uint8_t> payload;
        payload.reserve(6 + points.size() * 4);
        auto push_u16_be = [&payload](uint16_t v) {
            payload.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
            payload.push_back(static_cast<uint8_t>(v & 0xFF));
        };
        push_u16_be(kCurveFitCmd);
        push_u16_be(static_cast<uint16_t>(points.size()));
        push_u16_be(rgb565);
        for (const auto& p : points) {
            push_u16_be(p.first);
            push_u16_be(p.second);
        }
        return payload;
    };
    std::vector<uint8_t> all_payload, left_payload, right_payload;
    left_payload = payload(left_points);
    right_payload = payload(right_points);
    all_payload.insert(all_payload.end(), left_payload.begin(), left_payload.end());
    all_payload.insert(all_payload.end(), right_payload.begin(), right_payload.end());
    
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "左侧车道警告线--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(left_payload.data()), left_payload.size()).c_str());
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "右侧车道警告线--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(right_payload.data()), right_payload.size()).c_str());
    RCLCPP_INFO(rclcpp::get_logger("TftModule"), "总车道警告线--发送DCBUS写指令: %s", bytesToHexStringTft(reinterpret_cast<const char*>(all_payload.data()), all_payload.size()).c_str());
    
    return sendDcBusLaneWarningWrite(addr, reg, all_payload);
}

bool TftModule::sendDcBusRead(uint8_t addr, uint16_t reg, uint8_t count)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!session_ || !codec_) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "DCBUS未初始化，无法发送读指令");
        return false;
    }
    auto bytes = codec_->encodeRead(addr, reg, count);
    session_->sendRaw(bytes);
    return true;
}

std::vector<uint8_t> TftModule::buildDcBusReadFrame(uint16_t reg, uint8_t count) const
{
    DcBusCodec codec(crc_enabled_, frame_header_);
    return codec.encodeRead(station_addr_, reg, count);
}

std::optional<uint16_t> TftModule::parseDcBusReadResponse(const std::vector<uint8_t>& raw,
                                                          uint16_t expected_reg,
                                                          uint8_t expected_addr) const
{
    DcBusCodec codec(crc_enabled_, frame_header_);
    auto buffer = raw; // tryDecode 会消费 buffer 内容，使用副本避免影响调用方
    auto frames = codec.tryDecode(buffer);
    for (const auto& frame : frames) {
        if (frame.func == 0xF2 && frame.reg == expected_reg && frame.addr == expected_addr && frame.payload.size() >= 3) {
            // payload[0] = word count, payload[1..] = data
            return static_cast<uint16_t>(frame.payload[1] << 8 | frame.payload[2]);
        }
    }
    return std::nullopt;
}

void TftModule::setReadResponseHandler(std::function<void(uint16_t, uint16_t)> handler)
{
    read_response_handler_ = std::move(handler);
}

void TftModule::setTouchEventCallback(std::function<void(uint16_t)> handler)
{
    touch_event_handler_ = std::move(handler);
}

void TftModule::setupProtocolSession()
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (session_ || !_serial_port_ptr) {
        return;
    }

    codec_ = std::make_shared<DcBusCodec>(crc_enabled_, frame_header_);
    session_ = std::make_unique<SerialSession>(
        _serial_port_ptr,
        codec_,
        [this](const DcBusFrame& frame) { handleDcBusFrame(frame); });
    session_->start();
}

void TftModule::stopProtocolSession()
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (session_) {
        session_->stop();
        session_.reset();
    }
    codec_.reset();
}

void TftModule::handleDcBusFrame(const DcBusFrame& frame)
{
    RCLCPP_DEBUG(
        rclcpp::get_logger("TftModule"),
        "DCBUS frame func=0x%02X reg=0x%04X payload=%zu", frame.func, frame.reg, frame.payload.size());

    // 处理读寄存器（F2）
    if (frame.func == 0xF2) {
        // 判断是读寄存器请求还是响应
        // 请求格式：F2 + reg(2字节) + count(1字节)，payload只有count
        // 响应格式：F2 + reg(2字节) + word_count(1字节) + data(2字节或更多)，payload包含word_count和数据
        
        if (frame.payload.size() == 1) {
            // 读寄存器请求：TFT 触屏时主动上报，payload[0] = count，frame.reg 为触屏寄存器地址
            uint8_t count = frame.payload[0];
            RCLCPP_INFO(rclcpp::get_logger("TftModule"), 
                       "DCBUS读寄存器请求(触屏): 站号=0x%02X, 变量起始地址=0x%04X, 读取个数=%u", 
                       frame.addr, frame.reg, count);
            if (touch_event_handler_) {
                touch_event_handler_(frame.reg);
            }
        } else if (frame.payload.size() >= 3) {
            // 读寄存器响应：payload[0]=word count, payload[1..]=data
            uint8_t word_count = frame.payload[0];
            // 提取第一个寄存器的值（高字节在前）
            uint16_t value = static_cast<uint16_t>(frame.payload[1] << 8 | frame.payload[2]);

            {
                std::lock_guard<std::mutex> vlk(verify_mtx_);
                if (verify_pending_.active && frame.addr == verify_pending_.addr &&
                    frame.reg == verify_pending_.reg) {
                    verify_pending_.last_read = value;
                    verify_pending_.success = (value == verify_pending_.expected);
                    verify_pending_.finished = true;
                    verify_cv_.notify_all();
                }
            }
            
            RCLCPP_INFO(rclcpp::get_logger("TftModule"), 
                       "DCBUS读寄存器响应: 站号=0x%02X, 变量起始地址=0x%04X, 寄存器值=%u (0x%04X), word_count=%u", 
                       frame.addr, frame.reg, value, value, word_count);
            
            // 调用回调函数
            if (read_response_handler_) {
                read_response_handler_(frame.reg, value);
            }
            if (touch_event_handler_) {
                touch_event_handler_(frame.reg);
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("TftModule"), 
                       "DCBUS F2帧payload长度异常: reg=0x%04X, payload_size=%zu", 
                       frame.reg, frame.payload.size());
        }
        return;
    }
}

}