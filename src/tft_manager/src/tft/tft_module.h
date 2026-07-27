#pragma once
#include "tft_helper.h"
#include "tft_protocol.h"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include <utils/serial_port.h>
#define REC_BUFF_LEN 512
#define BUFFER_128 128
#define HEALTHREADINTERVAL 5000

#ifdef __UNIT_TEST__
#define private public
#define protected public
#endif
using namespace utils;
namespace tft_service {

class TftModule {
public:
    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        ERROR
    };

    TftModule(std::string device_address, bool crc_enabled = true, uint16_t frame_header = 0xAA55, uint8_t station_addr = 0);
    ~TftModule();
    void startWrite(const char* buffer, int buf_len);
    void setStart(bool status);
    
    /**
     * @brief 初始化串口，带重试机制
     * @param max_retries 最大重试次数，默认3次
     * @return 是否成功
     */
    bool initializeWithRetry(int max_retries = 3);
    
    /**
     * @brief 获取连接状态
     */
    ConnectionState getConnectionState() const { return connection_state_; }
    
    /**
     * @brief 检查串口是否可用
     */
    bool isConnected() const { return connection_state_ == ConnectionState::CONNECTED; }

    // DCBUS helpers
    bool sendDcBusWrite(uint8_t addr, uint16_t reg, uint16_t value);
    /**
     * @brief 写 16 位寄存器后立即发 F2 读 1 字，校验屏幕回包中的值是否与写入一致（需已开启与屏幕一致的 CRC）
     * @param timeout 等待读回响应超时
     * @return 收到合法 F2 响应且值一致为 true；超时、未初始化或值不一致为 false
     */
    bool sendDcBusWriteAndVerify(uint8_t addr, uint16_t reg, uint16_t value,
                                 std::chrono::milliseconds timeout = std::chrono::milliseconds(300));
    /** 发送字符串到指定寄存器（payload 为字符串 UTF-8 字节） */
    bool sendDcBusTextWrite(uint8_t addr, uint16_t reg, const std::string& text);
    /* 发送float类型数据到指定寄存器 */
    bool sendDcBusFloatWrite(uint8_t addr, uint16_t reg, float value);
    /*
     * @brief 发送DCBUS写寄存器帧
     * @param addr 站号
     * @param reg 寄存器地址
     * @param payload 数据
     * @return 是否成功
    */
    bool sendDcBusWritePayload(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload);
    /*
     * @brief 发送总车道警告线DCBUSBUS写寄存器帧
     * @param addr 站号
     * @param reg 寄存器地址
     * @param payload 数据
     * @return 是否成功
    */
    bool sendDcBusLaneWarningWrite(uint8_t addr, uint16_t reg, const std::vector<uint8_t>& payload);
    /**
     * @brief 屏幕 VT 曲线拟合（与 Lua set_array 中 0x20 一致）：payload = 0x0020 | 点数 | RGB565 | (x,y)×N，均为 16bit 大端
     * @param addr 站号 STATION_ADDRESS 0
     * @param reg 寄存器地址 BACKWARD_LINE_ADDRESS 0x2000  倒车引导线地址
     * @param rgb565 线条颜色，如红色 0xF800
     * @param points 采样点序列；点数须与屏工程一致（常见 6～7 点）
     */
    bool sendDcBusCurveFitWrite(uint8_t addr, uint16_t reg, uint16_t rgb565,
                                const std::vector<std::pair<uint16_t, uint16_t>>& points);
    bool sendDcBusRead(uint8_t addr, uint16_t reg, uint8_t count);

    // 构造/解析 DCBUS 读寄存器帧，便于仿真与真实串口复用
    std::vector<uint8_t> buildDcBusReadFrame(uint16_t reg, uint8_t count = 1) const;
    std::optional<uint16_t> parseDcBusReadResponse(const std::vector<uint8_t>& raw,
                                                   uint16_t expected_reg,
                                                   uint8_t expected_addr) const;

    void setReadResponseHandler(std::function<void(uint16_t reg, uint16_t value)> handler);

    /** 设置触屏事件回调：收到 TFT 触屏读请求（F2 payload.size==1）时调用，reg 为触屏寄存器地址 */
    void setTouchEventCallback(std::function<void(uint16_t reg)> handler);

    uint8_t stationAddr() const { return station_addr_; }

private:
    // void threadReadSerialData();
    void thread_read_serial_data();
    bool serial_data_analyse(TFT_PACK& pack);
    void setupProtocolSession();
    void stopProtocolSession();
    void handleDcBusFrame(const DcBusFrame& frame);

private:
    int _index;
    bool i_start_ = false;
    std::shared_ptr<SerialPort> _serial_port_ptr;
    ConnectionState connection_state_ = ConnectionState::DISCONNECTED;
    std::string device_address_;

    std::shared_ptr<ProtocolCodec> codec_;
    std::unique_ptr<SerialSession> session_;
    std::mutex session_mutex_;
    bool crc_enabled_ = true;
    uint16_t frame_header_ = 0xAA55;
    uint8_t station_addr_ = 0;

    std::function<void(uint16_t, uint16_t)> read_response_handler_{};
    std::function<void(uint16_t)> touch_event_handler_{};

    struct VerifyPending {
        bool active = false;
        bool finished = false;
        bool success = false;
        uint8_t addr = 0;
        uint16_t reg = 0;
        uint16_t expected = 0;
        uint16_t last_read = 0;
    };
    std::mutex verify_mtx_;
    std::condition_variable verify_cv_;
    VerifyPending verify_pending_{};

    uint64_t _last_timestamp = 0;
    TFT_PACK _lastest_hrvinfo{}; // 最近一次更新的 HEALTH 坐标

    char* _buff = nullptr;
    int _total_len = 0;
    uint64_t _frame_id = 0;
};

} // namespace sensor::gps