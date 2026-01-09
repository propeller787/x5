#pragma once

#include "tft_decoder.h"
#include "tft_helper.h"
#include <functional>
#include <iostream>
#include <memory>

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

    TftModule(std::string device_address);
    ~TftModule();
    void startWrite(char* buffer, int buf_len);
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

private:
    // void threadReadSerialData();
    void thread_read_serial_data();
    bool serial_data_analyse(TFT_PACK& pack);

private:
    int _index;
    bool i_start_ = false;
    std::shared_ptr<SerialPort> _serial_port_ptr;
    TftDecoder _decoder;
    ConnectionState connection_state_ = ConnectionState::DISCONNECTED;
    std::string device_address_;

    uint64_t _last_timestamp = 0;
    TFT_PACK _lastest_hrvinfo = { 0 }; // 最近一次更新的 HEALTH 坐标

    char* _buff = nullptr;
    int _total_len = 0;
    uint64_t _frame_id = 0;
};

} // namespace sensor::gps