#include "tft_module.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>

using namespace std;

namespace tft_service {

bool TftModule::serial_data_analyse(TFT_PACK& pack)
{
    bool ret = true;
    int len = 0;
    char once_data[REC_BUFF_LEN];
    memset(once_data, 0, REC_BUFF_LEN * sizeof(char));
    len = _serial_port_ptr->read(once_data, REC_BUFF_LEN);
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
        
        TFT_PACK pack { 0 };
        bool ret = serial_data_analyse(pack);
        if (ret) {
            // TODO: 处理接收到的数据包
        }
    }
}

TftModule::TftModule(std::string device_address)
    : device_address_(device_address)
{
    _total_len = 0;
    connection_state_ = ConnectionState::DISCONNECTED;
    _index = 1;
    
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
void TftModule::startWrite(char* buffer, int buf_len)
{
    if (connection_state_ != ConnectionState::CONNECTED || !_serial_port_ptr) {
        RCLCPP_WARN(rclcpp::get_logger("TftModule"), "串口未连接，无法发送数据");
        return;
    }
    
    _serial_port_ptr->write(buffer, buf_len);
}
TftModule::~TftModule()
{
}

}