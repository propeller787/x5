#include "gps_module.h"

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <chrono>
#include "../../global.h"
#include "../../../common/utils/error_event.h"
#include "interface/msg/hex_data.hpp"
using namespace std;
namespace sensor::gps::cs8 {

static muduo::net::EventLoopThread GpsReceiverThread;

int find_sequence(const char* data, int data_len, const char* sequence, int sequence_len)
{
    for (int i = 0; i <= data_len - sequence_len; i++) {
        if (memcmp(&data[i], sequence, sequence_len) == 0) {
            return i; // 返回序列的起始位置
        }
    }
    return -1; // 如果没有找到，返回-1
}

bool GPSModule::serial_data_analyse()
{
    bool ret = true;
    int len = 0;
    char once_data[REC_BUFF_LEN];
    memset(once_data, 0, REC_BUFF_LEN * sizeof(char));
    len = _serial_port_ptr->read(once_data, REC_BUFF_LEN);
    if (len <= 0) {
        consecutive_read_failures_++;
        
        // 检查是否超时（超过指定时间没有成功读取）
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_success = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_successful_read_time_).count();
        
        if (time_since_last_success > TIMEOUT_SECONDS) {
            // 报告数据丢失错误（超时无数据）
            auto* global = sensor::Global::getInstance();
            if (global && global->node_ && current_error_code_ != GPS_DATA_LOSS) {
                interface::msg::HexData error_msg;
                error_msg.hex_value = encodeErrorLevel(GPS_DATA_LOSS, ErrorEventLevel::OCCUR);
                global->sendHexData(error_msg);
                current_error_code_ = GPS_DATA_LOSS; // 记录错误状态
            }
        }
        
        // 检查设备是否断连
        if (consecutive_read_failures_ >= MAX_READ_FAILURES) {
            auto* global = sensor::Global::getInstance();
            if (global && global->node_ && current_error_code_ != GPS_DISCONNECTED) {
                interface::msg::HexData error_msg;
                error_msg.hex_value = encodeErrorLevel(GPS_DISCONNECTED, ErrorEventLevel::OCCUR);
                global->sendHexData(error_msg);
                current_error_code_ = GPS_DISCONNECTED; // 记录错误状态
            }
        }
        
        return false;
    }
    
    // 读取成功，重置失败计数
    consecutive_read_failures_ = 0;
    last_successful_read_time_ = std::chrono::steady_clock::now();
    
    // 如果之前处于错误状态，现在恢复，发布恢复消息
    if (current_error_code_ != 0) {
        auto* global = sensor::Global::getInstance();
        if (global && global->node_) {
            interface::msg::HexData recover_msg;
            recover_msg.hex_value = encodeErrorLevel(current_error_code_, ErrorEventLevel::RECOVER);
            global->sendHexData(recover_msg);
            current_error_code_ = 0; // 清除错误状态
        }
    }
    // 查找并解析GNGGA数据
    char* gngga_pos = strstr(once_data, "$GNGGA");
    if (gngga_pos != nullptr) {
        char* end_pos = strchr(gngga_pos, '\r\n');
        if (end_pos != nullptr) {
            size_t len = end_pos - gngga_pos + 1;
            if (len < 50) {
                return false;
            }
            strncpy(_rtk_data, gngga_pos, len);
            _rtk_data[len] = '\0';

            // 解析GNGGA数据
            char* token = strtok(_rtk_data, ",");
            int field = 0;
            double latitude = 0.0;
            double longitude = 0.0;
            double altitude = 0.0;

            // 循环处理GNGGA消息中的每个字段
            while (token != nullptr) {
                switch (field) {
                case 2: // 纬度字段
                    if (strlen(token) > 0) {
                        // GNGGA中纬度格式为ddmm.mmmmm,需要转换为度
                        // 首先除以100得到度和分的小数形式
                        latitude = std::stod(token) / 100.0;
                        // 提取整数部分(度)
                        int degrees = (int)latitude;
                        // 将分转换为度的小数部分并加到整数度上
                        // (latitude - degrees)得到分的小数形式
                        // 乘以100/60将分转换为度
                        latitude = degrees + (latitude - degrees) * 100.0 / 60.0;
                    }
                    break;
                case 4: // 经度字段
                    if (strlen(token) > 0) {
                        // 经度转换过程与纬度相同
                        // GNGGA中经度格式为dddmm.mmmmm
                        longitude = std::stod(token) / 100.0;
                        int degrees = (int)longitude;
                        longitude = degrees + (longitude - degrees) * 100.0 / 60.0;
                    }
                    break;
                case 9: // 海拔高度字段(单位:米)
                    if (strlen(token) > 0) {
                        // 直接转换为double,无需其他处理
                        altitude = std::stod(token);
                    }
                    break;
                }
                // 获取下一个字段
                token = strtok(nullptr, ",");
                field++;
            }

            // 创建并发送NavSatFix消息
            auto now = muduo::Timestamp::now();
            auto messageGps = sensor_msgs::msg::NavSatFix();
            messageGps.header.stamp.sec = now.secondsSinceEpoch();
            messageGps.header.stamp.nanosec = (now.microSecondsSinceEpoch() % 1000000) * 1000;
            messageGps.set__latitude(latitude);
            messageGps.set__longitude(longitude);
            messageGps.set__altitude(altitude);
            messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            messageGps.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            if (latitude != 0.0 && longitude != 0.0) {
                cb_(messageGps);
                return true;
            }
        }
    }

    return false;
}

void GPSModule::setGpsCallback(const onGpsCallback& cb)
{
    cb_ = cb;
}

void GPSModule::thread_read_serial_data()
{
    while (true) {
        usleep(10000);
        bool ret = serial_data_analyse();
        if (ret) {
            IMUInfo imu_msg = { 0 };
            GPSData gps_data_cur = { 0 };

            /*Decode message*/
            // _decoder.decode_pvt(_pvt_data, gps_data_cur);
            // _decoder.decode_imu(_imu_data, imu_msg, gps_data_cur);
            _decoder.decode_rmc(_rmc_data, gps_data_cur);
            // _decoder.decode_rtk(_rtk_data, gps_data_cur);
            auto now = muduo::Timestamp::now();
            auto messageGps = sensor_msgs::msg::NavSatFix();
            messageGps.header.stamp.sec = now.secondsSinceEpoch();
            messageGps.header.stamp.nanosec = (now.microSecondsSinceEpoch() % 1000000) * 1000;
            messageGps.set__latitude(gps_data_cur.lla.latitude);
            messageGps.set__longitude(gps_data_cur.lla.longitude);
            messageGps.set__altitude(gps_data_cur.lla.height_over_sea);
            std::array<double, 9> arry;
            arry[0] = gps_pvt_.headMot * 0.00001;
            messageGps.set__position_covariance(arry);
            if (gps_data_cur.lla.latitude != 0 && gps_data_cur.lla.longitude != 0) {
                cb_(messageGps);
            }
            // // imucb_(messageImu);
            // // std::cout << "Sending one GPS message." << std::endl;
            // ++_index;
        }
    }
}

// TODO: add error reconnect process
GPSModule::GPSModule(std::string device_address, muduo::net::EventLoop* loop)
{
    _buff = new char[REC_BUFF_LEN];
    memset(_buff, 0, REC_BUFF_LEN * sizeof(char));
    _total_len = 0;
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

    _index = 1;

    _serial_port_ptr = std::make_shared<SerialPort>(device_address, gps_serial_port_option);
    bool ret = _serial_port_ptr->start();
    if (!ret) {
        std::cout << "串口初始化失败" << std::endl;
        // 报告设备未找到错误
        auto* global = sensor::Global::getInstance();
        if (global && global->node_) {
            interface::msg::HexData error_msg;
            error_msg.hex_value = encodeErrorLevel(GPS_NOT_FOUND, ErrorEventLevel::OCCUR);
            global->sendHexData(error_msg);
            current_error_code_ = GPS_NOT_FOUND; // 记录错误状态
        }
    } else {
        // 串口打开成功，初始化时间戳
        last_successful_read_time_ = std::chrono::steady_clock::now();
        
        // 如果之前处于错误状态，现在恢复，发布恢复消息
        if (current_error_code_ != 0) {
            auto* global = sensor::Global::getInstance();
            if (global && global->node_) {
                interface::msg::HexData recover_msg;
                recover_msg.hex_value = encodeErrorLevel(current_error_code_, ErrorEventLevel::RECOVER);
                global->sendHexData(recover_msg);
                current_error_code_ = 0; // 清除错误状态
            }
        }
    }
    auto pool = read_frame_loop_.startLoop();
    pool->runInLoop([=] {
        thread_read_serial_data();
    });

    // end of ctor
}

GPSModule::~GPSModule()
{
    if (_buff) {
        delete[] _buff;
    }
}

} // namespace sensor::gps
