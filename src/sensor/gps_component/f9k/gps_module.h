/*
 * @copyright: Copyright (c) 2024 zhengqi.com, Inc. All Rights Reserved
 * @Author: likang(likang@zhengqi.com)
 * @Date: 2024-01-05 16:32:38
 * @FilePath: mem/src/sensor/gps_component/f9k/gps_module.h
 * @brief:
 */
#pragma once
// #include "../serial_port/serial_port.h"
#include "gps_decoder.h"
#include "gps_helper.h"
#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>
#include <memory>
// #include <muduo/net/EventLoopManager.h>
#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <utils/serial_port.h>
#define REC_BUFF_LEN 1792
#define BUFFER_128 128
#define BUFFER_69 69
#define BUFFER_75 75
#define GPSREADINTERVAL 5000

#ifdef __UNIT_TEST__
#define private public
#define protected public
#endif
using namespace utils;
using namespace std::placeholders;
namespace sensor::gps {

class GPSModule {
protected:
    // 优化：使用const引用避免不必要的拷贝
    using onGpsCallback = std::function<void(const sensor_msgs::msg::NavSatFix&)>;
    using onImuCallback = std::function<void(const sensor_msgs::msg::Imu&)>;

public:
    GPSModule(std::string device_address, muduo::net::EventLoop* loop);
    ~GPSModule();
    void setGpsCallback(const onGpsCallback& cb);
    void setImuCallback(const onImuCallback& cb);
    void rtkWrite(const char* buff, size_t len);

private:
    // void threadReadSerialData();
    void thread_read_serial_data();

private:
    int _index;

    muduo::net::EventLoopThread read_frame_loop_;

    std::shared_ptr<SerialPort> _serial_port_ptr;
    GPSDecoder _decoder;

    bool serial_data_analyse();

    float _distance_total = 0.0; // 车辆行驶的总距离
    float _travel_distance = 0.0; // 基于速度计算的车辆行驶距离（与上一帧之间）
    // int m_interval_distance = 5;                    //采样区间距离

    uint64_t _last_timestamp = 0;
    GPSData _lastest_gpsinfo = { 0 }; // 最近一次更新的 GPS 坐标
    // Eigen::Quaterniond m_qwb(0,0,0,0);         //四元数,供坐标系转换使用
    // Eigen::Vector3d m_postion(0,0,0);          //IMU系统中 惯导的位移
    // Eigen::Vector3d m_velocity(0,0,0);         //IMU系统中 惯导的速度
    // Eigen::Vector3d m_acc(0,0,0);              //最近一次的3轴加速度
    // Eigen::Vector3d m_gyr(0,0,0);              //最近一次的3轴角速度
    // Eigen::Vector3d m_acc_bias(0,0,0);         //当前的加速度偏差(bias)
    // Eigen::Vector3d m_gyr_bias(0,0,0);         //当前的角速度偏差(bias)

    char _imu_data[BUFFER_128] = { 0 };
    char _pvt_data[BUFFER_128] = { 0 };
    char _rmc_data[BUFFER_128] = { 0 };
    char _rtk_data[BUFFER_128] = { 0 };
    char* _buff = nullptr;
    int _total_len = 0;
    uint64_t _frame_id = 0;
    onGpsCallback cb_;
    onImuCallback imucb_;
    GPSPVT gps_pvt_ { 0 };
    // QxwzSdkDemo rtk_ = {};
};

} // namespace sensor::gps