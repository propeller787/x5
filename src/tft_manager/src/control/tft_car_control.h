#pragma once
#include "control.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/TimerId.h"
#include <boost/bind.hpp>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <interface/msg/car_info.hpp>
#include <thread>

namespace tft_service::control {
/**
 * @brief 车辆控制类，继承自Control基类
 * 负责处理车辆控制相关逻辑，包括底盘控制、车辆信息获取等
 */
class CarControl : public Control {

public:
    /**
     * @brief 构造函数
     * 初始化车辆控制服务
     */
    CarControl();
    ~CarControl();

private:
    void backupCarCallback(const interface::msg::BackupCar::SharedPtr msg);

private:
    void handleCarInfo(const interface::msg::CarInfo msg);
    // 发送停车波浪
    void sendParkingWave();
    // 发送剩余里程
    void sendRemainingMileage();
    // 发送日期
    void sendDate();
    // 发送时间
    void sendTime();
    // 发送档位
    void sendGear();
    // 发送网络
    void sendNetworkStatus();
    // 发送温度
    void sendTemperature();
    // 发送电量
    void sendElectricQuantity();

    std::vector<char> floatToHexBytes(float value);
    void sendSpeedKmh();

    void sendOpenCamera();
    void sendCloseCamera();
    void sendChargingPage();

private:
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_subs_;
    rclcpp::Subscription<interface::msg::BackupCar>::SharedPtr backup_car_subs_;
    interface::msg::CarInfo car_info_ {};
    int16_t parking_wave_level_ = 0;
    int16_t charging_page_level_ = 0;
    bool low_battery_10_ = true; // 是否低电量
    bool low_battery_20_ = true; // 是否低电量
};

}