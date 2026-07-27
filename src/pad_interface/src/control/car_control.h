#pragma once
#include "control.h"
#include <muduo/net/EventLoop.h>
#include <muduo/net/TimerId.h>
#include <boost/bind.hpp>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>

namespace service::control {

// 车辆控制相关常量定义
const float VX_MIN_VAL = 0.1; // 车辆前进方向最小速度阈值 (m/s)
const float VX_MAX_VAL = 6.0 / 3.6; // 车辆前进方向最大速度 (6 km/h 转换为 m/s)
const float W_MIN_VAL = 0.1; // 车辆旋转方向最小速度阈值 (rad/s)
const float W_MAX_VAL = 6.0 / 3.6; // 车辆旋转方向最大速度 (6 km/h 转换为 rad/s)

/**
 * @brief 车辆控制类，继承自Control基类
 * 负责处理车辆控制相关逻辑，包括底盘控制、车辆信息获取等
 */
class CarControl : public Control {

    /**
     * @brief 车辆信息结构体
     * 用于存储车辆当前状态信息
     */
    typedef struct carInformation {
        float vx; // 车辆前进速度 (m/s)
        float w; // 车辆旋转速度 (rad/s)
        int16_t speed; // 车辆当前速度 (km/h)
        int16_t electric; // 车辆电量百分比
        int16_t remaining; // 车辆剩余里程 (km)
        int16_t gear; // 车辆档位
        int16_t helpcar; // 车辆是否处于求助状态
        int16_t sos; // 车辆是否处于 SOS 状态
        int16_t turnswitch; // 车辆处于转弯 是否显示
        int16_t lighttype; // 车辆灯光类型
        int16_t lockswitch; // 车辆是否处于锁车状态
        int16_t health; // 车辆健康状态
        int16_t getcharging; // 车辆是否处于充电状态
    } CarInformation;

public:
    /**
     * @brief 构造函数
     * 初始化车辆控制服务
     */
    CarControl();
    ~CarControl();

private:
    /**
     * @brief 处理车辆控制请求
     * @param request 控制请求，包含控制指令
     * @param response 控制响应，返回执行结果
     */
    void handleCarControls(const std::shared_ptr<interface::srv::CarControls::Request> request,
        std::shared_ptr<interface::srv::CarControls::Response> response);

    /**
     * @brief 处理控制结果回调
     * @param msg 控制消息，包含车辆最新状态
     */
    void onControlResult(const interface::msg::Control msg);

private:
    // 底盘控制服务，用于接收控制指令
    rclcpp::Service<interface::srv::CarControls>::SharedPtr car_controls_ { nullptr };

    // 当前车辆状态信息
    CarInformation car_infor_ { 0 };

    // 控制指令发布器，用于发送控制指令到底盘
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pubs_;

    // 控制结果订阅器，用于接收底盘反馈信息
    rclcpp::Subscription<interface::msg::Control>::SharedPtr control_sub_ { nullptr };

    // 定时器，用于定时发送车辆状态信息
    muduo::net::EventLoop loop_;
    std::thread loop_thread_;
    muduo::net::TimerId timer_id_; // 定时器ID
    bool is_timer_running_ = false; // 定时器运行状态
    bool is_running_ = false;

    float vx_speed_ = 0.0; // 底盘前进速度
    float w_speed_ = 0.0; // 底盘旋转速度

    int16_t stop_index_ = 0;

    // 定时器回调函数
    void onTimer();
    // 启动定时器
    void startTimer();
    // 停止定时器
    void stopTimer();
    void joystickDistribution(double val, double angle, unsigned int gear, float& vx, float& w);
};

}