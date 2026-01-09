#pragma once

#include "dbc/odrive.h"
#include "interface/msg/backup_car.hpp"
#include "interface/msg/car_info.hpp"
#include "interface/msg/car_information.hpp"
#include "interface/msg/colour_screen.hpp"
#include "interface/msg/control.hpp"
#include "interface/msg/encoder.hpp"
#include "interface/msg/gear.hpp"
#include "interface/msg/gnss.hpp"
#include "interface/msg/health.hpp"
#include "interface/msg/health_start.hpp"
#include "interface/msg/objects.hpp"
#include "interface/msg/recharge_status.hpp"
#include "interface/msg/joystick.hpp"
#include "interface/srv/gears_controls.hpp"
#include "interface/srv/mode_enable.hpp"
#include "interface/srv/tft_controls.hpp"
#include "muduo/net/Buffer.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <interface/srv/key_value_int.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <utils/topic.h>
#include <visualization_msgs/msg/marker_array.hpp>
const float VX_MAX_VAL = 6.0 / 3.6;
// const float VX_MAX_VAL = 0.18;
namespace sensor::chassis {

class CanTransreceiver;

class CanAdapter : public rclcpp::Node {
  public:
    CanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs, const std::string &adapter_name);
    void canSend(std::string dev, int frameid, muduo::net::Buffer &buf);
    void canSendLoog(std::string dev, int frameid);

  public:
    virtual void sendCarInfo() = 0;
    virtual void sendBattery() = 0;

  private:
    virtual void onReceiveCanMessage(const std::string &dev, int fid, muduo::net::Buffer &buffer) = 0;
    virtual void onReceiveGnssMessage(const interface::msg::Gnss::SharedPtr msg) = 0;
    virtual void onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;
    virtual void onRechargeStatusMessage(const interface::msg::RechargeStatus::SharedPtr msg) = 0;
    void onReceiveHealthMessage(const interface::msg::Health::SharedPtr msg);
    void onReceiveNarrowMessage(const std_msgs::msg::Empty::SharedPtr msg);
    virtual void onRecieveSpeedbumpMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg) = 0;
    virtual void onRecievePaDaoMessage(const std::shared_ptr<std_msgs::msg::Int32> msg) = 0;
    // void onReceiveColourScreenMessage(const interface::msg::ColourScreen::SharedPtr msg);
    // void onReceiveGearMessage(const interface::msg::Gear::SharedPtr msg);

    void onReceiveAudioMessage(const std_msgs::msg::Int32::SharedPtr msg);

  public:
    /**
     * @brief 限幅 int 型
     *
     * @param input 输入
     * @param max_ouput 最大输出值
     * @return ** int 限幅结果
     */
    int output_limit(int input, int max_ouput);
    void speedDistribution(float vx, float w, int &pcmd_a, int &pcmd_b);
    // 转换为前进速度 底盘旋转速度
    /**
     * @brief 底盘速度分配函数
     *
     * @param[in] cmd_a 左轮速度 CAN 下发指令值
     * @param[in] cmd_b 右轮速度 CAN 下发指令值
     * @param[out] vx 底盘前进速度 m/s
     * @param[out] w 底盘旋转速度 rad/s
     * @return ** void
     */
    void distributionSpeed(int cmd_a, int cmd_b, float &pvx, float &pw);

    /** 自研
     * *@brief底盘速度分配函数**
     * @param[in]vx底盘前进速度m/s*
     * @param[in]w底盘旋转速度rad/s*
     * @param[out]cmd_a左轮速度CAN下发指令值*
     * @param[out]cmd_b右轮速度CAN下方指令值*
     * @return**void
     */
    void ZqSpeedDistribution(float vx, float w, float &cmd_a, float &cmd_b);
    void ZqDistributionSpeed(float cmd_a, float cmd_b, float &pvx, float &pw);

  protected:
    std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> can_devs_;
    rclcpp::Logger logger_;
    double vx_max_val_; // 添加成员变量
    std::unordered_map<std::string, rclcpp::Publisher<interface::msg::Objects>::SharedPtr> radar_obj_pubs_;
    // std::unordered_map<std::string, rclcpp::Subscription<interface::msg::Gnss>::SharedPtr> gnss_subs_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pubs_;
    rclcpp::Publisher<interface::msg::Control>::SharedPtr control_car_pubs_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr control_car_spubs_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_subs_;
    rclcpp::Publisher<interface::msg::Encoder>::SharedPtr control_car_encoder_pubs_;
    rclcpp::Publisher<interface::msg::BackupCar>::SharedPtr backup_car_pubs_;

    rclcpp::Subscription<interface::msg::ColourScreen>::SharedPtr colour_screen_subs_;
    rclcpp::Subscription<interface::msg::Gear>::SharedPtr gear_subs_;
    // 上控健康计
    rclcpp::Publisher<interface::msg::HealthStart>::SharedPtr health_pubs_;
    // 上控健康数据接收
    rclcpp::Subscription<interface::msg::Health>::SharedPtr health_subs_;
    // 发送车辆信息
    rclcpp::Publisher<interface::msg::CarInfo>::SharedPtr car_info_pubs_;
    rclcpp::Subscription<interface::msg::RechargeStatus>::SharedPtr recharge_status_subs_;
    // 减速带数据接收
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr speedbump_subs_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr podao_subs_;
    // 减速带事件播报
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speedbump_pubs_;

    // 下坡事件播报
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr podao_pubs_;
    // 上坡事件播报
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr uppd_pubs_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr narrow_sub_;

    // 设置状态
    rclcpp::Client<interface::srv::KeyValueInt>::SharedPtr mode_enable_client_;
    // 音频发送
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr audio_sub_;
    // 舞蹈秀开始
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dance_puts_;
    // 摇杆
    rclcpp::Publisher<interface::msg::Joystick>::SharedPtr joystick_pubs_;
};

} // namespace sensor::chassis