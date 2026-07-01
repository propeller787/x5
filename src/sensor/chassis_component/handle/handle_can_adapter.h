#pragma once
#include "../../global.h"
#include "../../include/define.h"
#include "../can_adapter.h"
#include "../can_transreceiver.h"
#include <interface/srv/bluetooth_mac.hpp>
#include <interface/msg/bluetooth_status.hpp>
#include "../dbc/odrive.h"
#include <muduo/net/EventLoop.h>
#include <thread>
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cstdint>
#include <unordered_set>
#include <atomic>
namespace sensor::chassis {
#define BLUETOOTH_CTRL_2 2   // 蓝牙控制模式
#define BLUETOOTH_CTRL_3 3   // 蓝牙控制模式
#define HANDLE_FRAME_ID (0x801400u)

typedef struct SystemVersionRequest {
    std::string hardware_version;
    std::string software_version;
    std::string version_data;
} SystemVersionRequest;

class HandleCanAdapter : public CanAdapter {
public:
    using ModeTypeCallback = std::function<sensor::ModeType()>;  // 获取模式类型的回调函数类型
    
    explicit HandleCanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs,
                             ModeTypeCallback mode_type_callback = nullptr);
    virtual void onReceiveCanMessage(const std::string& dev, int fid, muduo::net::Buffer& buffer) override;
    virtual void onReceiveGnssMessage(const interface::msg::Gnss::SharedPtr msg) override;
    virtual void onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) override;
    virtual void onRechargeStatusMessage(const interface::msg::RechargeStatus::SharedPtr msg) override;
    virtual void onRecieveSpeedbumpMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg) override;
    virtual void onRecievePaDaoMessage(const std::shared_ptr<std_msgs::msg::Int32> msg) override;
    virtual void sendCarInfo() override;
    virtual void sendBattery() override;

private:
    // 摇杆
    void onReceive8014Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive220Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive217Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    // void onReceive219Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive21aInfo(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive21cInfo(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive225Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive222Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive224Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive201Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive301Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive226Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive229Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive228Info(const std::string&, int fid, muduo::net::Buffer& buffer);

    void onReceive101Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive100Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    //错误描述信息
    void onReceive270Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive271Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive272Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive273Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive274Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive27FInfo(const std::string&, int fid, muduo::net::Buffer& buffer);

    void onReceive280Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive281Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive282Info(const std::string&, int fid, muduo::net::Buffer& buffer);

    void onReceive21DInfo(const std::string&, int fid, muduo::net::Buffer& buffer);
    void onReceive227Info(const std::string&, int fid, muduo::net::Buffer& buffer);
    void send90108();

    // 系统版本
    void handleUCVersionRequest(const std::shared_ptr<interface::srv::SystemVersion::Request> request,
                                std::shared_ptr<interface::srv::SystemVersion::Response> response);
    void handleDBVersionRequest(const std::shared_ptr<interface::srv::SystemVersion::Request> request,
                                std::shared_ptr<interface::srv::SystemVersion::Response> response);
    void handleBluetoothMacRequest(const std::shared_ptr<interface::srv::BluetoothMac::Request> request,
                                    std::shared_ptr<interface::srv::BluetoothMac::Response> response);
    void handleBluetoothStatusMessage(const interface::msg::BluetoothStatus::SharedPtr msg);

private:
    using CanMessageUnpackFunction = std::function<void(const std::string& dev, int fid, muduo::net::Buffer& buffer)>;
    std::map<int, CanMessageUnpackFunction> message_unpack_table_;
    interface::msg::Control control_info_;
    int i = 0;
    muduo::Timestamp last_backup_car_time_;
    bool is_backup_car_ = false;
    bool backup_car_ = true;
    interface::msg::CarInfo car_info_ {}; // 使用花括号进行值初始化
    size_t speep_dump_count_ = 0;
    size_t pa_dao_type_ = 0;
    bool speedbump_flag_ { false };
    bool podao_flag_ { false };
    bool uppd_flag_ { false };
    // 线充状态 原子操作
    std::atomic_bool line_charging_status_ { false };
    // 倒车状态 true 倒车 false 前进(静止)
    std::atomic_bool reverse_status_ { false };
    // 有速度时为true 原子操作
    std::atomic_bool speed_status_ { true };
    // 获取模式类型的回调函数
    ModeTypeCallback mode_type_callback_;
    rclcpp::Service<interface::srv::SystemVersion>::SharedPtr uc_version_service_;
    SystemVersionRequest uc_version_request_;
    rclcpp::Service<interface::srv::SystemVersion>::SharedPtr db_version_service_;
    SystemVersionRequest db_version_request_;

    // 接收蓝牙mac地址
    rclcpp::Service<interface::srv::BluetoothMac>::SharedPtr bluetooth_mac_service_;
    std::string bluetooth_mac_ {""};

    // 接收蓝牙状态 bluetoothStatusTopic
    rclcpp::Subscription<interface::msg::BluetoothStatus>::SharedPtr bluetooth_status_sub_;
    // 基站充电状态 true 充电 false 未充电
    bool charging_status_ { false };
    // 电压
    int16_t voltage_ { 0 };

    // CAN数据发布器 - 用于实时发布CAN数据到话题供recorder订阅
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;
    // 帧 ID 白名单（CAN_EFF_MASK 后的 ID）。空 = 发布所有；非空 = 仅发布列表中的 ID。
    // 通过参数 record.can_publish_frame_ids（int 数组）配置。
    std::unordered_set<uint32_t> can_publish_id_filter_;
    bool can_publish_filter_enabled_{false};
    // 节流统计：每秒丢弃数（被过滤）与发布数，仅做采样日志。
    std::atomic<uint32_t> can_publish_count_{0};
    std::atomic<uint32_t> can_publish_dropped_{0};

    // 发布CAN数据到话题
    void publishCanFrame(const std::string& dev, int fid, muduo::net::Buffer& buffer);

    /** 左右轮编码器计数更新后发布 /car/encoder 并积分里程计 */
    void publishEncoderAndOdometry();
    void updateWheelOdometryFromEncoders();
    static int32_t encoderCountToMsgField(int64_t raw);
    static int64_t encoderDeltaCounts(int32_t current, int32_t previous);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;

    bool wheel_odom_enabled_{false};
    double wheel_radius_m_{0.126};
    double wheel_separation_m_{0.58};
    double counts_per_revolution_{278511.0};
    double m_per_count_{0.0};  // (2*pi*radius) / counts_per_rev 预计算
    std::string wheel_odom_frame_id_;
    std::string wheel_odom_child_frame_id_;

    int32_t latest_left_count_{0};
    int32_t latest_right_count_{0};
    bool encoder_valid_{false};

    int32_t prev_left_count_{0};
    int32_t prev_right_count_{0};
    bool wheel_odom_initialized_{false};

    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_theta_{0.0};
    rclcpp::Time last_wheel_odom_stamp_;
    bool have_wheel_odom_last_stamp_{false};

    // EMA 滤波后的速度
    double filtered_vx_{0.0};
    double filtered_omega_{0.0};

    // 时变协方差累积量（随行驶距离增长）
    double pose_cov_xx_{0.0};
    double pose_cov_yy_{0.0};
    double pose_cov_yaw_{0.0};
};

}