#include "handle_can_adapter.h"
#include "../chassis/chassis_can_adapter.h"
#include "../../global.h"
#include "sensor_utils.hpp"
#include <linux/can.h>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace sensor::chassis;
using namespace sensor::utils;

namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

HandleCanAdapter::HandleCanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs,
                                   ModeTypeCallback mode_type_callback) 
    : CanAdapter(devs, "handle"), mode_type_callback_(mode_type_callback) {
    car_info_.vx = 0;
    car_info_.w = 0;
    car_info_.speed = 0;
    car_info_.electricquantity = 0;
    car_info_.remainingmileage = 0;
    car_info_.gear = 1;
    car_info_.helpcar = 0;
    car_info_.sos = 0;
    car_info_.turnswitch = -1;
    car_info_.lighttype = -1;
    car_info_.lockswitch = 0;
    car_info_.health = -1;
    car_info_.getcharging = -1;
    using namespace std::placeholders;
    message_unpack_table_ = {
        {HANDLE_FRAME_ID, std::bind(&HandleCanAdapter::onReceive8014Info, this, _1, _2, _3)},
        {ODRIVE_TOP_INPUT_VEL_FRAME_ID, std::bind(&HandleCanAdapter::onReceive220Info, this, _1, _2, _3)},
        {ODRIVE_VBUS_VOLTAGE_FRAME_ID, std::bind(&HandleCanAdapter::onReceive217Info, this, _1, _2, _3)},
        //{ ODRIVE_INPUT_VEL_FRAME_ID, std::bind(&HandleCanAdapter::onReceive219Info, this, _1, _2, _3) },
        {ODRIVE_COUNT_FRAME_ID, std::bind(&HandleCanAdapter::onReceive21aInfo, this, _1, _2, _3)},
        {ODRIVE_VEL_ESTIMATE_FRAME_ID, std::bind(&HandleCanAdapter::onReceive21cInfo, this, _1, _2, _3)},
        {ODRIVE_KEY_STATUS_FRAME_ID, std::bind(&HandleCanAdapter::onReceive225Info, this, _1, _2, _3)},
        {ODRIVE_COLOUR_SCREEN_FRAME_ID, std::bind(&HandleCanAdapter::onReceive224Info, this, _1, _2, _3)},
        {ODRIVE_VES_INFOR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive222Info, this, _1, _2, _3)},
        {ODRIVE_CELL_FRAME_ID, std::bind(&HandleCanAdapter::onReceive101Info, this, _1, _2, _3)},
        {ODRIVE_CELL_PACK_FRAME_ID, std::bind(&HandleCanAdapter::onReceive100Info, this, _1, _2, _3)},
        {ODRIVE_LEFT_DRIVER_FRAME_ID, std::bind(&HandleCanAdapter::onReceive201Info, this, _1, _2, _3)},
        {ODRIVE_RIGHT_DRIVER_FRAME_ID, std::bind(&HandleCanAdapter::onReceive301Info, this, _1, _2, _3)},
        {ODRIVE_ROCKER_FRAME_ID, std::bind(&HandleCanAdapter::onReceive226Info, this, _1, _2, _3)},

        {ODRIVE_AXLE_1_ERROR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive270Info, this, _1, _2, _3)},
        {ODRIVE_AXLE_2_ERROR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive271Info, this, _1, _2, _3)},
        {ODRIVE_BRAKE_ERROR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive272Info, this, _1, _2, _3)},
        {ODRIVE_BRAKE_ERROR1_FRAME_ID, std::bind(&HandleCanAdapter::onReceive273Info, this, _1, _2, _3)},
        {ODRIVE_BRAKE_SHOE_FRAME_ID, std::bind(&HandleCanAdapter::onReceive274Info, this, _1, _2, _3)},
        {ODRIVE_DETAILED_ERRORS_FRAME_ID, std::bind(&HandleCanAdapter::onReceive27FInfo, this, _1, _2, _3)},

        {ODRIVE_UPPER_STATUS_ERROR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive280Info, this, _1, _2, _3)},
        {ODRIVE_ROCKER_ERROR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive281Info, this, _1, _2, _3)},
        {ODRIVE_MOTOR_ERROR_FRAME_ID, std::bind(&HandleCanAdapter::onReceive282Info, this, _1, _2, _3)},
        {ODRIVE_DB_VERSION_FRAME_ID, std::bind(&HandleCanAdapter::onReceive21DInfo, this, _1, _2, _3)},
        {ODRIVE_UC_VERSION_FRAME_ID, std::bind(&HandleCanAdapter::onReceive227Info, this, _1, _2, _3)},

        {ODRIVE_UP_SETTING_FRAME_ID, std::bind(&HandleCanAdapter::onReceive229Info, this, _1, _2, _3)},
        {ODRIVE_BLUETOOTH_MAC_FRAME_ID, std::bind(&HandleCanAdapter::onReceive228Info, this, _1, _2, _3)},
    };
    std::map<uint16_t, unsigned int> canIdMap = {
        {HANDLE_FRAME_ID, CAN_EFF_MASK},
        {ODRIVE_TOP_INPUT_VEL_FRAME_ID, CAN_EFF_MASK},
    };
    
    uc_version_service_ = this->create_service<interface::srv::SystemVersion>(UCVersionService, std::bind(&HandleCanAdapter::handleUCVersionRequest, this, _1, _2));
    db_version_service_ = this->create_service<interface::srv::SystemVersion>(DBVersionService, std::bind(&HandleCanAdapter::handleDBVersionRequest, this, _1, _2));
    bluetooth_mac_service_ = this->create_service<interface::srv::BluetoothMac>(bluetoothMacService, std::bind(&HandleCanAdapter::handleBluetoothMacRequest, this, _1, _2));

    // /can/frames 发布器：CAN 总线 500-1000Hz，对录制非关键帧使用 best_effort + KEEP_LAST(50)，
    // 避免 reliable QoS 在订阅端阻塞时反向卡住 muduo 接收线程；丢帧由 best_effort 容忍。
    rclcpp::QoS can_frames_qos = rclcpp::SensorDataQoS().keep_last(50);
    can_frame_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/can/frames", can_frames_qos);

    // 帧 ID 白名单：通过参数 record.can_publish_frame_ids 注入；为空 = 发布全部。
    // 默认放空数组，向后兼容；运行时可通过 ros2 param set 调整。
    if (!this->has_parameter("record.can_publish_frame_ids")) {
        this->declare_parameter<std::vector<int64_t>>("record.can_publish_frame_ids", std::vector<int64_t>{});
    }
    auto filter_list = this->get_parameter("record.can_publish_frame_ids").as_integer_array();
    for (auto v : filter_list) {
        can_publish_id_filter_.insert(static_cast<uint32_t>(v) & CAN_EFF_MASK);
    }
    can_publish_filter_enabled_ = !can_publish_id_filter_.empty();

    bluetooth_status_sub_ = this->create_subscription<interface::msg::BluetoothStatus>(bluetoothStatusTopic, 10, std::bind(&HandleCanAdapter::handleBluetoothStatusMessage, this, _1));
    RCLCPP_INFO(logger_, "CAN data publisher created on /can/frames (SensorDataQoS, KEEP_LAST(50)), filter %s, %zu ids",
                can_publish_filter_enabled_ ? "ENABLED" : "DISABLED",
                can_publish_id_filter_.size());

    this->declare_parameter("wheel_odom_wheel_radius_m", 0.126);
    this->declare_parameter("wheel_odom_wheel_separation_m", 0.612);
    // 16383 * 17 编码器计数 / 圈
    this->declare_parameter("wheel_odom_counts_per_revolution", 278511.0);
    this->declare_parameter("wheel_odom_frame_id", std::string("odom"));
    this->declare_parameter("wheel_odom_child_frame_id", std::string("base_link"));
    wheel_radius_m_ = this->get_parameter("wheel_odom_wheel_radius_m").as_double();
    wheel_separation_m_ = this->get_parameter("wheel_odom_wheel_separation_m").as_double();
    counts_per_revolution_ = this->get_parameter("wheel_odom_counts_per_revolution").as_double();
    m_per_count_ = (2.0 * kPi * wheel_radius_m_) / counts_per_revolution_;
    wheel_odom_frame_id_ = this->get_parameter("wheel_odom_frame_id").as_string();
    wheel_odom_child_frame_id_ = this->get_parameter("wheel_odom_child_frame_id").as_string();
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(wheelOdomTopic, 10);
    RCLCPP_INFO(logger_, "Wheel odometry publisher on %s (radius=%.4fm separation=%.4fm counts/rev=%.1f)",
                wheelOdomTopic, wheel_radius_m_, wheel_separation_m_, counts_per_revolution_);
}

void HandleCanAdapter::handleUCVersionRequest(const std::shared_ptr<interface::srv::SystemVersion::Request> request,
                                std::shared_ptr<interface::srv::SystemVersion::Response> response) {
    response->hardware_version = uc_version_request_.hardware_version;
    response->software_version = uc_version_request_.software_version;
    response->version = uc_version_request_.version_data;
}

void HandleCanAdapter::handleDBVersionRequest(const std::shared_ptr<interface::srv::SystemVersion::Request> request,
                                std::shared_ptr<interface::srv::SystemVersion::Response> response) {
    response->hardware_version = db_version_request_.hardware_version;
    response->software_version = db_version_request_.software_version;
    response->version = db_version_request_.version_data;
}

void HandleCanAdapter::handleBluetoothMacRequest(const std::shared_ptr<interface::srv::BluetoothMac::Request> request,
                                std::shared_ptr<interface::srv::BluetoothMac::Response> response) {
    response->bluetooth_mac = bluetooth_mac_;
    response->code = 0;
    response->message = "success";
}

void HandleCanAdapter::onReceiveCanMessage(const std::string &dev, int fid, muduo::net::Buffer &buffer) {
    // 发布CAN数据到话题供recorder订阅
    publishCanFrame(dev, fid, buffer);
    
    auto iter = message_unpack_table_.find(fid);
    if (iter == message_unpack_table_.end()) {
        // RCLCPP_WARN(logger_, "No Handle to unpack frame %04x", fid);
        return;
    }

    iter->second(dev, fid, buffer);
}
void HandleCanAdapter::onReceiveGnssMessage(const interface::msg::Gnss::SharedPtr msg) {}

int16_t parseTwoBytesToInt16(uint8_t byte1, uint8_t byte2) {
    // Function definition
    return (byte1 << 8) | byte2;
}

// 701
void HandleCanAdapter::onReceive8014Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    // return;
    // RCLCPP_INFO(logger_, "onReceive8014Info --- %04x %d %s", fid, buffer.readableBytes(),
    //     encodeToHexString(buffer.toStringPiece().as_string()).c_str());
    const uint8_t *data = reinterpret_cast<const uint8_t *>(buffer.peek());
    auto now = muduo::Timestamp::now();
    // D0 左轮速度值低8位 D1 左轮速度值高8位 D2 右轮速度值低8位 D3 右轮速度值高8位
    uint8_t lowByte = data[0];
    uint8_t highByte = data[1];
    int16_t leftData = parseTwoBytesToInt16(highByte, lowByte);

    lowByte = data[2];
    highByte = data[3];
    int16_t hightData = parseTwoBytesToInt16(highByte, lowByte);

    float pvx, pw = 0.0;
    distributionSpeed(leftData, hightData, pvx, pw);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = pvx;
    twist.angular.z = pw;
    control_pubs_->publish(twist);
}
// 假设我们有一个简化的电压-电量映射表（这些值是根据经验得到的，并不准确）
std::map<double, double> voltageToCapacityMapH = {
    {3.2 * 7, 0.0},  // 当电压低于3.2V时，认为电量为0%
    {3.4 * 7, 10.0}, // 电压3.4V对应10%电量
    {3.6 * 7, 30.0}, // 电压3.6V对应30%电量
    {3.7 * 7, 70.0}, // 电压3.7V对应70%电量
    {3.8 * 7, 90.0}, // 额定电压3.8V对应90%电量
    {4.2 * 7, 100.0} // 当电压高于4.0V时（通常不会这样），认为电量为100%
};

// 线性插值函数，用于估算电压和映射表中两个点之间的电量
double linearInterpolateH(double x, const std::pair<double, double> &p1, const std::pair<double, double> &p2) {
    return p1.second + (x - p1.first) * (p2.second - p1.second) / (p2.first - p1.first);
}

// 根据电压估算电量的函数
double estimateCapacityFromVoltageH(double voltage) {
    // 找到电压所在的区间
    auto it = voltageToCapacityMapH.lower_bound(voltage);

    if (it == voltageToCapacityMapH.begin()) {
        // 电压低于映射表中的最小值
        return 0.0;
    } else if (it == voltageToCapacityMapH.end()) {
        // 电压高于映射表中的最大值
        return 100.0;
    } else {
        // 电压位于两个映射点之间，进行线性插值
        auto prev = std::prev(it);
        return linearInterpolateH(voltage, *prev, *it);
    }
}
void HandleCanAdapter::onReceive217Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_vbus_voltage_t obj;
    if (buffer.readableBytes() < 4) {
        return;
    }
    int retv = odrive_vbus_voltage_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    double estimeat = estimateCapacityFromVoltageH(obj.vbus_voltage1);
    if ((int)estimeat > 10 && (int)estimeat < 100) {
        estimeat = ((int)estimeat / 10 + 1) * 10;
    }
    // car_info_.electricquantity = (int)estimeat;

    // std::cout << " HandleCanAdapter::onReceive217Info - electricquantity " << car_info_.electricquantity << std::endl;
}

void HandleCanAdapter::onReceive21aInfo(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_count_t obj;
    int retv = odrive_count_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    (void)retv;
    latest_left_count_ = obj.count0;
    latest_right_count_ = obj.count1;
    encoder_valid_ = true;
    publishEncoderAndOdometry();
}
void HandleCanAdapter::onReceive21cInfo(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_vel_estimate_t obj;
    int retv = odrive_vel_estimate_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    float pvx, pw = 0.0;
    ZqDistributionSpeed(obj.vel_estimate0, obj.vel_estimate1, pvx, pw);
    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";
    message.twist.linear.x = pvx;
    message.twist.linear.y = 0.0;
    message.twist.linear.z = 0.0;
    message.twist.angular.x = 0.0;
    message.twist.angular.y = 0.0;
    message.twist.angular.z = pw;
    if (pvx <= 0.0) {
        speed_status_ = false;
    } else {
        speed_status_ = true;
    }
    // car_info_.vx = std::abs(message.twist.linear.x) > 0.001 ? 1 : 0;
    // car_info_.w = std::abs(message.twist.angular.z) > 0.001 ? 1 : 0;
    car_info_.speedkmh = std::abs(message.twist.linear.x * 3.6); // 转换为km/h
    car_info_.speed = car_info_.speedkmh;
    //
    // std::cout << " HandleCanAdapter::onReceive21cInfo car_info_.speedkmh " << car_info_.speedkmh << std::endl;
    control_car_spubs_->publish(message);
}

void HandleCanAdapter::onReceive220Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_top_input_vel_t obj;
    int retv = odrive_top_input_vel_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    float pvx, pw = 0.0;
    ZqDistributionSpeed(obj.input_vel0, obj.input_vel1, pvx, pw);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = pvx;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = pw;
    control_pubs_->publish(twist);
    // std::cout << " -----------twist.linear.x -----------" << std::fixed << std::setprecision(3) << twist.linear.x << std::endl;
    if (twist.linear.x < -0.01 && !backup_car_) {
        if (!is_backup_car_) {
            last_backup_car_time_ = muduo::Timestamp::now();
            is_backup_car_ = true;
        } else {
            auto now = muduo::Timestamp::now();
            if (muduo::timeDifference(now, last_backup_car_time_) > 0.1) {
                auto message = interface::msg::BackupCar();
                message.header.stamp = this->now();
                message.header.frame_id = "base_link";
                message.is_backup_car = true;
                backup_car_pubs_->publish(message);
                backup_car_ = true;
                is_backup_car_ = false;
            }
        }
    } else if (twist.linear.x >= -0.01 && backup_car_) {
        if (!is_backup_car_) {
            last_backup_car_time_ = muduo::Timestamp::now();
            is_backup_car_ = true;
        } else {
            auto now = muduo::Timestamp::now();
            if (muduo::timeDifference(now, last_backup_car_time_) > 0.1) {
                auto message = interface::msg::BackupCar();
                message.header.stamp = this->now();
                message.header.frame_id = "base_link";
                message.is_backup_car = false;
                backup_car_pubs_->publish(message);
                backup_car_ = false;
                is_backup_car_ = false;
            }
        }
    }
}

void HandleCanAdapter::onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float pcmda, pcmdb = 0.0;
    // 过滤最大速度 不超过6迈
    // if (msg->linear.x > 1.667) {
    //     msg->linear.x = 1.666;
    // }
    // if (speep_dump_count_ > 0) {
    //     speep_dump_count_--;
    //     if (speep_dump_count_ == 0) {
    //         speedbump_flag_ = false;
    //         send90108();
    //     }
    //     if (msg->linear.x > 0.4) {
    //         msg->linear.x = 0.4;
    //     }
    // }
    // if (podao_flag_ && pa_dao_type_ == 100) {
    //     podao_flag_ = false;
    //     send90108();
    // }
    // if (uppd_flag_ && pa_dao_type_ == 100) {
    //     uppd_flag_ = false;
    // }
    // if (pa_dao_type_ == 104 || pa_dao_type_ == 102) {
    //     if (msg->linear.x > 0.4) {
    //         msg->linear.x = 0.3;
    //     }
    //     if (std::fabs(msg->angular.z) > 0.4) {
    //         msg->angular.z = (msg->angular.z > 0 ? 0.3 : -0.3);
    //     }
    // }
    // msg->linear.x = 0.5;
    ZqSpeedDistribution(msg->linear.x, msg->angular.z, pcmda, pcmdb);
    odrive_input_vel_t velt;
    velt.input_vel0 = pcmda;
    velt.input_vel1 = pcmdb;
    uint8_t dstp[8];
    if (int ret = odrive_input_vel_pack(dstp, &velt, sizeof(dstp))) {
        muduo::net::Buffer buffer;
        buffer.append(dstp, ret);
        canSend("can0", (ODRIVE_INPUT_VEL_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
}

void HandleCanAdapter::onReceive225Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_key_status_t obj;
    int retv = odrive_key_status_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    if (car_info_.sos != obj.sos || car_info_.helpcar != obj.help_implement || car_info_.health != obj.health) {
        car_info_.sos = obj.sos;
        car_info_.helpcar = obj.help_implement;
        car_info_.health = obj.health;
        car_info_pubs_->publish(car_info_);
    }
    if(obj.help_implement == 1){
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 1;
        auto response = scene_helpcar_client_->async_send_request(request);
    }else if(obj.help_implement == 0){
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 0;
        auto response = scene_helpcar_client_->async_send_request(request);
    }
    // 获取当前模式类型（如果回调函数可用）
    sensor::ModeType mode_type = sensor::ModeType::MODE_TYPE_STANDARD;
    if (mode_type_callback_) {
        mode_type = mode_type_callback_();
        RCLCPP_DEBUG(logger_, "当前模式类型: %d", mode_type);
    }
    if((int)obj.key_auxiliary >=0){
        std_msgs::msg::Int32 message;
        message.data = (int)obj.key_auxiliary;
        safety_function_pubs_->publish(message);
        /*
        if(car_info_.left_brakests == (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN || 
            car_info_.right_brakests == (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN){
            std_msgs::msg::Int32 message;
            message.data = int(sensor::PopupType::POPUP_TYPE_CLOSE_HAND_BRAKE);
            popup_pubs_->publish(message);
            return;
        }
        if(mode_type == sensor::ModeType::MODE_TYPE_HELPCAR){
            std_msgs::msg::Int32 message;
            message.data = int(sensor::PopupType::POPUP_TYPE_JOYSTICK);
            popup_pubs_->publish(message);
            return;
        }
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = obj.key_auxiliary;  // 参考 scene_node.cpp 中的参数格式：start=1 表示开启
        auto response = scene_auxiliary_client_->async_send_request(request);
        */
    }
    if ((int)obj.key_cruise >=0) {
        // if(car_info_.getcharging != 1){
        std_msgs::msg::Int32 message;
        message.data = (int)obj.key_cruise;
        cruise_mode_pubs_->publish(message);
        // }
        /*
        if(line_charging_status_){
            message.data = int(sensor::PopupType::POPUP_TYPE_PLUG_OUT_POWER);
            popup_pubs_->publish(message);
            return;
        }else if(car_info_.lockswitch == 1){
            message.data = int(sensor::PopupType::POPUP_TYPE_RELEASE_PARKING);
            popup_pubs_->publish(message);
            return;
        }
        // 
        if(reverse_status_){
            message.data = int(sensor::PopupType::POPUP_TYPE_FORWARD_JOYSTICK);
            popup_pubs_->publish(message);
            return;
        }
        sensor::ModeType mode_type = sensor::ModeType::MODE_TYPE_STANDARD;
        if (mode_type_callback_) {
            mode_type = mode_type_callback_();
            RCLCPP_DEBUG(logger_, "当前模式类型: %d", mode_type);
        }
        if(mode_type == sensor::ModeType::MODE_TYPE_HELPCAR){
            message.data = int(sensor::PopupType::POPUP_TYPE_JOYSTICK);
            popup_pubs_->publish(message);
            return;
        }

        if(car_info_.left_brakests == (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN || 
            car_info_.right_brakests == (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN){
            message.data = int(sensor::PopupType::POPUP_TYPE_CLOSE_HAND_BRAKE);
            popup_pubs_->publish(message);
            return;
        }
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = obj.key_cruise;
        auto response = scene_cruise_client_->async_send_request(request);*/
    }

    if ((int)obj.key_setting > 0)
    {
        std_msgs::msg::Int32 message;
        message.data = (int)obj.key_setting;
        gear_pubs_->publish(message);
    }
}

void HandleCanAdapter::onReceive222Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_ves_infor_t obj;
    int retv = odrive_ves_infor_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    if (car_info_.gear != obj.get_gear || car_info_.lockswitch != obj.lock_switch || car_info_.bluetooth_ctrl != obj.bluetooth_ctrl 
            || (obj.get_charging == 1) != line_charging_status_) {
        car_info_.gear = obj.get_gear;
        line_charging_status_ = (obj.get_charging == 1) ? true : false;
        // car_info_.getcharging = obj.get_charging;
        car_info_.lockswitch = obj.lock_switch;
        car_info_.bluetooth_ctrl = obj.bluetooth_ctrl;
        car_info_pubs_->publish(car_info_);
    }
    if(obj.lock_switch)
    {
        //启动标准模式
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 1;
        auto response = scene_standard_client_->async_send_request(request);
    }
    if(obj.bluetooth_ctrl == BLUETOOTH_CTRL_2)
    {
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 1;
        auto response = scene_bluetooth_client_->async_send_request(request);
    }
    if(obj.bluetooth_ctrl == BLUETOOTH_CTRL_3)
    {
        auto request = std::make_shared<interface::srv::SceneControls::Request>();
        request->start = 0;
        auto response = scene_bluetooth_client_->async_send_request(request);
    }
}
//  暂时用作按键失效播报
void HandleCanAdapter::onReceive224Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_colour_screen_k_t obj;
    int retv = odrive_colour_screen_k_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    std_msgs::msg::Int32 message;
    message.data = obj.light_mode;
    physical_button_failure_pubs_->publish(message);
}

void HandleCanAdapter::sendCarInfo() {
    // std::cout << "sendCarInfo" << std::endl;
    auto car_info = interface::msg::CarInfo();
    car_info.vx = car_info_.vx;
    car_info.w = car_info_.w;
    car_info.speed = std::abs(car_info_.vx + car_info_.w) > 0.0 ? car_info_.gear : 0;
    car_info.electricquantity = car_info_.electricquantity;
    car_info.remainingmileage = car_info_.remainingmileage;
    car_info.gear = car_info_.gear;
    car_info.helpcar = car_info_.helpcar;
    car_info.sos = car_info_.sos;
    car_info.turnswitch = car_info_.turnswitch;
    car_info.lighttype = car_info_.lighttype;
    car_info.lockswitch = car_info_.lockswitch;
    car_info.health = car_info_.health;
    car_info.getcharging = car_info_.getcharging;
    car_info.speedkmh = std::abs(car_info_.speedkmh); // 转换为km/h
    car_info.bluetooth_ctrl = car_info_.bluetooth_ctrl;
    car_info.mode_type = car_info_.mode_type;
    car_info.voice_ctrl = car_info_.voice_ctrl;
    car_info.left_brakests = car_info_.left_brakests;
    car_info.right_brakests = car_info_.right_brakests;
    car_info_pubs_->publish(car_info);
}

void HandleCanAdapter::onReceive201Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_left_driver_t obj;
    int retv = odrive_left_driver_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    car_info_.left_brakests = obj.left_driver_sts;
}

void HandleCanAdapter::onReceive301Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_right_driver_t obj;
    int retv = odrive_right_driver_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    car_info_.right_brakests = obj.right_driver_sts;
}

void HandleCanAdapter::onReceive226Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_rocker_t obj;
    int retv = odrive_rocker_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    auto message = interface::msg::Joystick();
    message.rocker_x = obj.rocker_x;
    message.rocker_y = obj.rocker_y;
    // 前拨 x 为正 后拨 x 为负(-100,100) 右拨y为正 左拨y为负(-100,100)
    // 通过x y 计算出方向和角度
    double direction = atan2(obj.rocker_y, obj.rocker_x);
    double angle = direction * 180.0 / M_PI;
    message.rocker_direction = direction;
    message.rocker_angle = angle;
    joystick_pubs_->publish(message);
    // 检测到摇杆倒车 判断倒车时间
    /*
    if(obj.rocker_x < 0.01) {
        reverse_status_ = true;
        std_msgs::msg::Int32 message;
        // 拔出电源播报
        if(line_charging_status_){
            message.data = int(sensor::PopupType::POPUP_TYPE_PLUG_OUT_POWER);
            popup_pubs_->publish(message);
            return;
        }
        // 驻车播报
        if(car_info_.lockswitch == 1){
            message.data = int(sensor::PopupType::POPUP_TYPE_RELEASE_PARKING);
            popup_pubs_->publish(message);
            return;
        }
        // 驶离基站播报
        if(car_info_.getcharging == 2){
            message.data = int(sensor::PopupType::POPUP_TYPE_LEAVE_STATION);
            popup_pubs_->publish(message);
            return;
        }
        sensor::ModeType mode_type = sensor::ModeType::MODE_TYPE_STANDARD;
        if (mode_type_callback_) {
            mode_type = mode_type_callback_();
            RCLCPP_DEBUG(logger_, "当前模式类型: %d", mode_type);
        }
        if(mode_type == sensor::ModeType::MODE_TYPE_HELPCAR){
            message.data = int(sensor::PopupType::POPUP_TYPE_JOYSTICK);
            popup_pubs_->publish(message);
            return;
        }
        if(car_info_.left_brakests == (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN || 
            car_info_.right_brakests == (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN){
            message.data = int(sensor::PopupType::POPUP_TYPE_CLOSE_HAND_BRAKE);
            popup_pubs_->publish(message);
            return;
        }
    }else{
        reverse_status_ = false;
    }
    */
}

void HandleCanAdapter::sendBattery() {
    odrive_battery_t obj;
    obj.charging = (car_info_.getcharging == 0) ? 1 : 0;
    obj.residual_electricity = car_info_.electricquantity;
    obj.remaining_mileage = car_info_.remainingmileage;
    uint8_t dstp[8];
    if (int ret = odrive_battery_pack(dstp, &obj, sizeof(dstp))) {
        muduo::net::Buffer buffer;
        buffer.append(dstp, ret);
        //canSend("can2", (ODRIVE_BATTERY_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
}

void HandleCanAdapter::handleBluetoothStatusMessage(const interface::msg::BluetoothStatus::SharedPtr msg) {
    charging_status_ = msg->charging_status;
    voltage_ = msg->voltage;
    odrive_battery_t obj;
    if(msg->charging_status) {
        car_info_.getcharging = 2;
        obj.charging = 2;//回充中
    }else {
        obj.charging = 0;
    }
    obj.residual_electricity = car_info_.electricquantity;
    obj.remaining_mileage = car_info_.remainingmileage;
    uint8_t dstp[8];
    if (int ret = odrive_battery_pack(dstp, &obj, sizeof(dstp))) {
        muduo::net::Buffer buffer;
        buffer.append(dstp, ret);
        canSend("can0", (ODRIVE_BATTERY_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
}

void HandleCanAdapter::onReceive101Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_cell_t obj;
    int retv = odrive_cell_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    if (obj.percent_capacitance < 101) {
        car_info_.electricquantity = mapBatteryPercent(obj.percent_capacitance);
        car_info_.remainingmileage = calcRemainingMileage(
            sensor::Global::getInstance()->getBatteryCapacity(), car_info_.electricquantity);
    }
}

void HandleCanAdapter::onReceive100Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_cell_pack_t obj;
    int retv = odrive_cell_pack_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    std_msgs::msg::Int32 chg;
    // 回充使用
    // if(obj.electricity > 20) {
    //     chg.data = 1;
    //     recharge_charging_status_pub_->publish(chg);
    // }else if(obj.electricity < -10) {
    //     // 未回充
    //     chg.data = 0;
    //     recharge_charging_status_pub_->publish(chg);
    // }
    if (obj.electricity > 0.01) {
        if(line_charging_status_) {
            car_info_.getcharging = 1;
        }
    } 
    if(obj.electricity < -0.01 && !line_charging_status_ && !charging_status_) {
        car_info_.getcharging = 0;
    }
}

void HandleCanAdapter::onReceive270Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_axle_1_error_t obj;
    int retv = odrive_axle_1_error_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
    // std::cout << " axle_1_error " << obj.axle_1_error << std::endl;
}

void HandleCanAdapter::onReceive271Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_axle_2_error_t obj;
    int retv = odrive_axle_2_error_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
    // std::cout << " axle_2_error " << obj.axle_2_error << std::endl;
}

void HandleCanAdapter::onReceive272Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_brake_error_t obj;
    int retv = odrive_brake_error_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
    // std::cout << " brake_error " << obj.brake_error << std::endl;
}

void HandleCanAdapter::onReceive273Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_brake_error1_t obj;
    int retv = odrive_brake_error1_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
}

void HandleCanAdapter::onReceive274Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    //第四个字节01 时为抱闸异常 02 时表示抱闸解除
    if(buffer.peek()[3] == 0x01) {
        car_info_.left_brakests = (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN;
        car_info_.right_brakests = (int8_t)sensor::BrakeStatus::BRAKE_STATUS_OPEN;
    } else if(buffer.peek()[3] == 0x02) {
        car_info_.left_brakests = (int8_t)sensor::BrakeStatus::BRAKE_STATUS_CLOSE;
        car_info_.right_brakests = (int8_t)sensor::BrakeStatus::BRAKE_STATUS_CLOSE;
    }
    sendCarInfo();
}

void HandleCanAdapter::onReceive27FInfo(const std::string &, int fid, muduo::net::Buffer &buffer) {
    
    // std::cout << " detailed_errors " << obj.detailed_errors << std::endl;
}

void HandleCanAdapter::onReceive280Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_upper_status_error_t obj;
    int retv = odrive_upper_status_error_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
    // std::cout << " upper_status_error " << obj.upper_status_error << std::endl;
}

void HandleCanAdapter::onReceive281Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_rocker_error_t obj;
    int retv = odrive_rocker_error_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
    // std::cout << " rocker_error " << obj.rocker_error << std::endl;
}

void HandleCanAdapter::onReceive282Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_motor_error_t obj;
    int retv = odrive_motor_error_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    interface::msg::HexData error_msg;
    error_msg.hex_value = obj.error_code;
    auto* global = sensor::Global::getInstance();
    if (global) {
        global->sendHexData(error_msg);
    }
    // std::cout << " motor_error " << obj.motor_error << std::endl;
}

static SystemVersionRequest decodeBcdVersion(const uint8_t* data) {
    // 8字节BCD编码版本信息:
    // data[0-1]: hardware_version (BCD), data[2-3]: software_version (BCD)
    // data[4-5]: 年份(BCD), data[6]: 月份(BCD), data[7]: 日期(BCD)
    SystemVersionRequest ver;
    uint8_t hw_major = (data[0] >> 4) * 10 + (data[0] & 0x0F);
    uint8_t hw_minor = (data[1] >> 4) * 10 + (data[1] & 0x0F);
    uint8_t sw_major = (data[2] >> 4) * 10 + (data[2] & 0x0F);
    uint8_t sw_minor = (data[3] >> 4) * 10 + (data[3] & 0x0F);
    uint16_t year = (data[4] >> 4) * 1000 + (data[4] & 0x0F) * 100 +
                    (data[5] >> 4) * 10 + (data[5] & 0x0F);
    uint8_t month = (data[6] >> 4) * 10 + (data[6] & 0x0F);
    uint8_t day = (data[7] >> 4) * 10 + (data[7] & 0x0F);
    std::ostringstream hw_ver, sw_ver, ver_data;
    hw_ver << "V" << std::setfill('0') << std::setw(2) << (int)hw_major << "."
           << std::setfill('0') << std::setw(2) << (int)hw_minor << ".00";
    sw_ver << "V" << std::setw(1) << (int)sw_major << "." << std::setfill('0') << std::setw(2) << (int)sw_minor;
    ver_data << year << "."
             << std::setfill('0') << std::setw(2) << (int)month << "."
             << std::setfill('0') << std::setw(2) << (int)day;
    ver.hardware_version = hw_ver.str();
    ver.software_version = sw_ver.str();
    ver.version_data = ver_data.str();
    return ver;
}

void HandleCanAdapter::onReceive21DInfo(const std::string &, int fid, muduo::net::Buffer &buffer) {
    if (buffer.readableBytes() >= 8) {
        db_version_request_ = decodeBcdVersion(reinterpret_cast<const uint8_t*>(buffer.peek()));
    }
}

void HandleCanAdapter::onReceive227Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    if (buffer.readableBytes() >= 8) {
        uc_version_request_ = decodeBcdVersion(reinterpret_cast<const uint8_t*>(buffer.peek()));
    }
}

void HandleCanAdapter::onReceive229Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    struct odrive_up_setting_t obj;
    int retv = odrive_up_setting_unpack(&obj, (const uint8_t *)(buffer.peek()), buffer.readableBytes());
    if (obj.mode_type != 0xFF) {
        car_info_.mode_type = obj.mode_type;
        car_info_pubs_->publish(car_info_);
    }
    if (obj.voice_ctrl != 0xFF) {
        car_info_.voice_ctrl = obj.voice_ctrl;
        car_info_pubs_->publish(car_info_);
    }
}

void HandleCanAdapter::onReceive228Info(const std::string &, int fid, muduo::net::Buffer &buffer) {
    // 转换为12位 MAC（十六进制字母大写）
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (int i = 0; i < 6; ++i) {
        ss << std::setw(2) << static_cast<int>(static_cast<unsigned char>(buffer.peek()[i]));
    }
    bluetooth_mac_ = ss.str();
    std::cout << " bluetooth_mac " << bluetooth_mac_ << std::endl;
}

void HandleCanAdapter::onRechargeStatusMessage(const interface::msg::RechargeStatus::SharedPtr msg) {
    // sendBattery();
}

void HandleCanAdapter::onRecieveSpeedbumpMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {

    // 处理速度带信息
    if (msg->markers.empty()) {
        return;
    }
    auto &marker = msg->markers[0];
    if (marker.points.size() != 2) {
        return;
    }
    // std::cout << " HandleCanAdapter::onRecieveSpeedbumpMessage 1 " << marker.points[0].x << " " << marker.points[1].x
    //           << " " << marker.points[0].y << " " << marker.points[1].y << std::endl;
    double distance = 2.8;
    if (car_info_.gear == 3) {
        distance = 4.0;
    }
    // 判断两个点的y值是否为一个正数一个负数
    if (speed_status_) {
        if ((marker.points[0].y > 0 && marker.points[1].y < 0) || (marker.points[0].y < 0 && marker.points[1].y > 0)) {
            if (marker.points[0].x < distance || marker.points[1].x < distance) {
                speep_dump_count_ = 280; // 设置速度带倒计时
                if (!speedbump_flag_) {
                    std_msgs::msg::Int32 msg;
                    msg.data = 1; // 速度带触发
                    speedbump_pubs_->publish(msg);
                    speedbump_flag_ = true;
                }
                // std::cout << " HandleCanAdapter::onRecieveSpeedbumpMessage 2 " << marker.points[0].x << " " << marker.points[1].x
                //           << " " << marker.points[0].y << " " << marker.points[1].y << std::endl;
            }
        }
    }
}

void HandleCanAdapter::onRecievePaDaoMessage(const std::shared_ptr<std_msgs::msg::Int32> msg) {
    pa_dao_type_ = msg->data;
    if (pa_dao_type_ == 104 && !podao_flag_ && speed_status_) {
        std_msgs::msg::Int32 msg;
        msg.data = 1; // 速度带触发
        podao_pubs_->publish(msg);
        podao_flag_ = true;
    }
    if (pa_dao_type_ == 103 && !uppd_flag_ && speed_status_) {
        std_msgs::msg::Int32 msg;
        msg.data = 1; // 速度带触发
        uppd_pubs_->publish(msg);
        uppd_flag_ = true;
    }
    // std::cout << "HandleCanAdapter::onRecievePaDaoMessage pa_dao_type_ " << pa_dao_type_ << std::endl;
}

void HandleCanAdapter::send90108() {
    odrive_colour_screen_k_t obj;
    obj.light_status = 0;
    obj.light_mode = 8;
    uint8_t data[8];
    odrive_colour_screen_k_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_COLOUR_SCREEN_K_FRAME_ID | CAN_EFF_FLAG), buffer);
}

void HandleCanAdapter::publishCanFrame(const std::string& dev, int fid, muduo::net::Buffer& buffer) {
    if (!can_frame_publisher_) {
        return;
    }

    // 帧 ID 过滤：白名单非空时只发布列表中的 ID，避免把 500-1000Hz 全量帧推向 ROS。
    const uint32_t masked_id = static_cast<uint32_t>(fid) & CAN_EFF_MASK;
    if (can_publish_filter_enabled_ &&
        can_publish_id_filter_.find(masked_id) == can_publish_id_filter_.end()) {
        can_publish_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    can_msgs::msg::Frame frame;
    frame.header.stamp = this->now();
    frame.header.frame_id = dev;
    frame.id = fid;
    frame.dlc = static_cast<uint8_t>(buffer.readableBytes());

    const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer.peek());
    size_t data_len = std::min(static_cast<size_t>(buffer.readableBytes()), static_cast<size_t>(8));
    std::fill(frame.data.begin(), frame.data.end(), 0);
    for (size_t i = 0; i < data_len; ++i) {
        frame.data[i] = data[i];
    }

    frame.is_extended = (fid & CAN_EFF_FLAG) != 0;
    frame.is_rtr = false;
    frame.is_error = false;

    can_frame_publisher_->publish(frame);

    // 采样日志：每发出 5000 帧打印一次过滤统计，便于观察 CAN 实际频率。
    uint32_t cnt = can_publish_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if ((cnt & 0x1FFF) == 0) {  // 每 8192 次
        RCLCPP_DEBUG(logger_, "/can/frames published=%u dropped=%u",
                     cnt, can_publish_dropped_.load(std::memory_order_relaxed));
    }
}

int32_t HandleCanAdapter::encoderCountToMsgField(int64_t raw) {
    if (raw > std::numeric_limits<int32_t>::max()) {
        return std::numeric_limits<int32_t>::max();
    }
    if (raw < std::numeric_limits<int32_t>::min()) {
        return std::numeric_limits<int32_t>::min();
    }
    return static_cast<int32_t>(raw);
}

void HandleCanAdapter::publishEncoderAndOdometry() {
    if (!encoder_valid_) {
        return;
    }
    interface::msg::Encoder message;
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";
    message.left_wheel_count = encoderCountToMsgField(latest_left_count_);
    message.right_wheel_count = encoderCountToMsgField(latest_right_count_);
    control_car_encoder_pubs_->publish(message);
    updateWheelOdometryFromEncoders();
}

void HandleCanAdapter::updateWheelOdometryFromEncoders() {
    if (!encoder_valid_) {
        return;
    }
    if (wheel_separation_m_ <= 1e-9 || counts_per_revolution_ <= 1e-9) {
        RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000,
                             "wheel_odom: invalid wheel_separation_m or counts_per_revolution");
        return;
    }

    const auto stamp = this->now();

    if (!wheel_odom_initialized_) {
        prev_left_count_ = latest_left_count_;
        prev_right_count_ = latest_right_count_;
        wheel_odom_initialized_ = true;
        last_wheel_odom_stamp_ = stamp;
        have_wheel_odom_last_stamp_ = true;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = wheel_odom_frame_id_;
        odom.child_frame_id = wheel_odom_child_frame_id_;
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom.pose.pose.position.z = 0.0;
        const double half_theta = odom_theta_ * 0.5;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = std::sin(half_theta);
        odom.pose.pose.orientation.w = std::cos(half_theta);
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.angular.z = 0.0;
        wheel_odom_pub_->publish(odom);
        return;
    }

    const int64_t dl = latest_left_count_ - prev_left_count_;
    const int64_t dr = latest_right_count_ - prev_right_count_;
    prev_left_count_ = latest_left_count_;
    prev_right_count_ = latest_right_count_;

    const double ds_l = static_cast<double>(dl) * m_per_count_;
    const double ds_r = static_cast<double>(dr) * m_per_count_;
    const double ds = (ds_l + ds_r) * 0.5;
    const double dtheta = (ds_r - ds_l) / wheel_separation_m_;

    const double theta_mid = odom_theta_ + dtheta * 0.5;
    odom_x_ += ds * std::cos(theta_mid);
    odom_y_ += ds * std::sin(theta_mid);
    odom_theta_ += dtheta;

    double vx = 0.0;
    double omega = 0.0;
    if (have_wheel_odom_last_stamp_) {
        const double dt = (stamp - last_wheel_odom_stamp_).seconds();
        if (dt > 1e-6 && dt < 1.0) {
            vx = ds / dt;
            omega = dtheta / dt;
        }
    }
    last_wheel_odom_stamp_ = stamp;
    have_wheel_odom_last_stamp_ = true;

    // EMA 低通滤波平滑速度
    constexpr double kEmaAlpha = 0.3;
    filtered_vx_ = kEmaAlpha * vx + (1.0 - kEmaAlpha) * filtered_vx_;
    filtered_omega_ = kEmaAlpha * omega + (1.0 - kEmaAlpha) * filtered_omega_;

    // 时变协方差：随行驶距离线性增长
    const double dist_step = std::abs(ds) + std::abs(dtheta) * wheel_separation_m_ * 0.5;
    constexpr double kTransDrift = 0.002;   // 位置漂移率 2mm / 每行驶米
    constexpr double kRotDrift  = 0.01;     // 角度漂移率 0.01rad / 每行驶米
    pose_cov_xx_  += kTransDrift * dist_step;
    pose_cov_yy_  += kTransDrift * dist_step;
    pose_cov_yaw_ += kRotDrift  * dist_step;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = wheel_odom_frame_id_;
    odom.child_frame_id = wheel_odom_child_frame_id_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.position.z = 0.0;
    const double half_theta = odom_theta_ * 0.5;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_theta);
    odom.pose.pose.orientation.w = std::cos(half_theta);
    odom.twist.twist.linear.x = filtered_vx_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = filtered_omega_;

    odom.pose.covariance[0] = 0.001 + pose_cov_xx_;
    odom.pose.covariance[7] = 0.001 + pose_cov_yy_;
    odom.pose.covariance[35] = 0.001 + pose_cov_yaw_;
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[35] = 0.05;

    wheel_odom_pub_->publish(odom);
}
