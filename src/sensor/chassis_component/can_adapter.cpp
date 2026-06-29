#include "can_adapter.h"
#include "can_transreceiver.h"
#include "sensor_utils.hpp"
#include <utils/topic.h>

using namespace sensor::chassis;
using namespace sensor::utils;
namespace {
constexpr int kTftRemoteFrame909 = 0x909;
}

CanAdapter::CanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs, const std::string &adapter_name)
    : rclcpp::Node(adapter_name), can_devs_(devs), logger_(rclcpp::get_logger(adapter_name)) {
    // 声明并初始化参数
    this->declare_parameter("vx_max_val", 8.0 / 3.6);
    vx_max_val_ = this->get_parameter("vx_max_val").as_double();

    using namespace std::placeholders;

    for (auto &can : can_devs_) {
        can.second->setReceiveCallback(std::bind(&CanAdapter::onReceiveCanMessage, this, can.first, _1, _2));
    }
    // gnss_subs_["gnss1"] = this->create_subscription<interface::msg::Objects>("/sensor/gnss/location/gnss1", 10,
    //     std::bind(&CanAdapter::onReceiveGnssMessage, this, _1));
    // 自测
    // control_pubs_ = this->create_publisher<interface::msg::Control>(std::string(handleControlTopic), 10);
    control_pubs_ = this->create_publisher<geometry_msgs::msg::Twist>(std::string(handleControlTopic), 10);
    control_car_pubs_ = this->create_publisher<interface::msg::Control>(std::string(carWheelSpeedTopic), 10);
    control_car_spubs_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(std::string(carWheelSpeedStaTopic), 10);
    control_subs_ = this->create_subscription<geometry_msgs::msg::Twist>(nav2ControlTopic, 10, std::bind(&CanAdapter::onReceiveControlMessage, this, _1));
    control_car_encoder_pubs_ = this->create_publisher<interface::msg::Encoder>(std::string(carEncoderTopic), 10);
    backup_car_pubs_ = this->create_publisher<interface::msg::BackupCar>(std::string(backupCarTopic), 10);
    // colour_screen_subs_ = this->create_subscription<interface::msg::ColourScreen>(std::string(serviceColourScreenTopic), 10,
    //     std::bind(&CanAdapter::onReceiveColourScreenMessage, this, _1));
    // gear_subs_ = this->create_subscription<interface::msg::Gear>(std::string(serviceGearTopic), 10,
    //     std::bind(&CanAdapter::onReceiveGearMessage, this, _1));
    health_subs_ =
        this->create_subscription<interface::msg::Health>(std::string(healthDataTopic), 10, std::bind(&CanAdapter::onReceiveHealthMessage, this, _1));
    car_info_pubs_ = this->create_publisher<interface::msg::CarInfo>(std::string(carInfoTopic), 10);
    recharge_charging_status_pub_ = this->create_publisher<std_msgs::msg::Int32>(std::string(sensorTftChargingStatusTopic), 10);
    health_pubs_ = this->create_publisher<interface::msg::HealthStart>(std::string(sensorhandleControlTopic), 10);
    recharge_status_subs_ = this->create_subscription<interface::msg::RechargeStatus>(std::string(rechargeStatusTopic), 10,
                                                                                      std::bind(&CanAdapter::onRechargeStatusMessage, this, _1));
    speedbump_subs_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(std::string(perSpeedbumpTopic), 10,
                                                                                      std::bind(&CanAdapter::onRecieveSpeedbumpMessage, this, _1));
    podao_subs_ = this->create_subscription<std_msgs::msg::Int32>(std::string(perPoDaoTopic), 10, std::bind(&CanAdapter::onRecievePaDaoMessage, this, _1));
    light_type_sub_ = this->create_subscription<std_msgs::msg::Int32>(std::string(lightTypeTopic), 10, std::bind(&CanAdapter::onReceiveLightTypeMessage, this, _1));

    car_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(std::string(tftCarModeTopic), 10, std::bind(&CanAdapter::onReceiveCarModeMessage, this, _1));
    sound_switch_sub_ = this->create_subscription<std_msgs::msg::Int32>(std::string(tftSoundSwitchTopic), 10, std::bind(&CanAdapter::onReceiveSoundSwitchMessage, this, _1));
    language_sub_ = this->create_subscription<std_msgs::msg::Int32>(std::string(tftLanguageTopic), 10, std::bind(&CanAdapter::onReceiveLanguageMessage, this, _1));
    speedbump_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(speedbumpAlarmTopic), 10);
    podao_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(downhillAlarmTopic), 10);
    uppd_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(uphillAlarmTopic), 10);
    //dance_puts_ = this->create_publisher<std_msgs::msg::Int32>(std::string(sceneKeyDanceTopic), 10);
    narrow_sub_ = this->create_subscription<std_msgs::msg::Empty>(std::string("/narrow/end"), 10, std::bind(&CanAdapter::onReceiveNarrowMessage, this, _1));
    
    mode_enable_client_ = this->create_client<interface::srv::KeyValueInt>(vlagoModeEnableService);
    scene_helpcar_client_ = this->create_client<interface::srv::SceneControls>(manageHelpcarService);
    scene_standard_client_ = this->create_client<interface::srv::SceneControls>(manageStandardService);
    scene_bluetooth_client_ = this->create_client<interface::srv::SceneControls>(manageBluetoothService);
    
    audio_sub_ = this->create_subscription<std_msgs::msg::Int32>(audioTopic, 10, std::bind(&CanAdapter::onReceiveAudioMessage, this, _1));
    tft_touch_event_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        std::string(tftTouchEventTopic), 10, std::bind(&CanAdapter::onReceiveTftTouchEventMessage, this, _1));
    joystick_pubs_ = this->create_publisher<interface::msg::Joystick>(std::string(joystickTopic), 10);

    //popup_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(popupTypeTopic), 2);
    physical_button_failure_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(physicalButtonFailureTopic), 10);
    safety_function_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(safetyFunctionTopic), 10);
    cruise_mode_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(cruiseModeTopic), 10);
    gear_pubs_ = this->create_publisher<std_msgs::msg::Int32>(std::string(gearTopic), 10);

    
}

void CanAdapter::canSend(std::string dev, int frameid, muduo::net::Buffer &buf) {
    if (!can_enabled_) return;
    can_devs_[dev]->send(frameid, buf);
}

void CanAdapter::canSendLoog(std::string dev, int frameid) {
    if (!can_enabled_) return;
    can_devs_[dev]->sendLoog(frameid);
}

void CanAdapter::speedDistribution(float vx, float w, int &pcmd_a, int &pcmd_b) {
    float car_width = 0.58f;        // 代步车两轮间距 m
    float k_radps2rpm = 9.5492967f; // rad/s -> rpm 系数
    float radius = 0.120f;          // 轮子的半径 m
    float left_radius = 0.1237429f;
    float hight_radius = 0.1249366f;
    float k_rpm2cmd = 141.2104f; // rpm --> can 下发值 系数
    // 速度分配 负号和电机安装有关
    float v_a = -1.0f * vx + w * car_width * 0.5f;
    float v_b = -1.0f * vx - w * car_width * 0.5f;
    // printf("%f\t %f\n", v_a, v_b);
    // 电机转速 rpm
    float rpm_a = k_radps2rpm * (v_a / left_radius);
    float rpm_b = k_radps2rpm * (v_b / hight_radius);
    // printf("%f\t %f\n", rpm_a, rpm_b);
    // CAN 下发指令
    int cmd_a = k_rpm2cmd * rpm_a;
    int cmd_b = k_rpm2cmd * rpm_b;
    // printf("%d\t %d\n", cmd_a, cmd_b);
    cmd_a = output_limit(cmd_a, 16380);
    cmd_b = output_limit(cmd_b, 16380);
    // printf("%d\t %d\n", cmd_a, cmd_b);
    pcmd_a = cmd_a;
    pcmd_b = cmd_b;
}
int CanAdapter::output_limit(int input, int max_ouput) {
    int output = input;
    if (output > max_ouput) {
        output = max_ouput;
    }
    if (output < -max_ouput) {
        output = -max_ouput;
    }
    return output;
}
void CanAdapter::distributionSpeed(int cmd_a, int cmd_b, float &pvx, float &pw) {
    cmd_a = output_limit(cmd_a, 16380);
    cmd_b = output_limit(cmd_b, 16380);
    float car_width = 0.58f;        // 代步车两轮间距 m
    float k_radps2rpm = 9.5492967f; // rad/s -> rpm 系数
    float radius = 0.128f;          // 轮子的半径 m
    float left_radius = 0.1237429f;
    float hight_radius = 0.1259366f;
    float k_cmd2rpm = 1.0f / 141.2104f; // can 下发值 -> rpm 系数

    // 电机转速 rpm
    float rpm_a = cmd_a * k_cmd2rpm;
    float rpm_b = cmd_b * k_cmd2rpm;

    // 计算左右轮速度
    float v_a = (rpm_a / k_radps2rpm) * left_radius;
    float v_b = (rpm_b / k_radps2rpm) * hight_radius;

    // 计算前进速度和旋转速度
    float vx = -0.5f * (v_a + v_b);
    float w = (v_a - v_b) / car_width;

    // 将结果赋值给输出参数
    pvx = vx;
    pw = w;
}

void CanAdapter::ZqSpeedDistribution(float vx, float w, float &cmd_a, float &cmd_b) {
    w = -w;
    float car_width = 0.595f;       // 代步车两轮间距 m
    float k_radps2rpm = 9.5492967f; // rad/s -> rpm 系数
    float radius = 0.1268f;          // 轮子的半径 m
    float gear_ratio = 17.00f;      // 电机减速机减速比
    // if (vx > this->get_parameter("vx_max_val").as_double()) {
    //     vx = this->get_parameter("vx_max_val").as_double();
    // }
    // 速度分配 负号和电机安装有关
    float v_a = 1.0f * vx - w * car_width * 0.5f;
    float v_b = 1.0f * vx + w * car_width * 0.5f;
    float rpm_a = v_a / radius / 2 / M_PI * gear_ratio;
    float rpm_b = v_b / radius / 2 / M_PI * gear_ratio;
    cmd_a = rpm_a;
    cmd_b = rpm_b;
}

void CanAdapter::ZqDistributionSpeed(float cmd_a, float cmd_b, float &pvx, float &pw) {
    float car_width = 0.595f;  // 代步车两轮间距 m
    float radius = 0.1268f;     // 轮子的半径 m
    float gear_ratio = 17.00f; // 电机减速机减速比

    // 计算电机转速对应的线速度
    float v_a = cmd_a * radius * 2 * M_PI / gear_ratio;
    float v_b = cmd_b * radius * 2 * M_PI / gear_ratio;

    // 计算车体的线速度和角速度
    pvx = (v_a + v_b) * 0.5f;
    pw = (v_a - v_b) / car_width;

    // 如果线速度超过最大值，进行限制
    // if (pvx > this->get_parameter("vx_max_val").as_double()) {
    //     pvx = this->get_parameter("vx_max_val").as_double();
    // }
}

void CanAdapter::onReceiveHealthMessage(const interface::msg::Health::SharedPtr msg) {
    odrive_health_info_t obj;
    obj.heartrate = msg->heartrate;
    obj.bloodoxygen = msg->bloodoxygen;
    obj.microcirculation = msg->microcirculation;
    uint8_t data[8];
    odrive_health_info_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_HEALTH_INFO_FRAME_ID | CAN_EFF_FLAG), buffer);
}

void CanAdapter::onReceiveNarrowMessage(const std_msgs::msg::Empty::SharedPtr msg) {
    odrive_colour_screen_k_t obj;
    obj.light_status = 0;
    obj.light_mode = 8;
    uint8_t data[8];
    odrive_colour_screen_k_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_COLOUR_SCREEN_K_FRAME_ID | CAN_EFF_FLAG), buffer);
}

void CanAdapter::onReceiveAudioMessage(const std_msgs::msg::Int32::SharedPtr msg) {
    odrive_audio_t obj;
    obj.audio_type = msg->data;
    uint8_t data[8];
    odrive_audio_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_AUDIO_FRAME_ID | CAN_EFF_FLAG), buffer);
    RCLCPP_INFO(logger_, "Send audio type: %d", obj.audio_type);
}

void CanAdapter::onReceiveTftTouchEventMessage(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) {
        return;
    }
    canSendLoog("can0", (kTftRemoteFrame909 | CAN_EFF_FLAG));
    RCLCPP_INFO(logger_, "Receive tft touch event: %s, send 0x909 remote frame", msg->data ? "true" : "false");
}

void CanAdapter::onReceiveLightTypeMessage(const std_msgs::msg::Int32::SharedPtr msg) {
    odrive_colour_screen_k_t obj;
    obj.light_status = 1;
    obj.light_mode = msg->data;
    uint8_t data[8];
    odrive_colour_screen_k_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_COLOUR_SCREEN_K_FRAME_ID | CAN_EFF_FLAG), buffer);
    RCLCPP_INFO(logger_, "Send light type: %d", obj.light_mode);
}

void CanAdapter::onReceiveCarModeMessage(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(logger_, "Receive car mode: %d", msg->data);
    odrive_car_setting_t obj;
    if (msg->data == 0xFF) { // 系统初始化获取车辆模式和声音开关
        obj.mode_type = 0xFF;
        obj.voice_ctrl = 0xFF;
        uint8_t data[8];
        odrive_car_setting_pack(data, &obj, 8);
        muduo::net::Buffer buffer;
        buffer.append(data, 8);
        canSend("can0", (ODRIVE_CAR_SETTING_FRAME_ID | CAN_EFF_FLAG), buffer);
        return;
    }
    obj.mode_type = msg->data;
    obj.voice_ctrl = 0xFF;
    obj.language = 0xFF;
    uint8_t data[8];
    odrive_car_setting_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_CAR_SETTING_FRAME_ID | CAN_EFF_FLAG), buffer);
}

void CanAdapter::onReceiveSoundSwitchMessage(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(logger_, "Receive sound switch: %d", msg->data);
    odrive_car_setting_t obj;
    obj.voice_ctrl = msg->data;
    obj.mode_type = 0xFF;
    obj.language = 0xFF;
    uint8_t data[8];
    odrive_car_setting_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_CAR_SETTING_FRAME_ID | CAN_EFF_FLAG), buffer);
}
void CanAdapter::onReceiveLanguageMessage(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(logger_, "Receive language: %d", msg->data);
    odrive_car_setting_t obj;
    obj.language = msg->data;
    obj.voice_ctrl = 0xFF;
    obj.mode_type = 0xFF;
    uint8_t data[8];
    odrive_car_setting_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    canSend("can0", (ODRIVE_CAR_SETTING_FRAME_ID | CAN_EFF_FLAG), buffer);
}