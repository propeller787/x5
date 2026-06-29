#include "chassis.hpp"
#include "can_configurator.h"
#include "can_transreceiver.h"
#include "chassis/chassis_can_adapter.h"
#include "handle/handle_can_adapter.h"
#include "dbc/odrive.h"
#include <utils/topic.h>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace sensor::chassis;
using namespace std::placeholders;
static muduo::net::EventLoopThread ChassisTransreceiverThread;
static muduo::net::EventLoopThread ChassisAdapterThread;

ChassisComponent::ChassisComponent(const rclcpp::NodeOptions &options) : rclcpp::Node("chassis", options) {
    auto logger = rclcpp::get_logger("sensor::chassis");
    RCLCPP_INFO(logger, "sensor::chassis start!");

    setupConfigures();
    setupTansreceivers();
    setupAdapters();

    for (auto &dev : can_devs_) {
        dev.second->start();
    }

    // 为 chassis 申请独立的 MutuallyExclusive callback group：
    //   - 节点内部的 timer/service/subscription 仍然串行执行，无需额外锁；
    //   - MultiThreadedExecutor 会把这个 group 分配到独立线程，不再与 recorder 等
    //     节点的高频回调竞争同一线程，保证 200ms 心跳定时器不被 IO 阻塞。
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&ChassisComponent::onTimer, this), cb_group_);
    car_info_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                              std::bind(&ChassisComponent::sendCarInfo, this), cb_group_);
    gears_control_service_ = this->create_service<interface::srv::GearsControls>(
        sensorGearControlService, std::bind(&ChassisComponent::handleGearControls, this, _1, _2),
        rmw_qos_profile_services_default, cb_group_);
    tft_control_service_ = this->create_service<interface::srv::TftControls>(
        sensorTftControlService, std::bind(&ChassisComponent::handleTftControls, this, _1, _2),
        rmw_qos_profile_services_default, cb_group_);
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        sceneServiceModeTopic, 10, std::bind(&ChassisComponent::modeCallback, this, _1), sub_opts);
    auxiliary_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        auxiliaryStatusTopic, 10, std::bind(&ChassisComponent::auxiliaryStatusCallback, this, _1), sub_opts);
    upgrade_prepare_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        otaSystemUpgradePrepare, 10, std::bind(&ChassisComponent::upgradePrepareCallback, this, _1), sub_opts);

    max_gear_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        tftMaxGearTopic, 10, std::bind(&ChassisComponent::maxGearCallback, this, _1), sub_opts);

    language_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        tftLanguageTopic, 10, std::bind(&ChassisComponent::languageCallback, this, _1), sub_opts);

    sign_status_sub_ = this->create_subscription<interface::msg::VehicleBindStatus>(
        cloudManagerDeviceBindPubTopic, 10, std::bind(&ChassisComponent::signStatusCallback, this, _1), sub_opts);

    RCLCPP_INFO(logger, "sensor::chassis init done!");
}

ChassisComponent::~ChassisComponent() {
    // 通知运行 HandleCanAdapter 的 SingleThreadedExecutor 退出，避免 muduo 线程
    // 永久卡在 executor->spin() 上。
    if (adapter_executor_) {
        adapter_executor_->cancel();
    }
}

void ChassisComponent::sendCarInfo() {
    adapters_["handle"]->sendCarInfo();
    if (send_cell_pack_next_) {
        adapters_["handle"]->canSendLoog("can0", ODRIVE_CELL_PACK_FRAME_ID);
    } else {
        adapters_["handle"]->canSendLoog("can0", ODRIVE_CELL_FRAME_ID);
    }
    send_cell_pack_next_ = !send_cell_pack_next_;
    // 累计发送3次
    if (bluetooth_mac_count_ < 3) {
        adapters_["handle"]->canSendLoog("can0", ODRIVE_BLUETOOTH_MAC_FRAME_ID | CAN_EFF_FLAG);
        bluetooth_mac_count_++;
    } 
    // 发送系统版本
    adapters_["handle"]->canSendLoog("can0", ODRIVE_UC_VERSION_FRAME_ID | CAN_EFF_FLAG);
    adapters_["handle"]->canSendLoog("can0", ODRIVE_DB_VERSION_FRAME_ID | CAN_EFF_FLAG);
    if (cell_count_ == 0) {
        cell_count_ = 1;
        return;
    }
   //adapters_["handle"]->sendBattery();
}

void ChassisComponent::onTimer() {
    is_process_ = false;
    odrive_heart_t obj;
    odrive_heart_init(&obj);  // 使用官方初始化函数确保所有字段被正确初始化
    obj.heart_beat = is_process_ ? 0xFF : 0;
    
    // 设置模式类型、安全功能状态、绑定状态、语言和最大档位
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        obj.mode_type = (int8_t)current_mode_type_;
        obj.fafety_status = (int8_t)current_safety_status_;
        obj.sign_status = (int8_t)sign_status_;    // 24-32位：车辆绑定状态
        obj.language = (int8_t)language_;          // 32-40位：语言设置
        obj.max_gear = (int8_t)max_gear_;          // 40-48位：最大档位
    }
    
    uint8_t data[8];
    odrive_heart_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    adapters_["handle"]->canSend("can0", (ODRIVE_HEART_FRAME_ID | CAN_EFF_FLAG), buffer);
}

void ChassisComponent::setupConfigures() {
    CanConfigurator cfg2("can0");
    cfg2.setup();
}

void ChassisComponent::setupTansreceivers() {
    auto loop = ChassisTransreceiverThread.startLoop();
    can_devs_ = {
        //{"can1", std::make_shared<sensor::chassis::CanTransreceiver>("can1", loop)},
        {"can0", std::make_shared<sensor::chassis::CanTransreceiver>("can0", loop)},
        //{"can2", std::make_shared<sensor::chassis::CanTransreceiver>("can2", loop)},
    };
}

void ChassisComponent::setupAdapters() {
    auto loop1 = ChassisAdapterThread.startLoop();
    // 仅创建并 spin 实际存在的 HandleCanAdapter，避免对空 shared_ptr 调用 spin。
    // 若未来恢复 ChassisCanAdapter，请用同样的方式新增其 executor + spin，而不是
    // 使用 unordered_map::operator[] 触发空值默认构造再 spin。
    adapters_["handle"] = std::make_shared<HandleCanAdapter>(can_devs_,
        [this]() -> sensor::ModeType {
            return this->getCurrentModeType();
        });
    loop1->runInLoop([this] {
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(adapters_["handle"]);
        adapter_executor_ = executor;
        executor->spin();
    });
}

void ChassisComponent::handleGearControls(const std::shared_ptr<interface::srv::GearsControls::Request> request,
                                          std::shared_ptr<interface::srv::GearsControls::Response> response) {
    std::cout << " ChassisComponent::handleGearControls " << std::endl;
    if (0 < request->gear && request->gear < 4) {
        odrive_gear_k_t obj;
        obj.get_gear = request->gear;
        uint8_t data[8];
        odrive_gear_k_pack(data, &obj, 8);
        muduo::net::Buffer buffer;
        buffer.append(data, 8);
        adapters_["handle"]->canSend("can0", (ODRIVE_GEAR_K_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
    if (request->helpcar != -1 || request->sos != -1 || request->lock != -1) {
        odrive_key_status_k_t obj;
        obj.sos = request->sos;
        obj.help_implement = request->helpcar;
        obj.lock_switch = request->lock;
        uint8_t data[8];
        odrive_key_status_k_pack(data, &obj, 8);
        muduo::net::Buffer buffer;
        buffer.append(data, 8);
        adapters_["handle"]->canSend("can0", (ODRIVE_KEY_STATUS_K_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
    response->code = 0;
}

void ChassisComponent::handleTftControls(const std::shared_ptr<interface::srv::TftControls::Request> request,
                                         std::shared_ptr<interface::srv::TftControls::Response> response) {
    if (request->turns_switch != -1 || request->light_type != -1) {
        odrive_colour_screen_k_t obj;
        obj.light_status = request->turns_switch;
        obj.light_mode = request->light_type;
        uint8_t data[8];
        odrive_colour_screen_k_pack(data, &obj, 8);
        muduo::net::Buffer buffer;
        buffer.append(data, 8);
        adapters_["handle"]->canSend("can0", (ODRIVE_COLOUR_SCREEN_K_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
    response->code = 0;
}

// 模式回调函数
void ChassisComponent::modeCallback(const std_msgs::msg::String::SharedPtr msg) {
    auto logger = rclcpp::get_logger("sensor::chassis");
    std::string mode_str = msg->data;
    sensor::ModeType mode_type = getModeType(mode_str);
    if(current_mode_type_ != mode_type)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_mode_type_ = mode_type;
    }
    RCLCPP_DEBUG(logger, "Received mode: %s, mode_type: %d", mode_str.c_str(), mode_type);
}

// OTA 升级准备回调：1 结束 CAN 通信，0 恢复 CAN 通信
void ChassisComponent::upgradePrepareCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    auto logger = rclcpp::get_logger("sensor::chassis");
    if (msg->data == 1) {
        for (auto& dev : can_devs_) {
            dev.second->stop();
        }
        for (auto& adp : adapters_) {
            adp.second->setCanEnabled(false);
        }
        RCLCPP_INFO(logger, "upgrade_prepare=1: CAN communication stopped");
    } else if (msg->data == 0) {
        for (auto& dev : can_devs_) {
            dev.second->resume();
        }
        for (auto& adp : adapters_) {
            adp.second->setCanEnabled(true);
        }
        RCLCPP_INFO(logger, "upgrade_prepare=0: CAN communication resumed");
    }
}

// 安全功能状态回调函数
void ChassisComponent::auxiliaryStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    auto logger = rclcpp::get_logger("sensor::chassis");
    bool safety_enabled = msg->data;
    sensor::SafetyStatus safety_status = safety_enabled ? sensor::SafetyStatus::SAFETY_STATUS_ON : sensor::SafetyStatus::SAFETY_STATUS_OFF;  // true=1(开启), false=0(关闭)
    if(current_safety_status_ != safety_status)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_safety_status_ = safety_status;
    }
}

// 将模式字符串转换为模式类型
// 1=巡航 2=回充 3=推行 4=标准
sensor::ModeType ChassisComponent::getModeType(const std::string &mode_str) {
    if (mode_str == "cruise") {
        return sensor::ModeType::MODE_TYPE_CRUISE;  // 巡航
    } else if (mode_str == "recharge") {
        return sensor::ModeType::MODE_TYPE_RECHARGE;  // 回充
    } else if (mode_str == "helpcar") {
        return sensor::ModeType::MODE_TYPE_HELPCAR;  // 推行
    } else if (mode_str == "standard") {
        return sensor::ModeType::MODE_TYPE_STANDARD;  // 标准
    } else if (mode_str == "bluetooth") {
        return sensor::ModeType::MODE_TYPE_BLUETOOTH;  // 蓝牙
    }
    // 默认返回0（未知模式）
    return sensor::ModeType::MODE_TYPE_STANDARD;
}

// 获取当前模式类型
sensor::ModeType ChassisComponent::getCurrentModeType() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_mode_type_;
}

void ChassisComponent::maxGearCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    max_gear_ = msg->data;
}

/**
 * @brief 车辆绑定状态回调函数
 * @param msg 车辆绑定状态消息，包含绑定状态信息
 * @details 该函数是ROS2订阅回调，用于接收车辆绑定状态信息
 *          当接收到绑定状态消息时，将sign_status_设置为1（已绑定）或0（未绑定）
 *          该值会在onTimer函数中通过CAN总线发送给底层控制器
 */
void ChassisComponent::signStatusCallback(const interface::msg::VehicleBindStatus::SharedPtr msg) {
    sign_status_ = (msg->status == 1) ? 1 : 0;
}

void ChassisComponent::languageCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    language_ = msg->data;
}


// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(sensor::chassis::ChassisComponent)