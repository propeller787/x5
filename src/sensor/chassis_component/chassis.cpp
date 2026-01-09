#include "chassis.hpp"
#include "can_configurator.h"
#include "can_transreceiver.h"
#include "chassis/chassis_can_adapter.h"
#include "handle/handle_can_adapter.h"

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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ChassisComponent::onTimer, this));
    car_info_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ChassisComponent::sendCarInfo, this));
    gears_control_service_ =
        this->create_service<interface::srv::GearsControls>(sensorGearControlService, std::bind(&ChassisComponent::handleGearControls, this, _1, _2));
    tft_control_service_ =
        this->create_service<interface::srv::TftControls>(sensorTftControlService, std::bind(&ChassisComponent::handleTftControls, this, _1, _2));
    RCLCPP_INFO(logger, "sensor::chassis init done!");
}

ChassisComponent::~ChassisComponent() {}

void ChassisComponent::sendCarInfo() {
    adapters_["handle"]->sendCarInfo();
    // adapters_["handle"]->canSendLoog("can1", ODRIVE_CELL_FRAME_ID);
    // adapters_["handle"]->canSendLoog("can1", ODRIVE_CELL_PACK_FRAME_ID);
    if (cell_conut_ == 0) {
        cell_conut_ = 1;
        return;
    }
    adapters_["handle"]->sendBattery();
}

void ChassisComponent::onTimer() {
    is_process_ = isProcessRunning("control_service");
    odrive_heart_t obj;
    if (is_process_) {
        obj.heart_beat = 0xFF;
    } else {
        obj.heart_beat = 0;
    }
    uint8_t data[8];
    odrive_heart_pack(data, &obj, 8);
    muduo::net::Buffer buffer;
    buffer.append(data, 8);
    adapters_["handle"]->canSend("can2", (ODRIVE_HEART_FRAME_ID | CAN_EFF_FLAG), buffer);
}

void ChassisComponent::setupConfigures() {
    CanConfigurator cfg0("can1");
    cfg0.setup();
    CanConfigurator cfg1("can2");
    cfg1.setup();
    // CanConfigurator cfg1("can0");
    // cfg1.setup();
}

void ChassisComponent::setupTansreceivers() {
    auto loop = ChassisTransreceiverThread.startLoop();
    can_devs_ = {
        //{"can1", std::make_shared<sensor::chassis::CanTransreceiver>("can1", loop)},
        {"can2", std::make_shared<sensor::chassis::CanTransreceiver>("can2", loop)},
    };
}

void ChassisComponent::setupAdapters() {
    auto loop1 = ChassisAdapterThread.startLoop();
    adapters_["chassis"] = std::make_shared<ChassisCanAdapter>(can_devs_);
    adapters_["handle"] = std::make_shared<HandleCanAdapter>(can_devs_);
    loop1->runInLoop([&] {
        rclcpp::spin(adapters_["handle"]);
        rclcpp::spin(adapters_["chassis"]);
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
        adapters_["handle"]->canSend("can2", (ODRIVE_GEAR_K_FRAME_ID | CAN_EFF_FLAG), buffer);
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
        adapters_["handle"]->canSend("can2", (ODRIVE_KEY_STATUS_K_FRAME_ID | CAN_EFF_FLAG), buffer);
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
        adapters_["handle"]->canSend("can2", (ODRIVE_COLOUR_SCREEN_K_FRAME_ID | CAN_EFF_FLAG), buffer);
    }
    response->code = 0;
}

// 判断某个进程名是否存在
bool ChassisComponent::isProcessRunning(const std::string &processName) {
    DIR *dir = opendir("/proc");
    if (!dir)
        return false;

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        // 只遍历数字目录（代表PID）
        if (entry->d_type == DT_DIR) {
            std::string pidStr(entry->d_name);
            if (std::all_of(pidStr.begin(), pidStr.end(), ::isdigit)) {
                std::string cmdPath = "/proc/" + pidStr + "/comm";
                std::ifstream cmdFile(cmdPath);
                std::string name;
                if (cmdFile && std::getline(cmdFile, name)) {
                    if (name == processName) {
                        closedir(dir);
                        return true;
                    }
                }
            }
        }
    }
    closedir(dir);
    return false;
}

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(sensor::chassis::ChassisComponent)
