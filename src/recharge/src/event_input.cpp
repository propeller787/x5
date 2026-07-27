// event_input.cpp
#include "event_input.hpp"
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/bool.hpp>
using namespace std::chrono_literals;

using std::placeholders::_1;
EventInputHandler::EventInputHandler(rclcpp::Node *node)
    : node_(node), battery_manager_(nullptr), lidar_controller_(nullptr),
      ipad_manager_(nullptr), workflow_manager_(nullptr), report_manager_(nullptr)
{
    RCLCPP_INFO(node_->get_logger(), "事件输入处理器已初始化");
}

void EventInputHandler::initializeModules()
{
    // 初始化电池管理器
    battery_manager_ = std::make_unique<BatteryManager>(node_, 20.0f, 10.0f);
    battery_manager_->initialize();

    // 初始化激光控制器
    lidar_controller_ = std::make_unique<LidarScanController>(node_);
    lidar_controller_->initialize();

    // 初始化iPad管理器
    ipad_manager_ = std::make_unique<IpadManager>(node_);

    // 初始化工作流管理器
    workflow_manager_ = std::make_unique<WorkflowManager>(node_);
    // 初始化报告管理器
    report_manager_ = std::make_unique<ReportManager>(node_,ipad_manager_.get());

    setupModuleCallbacks();

    bluetooth_status_sub_ = node_->create_subscription<interface::msg::BluetoothStatus>(bluetoothStatusTopic, 10, [this](const interface::msg::BluetoothStatus::SharedPtr msg) {
        onBluetoothStatus(msg);
    });
    RCLCPP_INFO(node_->get_logger(), "所有模块初始化完成");
}

void EventInputHandler::setupModuleCallbacks()
{
    // 1. 设置电池管理器回调
    battery_manager_->setBatteryStateCallback(
        [this](BatteryState state, float percentage)
        {
            // 报告电池状态
            report_manager_->reportBatteryStatus(state, percentage);

            // 触发电池状态事件
            if (state == BatteryState::CHARGING)
            {
                triggerEvent("battery_start_charging");
            }
            else if (state == BatteryState::FULL)
            {
                triggerEvent("battery_full");
            }
        });

    battery_manager_->setLowBatteryCallback(
        [this](BatteryState state, float percentage)
        {
            // 触发低电量事件
            triggerLowBattery(state, percentage);
        });

    // 2. 设置iPad管理器回调
    ipad_manager_->setRechargeStartCallback(
        [this]()
        {
            RCLCPP_INFO(node_->get_logger(), "iPad启动回充");
            if (lidar_controller_)
            {
                lidar_controller_->resetStationDetection();
                RCLCPP_INFO(node_->get_logger(), "已重置基站检测状态为false");
            }
            triggerEvent("ipad_phone_interface_start");
        });

    ipad_manager_->setRechargeCancelCallback(
        [this]()
        {
            RCLCPP_INFO(node_->get_logger(), "iPad取消回充");
            triggerEvent("ipad_phone_interface_cancel");
        });

    // 3. 设置工作流管理器状态更新回调
    workflow_manager_->setStateUpdateCallback(
        [this](const std::string &state_str, const std::string &message)
        {
            // 报告状态更新
            report_manager_->reportInfo(state_str + ": " + message);

            // 根据状态变化控制导航
            WheelchairState current_state = workflow_manager_->getCurrentWheelchairState();
            if (current_state == WheelchairState::SEARCHING)
            {
                startNavigationControl();
            }
            else if (current_state == WheelchairState::ROTATING) // 新增：旋转状态
            {
                // 旋转状态由旋转管理器控制，不需要启动导航控制
                // 可以添加其他处理逻辑
                RCLCPP_INFO(node_->get_logger(), "进入旋转搜索状态");
            }
            else if (current_state == WheelchairState::CHARGING)
            {
                stopNavigationControl();
                lidar_controller_->stopMotion();
                lidar_controller_->setChargingVoltage(true);
            }
            else if (current_state == WheelchairState::READY)
            {
                stopNavigationControl();
            }
            else if (current_state == WheelchairState::WALKING)
            {
                RCLCPP_INFO(node_->get_logger(), "进入行走对接状态");
            }
        });

    // 4. 设置工作流管理器回充状态回调
    workflow_manager_->setRechargeStatusCallback(
        [this](const std::string &status, const std::string &message)
        {
            // 处理回充状态更新
            RCLCPP_INFO(node_->get_logger(), "[回充状态] %s: %s", status.c_str(), message.c_str());

            // 所有回充状态都通过报告管理器反馈
            if (status == "RECHARGE_FAILED")
            {
                report_manager_->reportWarning("回充失败: " + message);
            }
            else if (status == "RECHARGE_SUCCESS")
            {
                report_manager_->reportInfo("回充成功: " + message);
            }
            else if (status == "CHARGING_INTERRUPTED")
            {
                report_manager_->reportWarning("充电中断: " + message);
            }
            else if (status == "CHARGING_COMPLETED")
            {
                report_manager_->reportInfo("充电完成: " + message);
            }
        });

    // 5. 设置事件处理器回调（仅外部输入事件）
    registerEvent("ipad_phone_interface_start",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理iPad启动事件");
                      workflow_manager_->handleEvent("ipad_phone_interface_start");
                  });

    registerEvent("ipad_phone_interface_cancel",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理iPad取消事件");
                      workflow_manager_->handleEvent("ipad_phone_interface_cancel");
                  });

    registerEvent("low_battery_warning",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理低电量事件");
                      workflow_manager_->handleEvent("low_battery_warning");
                  });

    registerEvent("battery_start_charging",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理开始充电事件");
                      workflow_manager_->handleEvent("battery_start_charging");
                  });

    registerEvent("battery_stop_charging",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理停止充电事件");
                      workflow_manager_->handleEvent("battery_stop_charging");
                  });

    registerEvent("battery_full",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理电池充满事件");
                      workflow_manager_->handleEvent("battery_full");
                  });

    registerEvent("bluetooth_disconnected",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理蓝牙断开事件");
                      workflow_manager_->handleEvent("bluetooth_disconnected");
                  });

    registerEvent("bluetooth_connected",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理蓝牙连接事件");
                      workflow_manager_->handleEvent("bluetooth_connected");
                  });

    registerEvent("base_station_power_off",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理基站断电事件");
                      workflow_manager_->handleEvent("base_station_power_off");
                  });

    registerEvent("lock_vehicle",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理锁车事件");
                      workflow_manager_->handleEvent("lock_vehicle");
                  });

    registerEvent("unlock_vehicle",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理解锁事件");
                      workflow_manager_->handleEvent("unlock_vehicle");
                  });

    registerEvent("joystick_pull_back",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理摇杆后拉事件");
                      workflow_manager_->handleEvent("joystick_pull_back");
                  });

    registerEvent("joystick_stop",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理摇杆停止事件");
                      workflow_manager_->handleEvent("joystick_stop");
                  });

    registerEvent("push_start",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理推行启动事件");
                      workflow_manager_->handleEvent("push_start");
                  });

    registerEvent("push_stop",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理推行停止事件");
                      workflow_manager_->handleEvent("push_stop");
                  });

    registerEvent("base_station_lost",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理基站丢失事件");
                      workflow_manager_->handleEvent("base_station_lost");
                  });

    registerEvent("search_30s_timeout",
                  [this](const std::string &data)
                  {
                      RCLCPP_INFO(node_->get_logger(), "处理搜索超时事件");
                      workflow_manager_->handleEvent("search_30s_timeout");
                  });

    RCLCPP_INFO(node_->get_logger(), "模块回调设置完成");
}

void EventInputHandler::onBluetoothStatus(const interface::msg::BluetoothStatus::SharedPtr msg) {
    if (!msg) return;
    // bluetooth_status_ = msg->status;
    // bluetooth_voltage_ = msg->voltage;
    // bluetooth_key_status_ = msg->key_status;
    // bluetooth_charging_status_ = msg->charging_status;

    // msg.charging_status - true: 通电-充电中  false: 未通电
    int32_t start = false;
    if (msg->charging_status) {
        start = true;
    }
    if (ipad_manager_->isRechargeActive()) {
        std::thread([this, start]() {
            if (start) {
                RCLCPP_WARN(node_->get_logger(), "电池状态 - 通电 1");
                triggerBatteryStopCharging();
                stopNavigationControl();
                lidar_controller_->stopMotion();
                lidar_controller_->setChargingVoltage(true);
                RCLCPP_WARN(node_->get_logger(), "电池状态 - 通电 end");
            } else {
                RCLCPP_INFO(node_->get_logger(), "电池状态 - 掉电 1");
                // triggerBatteryStartCharging();
                lidar_controller_->setChargingVoltage(false);
                RCLCPP_INFO(node_->get_logger(), "电池状态 - 掉电 end");
            }
        }).detach();
    }
    battery_status_.store(msg->charging_status);
}

void EventInputHandler::startNavigationControl()
{
    if (!interface_active_)
    {
        lidar_controller_->startNavigation();
        interface_active_ = true;
        RCLCPP_INFO(node_->get_logger(), "启动导航控制");
    }
}

void EventInputHandler::stopNavigationControl()
{
    if (interface_active_)
    {
        lidar_controller_->stopNavigation();
        lidar_controller_->stopMotion();
        interface_active_ = false;
        RCLCPP_INFO(node_->get_logger(), "停止导航控制");
    }
}

void EventInputHandler::registerEvent(const std::string &event_name, EventCallback callback)
{
    event_callbacks_[event_name] = callback;
    RCLCPP_INFO(node_->get_logger(), "已注册事件: %s", event_name.c_str());
}

void EventInputHandler::triggerEvent(const std::string &event_name)
{
    triggerEvent(event_name, "");
}

void EventInputHandler::triggerEvent(const std::string &event_name, const std::string &data)
{
    logEvent(event_name, data);

    if (event_callbacks_.find(event_name) != event_callbacks_.end())
    {
        event_callbacks_[event_name](data);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "未注册的事件: %s", event_name.c_str());
    }
}

void EventInputHandler::triggerIpadStart()
{
    triggerEvent("ipad_phone_interface_start", "iPad启动回充");
}

void EventInputHandler::triggerIpadCancel()
{
    triggerEvent("ipad_phone_interface_cancel", "iPad取消回充");
}

void EventInputHandler::triggerBluetoothDisconnected()
{
    triggerEvent("bluetooth_disconnected", "蓝牙断开连接");
}

void EventInputHandler::triggerBluetoothConnected()
{
    triggerEvent("bluetooth_connected", "蓝牙已连接");
}

void EventInputHandler::triggerBaseStationPowerOff()
{
    triggerEvent("base_station_power_off", "基站断电");
}

void EventInputHandler::triggerLowBattery(BatteryState state, float percentage)
{
    std::string data = "低电量警告 - 状态: ";
    data += std::to_string(static_cast<int>(state));
    data += ", 电量: " + std::to_string(percentage) + "%";
    triggerEvent("low_battery_warning", data);
}

void EventInputHandler::triggerLockVehicle()
{
    triggerEvent("lock_vehicle", "锁车操作");
}

void EventInputHandler::triggerUnlockVehicle()
{
    triggerEvent("unlock_vehicle", "解锁操作");
}

void EventInputHandler::triggerJoystickPullBack()
{
    triggerEvent("joystick_pull_back", "摇杆后拉");
}

void EventInputHandler::triggerJoystickStop()
{
    triggerEvent("joystick_stop", "摇杆停止");
}

void EventInputHandler::triggerPushStart()
{
    triggerEvent("push_start", "推行启动");
}

void EventInputHandler::triggerPushStop()
{
    triggerEvent("push_stop", "推行停止");
}

void EventInputHandler::triggerBatteryStartCharging()
{
    triggerEvent("battery_start_charging", "开始充电");
}

void EventInputHandler::triggerBatteryStopCharging()
{
    triggerEvent("battery_stop_charging", "停止充电");
}

void EventInputHandler::triggerBatteryFull()
{
    triggerEvent("battery_full", "电池充满");
}

void EventInputHandler::triggerBaseStationLost()
{
    triggerEvent("base_station_lost", "基站检测丢失");
}

void EventInputHandler::triggerSearchTimeout()
{
    triggerEvent("search_30s_timeout", "搜索30秒超时");
}

void EventInputHandler::logEvent(const std::string &event_name, const std::string &details)
{
    if (details.empty())
    {
        RCLCPP_INFO(node_->get_logger(), "触发事件: %s", event_name.c_str());
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "触发事件: %s - %s",
                    event_name.c_str(), details.c_str());
    }
}

std::string EventInputHandler::getCurrentState() const
{
    if (workflow_manager_)
        return workflow_manager_->getCurrentState();
    return "未初始化";
}

bool EventInputHandler::isRechargeActive() const
{
    return interface_active_;
}

void EventInputHandler::displaySystemStatus()
{
    if (!workflow_manager_ || !battery_manager_ || !lidar_controller_ || !report_manager_)
    {
        RCLCPP_ERROR(node_->get_logger(), "系统未初始化");
        return;
    }

    std::string control_mode = interface_active_ ? "iPad控制" : "本地控制";

    report_manager_->reportSystemStatus(
        workflow_manager_->getCurrentWheelchairState(),
        battery_manager_->getBatteryState(),
        battery_manager_->getBatteryPercentage(),
        lidar_controller_->getChargingState(),
        lidar_controller_->getChargingVoltage(),
        isRechargeActive(),
        control_mode);
}