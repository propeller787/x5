// event_input.hpp
#ifndef EVENT_INPUT_HANDLER_HPP
#define EVENT_INPUT_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>
#include <map>
#include <memory>
#include "wheelchair_types.hpp"
#include "battery_manager.hpp"
#include "lidascan_ctrl.hpp"
#include "ipad_manager.hpp"
#include "workflow.hpp"
#include "report.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include<std_msgs/msg/int32.hpp>
#include"recharge_status.hpp"
#include <std_msgs/msg/int32.hpp>
#include <interface/msg/bluetooth_status.hpp>

class EventInputHandler
{
public:
    using EventCallback = std::function<void(const std::string &)>;

    EventInputHandler(rclcpp::Node *node);

    // 初始化接口
    void initializeModules();

    // 事件注册接口
    void registerEvent(const std::string &event_name, EventCallback callback);

    // 事件触发接口
    void triggerEvent(const std::string &event_name);
    void triggerEvent(const std::string &event_name, const std::string &data);

    // 特定事件触发（仅外部输入事件）
    void triggerIpadStart();
    void triggerIpadCancel();
    void triggerBluetoothDisconnected();
    void triggerBluetoothConnected();
    void triggerBaseStationPowerOff();
    void triggerLowBattery(BatteryState state, float percentage);
    void triggerLockVehicle();
    void triggerUnlockVehicle();
    void triggerJoystickPullBack();
    void triggerJoystickStop();
    void triggerPushStart();
    void triggerPushStop();
    void triggerBatteryStartCharging();
    void triggerBatteryStopCharging();
    void triggerBatteryFull();
    void triggerBaseStationLost();
    void triggerSearchTimeout();

    // 系统状态查询
    std::string getCurrentState() const;
    bool isRechargeActive() const;

    // 系统状态显示
    void displaySystemStatus();

    void onBluetoothStatus(const interface::msg::BluetoothStatus::SharedPtr msg);

private:
    rclcpp::Node *node_;

    // 模块化组件
    std::unique_ptr<BatteryManager> battery_manager_;
    std::unique_ptr<LidarScanController> lidar_controller_;
    std::unique_ptr<IpadManager> ipad_manager_;
    std::unique_ptr<WorkflowManager> workflow_manager_;
    std::unique_ptr<ReportManager> report_manager_;

    // ROS订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_enable_sub_;
     rclcpp::Subscription<interface::msg::BluetoothStatus>::SharedPtr bluetooth_status_sub_;
    std::atomic<bool> battery_status_{false}; // false未通电 true通电

    // 事件回调映射
    std::map<std::string, EventCallback> event_callbacks_;

    // 状态标志
    bool interface_active_ = false;

    // 私有方法
    void setupModuleCallbacks();
    void startNavigationControl();
    void stopNavigationControl();
    void logEvent(const std::string &event_name, const std::string &details = "");
};

#endif // EVENT_INPUT_HANDLER_HPP