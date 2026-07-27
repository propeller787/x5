// workflow.hpp
#ifndef WORKFLOW_HPP
#define WORKFLOW_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>
#include <memory>
#include <functional>
#include "wheelchair_types.hpp"
#include "error_code.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <rclcpp/timer.hpp>
#include "rotation_manager.hpp" // 新增头文件
#include <std_msgs/msg/bool.hpp>
#include<std_msgs/msg/int32.hpp>
#include "recharge_status.hpp"

class WorkflowManager
{
public:
    using StateUpdateCallback = std::function<void(const std::string &, const std::string &)>;

    WorkflowManager(rclcpp::Node *node);

    // ==================== 状态机核心接口 ====================

    // 事件处理接口
    bool handleEvent(const std::string &event);

    // 状态查询接口
    std::string getCurrentState() const;
    WheelchairState getCurrentWheelchairState() const { return current_state_; }

    // 状态设置接口
    void setState(WheelchairState new_state);

    // ==================== 状态转移控制 ====================

    // 标准状态转移
    void transitionToWalking();
    void transitionToSearching();
    void transitionToRotating();
    void transitionToCharging();
    void transitionToReady();

    // 紧急状态转移
    void emergencyTransitionToReady(const std::string &reason = "未知原因");

    // ==================== 旋转管理接口 ====================
    void startRotationTimer();      // 新增
    void checkRotationTransition(); // 新增

    // ==================== 回调设置接口 ====================

    void setStateUpdateCallback(StateUpdateCallback callback);
    void setRechargeStatusCallback(StateUpdateCallback callback);
    void setRechargeErrorCallback(std::function<void(const ErrorInfo &)> callback);

    // ==================== 定时器管理 ====================

    void startSearchTimeoutTimer();
    void stopSearchTimeoutTimer();
    void stopRotationTimer();

    // ==================== 事件处理接口 ====================

    // 外部事件处理
    void handleLowBatteryEvent(BatteryState state, float percentage);
    void handleChargingStartEvent();
    void handleChargingStopEvent();
    void handleChargingFullEvent();
    void handleBaseStationLostEvent();
    void handleSearchTimeoutEvent();
    void handleRechargeFailure(const std::string &reason);

    // 内部事件触发
    void triggerEvent(const std::string &event_name);

    void startRotationTimeoutTimer();
    void stopRotationTimeoutTimer();

private:
    rclcpp::Node *node_;

    // 新增：基站检测处理相关
    bool station_detected_while_rotating_ = false;      // 旋转中检测到基站
    rclcpp::Time station_detected_time_;                // 基站检测时间
    rclcpp::TimerBase::SharedPtr rotation_delay_timer_; // 旋转延迟定时器

    // 状态机状态
    WheelchairState current_state_;
    std::map<std::string, std::map<WheelchairState, bool>> transition_table_;

    // 旋转管理器
    std::unique_ptr<RotationManager> rotation_manager_;

    // ROS发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recharge_timeout_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr station_detected_sub_; // 新增
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub1_;//对外
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rotation_timeout_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr search_timeout_timer_;
    rclcpp::TimerBase::SharedPtr rotation_check_timer_; // 新增
    rclcpp::Time search_start_time_;                    // 新增

    // 回调函数
    StateUpdateCallback state_update_callback_;
    StateUpdateCallback recharge_status_callback_;
    std::function<void(const ErrorInfo &)> recharge_error_callback_;

    // ==================== 私有方法 ====================

    // 初始化方法
    void initializeTransitionTable();
    void initializeRotationManager();
    void initializeSubscriptions(); // 新增

    // 事件权限检查
    bool checkEventPermission(const std::string &event);

    // 事件执行
    void executeEvent(const std::string &event);
    void processStateTransition(const std::string &event);

    // 状态发布
    void publishState(const std::string &state_str, const std::string &message = "");
    std::string stateToString(WheelchairState state) const;

    // 回充状态管理
    void publishRechargeStatus(const std::string &status, const std::string &message);
    void publishRechargeErrorCode(uint32_t error_code);

    // ==================== 具体事件处理函数 ====================

    // 外部事件处理
    void processIpadStartEvent();
    void processIpadCancelEvent();
    void processBluetoothDisconnected();
    void processBluetoothConnected();
    void processBaseStationPowerOff();
    void processLockVehicle();
    void processUnlockVehicle();
    void processJoystickPullBack();
    void processJoystickStop();
    void processPushStart();
    void processPushStop();

    // ==================== 异常处理函数 ====================

    void handleException(const std::string &context, const std::exception &e);

    // ==================== 定时器回调 ====================

    void searchTimeoutCallback();
    void rotationCheckCallback();

    void stationDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 新增：延迟处理相关方法
    void startRotationDelayTimer();
    void stopRotationDelayTimer();
    void rotationDelayCallback();

    // 修改现有的方法声明
};
#endif