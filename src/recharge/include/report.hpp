// report.hpp
#ifndef REPORT_HPP_
#define REPORT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include "wheelchair_types.hpp"
#include "error_code.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include "../../common/utils/topic.h"
#include <interface/srv/scene_controls.hpp>
#include "ipad_manager.hpp"
#include <memory>
class IpadManager; 

class ReportManager
{
public:
    using StatusReportCallback = std::function<void(const std::string &)>;
    using ErrorCallback = std::function<void(const ErrorInfo &)>;

    ReportManager(rclcpp::Node *node, IpadManager* ipad);
    void handleRechargeService(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    // 报告生成接口
    std::string generateSystemStatusReport(
        WheelchairState wheelchair_state,
        BatteryState battery_state,
        float battery_percentage,
        ChargingState charging_state,
        bool charging_voltage_detected,
        bool recharge_active,
        const std::string &control_mode);

    // 状态报告发布
    void publishStatusReport(const std::string &report);

    // 特定状态报告
    void reportSystemStatus(
        WheelchairState wheelchair_state,
        BatteryState battery_state,
        float battery_percentage,
        ChargingState charging_state,
        bool charging_voltage_detected,
        bool recharge_active,
        const std::string &control_mode);

    void reportBatteryStatus(BatteryState state, float percentage);
    void reportChargingStatus(ChargingState state);
    void reportNavigationStatus(const std::string &status);
    void reportError(const std::string &error_message);
    void reportWarning(const std::string &warning_message);
    void reportInfo(const std::string &info_message);

    // 错误处理接口
    void processErrorCode(uint32_t error_code);
    void processErrorCode(const ErrorInfo &error_info);
    void notifyErrorRecovered(uint32_t error_code);

    // 错误查询接口
    bool hasCriticalErrors() const;
    bool hasError(uint32_t error_code) const;
    std::vector<ErrorInfo> getActiveErrors() const;
    std::vector<ErrorInfo> getCriticalErrors() const;

    // 回调设置
    void setStatusReportCallback(StatusReportCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setCriticalErrorCallback(ErrorCallback callback);
    void setRecoveryCallback(ErrorCallback callback);

private:
    rclcpp::Node *node_;
    std::unique_ptr<IpadManager> _ipad;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr recharge_service_;     ///< 回充功能服务
    std::atomic<bool> recharge_mode_enabled_{false};        ///< 回充模式是否启用
    mutable std::mutex mode_mutex_;                       ///< 模式状态互斥锁

    StatusReportCallback status_report_callback_;

    // 错误管理相关
    std::map<uint32_t, ErrorInfo> active_errors_;
    mutable std::mutex error_mutex_;

    ErrorCallback error_callback_;
    ErrorCallback critical_error_callback_;
    ErrorCallback recovery_callback_;

    // 错误码映射
    static std::map<uint32_t, std::string> error_descriptions_;
    static std::map<ErrorModule, std::string> module_names_;
    static std::map<ErrorSubModule, std::string> sub_module_names_;

    // 辅助函数
    std::string wheelchairStateToString(WheelchairState state) const;
    std::string batteryStateToString(BatteryState state) const;
    std::string chargingStateToString(ChargingState state) const;

    // 错误处理函数
    void errorCodeCallback(const std_msgs::msg::UInt32::SharedPtr msg);
    ErrorInfo parseErrorCode(uint32_t error_code);
    void handleNewError(const ErrorInfo &error_info);
    void handleErrorRecovery(const ErrorInfo &error_info);

    static std::string getErrorDescription(uint32_t error_code);
    static std::string getModuleName(ErrorModule module);
    static std::string getSubModuleName(ErrorSubModule sub_module);
    static void initializeErrorMaps();

    void logReport(const std::string &category, const std::string &message);
};

#endif // REPORT_HPP_