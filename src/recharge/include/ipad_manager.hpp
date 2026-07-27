// ipad_manager.hpp
#ifndef IPAD_MANAGER_HPP
#define IPAD_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>

class IpadManager
{
public:
    // 回调函数类型
    using RechargeStartCallback = std::function<void()>;
    using RechargeCancelCallback = std::function<void()>;

    IpadManager(rclcpp::Node *node);

    // 回调设置
    void setRechargeStartCallback(RechargeStartCallback callback);
    void setRechargeCancelCallback(RechargeCancelCallback callback);

    // 状态查询（可选）
    bool isRechargeActive() const { return recharge_active_; }
    void startRecharge();
    void cancelRecharge();

private:
    rclcpp::Node *node_;

    // 回调函数
    RechargeStartCallback recharge_start_callback_;
    RechargeCancelCallback recharge_cancel_callback_;

    // 状态
    bool recharge_active_;
    std::string cmd_enable_;

    
    void logInfo(const std::string &message);
};

#endif // IPAD_MANAGER_HPP