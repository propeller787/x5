// ipad_manager.cpp
#include "ipad_manager.hpp"
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

IpadManager::IpadManager(rclcpp::Node *node)
    : node_(node), recharge_active_(false)
{
    logInfo("iPad管理器初始化完成");
}

void IpadManager::startRecharge()
{
    if (!recharge_active_)
    {
        recharge_active_ = true;
        logInfo("启动回充功能");

        if (recharge_start_callback_)
        {
            recharge_start_callback_();
        }
        else
        {
            logInfo("回充启动回调未设置");
        }
    }
    else
    {
        logInfo("回充功能已在运行中");
    }
}

void IpadManager::cancelRecharge()
{
    if (recharge_active_)
    {
        recharge_active_ = false;
        logInfo("取消回充功能");

        if (recharge_cancel_callback_)
        {
            recharge_cancel_callback_();
        }
        else
        {
            logInfo("回充取消回调未设置");
        }
    }
    else
    {
        logInfo("回充功能未激活，无需取消");
    }
}

void IpadManager::setRechargeStartCallback(RechargeStartCallback callback)
{
    recharge_start_callback_ = callback;
    logInfo("已设置回充启动回调");
}

void IpadManager::setRechargeCancelCallback(RechargeCancelCallback callback)
{
    recharge_cancel_callback_ = callback;
    logInfo("已设置回充取消回调");
}

void IpadManager::logInfo(const std::string &message)
{
    if (node_)
    {
        RCLCPP_INFO(node_->get_logger(), "[IpadManager] %s", message.c_str());
    }
}