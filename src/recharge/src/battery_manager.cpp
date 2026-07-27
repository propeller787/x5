// battery_manager.cpp
#include "battery_manager.hpp"
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

BatteryManager::BatteryManager(rclcpp::Node *node,
                               float low_threshold,
                               float critical_threshold)
    : node_(node), battery_state_(BatteryState::NORMAL), battery_percentage_(100.0f), low_battery_threshold_(low_threshold), critical_battery_threshold_(critical_threshold)
{
}

void BatteryManager::initialize()
{
    // 声明电池阈值参数
    node_->declare_parameter<float>("low_battery_threshold", low_battery_threshold_);
    node_->declare_parameter<float>("critical_battery_threshold", critical_battery_threshold_);

    // 获取电池阈值参数
    low_battery_threshold_ = node_->get_parameter("low_battery_threshold").as_double();
    critical_battery_threshold_ = node_->get_parameter("critical_battery_threshold").as_double();

    // 创建电池状态订阅者
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_state", 10,
        std::bind(&BatteryManager::batteryStateCallback, this, _1));

    RCLCPP_INFO(node_->get_logger(), "电池管理器已初始化");
    RCLCPP_INFO(node_->get_logger(), "电池阈值 - 低电量: %.1f%%, 严重低电量: %.1f%%",
                low_battery_threshold_, critical_battery_threshold_);
}

void BatteryManager::batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    updateFromMessage(msg);
}

void BatteryManager::updateFromMessage(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    // 更新电池百分比
    if (msg->percentage >= 0.0 && msg->percentage <= 1.0)
    {
        battery_percentage_ = msg->percentage * 100.0;
    }
    else if (msg->voltage > 0.0)
    {
        // 如果percentage无效，根据电压估算（示例）
        float voltage = msg->voltage;
        // 简单线性估算：假设12V为满电，10V为没电
        battery_percentage_ = std::clamp((voltage - 10.0f) / 2.0f * 100.0f, 0.0f, 100.0f);
    }
    else
    {
        battery_percentage_ = 100.0f; // 默认值
    }

    // 检查充电状态
    if (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
    {
        // 如果正在充电，更新状态
        battery_state_ = BatteryState::CHARGING;
        if (battery_state_callback_)
            battery_state_callback_(battery_state_, battery_percentage_);
    }
    else if (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL)
    {
        // 电池已充满
        battery_state_ = BatteryState::FULL;
        if (battery_state_callback_)
            battery_state_callback_(battery_state_, battery_percentage_);
    }
    else
    {
        // 更新电池状态
        updateBatteryState(battery_percentage_);
    }

    // 检查电池阈值并触发相应动作
    checkBatteryThresholds();

    // 定期记录电池状态（每10%记录一次）
    static int last_logged_percentage = -1;
    int current_10percent = static_cast<int>(battery_percentage_ / 10.0);
    if (current_10percent != last_logged_percentage)
    {
        RCLCPP_INFO(node_->get_logger(), "电池状态: %.1f%% [%s]",
                    battery_percentage_, stateToString().c_str());
        last_logged_percentage = current_10percent;
    }
}

void BatteryManager::updateBatteryState(float percentage)
{
    BatteryState old_state = battery_state_;

    if (percentage <= critical_battery_threshold_)
    {
        battery_state_ = BatteryState::CRITICAL;
    }
    else if (percentage <= low_battery_threshold_)
    {
        battery_state_ = BatteryState::LOW;
    }
    else
    {
        battery_state_ = BatteryState::NORMAL;
    }

    // 状态变化时记录
    if (old_state != battery_state_)
    {
        RCLCPP_WARN(node_->get_logger(), "电池状态变化: %s -> %s (%.1f%%)",
                    batteryStateToString(old_state).c_str(),
                    batteryStateToString(battery_state_).c_str(),
                    percentage);

        if (battery_state_callback_)
            battery_state_callback_(battery_state_, battery_percentage_);
    }
}

void BatteryManager::checkBatteryThresholds()
{
    static bool low_battery_triggered = false;
    static bool critical_battery_triggered = false;

    // 检查低电量警告
    if (battery_state_ == BatteryState::LOW && !low_battery_triggered)
    {
        RCLCPP_WARN(node_->get_logger(), "低电量警告: %.1f%% (阈值: %.1f%%)",
                    battery_percentage_, low_battery_threshold_);
        if (low_battery_callback_)
            low_battery_callback_(battery_state_, battery_percentage_);
        low_battery_triggered = true;
        critical_battery_triggered = false; // 重置严重低电量触发
    }

    // 检查严重低电量警告
    if (battery_state_ == BatteryState::CRITICAL && !critical_battery_triggered)
    {
        RCLCPP_ERROR(node_->get_logger(), "严重低电量警告: %.1f%% (阈值: %.1f%%)",
                     battery_percentage_, critical_battery_threshold_);
        if (low_battery_callback_)
            low_battery_callback_(battery_state_, battery_percentage_);
        critical_battery_triggered = true;
    }

    // 如果电池恢复到正常水平，重置触发标志
    if (battery_state_ == BatteryState::NORMAL)
    {
        low_battery_triggered = false;
        critical_battery_triggered = false;
    }
}

void BatteryManager::setBatteryStateCallback(BatteryStateCallback callback)
{
    battery_state_callback_ = callback;
}

void BatteryManager::setLowBatteryCallback(LowBatteryCallback callback)
{
    low_battery_callback_ = callback;
}

std::string BatteryManager::stateToString() const
{
    return batteryStateToString(battery_state_);
}

std::string BatteryManager::batteryStateToString(BatteryState state) const
{
    switch (state)
    {
    case BatteryState::NORMAL:
        return "正常";
    case BatteryState::LOW:
        return "低电量";
    case BatteryState::CRITICAL:
        return "严重低电量";
    case BatteryState::CHARGING:
        return "充电中";
    case BatteryState::FULL:
        return "充满";
    default:
        return "未知";
    }
}