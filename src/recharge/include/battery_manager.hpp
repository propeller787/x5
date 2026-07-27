// battery_manager.hpp
#ifndef BATTERY_MANAGER_HPP
#define BATTERY_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "wheelchair_types.hpp"
#include <functional>

class BatteryManager
{
public:
    using BatteryStateCallback = std::function<void(BatteryState, float)>;
    using LowBatteryCallback = std::function<void(BatteryState, float)>;

    BatteryManager(rclcpp::Node *node,
                   float low_threshold = 20.0f,
                   float critical_threshold = 10.0f);

    void initialize();
    void updateFromMessage(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void setBatteryStateCallback(BatteryStateCallback callback);
    void setLowBatteryCallback(LowBatteryCallback callback);

    BatteryState getBatteryState() const { return battery_state_; }
    float getBatteryPercentage() const { return battery_percentage_; }
    float getLowThreshold() const { return low_battery_threshold_; }
    float getCriticalThreshold() const { return critical_battery_threshold_; }

    std::string stateToString() const;

private:
    rclcpp::Node *node_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    BatteryState battery_state_;
    float battery_percentage_;
    float low_battery_threshold_;
    float critical_battery_threshold_;

    BatteryStateCallback battery_state_callback_;
    LowBatteryCallback low_battery_callback_;

    void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void updateBatteryState(float percentage);
    void checkBatteryThresholds();
    std::string batteryStateToString(BatteryState state) const;
};

#endif // BATTERY_MANAGER_HPPs