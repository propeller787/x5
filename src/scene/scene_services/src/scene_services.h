#pragma once
#include "scene_def.h"
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include "std_msgs/msg/bool.hpp"

//Recharge
#define USE_SCENE_SERVICE_RECHARGE 1

#define USE_SCENE_HELPCAR 1

namespace service::scene {

enum SecneMode{
    Mode_zero = 0,
    Mode_Standard,
	Mode_Cruise,
    Mode_Helpcar,
	Mode_Recharge,
	Mode_Narrow,
	Mode_Dance,
	Mode_Followme,
    Mode_Autoindoor,
	Mode_TOP,
    Mode_Bluetooth
};

// 定义枚举到字符串的映射表
static std::map<SecneMode, const char*> mode_str_map = {
    {Mode_zero, "Mode_zero"},
    {Mode_Standard, "standard"},
    {Mode_Cruise, "cruise"},
    {Mode_Helpcar, "helpcar"},
    {Mode_Recharge, "recharge"},
    {Mode_Narrow, "narrow"},
    {Mode_Dance, "dance"},
    {Mode_Followme, "followme"},
    {Mode_Autoindoor, "autoindoor"},
    {Mode_TOP, "Mode_TOP"},
    {Mode_Bluetooth, "bluetooth"}
};

class SceneServices : public rclcpp::Node
{
public:
    SceneServices(std::string name);
    ~SceneServices();
    rclcpp::Node::SharedPtr getnode();
private:
    void modeEnableSingle(const std::shared_ptr<interface::srv::KeyValueInt::Request> request, std::shared_ptr<interface::srv::KeyValueInt::Response> response);

    //模式可主动启停，只有回充、巡航
    void feedRecharge(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedCruise(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedHelpcar(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedStandard(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedBluetooth(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void OnlineReportMode(enum SecneMode opened_mode, enum SecneMode  closed_mode);

    //切换流程 OnTrigger -> doScencSwitch -> ChangeMode
    void OnTrigger(enum SecneMode mode, bool start);
    bool doScencSwitch(enum SecneMode new_mode, enum SecneMode old_mode);
    bool ChangeMode(enum SecneMode new_mode);

    bool doStart(enum SecneMode mode);
    bool doStop(enum SecneMode mode);

    bool waitControlService(enum SecneMode mode);
    SecneMode getDefaultMode();

private:
    std::mutex mutex_;
    std::mutex mode_mutex_;
    bool mode_enable_[Mode_TOP];
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr mode_client_list_[Mode_TOP];
    SecneMode last_mode_;
    SecneMode curr_mode_;
    rclcpp::Node::SharedPtr node_;

    rclcpp::Service<interface::srv::ModeEnable>::SharedPtr mode_enable_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::KeyValueInt>::SharedPtr mode_enable_single_ctrl_ { nullptr };

    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_recharge_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_cruise_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_help_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_bluetooth_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_standard_ctrl_ { nullptr };
    
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr standard_client_ { nullptr };
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr cruise_client_ { nullptr };
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr recharge_client_ { nullptr };

    rclcpp::Publisher<interface::msg::ReportMode>::SharedPtr mode_status_pub_ { nullptr };
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_str_pub_ { nullptr };
    rclcpp::TimerBase::SharedPtr mode_pub_timer_ {nullptr};
};

}
