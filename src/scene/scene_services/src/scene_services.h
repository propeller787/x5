#pragma once
#include "scene_def.h"
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include "std_msgs/msg/bool.hpp"

namespace service::scene {

enum SecneMode{
    Mode_zero = 0,
	Mode_Auxiliary,
	Mode_Narrow,
    Mode_Standard,
	Mode_Cruise,
	Mode_Dance,
	Mode_Recharge,
	Mode_Followme,
    Mode_Autoindoor,
	Mode_TOP
};

// 定义枚举到字符串的映射表
static std::map<SecneMode, const char*> mode_str_map = {
    {Mode_zero, "Mode_zero"},
    {Mode_Auxiliary, "auxiliary"},
    {Mode_Narrow, "narrow"},
    {Mode_Standard, "standard"},
    {Mode_Cruise, "cruise"},
    {Mode_Dance, "dance"},
    {Mode_Recharge, "recharge"},
    {Mode_Followme, "followme"},
    {Mode_Autoindoor, "autoindoor"},
    {Mode_TOP, "Mode_TOP"}
};

class SceneServices : public rclcpp::Node
{
public:
    SceneServices(std::string name);
    ~SceneServices();
    void test();
    rclcpp::Node::SharedPtr getnode();
private:
    void rechargeCtrl(const std::shared_ptr<interface::srv::RechargeControls::Request> request, std::shared_ptr<interface::srv::RechargeControls::Response> response);
    void followmeCtrl(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void modeEnable(const std::shared_ptr<interface::srv::ModeEnable::Request> request, std::shared_ptr<interface::srv::ModeEnable::Response> response);
    void modeEnableSingle(const std::shared_ptr<interface::srv::KeyValueInt::Request> request, std::shared_ptr<interface::srv::KeyValueInt::Response> response);
    void autoindoorCtrl(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void autoindoorNavGoto(const std::shared_ptr<interface::srv::AutoindoorNav::Request> request, std::shared_ptr<interface::srv::AutoindoorNav::Response> response);
    void autoindoorNavCannel(const std::shared_ptr<interface::srv::AutoindoorNav::Request> request, std::shared_ptr<interface::srv::AutoindoorNav::Response> response);

    void feedNarrow(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedRecharge(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedFollowme(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedDance(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);
    void feedCruise(const std::shared_ptr<interface::srv::SceneControls::Request> request, std::shared_ptr<interface::srv::SceneControls::Response> response);

    void OnConfigEnable(enum SecneMode, bool enable);
    void OnTrigger(enum SecneMode mode, bool start);
    
	void OnlineReportMode(enum SecneMode opened_mode, enum SecneMode  closed_mode);
	void OnlineReportStatus(enum SecneMode  mode, int status);

	void OnlineRechargeStatus(int status);

    bool waitControlService(enum SecneMode mode);
    bool doScencSwitch(enum SecneMode new_mode, enum SecneMode old_mode);
    SecneMode getDefaultMode();
    bool doStart(enum SecneMode mode);
    bool doStop(enum SecneMode mode);

    bool check_mode_in(SecneMode comming_mode);
    bool ChangeMode(enum SecneMode new_mode);

    bool mockRechargeCtrl(int on_off);
    bool do_recharge_ctrl(bool start);
private:
    std::mutex mutex_;
    std::mutex mode_mutex_;
    bool mode_enable_[Mode_TOP];
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr mode_client_list_[Mode_TOP];
    SecneMode last_mode_;
    SecneMode curr_mode_;

    rclcpp::Service<interface::srv::RechargeControls>::SharedPtr recharge_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr followme_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::ModeEnable>::SharedPtr mode_enable_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::KeyValueInt>::SharedPtr mode_enable_single_ctrl_ { nullptr };

    rclcpp::Service<interface::srv::SceneControls>::SharedPtr autoindoor_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::AutoindoorNav>::SharedPtr autoindoor_nav_goto_ { nullptr };
    rclcpp::Service<interface::srv::AutoindoorNav>::SharedPtr autoindoor_nav_cannel_ { nullptr };

    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_narrow_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_recharge_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_followme_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_dance_ctrl_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr feed_cruise_ctrl_ { nullptr };

    rclcpp::Node::SharedPtr node_;
    // rclcpp::CallbackGroup::SharedPtr callback_group_;
    // rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    // std::thread callback_group_executor_thread;

    rclcpp::Client<interface::srv::SceneControls>::SharedPtr auxiliary_client_ { nullptr };
    // rclcpp::Client<interface::srv::SceneControls>::SharedPtr narrow_client_ { nullptr };
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr standard_client_ { nullptr };
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr cruise_client_ { nullptr };
    // rclcpp::Client<interface::srv::SceneControls>::SharedPtr dance_client_ { nullptr };
    // rclcpp::Client<interface::srv::SceneControls>::SharedPtr recharge_client_ { nullptr };
    // rclcpp::Client<interface::srv::SceneControls>::SharedPtr followme_client_ { nullptr };
    
    rclcpp::Publisher<interface::msg::VlagoScene>::SharedPtr sence_pub_ { nullptr };
    rclcpp::Publisher<interface::msg::ReportMode>::SharedPtr mode_status_pub_ { nullptr };
    rclcpp::Publisher<interface::msg::RechargeStatus>::SharedPtr recharge_status_pub_ { nullptr };
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_ { nullptr };
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recharge_ctrl_pub_ { nullptr };
    
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr yisbot_goto_ { nullptr };
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr yisbot_cannel_ { nullptr };

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autobot_checkin_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr recharge_feed_sub_;
    

    int mock_recharge_count_ = 0;
    int mock_recharge_status_ = 0;
    rclcpp::TimerBase::SharedPtr mode_pub_timer_ {nullptr};
    rclcpp::TimerBase::SharedPtr mock_recharge_timer_ {nullptr};
    bool autobot_indoor_checkin_ = true;
};

}