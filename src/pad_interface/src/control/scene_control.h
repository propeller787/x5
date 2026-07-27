#pragma once
#include "control.h"

namespace service::control {

class SceneControl : public Control {
public:
    SceneControl();
    ~SceneControl();

private:
    void handleAuxiliaryControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
    void handleStandardControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
    void handleNarrowControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
        void handleCruiseControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
    
private:
    int auxiliary_status_; // 0 关闭  1 开启
    int standard_status_; // 0 关闭  1 开启
    int narrow_status_; // 0 关闭  1 开启
    int cruise_status_; // 0 关闭  1 开启
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr auxiliary_control_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr standard_control_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr narrow_control_ { nullptr };
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr cruise_control_ { nullptr };
};

}