#include "scene_control.h"

namespace service::control {

SceneControl::SceneControl()
    : Control("scene")
{
    auto global = service::Global::getInstance();
    auxiliary_control_ = global->node_->create_service<interface::srv::SceneControls>(auxiliaryControlService, std::bind(&SceneControl::handleAuxiliaryControl, this, _1, _2));
    standard_control_ = global->node_->create_service<interface::srv::SceneControls>(standardControlService, std::bind(&SceneControl::handleStandardControl, this, _1, _2));
    narrow_control_ = global->node_->create_service<interface::srv::SceneControls>(narrowControlService, std::bind(&SceneControl::handleNarrowControl, this, _1, _2));
    cruise_control_ = global->node_->create_service<interface::srv::SceneControls>(cruiseControlService, std::bind(&SceneControl::handleCruiseControl, this, _1, _2));
}

SceneControl::~SceneControl()
{
}

void SceneControl::handleAuxiliaryControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response)
{
    if (request->start == 1) {
        auto mode_switch_request = std::make_shared<interface::srv::ModeSwitch::Request>();
        mode_switch_request->str_mode = "auxiliary";
        auto result = mode_switch_client_->async_send_request(mode_switch_request);
        if (result.get()->result) {
            auxiliary_status_ = request->start;
            response->code = 0;
            response->message = "success";  
        } else {
            response->code = -1;
            response->message = "failed";
        }
    } else {
        auxiliary_status_ = request->start;
        response->code = 0;
        response->message = "success";
    }
}

void SceneControl::handleStandardControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response)
{
    if (request->start == 1) {
        auto mode_switch_request = std::make_shared<interface::srv::ModeSwitch::Request>();
        mode_switch_request->str_mode = "standard";
        auto result = mode_switch_client_->async_send_request(mode_switch_request);
        if (result.get()->result) {
            standard_status_ = request->start;
            response->code = 0;
            response->message = "success";
        } else {
            response->code = -1;
            response->message = "failed";
        }
    } else {
        standard_status_ = request->start;
        response->code = 0;
        response->message = "success";
    }
}

void SceneControl::handleNarrowControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response)   
{
    if (request->start == 1) {
        auto mode_switch_request = std::make_shared<interface::srv::ModeSwitch::Request>();
        mode_switch_request->str_mode = "narrow";
        auto result = mode_switch_client_->async_send_request(mode_switch_request);
        if (result.get()->result) {
            narrow_status_ = request->start;
            response->code = 0;
            response->message = "success";
        } else {
            response->code = -1;
            response->message = "failed";
        }
    } else {
        narrow_status_ = request->start;
        response->code = 0;
        response->message = "success";
    }
}

void SceneControl::handleCruiseControl(const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response)
{
    if (request->start == 1) {
        auto mode_switch_request = std::make_shared<interface::srv::ModeSwitch::Request>();
        mode_switch_request->str_mode = "cruise";
        auto result = mode_switch_client_->async_send_request(mode_switch_request);
        if (result.get()->result) {
            cruise_status_ = request->start;
            response->code = 0;
            response->message = "success";
        } else {
            response->code = -1;
            response->message = "failed";
        }
    } else {
        cruise_status_ = request->start;
        response->code = 0;
        response->message = "success";
    }
}
}
