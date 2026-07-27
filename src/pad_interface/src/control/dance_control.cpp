#include "dance_control.h"

namespace service::control {

DanceControl::DanceControl()
    : Control("dance")
{
    auto global = service::Global::getInstance();
    dance_control_ = global->node_->create_service<interface::srv::DanceControls>(danceControlService, std::bind(&DanceControl::handleDanceControl, this, _1, _2));
}

DanceControl::~DanceControl()
{
}

void DanceControl::handleDanceControl(const std::shared_ptr<interface::srv::DanceControls::Request> request,
    std::shared_ptr<interface::srv::DanceControls::Response> response)
{
    type_ = request->type;
    musicalscore_ = request->musicalscore;
    response->code = 0;
    response->message = "success";
}

}
