#include "high_control.h"

using namespace service::control;

HighControl::HighControl()
    : Control("HighControl")
{
    using namespace std::placeholders;
    auto global = service::Global::getInstance();
    car_high_ = global->node_->create_service<interface::srv::High>(highService, std::bind(&HighControl::handleHigh, this, _1, _2));
}

void HighControl::handleHigh(const std::shared_ptr<interface::srv::High::Request> request,
    std::shared_ptr<interface::srv::High::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    std::cout << " high song " << request->song << std::endl;
    response->set__code(0);
}
