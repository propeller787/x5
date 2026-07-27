#include "health_control.h"

using namespace service::control;

HealthControl::HealthControl()
    : Control("Health")
{
    using namespace std::placeholders;
    auto global = service::Global::getInstance();
    health_sta_ptr_ = global->node_->create_publisher<interface::msg::HealthStart>(hrvStartTopic, 30);
    hrv_start_ = global->node_->create_service<interface::srv::HrvStart>(hrvStartService, std::bind(&HealthControl::handleHrvStart, this, _1, _2));
    hrv_info_ = global->node_->create_service<interface::srv::HrvInfo>(hrvInfoService, std::bind(&HealthControl::handleHrvInfo, this, _1, _2));
    health_ptr_ = global->node_->create_subscription<interface::msg::Health>(healthDataTopic, 10, std::bind(&HealthControl::onHrvResult, this, _1));
    std::cout << " HealthControl::HealthControl " << std::endl;
}
HealthControl::~HealthControl()
{
    std::cout << " HealthControl::~HealthControl " << std::endl;
}
void HealthControl::handleHrvStart(const std::shared_ptr<interface::srv::HrvStart::Request> request,
    std::shared_ptr<interface::srv::HrvStart::Response> response)
{
    std::cout << " HealthControl::handleHrvStart start id " << id_ << std::endl;
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    // if (status_) {
    //     response->set__code(-1);
    //     return;
    // }
    std::cout << " id " << request->id << " isstart " << request->isstart << std::endl;
    interface::msg::HealthStart hStr;
    if (request->isstart) {
        hStr.status = true;
        status_ = true;
        health_.hightension = 0;
        sendHrvStop();
    } else {
        hStr.status = false;
        acdata_.clear();
        health_sta_ptr_->publish(hStr);
        response->set__code(-1);
        std::cout << " HealthControl::handleHrvStop end " << std::endl;
        return;
    }
    health_sta_ptr_->publish(hStr);
    response->set__code(0);
    std::cout << " HealthControl::handleHrvStart end " << std::endl;
}

void HealthControl::sendHrvStop()
{
    std::cout << "HealthControl::sendHrvStop start" << std::endl;
    if (is_triggered_) {
        std::cout << "Switch is already triggered!" << std::endl;
        return;
    }
    is_triggered_ = true;
    {
        // 启动异步任务
        future_callback = std::async(std::launch::async, [this]() {
            auto start_time = std::chrono::steady_clock::now();
            auto wait_duration = std::chrono::seconds(50);
            while ((std::chrono::steady_clock::now() - start_time < wait_duration)) {
                // if (health_.lowpressure != 0 && health_.hightension != 0 && health_.heartrate != 0) {
                //     std::cout << " 检测结束 " << std::endl;
                //     break;
                // }
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 间隔 100 毫秒检查一次条件
            }
            // std::this_thread::sleep_for(std::chrono::seconds(60)); // 等待 60 秒
            interface::msg::HealthStart hStr;
            hStr.status = false;
            health_sta_ptr_->publish(hStr);
            is_triggered_ = false; // 重置开关
            status_ = false;
            std::cout << "HealthControl::sendHrvStop end" << std::endl;
        });
        // detach async 任务
        future_callback.wait_for(std::chrono::seconds(0)); // 不阻塞主线程，立即返回
    }
}

void HealthControl::handleHrvInfo(const std::shared_ptr<interface::srv::HrvInfo::Request> request,
    std::shared_ptr<interface::srv::HrvInfo::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    if (health_.lowpressure == 0 || health_.hightension == 0 || health_.heartrate == 0 || acdata_.size() < 639) {
        response->set__code(-1);
        return;
    }
    if (status_) {
        response->set__code(-1);
        return;
    }
    response->data.header.frame_id = "Health";
    response->data.header.stamp = health_.header.stamp;
    response->data.bloodoxygen = health_.bloodoxygen;
    response->data.heartrate = health_.heartrate;
    response->data.hightension = health_.hightension;
    response->data.lowpressure = health_.lowpressure;
    response->data.microcirculation = health_.microcirculation;
    // for (int i = 0; i < 64; ++i) {
    //     response->data.acdata[i] = (int)(health_.acdata[i]); // 填充一些测试数据
    // }
    for (size_t i = 0; i < acdata_.size(); i++) {
        response->acdata[i] = (int)acdata_[i];
    }
    response->set__code(0);
}

void HealthControl::onHrvResult(const interface::msg::Health msg)
{
    if (msg.lowpressure == 0 || msg.hightension == 0 || msg.heartrate == 0) {
        return;
    }
    health_ = msg;
    for (int i = 0; i < 64; ++i) {
        acdata_.push_back((health_.acdata[i]));
        if (acdata_.size() > 640) {
            acdata_.pop_front();
        }
    }
}