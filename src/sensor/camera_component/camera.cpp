#include "camera.hpp"
#include <utils/topic.h>
using namespace std::placeholders;
namespace sensor::camera {
CameraComponent::CameraComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("camera", options)
{
    auto logger = rclcpp::get_logger("camera");
    RCLCPP_INFO(logger, "camera start");
    backup_car_sub_ = this->create_subscription<interface::msg::BackupCar>(
        backupCarTopic, 10, std::bind(&CameraComponent::backupCarCallback, this, _1));
    h264_uvc_ = std::make_shared<H264UVC>("/dev/video0", options);
}

CameraComponent::~CameraComponent()
{
    h264_uvc_->stopCapture();
}

void CameraComponent::backupCarCallback(const interface::msg::BackupCar::SharedPtr msg)
{
    // is_backup_car_ = msg->is_backup_car;
    // if (is_backup_car_) {
    //     h264_uvc_->setBackupCar(true);
    // } else {
    //     h264_uvc_->setBackupCar(false);
    // }
}
} // namespace sensor::camera