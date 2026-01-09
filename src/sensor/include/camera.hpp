#pragma once
#include "rclcpp/rclcpp.hpp"
#include <interface/msg/backup_car.hpp>
#include "../camera_component/h264_publisher.h"
namespace sensor::camera
{
class CameraComponent : public rclcpp::Node
{
public:
    CameraComponent(const rclcpp::NodeOptions& options);
    ~CameraComponent();
private:
    void backupCarCallback(const interface::msg::BackupCar::SharedPtr msg);

private:
    rclcpp::Subscription<interface::msg::BackupCar>::SharedPtr backup_car_sub_;
    bool is_backup_car_ = false;
    std::shared_ptr<H264UVC> h264_uvc_;
};
} // namespace sensor::camera