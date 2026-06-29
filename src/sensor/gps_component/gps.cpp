#include "gps.hpp"
#include "muduo/net/EventLoop.h"
#include "muduo/net/EventLoopThread.h"

namespace sensor::gps {
using namespace std::placeholders;
static muduo::net::EventLoopThread GpsReceiverThread;

GpsComponent::GpsComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gps", options)
{
    auto logger = rclcpp::get_logger("sensor::gps");
    RCLCPP_INFO(logger, "sersor::gps start !");
    auto loop = GpsReceiverThread.startLoop();
    /* cs8：/dev/ttyS5  lc760z:/dev/ttyAS3  */
    declare_parameter("gps_device_path", "ttyS5");
    get_parameter("gps_device_path", gps_port_);
    gps_module_ = std::make_shared<sensor::gps::lc760z::GPSModule>(gps_port_, loop);
    gps_module_->setGpsCallback(std::bind(&GpsComponent::GpsData, this, _1));
    // gps_module_->setImuCallback(std::bind(&GpsComponent::ImuData, this, _1));
    gpsPubPtr_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gpsTopic, 30);
    // imuPubPtr_ = this->create_publisher<sensor_msgs::msg::Imu>(imuTopic, 30);
}

void GpsComponent::GpsData(const sensor_msgs::msg::NavSatFix& gnss)
{
    // 优化：直接传递引用，避免拷贝
    gpsPubPtr_->publish(gnss);
}

void GpsComponent::ImuData(const sensor_msgs::msg::Imu& imu)
{
    // 优化：直接传递引用，避免拷贝
    imuPubPtr_->publish(imu);
}

}