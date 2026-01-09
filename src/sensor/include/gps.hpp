#pragma once
#include "../gps_component/cs8/gps_module.h"
// #include "../gps_component/f9k/gps_module.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <utils/topic.h>
namespace sensor::gps {

class GpsComponent : public rclcpp::Node {

public:
    GpsComponent(const rclcpp::NodeOptions& options);

private:
    // 优化：使用const引用避免不必要的拷贝
    void GpsData(const sensor_msgs::msg::NavSatFix& gps);
    void ImuData(const sensor_msgs::msg::Imu& imu);

private:
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPubPtr_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPubPtr_;
    std::shared_ptr<sensor::gps::cs8::GPSModule> gps_module_;
};
}