#pragma once
#include "control.h"
#include <thread>
#include "sensor_msgs/msg/imu.hpp"

namespace service::control {

struct BaseLinkPose
{
    double x;
    double y;
    double yaw;
};
typedef BaseLinkPose MapedPose;

struct Gps84
{
    double lat;
    double lng;
    Gps84(double latitude, double longitude) : lat(latitude), lng(longitude) {}
};

struct WayPoint : Gps84
{
    double target_dur;
    double max_speed;
    double lat_offset;
    double lng_offset;
    WayPoint() : Gps84(0.0, 0.0){
        target_dur = 0.0;
        max_speed = 0.0;
        lat_offset = 0.0;
        lng_offset = 0.0;
    }
};

class WaypointControl : public Control {

public:
    WaypointControl();

private:
    rcl_interfaces::msg::SetParametersResult onSetParam(const std::vector<rclcpp::Parameter> & parameters);
    void onGpsResult(const sensor_msgs::msg::NavSatFix msg);
    void onImuResult(const sensor_msgs::msg::Imu msg);

    bool changeSpeed(double speed);
    void get_parameter(std::shared_ptr<rclcpp::AsyncParametersClient> client, const std::string & param_name);
    void set_parameter(std::shared_ptr<rclcpp::AsyncParametersClient> client, const std::string & param_name, double value);
    void loopSendTarget();
    bool waitGps(Gps84* target, Gps84* last_target);
    void handleTarget(Gps84* target);
    void onCarGps(struct Gps84* carGPS, double imu_yaw_nofix);
    double fixImuYaw(double imu_yaw_nofix);
    void calcDiffGps(Gps84* car, Gps84* target, double& distance, double& diffEastYaw);     //返回偏差方向, yaw是路径<-->朝东
    void calcBaseLinkYaw(double imuFixedYaw, double diffEastYaw, double& baselinkYaw);    // baselinkYaw = imuFixedYaw - diffEastYaw
    void calcBaseLinkPose(double baselinkYaw, double distance, BaseLinkPose& baselinkPose);
    bool sendBasePose(BaseLinkPose* baselinkPose);

private:
    std::unique_ptr<std::thread> thread_;
    std::unique_ptr<std::thread> thread_param_;
    int send_interval_ms_;
    double target_dur_;
    double imu_yaw_nofix_;
    double imu_yaw_offset_;    //map朝东方向yaw为0 maped_yaw = imu_yaw + imu_yaw_offset_
    double lat_offset_;
    double lng_offset_;
    double max_speed_;
    struct Gps84 carGPS_;
    double imu_yaw_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_ { nullptr };
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_{ nullptr };
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
    // rclcpp::Service<sensor_msgs::msg::NavSatFix>::SharedPtr service_waypoint_ { nullptr };
};

}