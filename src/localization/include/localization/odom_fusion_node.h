#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace localization {

class OdomFusionNode : public rclcpp::Node {
public:
  OdomFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~OdomFusionNode() = default;

private:
  void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);
  void twistCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void publishOdometry(const rclcpp::Time& stamp);
  void reset();

  // 订阅
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  // 发布
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Reset 服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  // 状态
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double vx_ = 0.0;
  double vyaw_ = 0.0;
  rclcpp::Time last_stamp_;
  bool initialized_ = false;

  // 参数
  std::string chassis_twist_topic_;
  std::string imu_topic_;
  std::string fused_odom_topic_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
};

}  // namespace localization
