#include "localization/odom_fusion_node.h"
#include <cmath>

using namespace localization;

OdomFusionNode::OdomFusionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("odom_fusion_node", options) {

  // 声明参数（topic 可配置）
  this->declare_parameter("input.chassis_twist_topic",
                          std::string("/wheel/twist"));
  this->declare_parameter("input.imu_topic", std::string("/sensor/imu/data"));
  this->declare_parameter("output.fused_odom_topic", "/odom/fused");
  this->declare_parameter("output.odom_frame_id", std::string("odom"));
  this->declare_parameter("output.base_link_frame_id", std::string("base_link"));

  chassis_twist_topic_ =
      this->get_parameter("input.chassis_twist_topic").as_string();
  imu_topic_ = this->get_parameter("input.imu_topic").as_string();
  fused_odom_topic_ =
      this->get_parameter("output.fused_odom_topic").as_string();
  odom_frame_id_ = this->get_parameter("output.odom_frame_id").as_string();
  base_link_frame_id_ =
      this->get_parameter("output.base_link_frame_id").as_string();

  RCLCPP_INFO(get_logger(), "chassis_twist_topic: %s",
              chassis_twist_topic_.c_str());
  RCLCPP_INFO(get_logger(), "imu_topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "fused_odom_topic: %s",
              fused_odom_topic_.c_str());

  // 创建订阅
  twist_sub_ =
      this->create_subscription<geometry_msgs::msg::TwistStamped>(
          chassis_twist_topic_, 10,
          std::bind(&OdomFusionNode::twistCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&OdomFusionNode::imuCallback, this, std::placeholders::_1));

  // 创建发布
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      fused_odom_topic_, 10);

  // 创建 tf broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 创建 reset 服务
  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        reset();
        res->success = true;
        res->message = "odom reset to zero";
      });

  RCLCPP_INFO(get_logger(), "OdomFusionNode initialized");
}

void OdomFusionNode::twistCallback(
    geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  vx_ = msg->twist.linear.x;
}

void OdomFusionNode::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
  rclcpp::Time stamp = msg->header.stamp;
  vyaw_ = msg->angular_velocity.z;

  if (!initialized_) {
    last_stamp_ = stamp;
    initialized_ = true;
    return;
  }

  double dt = (stamp - last_stamp_).seconds();
  if (dt > 1e-6 && dt < 1.0) {
    x_ += vx_ * std::cos(yaw_) * dt;
    y_ += vx_ * std::sin(yaw_) * dt;
    yaw_ += vyaw_ * dt;
  }

  last_stamp_ = stamp;
  publishOdometry(stamp);
  publishTransform(stamp);
}

void OdomFusionNode::publishOdometry(const rclcpp::Time& stamp) {
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_link_frame_id_;

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  msg.twist.twist.linear.x = vx_;
  msg.twist.twist.angular.z = vyaw_;

  odom_pub_->publish(msg);
}

void OdomFusionNode::publishTransform(const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_id_;
  tf.child_frame_id = base_link_frame_id_;
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);
}

void OdomFusionNode::reset() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  vx_ = 0.0;
  vyaw_ = 0.0;
  initialized_ = false;
  RCLCPP_INFO(get_logger(), "OdomFusionNode reset");
}
