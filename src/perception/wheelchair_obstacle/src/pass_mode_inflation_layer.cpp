#include "wheelchair_obstacle/pass_mode_inflation_layer.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace wheelchair_obstacle
{

void PassModeInflationLayer::onInitialize()
{
  declareParameter("pass_mode_topic", rclcpp::ParameterValue(pass_mode_topic_));
  declareParameter("pass_mode_resolution_enable", rclcpp::ParameterValue(true));
  declareParameter("pass_mode_resolution", rclcpp::ParameterValue(0.02));

  nav2_costmap_2d::InflationLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PassModeInflationLayer: failed to lock node");
  }

  node->get_parameter(getFullName("pass_mode_topic"), pass_mode_topic_);
  node->get_parameter(getFullName("pass_mode_resolution_enable"), pass_mode_resolution_enable_);
  node->get_parameter(getFullName("pass_mode_resolution"), pass_mode_resolution_m_);

  pass_mode_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    pass_mode_topic_, rclcpp::QoS(1).transient_local(),
    std::bind(&PassModeInflationLayer::onPassMode, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "PassModeInflationLayer: topic=%s resolution_enable=%d resolution=%.2fm",
    pass_mode_topic_.c_str(), pass_mode_resolution_enable_, pass_mode_resolution_m_);
}

void PassModeInflationLayer::onPassMode(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg || msg->data == pass_mode_active_) {
    return;
  }

  pass_mode_active_ = msg->data;
  applyPassModeInflation(pass_mode_active_);
  applyPassModeResolution(pass_mode_active_);
}

void PassModeInflationLayer::applyPassModeInflation(bool active)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  if (active) {
    if (!saved_inflation_valid_) {
      saved_inflation_radius_ = inflation_radius_;
      saved_cost_scaling_factor_ = cost_scaling_factor_;
      saved_inflation_enabled_ = enabled_;
      saved_inflation_valid_ = true;
    }

    inflation_radius_ = 0.0;
    cost_scaling_factor_ = 0.0;
    enabled_ = false;
    cell_inflation_radius_ = 0;
    need_reinflation_ = true;

    RCLCPP_INFO(
      logger_,
      "pass_mode: inflation_layer disabled (restore enabled=%d r=%.2f scale=%.1f)",
      saved_inflation_enabled_, saved_inflation_radius_, saved_cost_scaling_factor_);
    return;
  }

  if (!saved_inflation_valid_) {
    return;
  }

  inflation_radius_ = saved_inflation_radius_;
  cost_scaling_factor_ = saved_cost_scaling_factor_;
  enabled_ = saved_inflation_enabled_;
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  need_reinflation_ = true;
  computeCaches();

  RCLCPP_INFO(
    logger_,
    "pass_mode: inflation_layer restored enabled=%d r=%.2f scale=%.1f",
    saved_inflation_enabled_, saved_inflation_radius_, saved_cost_scaling_factor_);
}

void PassModeInflationLayer::applyPassModeResolution(bool active)
{
  if (!pass_mode_resolution_enable_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  if (active) {
    if (!saved_resolution_valid_) {
      node->get_parameter("resolution", saved_resolution_m_);
      saved_resolution_valid_ = true;
    }

    if (std::abs(saved_resolution_m_ - pass_mode_resolution_m_) < 1e-6) {
      return;
    }

    node->set_parameter(rclcpp::Parameter("resolution", pass_mode_resolution_m_));
    RCLCPP_INFO(
      logger_, "pass_mode: costmap resolution -> %.2fm (restore %.2fm)",
      pass_mode_resolution_m_, saved_resolution_m_);
    return;
  }

  if (!saved_resolution_valid_) {
    return;
  }

  node->set_parameter(rclcpp::Parameter("resolution", saved_resolution_m_));
  RCLCPP_INFO(
    logger_, "pass_mode: costmap resolution restored %.2fm", saved_resolution_m_);
}

}  // namespace wheelchair_obstacle

PLUGINLIB_EXPORT_CLASS(wheelchair_obstacle::PassModeInflationLayer, nav2_costmap_2d::Layer)
