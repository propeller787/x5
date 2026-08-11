#ifndef WHEELCHAIR_OBSTACLE__PASS_MODE_INFLATION_LAYER_HPP_
#define WHEELCHAIR_OBSTACLE__PASS_MODE_INFLATION_LAYER_HPP_

#include <string>

#include "nav2_costmap_2d/inflation_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace wheelchair_obstacle
{

// Nav2 InflationLayer variant that reacts to /mppi/narrow_gate_pass_mode in-process.
// Shrinks inflation (default 2 cm) and optionally switches costmap resolution during
// pass_mode, avoiding unreliable cross-node parameter clients before the costmap is ready.
class PassModeInflationLayer : public nav2_costmap_2d::InflationLayer
{
public:
  void onInitialize() override;

private:
  void onPassMode(const std_msgs::msg::Bool::SharedPtr msg);
  void applyPassModeInflation(bool active);
  void applyPassModeResolution(bool active);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pass_mode_sub_;
  std::string pass_mode_topic_{"/mppi/narrow_gate_pass_mode"};
  bool pass_mode_resolution_enable_{true};
  double pass_mode_resolution_m_{0.02};
  double pass_mode_inflation_radius_m_{0.02};

  bool pass_mode_active_{false};
  bool saved_inflation_valid_{false};
  double saved_inflation_radius_{0.0};
  double saved_cost_scaling_factor_{0.0};
  bool saved_inflation_enabled_{true};

  bool saved_resolution_valid_{false};
  double saved_resolution_m_{0.05};
};

}  // namespace wheelchair_obstacle

#endif  // WHEELCHAIR_OBSTACLE__PASS_MODE_INFLATION_LAYER_HPP_
