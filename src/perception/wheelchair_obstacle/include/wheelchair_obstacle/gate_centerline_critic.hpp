#ifndef WHEELCHAIR_OBSTACLE__GATE_CENTERLINE_CRITIC_HPP_
#define WHEELCHAIR_OBSTACLE__GATE_CENTERLINE_CRITIC_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "interface/msg/joystick.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace mppi::critics
{

// ==== 窄门中心线引导 cost 试验版 BEGIN ====
// 窄门/窄道中心线引导代价，作为第一版试验 critic：
// 只在识别到宽度合适、中心未被占用、距离合适且摇杆意图匹配时启用；
// 启用后仅给 MPPI 候选轨迹增加“靠近中心线、航向对齐”的软偏好，不替代原避障 cost。
class GateCenterlineCritic : public mppi::critics::CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

private:
  struct GateInfo
  {
    double distance_m{0.0};
    double center_y_m{0.0};
    double width_m{0.0};
    double axis_yaw_body{0.0};
  };

  struct StickIntent
  {
    bool fresh{false};
    double forward{0.0};
    double lateral{0.0};
    double yaw_body{0.0};
  };

  struct StageWeights
  {
    double center_scale{1.0};
    double heading_scale{1.0};
  };

  struct GuidePoint
  {
    double x_body{0.0};
    double y_body{0.0};
    double yaw_body{0.0};
  };

  void onJoystick(const interface::msg::Joystick::SharedPtr msg);
  StickIntent getStickIntent();
  void publishDebug(
    bool active,
    const std::optional<GateInfo> & gate,
    const StickIntent & intent,
    const rclcpp::Time & now_time);
  std::optional<GateInfo> detectGate(const mppi::CriticData & data, const StickIntent & intent);
  bool isLethalInBody(double x_body, double y_body, const geometry_msgs::msg::Pose & pose) const;
  bool centerBandBlocked(
    double distance_m, double center_y_m, const geometry_msgs::msg::Pose & pose) const;
  bool stickMatchesGate(const GateInfo & gate, const StickIntent & intent);
  std::vector<GuidePoint> buildGuideCurve(const GateInfo & gate) const;
  GuidePoint findClosestGuidePoint(
    const std::vector<GuidePoint> & guide, double x_body, double y_body) const;
  GuidePoint makeHermiteGuidePoint(
    double u,
    double x0,
    double y0,
    double x1,
    double y1,
    double mx0,
    double my0,
    double mx1,
    double my1) const;
  StageWeights computeStageWeights(double x_body, const GateInfo & gate) const;
  double smoothstep01(double value) const;
  double lerp(double a, double b, double t) const;
  double getYaw(const geometry_msgs::msg::Quaternion & q) const;
  double normalizeAngle(double a) const;

  // 参数：全部放在 YAML 中，默认偏保守，方便第一版仿真验证。
  int cost_power_{1};
  double critic_weight_{4.0};
  double center_weight_{1.0};
  double heading_weight_{0.3};
  double gate_width_min_m_{0.80};
  double gate_width_max_m_{1.50};
  double gate_min_distance_m_{0.50};
  double gate_max_distance_m_{1.20};
  double scan_y_abs_m_{1.60};
  double scan_x_step_m_{0.05};
  double scan_y_step_m_{0.05};
  double center_clear_half_width_m_{0.25};
  double center_clear_x_margin_m_{0.12};
  double active_x_band_m_{0.45};
  double active_hold_s_{0.30};
  double stage_prepare_ratio_{0.35};
  double stage_turn_ratio_{0.20};
  double stage_exit_ratio_{0.45};
  double stage_far_center_scale_{0.35};
  double stage_prepare_center_scale_{0.85};
  double stage_turn_center_scale_{1.35};
  double stage_exit_center_scale_{0.90};
  double stage_far_heading_scale_{0.55};
  double stage_prepare_heading_scale_{1.00};
  double stage_turn_heading_scale_{1.25};
  double stage_exit_heading_scale_{0.95};
  double stage_weight_gain_{0.85};
  double critical_mix_ratio_{0.35};
  double stick_forward_min_{0.20};
  double stick_lateral_min_{0.12};
  double gate_center_y_deadband_m_{0.18};
  double intent_lateral_ratio_min_{0.20};
  unsigned char obstacle_cost_threshold_{253};
  int trajectory_point_step_{4};
  std::string joystick_topic_{"/sensor/joystick"};
  std::string debug_topic_{"/mppi/gate_centerline_critic/debug"};
  double debug_publish_period_s_{0.10};

  rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  std::mutex joystick_mutex_;
  interface::msg::Joystick::SharedPtr latest_joystick_;
  rclcpp::Time latest_joystick_stamp_{0, 0, RCL_ROS_TIME};

  std::optional<GateInfo> last_active_gate_;
  rclcpp::Time last_active_gate_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_stamp_{0, 0, RCL_ROS_TIME};
};
// ==== 窄门中心线引导 cost 试验版 END ====

}  // namespace mppi::critics

#endif  // WHEELCHAIR_OBSTACLE__GATE_CENTERLINE_CRITIC_HPP_
