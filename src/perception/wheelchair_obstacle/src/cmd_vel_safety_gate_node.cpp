// Safety gate between Nav2 MPPI (/cmd_vel) and the wheelchair CAN adapter (/nav/control).
//
// Always forwards MPPI output (with optional gate_enable bypass for testing).
// Retained safety layers:
//   - cmd_hold_timeout: hold last /cmd_vel when MPPI is slower than publish_rate_hz
//   - joystick stale -> fail-safe stop
//   - release soft-stop: joystick_local_path publishes /mppi/release_soft_stop_mode;
//     gate ramps vx/wz down (skip MPPI while active); /mppi/stick_release_halt -> hard stop
//   - proximity speed cap from obstacle cloud + travel intent
//   - tiered wz cap from nearby obstacle-point density
//   - gear vx cap from /vlago/car/info (1-4 gear -> configurable kph limits)
//   - stick deflection speed scale: cap vx/wz by |rocker|/100 (min ratio configurable)
//   - rocker sector intent filter: forward half-plane split into front / left-front /
//     right-front sectors; block reverse in front, opposite turn in lateral-forward sectors
//   - narrow gate pass mode (/mppi/narrow_gate_pass_mode): vx cap; rocker-intent filter
//   - side pass crawl (/mppi/narrow_gate_side_pass_mode): skip MPPI; footprint strips + crawl
//     (forward/backward vx and lateral wz when corresponding strips clear)
//   - open corridor cruise (/mppi/open_corridor_cruise_mode): skip MPPI; jerk-limited
//     vx ramp (open_corridor_cruise_jerk_mps3, cap open_corridor_cruise_accel_mps2)
//     toward gear vx cap when path corridor is clear
//   - after narrow gate exit: linear vx ramp-up to avoid sudden acceleration
//   - vx accel limit: max 1 m/s^2 when speeding up forward/reverse; decel/brake unchanged
//   - mppi fail soft-stop: /cmd_vel -> 0 while rocker active -> release logic (obstacle check);
//     recover when /cmd_vel becomes non-zero again (even during soft-stop lowpass)
//   - left_turn_wz_trick: optional +offset on output wz when |wz|>intent_eps

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <interface/msg/joystick.hpp>
#include <interface/msg/car_info.hpp>

namespace
{

double clamp(double v, double lo, double hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

double angleDiff(double a, double b)
{
  double d = std::fmod(a - b, 2.0 * M_PI);
  if (d < -M_PI) {
    d += 2.0 * M_PI;
  } else if (d > M_PI) {
    d -= 2.0 * M_PI;
  }
  return std::abs(d);
}

}  // namespace

class CmdVelSafetyGateNode : public rclcpp::Node
{
public:
  using Twist = geometry_msgs::msg::Twist;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using UInt8 = std_msgs::msg::UInt8;
  using Bool = std_msgs::msg::Bool;
  using Joystick = interface::msg::Joystick;
  using CarInfo = interface::msg::CarInfo;

  enum class WzDensityTier : uint8_t { OPEN = 0, MODERATE = 1, DENSE = 2 };

  enum class RockerForwardZone : uint8_t { NONE = 0, FRONT = 1, LEFT_FRONT = 2, RIGHT_FRONT = 3 };

  CmdVelSafetyGateNode()
  : rclcpp::Node("cmd_vel_safety_gate_node")
  {
    in_stamped_topic_ = declare_parameter<std::string>("cmd_vel_in", "/cmd_vel");
    out_topic_ = declare_parameter<std::string>("cmd_vel_out", "/nav/control");
    handle_topic_ = declare_parameter<std::string>("handle_control_topic", "/handle/control");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_points_topic", "/perception/obstacle_points");
    input_type_ = declare_parameter<std::string>("input_type", "twist");

    gate_enable_ = declare_parameter<bool>("gate_enable", true);
    bypass_source_ = declare_parameter<std::string>("bypass_source", "handle");
    if (bypass_source_ != "handle" && bypass_source_ != "cmd_vel") {
      RCLCPP_WARN(
        get_logger(),
        "bypass_source='%s' invalid; use 'handle' or 'cmd_vel', defaulting to handle",
        bypass_source_.c_str());
      bypass_source_ = "handle";
    }

    rocker_topic_ = declare_parameter<std::string>("rocker_topic", "/sensor/joystick");
    rocker_deadband_ = declare_parameter<double>("rocker_deadband", 5.0);
    rocker_timeout_ = declare_parameter<double>("rocker_timeout", 0.30);
    rocker_sector_intent_filter_enable_ = declare_parameter<bool>(
      "rocker_sector_intent_filter_enable", true);
    rocker_intent_front_half_deg_ = declare_parameter<double>(
      "rocker_intent_front_half_deg", 30.0);
    rocker_intent_side_span_deg_ = declare_parameter<double>(
      "rocker_intent_side_span_deg", 90.0);
    if (rocker_intent_front_half_deg_ < 0.0) {
      rocker_intent_front_half_deg_ = 0.0;
    }
    if (rocker_intent_side_span_deg_ < 0.0) {
      rocker_intent_side_span_deg_ = 0.0;
    }

    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.30);
    cmd_hold_timeout_ = declare_parameter<double>("cmd_hold_timeout", 0.50);
    if (cmd_hold_timeout_ < cmd_timeout_) {
      RCLCPP_WARN(
        get_logger(),
        "cmd_hold_timeout (%.2f) < cmd_timeout (%.2f); clamping hold to fresh window",
        cmd_hold_timeout_, cmd_timeout_);
      cmd_hold_timeout_ = cmd_timeout_;
    }
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    sector_half_angle_ = declare_parameter<double>("sector_half_angle", 0.785);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.5);
    intent_eps_ = declare_parameter<double>("intent_eps", 0.03);

    front_overhang_ = declare_parameter<double>("front_overhang", 0.935);
    rear_overhang_ = declare_parameter<double>("rear_overhang", 0.2);
    standoff_ = declare_parameter<double>("standoff", 0.4);
    brake_decel_ = declare_parameter<double>("brake_decel", 1.2);
    proximity_speed_cap_ = declare_parameter<bool>("proximity_speed_cap", true);

    wz_max_ = declare_parameter<double>("wz_max", 1.00);
    vx_reverse_max_ = declare_parameter<double>("vx_reverse_max", 0.5);
    left_turn_wz_trick_enable_ = declare_parameter<bool>(
      "left_turn_wz_trick_enable", false);
    left_turn_wz_trick_offset_deg_s_ = declare_parameter<double>(
      "left_turn_wz_trick_offset_deg_s", 2.0);
    left_turn_wz_trick_offset_rps_ =
      left_turn_wz_trick_offset_deg_s_ * M_PI / 180.0;

    car_info_topic_ = declare_parameter<std::string>("car_info_topic", "/vlago/car/info");
    gear_vx_cap_enable_ = declare_parameter<bool>("gear_vx_cap_enable", true);
    gear_default_index_ = declare_parameter<int>("gear_default_index", 3);
    const auto gear_vx_max_kph = declare_parameter<std::vector<double>>(
      "gear_vx_max_kph", std::vector<double>{2.0, 3.0, 4.0, 6.0});
    gear_vx_max_mps_.clear();
    gear_vx_max_mps_.reserve(gear_vx_max_kph.size());
    for (const double kph : gear_vx_max_kph) {
      gear_vx_max_mps_.push_back(std::max(0.0, kph) / 3.6);
    }
    if (gear_vx_max_mps_.empty()) {
      gear_vx_max_mps_ = {2.0 / 3.6, 3.0 / 3.6, 4.0 / 3.6, 6.0 / 3.6};
    }

    wz_density_cap_enable_ = declare_parameter<bool>("wz_density_cap_enable", true);
    density_check_radius_ = declare_parameter<double>("density_check_radius", 1.2);
    wz_density_caps_rps_ = declare_parameter<std::vector<double>>(
      "wz_density_caps_rps", std::vector<double>{1.30, 0.785, 0.524});
    obstacle_count_engage_moderate_ = declare_parameter<int>(
      "obstacle_count_engage_moderate", 12);
    obstacle_count_release_open_ = declare_parameter<int>(
      "obstacle_count_release_open", 6);
    obstacle_count_engage_dense_ = declare_parameter<int>(
      "obstacle_count_engage_dense", 40);
    obstacle_count_release_moderate_ = declare_parameter<int>(
      "obstacle_count_release_moderate", 28);
    wz_density_min_dwell_ = declare_parameter<double>("wz_density_min_dwell", 0.4);
    release_soft_stop_enable_ = declare_parameter<bool>("release_soft_stop_enable", true);
    release_soft_stop_radius_ = declare_parameter<double>("release_soft_stop_radius", 0.8);
    release_soft_stop_decel_ = declare_parameter<double>("release_soft_stop_decel", 1.0);
    release_soft_stop_wz_decel_ = declare_parameter<double>("release_soft_stop_wz_decel", 2.0);
    release_soft_stop_mode_topic_ = declare_parameter<std::string>(
      "release_soft_stop_mode_topic", "/mppi/release_soft_stop_mode");
    stick_release_halt_topic_ = declare_parameter<std::string>(
      "stick_release_halt_topic", "/mppi/stick_release_halt");
    release_soft_stop_time_constant_s_ = declare_parameter<double>(
      "release_soft_stop_time_constant_s", 0.30);
    release_soft_stop_angular_time_constant_s_ = declare_parameter<double>(
      "release_soft_stop_angular_time_constant_s", 0.30);
    if (release_soft_stop_time_constant_s_ < 1e-3) {
      release_soft_stop_time_constant_s_ = 0.30;
    }
    if (release_soft_stop_angular_time_constant_s_ < 1e-3) {
      release_soft_stop_angular_time_constant_s_ = release_soft_stop_time_constant_s_;
    }
    release_soft_stop_vel_eps_ = declare_parameter<double>("release_soft_stop_vel_eps", 0.02);
    release_soft_stop_brake_check_enable_ = declare_parameter<bool>(
      "release_soft_stop_brake_check_enable", true);
    release_soft_stop_standoff_ = declare_parameter<double>(
      "release_soft_stop_standoff", standoff_);
    release_soft_stop_brake_sector_half_angle_ = declare_parameter<double>(
      "release_soft_stop_brake_sector_half_angle", sector_half_angle_);
    mppi_fail_soft_stop_jerk_mps3_ = declare_parameter<double>(
      "mppi_fail_soft_stop_jerk_mps3", 0.3);
    if (mppi_fail_soft_stop_jerk_mps3_ < 0.01) {
      mppi_fail_soft_stop_jerk_mps3_ = 0.01;
    }
    mppi_fail_soft_stop_decel_mps2_ = declare_parameter<double>(
      "mppi_fail_soft_stop_decel_mps2", 1.0);
    if (mppi_fail_soft_stop_decel_mps2_ < 0.05) {
      mppi_fail_soft_stop_decel_mps2_ = 0.05;
    }
    mppi_fail_soft_stop_enable_ = declare_parameter<bool>(
      "mppi_fail_soft_stop_enable", false);
    mppi_fail_soft_stop_check_depth_m_ = declare_parameter<double>(
      "mppi_fail_soft_stop_check_depth_m", 0.10);
    mppi_fail_soft_stop_side_probe_depth_m_ = declare_parameter<double>(
      "mppi_fail_soft_stop_side_probe_depth_m", 0.20);
    mppi_fail_soft_stop_avoid_wz_accel_ = declare_parameter<double>(
      "mppi_fail_soft_stop_avoid_wz_accel", 0.50);
    mppi_fail_soft_stop_avoid_wz_max_ = declare_parameter<double>(
      "mppi_fail_soft_stop_avoid_wz_max", 0.35);
    mppi_fail_soft_stop_max_frames_ = declare_parameter<int>(
      "mppi_fail_soft_stop_max_frames", 50);

    narrow_gate_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_pass_mode_topic", "/mppi/narrow_gate_pass_mode");
    narrow_gate_vx_max_ = declare_parameter<double>("narrow_gate_vx_max", 0.20);
    narrow_gate_rocker_intent_filter_enable_ = declare_parameter<bool>(
      "narrow_gate_rocker_intent_filter_enable", true);
    narrow_gate_rocker_forward_min_ = declare_parameter<double>(
      "narrow_gate_rocker_forward_min", 5.0);
    narrow_gate_rocker_lateral_min_ = declare_parameter<double>(
      "narrow_gate_rocker_lateral_min", 5.0);
    narrow_gate_exit_ramp_enable_ = declare_parameter<bool>(
      "narrow_gate_exit_ramp_enable", true);
    narrow_gate_exit_ramp_sec_ = declare_parameter<double>(
      "narrow_gate_exit_ramp_sec", 2.0);
    narrow_gate_exit_ramp_vx_target_ = declare_parameter<double>(
      "narrow_gate_exit_ramp_vx_target", 1.50);

    narrow_gate_side_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_side_pass_mode_topic", "/mppi/narrow_gate_side_pass_mode");
    // true: crawl 由 jlp 发布 /cmd_vel；false: gate legacy crawl（默认关 jlp、开 gate）
    side_pass_crawl_enable_ = declare_parameter<bool>("side_pass_crawl_enable", false);
    side_pass_crawl_check_depth_m_ = declare_parameter<double>(
      "side_pass_crawl_check_depth_m", 0.06);
    side_pass_crawl_forward_strip_x_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_forward_strip_x", {0.685, 0.935, 0.685});
    side_pass_crawl_forward_strip_y_min_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_forward_strip_y_min", {0.2, -0.2, -0.34});
    side_pass_crawl_forward_strip_y_max_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_forward_strip_y_max", {0.34, 0.2, -0.2});
    if (side_pass_crawl_forward_strip_x_.size() !=
      side_pass_crawl_forward_strip_y_min_.size() ||
      side_pass_crawl_forward_strip_x_.size() !=
      side_pass_crawl_forward_strip_y_max_.size())
    {
      RCLCPP_WARN(
        get_logger(),
        "side_pass forward strip array size mismatch; using defaults");
      side_pass_crawl_forward_strip_x_ = {0.685, 0.935, 0.685};
      side_pass_crawl_forward_strip_y_min_ = {0.2, -0.2, -0.34};
      side_pass_crawl_forward_strip_y_max_ = {0.34, 0.2, -0.2};
    }
    if (side_pass_crawl_forward_strip_x_.empty()) {
      side_pass_crawl_forward_strip_x_ = {0.685, 0.935, 0.685};
      side_pass_crawl_forward_strip_y_min_ = {0.2, -0.2, -0.34};
      side_pass_crawl_forward_strip_y_max_ = {0.34, 0.2, -0.2};
    }
    side_pass_crawl_rear_strip_x_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_rear_strip_x", {-0.2, -0.2, -0.2});
    side_pass_crawl_rear_strip_y_min_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_rear_strip_y_min", {0.2, -0.2, -0.34});
    side_pass_crawl_rear_strip_y_max_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_rear_strip_y_max", {0.34, 0.2, -0.2});
    if (side_pass_crawl_rear_strip_x_.size() !=
      side_pass_crawl_rear_strip_y_min_.size() ||
      side_pass_crawl_rear_strip_x_.size() !=
      side_pass_crawl_rear_strip_y_max_.size())
    {
      RCLCPP_WARN(
        get_logger(),
        "side_pass rear strip array size mismatch; using defaults");
      side_pass_crawl_rear_strip_x_ = {-0.2, -0.2, -0.2};
      side_pass_crawl_rear_strip_y_min_ = {0.2, -0.2, -0.34};
      side_pass_crawl_rear_strip_y_max_ = {0.34, 0.2, -0.2};
    }
    if (side_pass_crawl_rear_strip_x_.empty()) {
      side_pass_crawl_rear_strip_x_ = {-0.2, -0.2, -0.2};
      side_pass_crawl_rear_strip_y_min_ = {0.2, -0.2, -0.34};
      side_pass_crawl_rear_strip_y_max_ = {0.34, 0.2, -0.2};
    }
    side_pass_crawl_lateral_strip_x_min_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_lateral_strip_x_min", {-0.2, 0.685, 0.7});
    side_pass_crawl_lateral_strip_x_max_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_lateral_strip_x_max", {0.685, 0.7, 0.935});
    side_pass_crawl_lateral_strip_y_edge_ = declare_parameter<std::vector<double>>(
      "side_pass_crawl_lateral_strip_y_edge", {0.33, 0.34, 0.20});
    if (side_pass_crawl_lateral_strip_x_min_.size() !=
      side_pass_crawl_lateral_strip_x_max_.size() ||
      side_pass_crawl_lateral_strip_x_min_.size() !=
      side_pass_crawl_lateral_strip_y_edge_.size())
    {
      RCLCPP_WARN(
        get_logger(),
        "side_pass lateral strip array size mismatch; using defaults");
      side_pass_crawl_lateral_strip_x_min_ = {-0.2, 0.685, 0.7};
      side_pass_crawl_lateral_strip_x_max_ = {0.685, 0.7, 0.935};
      side_pass_crawl_lateral_strip_y_edge_ = {0.33, 0.34, 0.20};
    }
    if (side_pass_crawl_lateral_strip_x_min_.empty()) {
      side_pass_crawl_lateral_strip_x_min_ = {-0.2, 0.685, 0.7};
      side_pass_crawl_lateral_strip_x_max_ = {0.685, 0.7, 0.935};
      side_pass_crawl_lateral_strip_y_edge_ = {0.33, 0.34, 0.20};
    }
    side_pass_crawl_vx_mps_ = declare_parameter<double>("side_pass_crawl_vx_mps", 0.10);
    side_pass_crawl_wz_rps_ = declare_parameter<double>("side_pass_crawl_wz_rps", 0.10);
    side_pass_crawl_rocker_forward_min_ = declare_parameter<double>(
      "side_pass_crawl_rocker_forward_min", 5.0);
    side_pass_crawl_rocker_lateral_min_ = declare_parameter<double>(
      "side_pass_crawl_rocker_lateral_min", 5.0);
    side_pass_exit_transition_enable_ = declare_parameter<bool>(
      "side_pass_exit_transition_enable", true);
    side_pass_exit_transition_frames_ = declare_parameter<int>(
      "side_pass_exit_transition_frames", 10);
    if (side_pass_exit_transition_frames_ < 1) {
      side_pass_exit_transition_frames_ = 1;
    }
    side_pass_avoid_turn_enable_ = declare_parameter<bool>(
      "side_pass_avoid_turn_enable", true);

    open_corridor_cruise_mode_topic_ = declare_parameter<std::string>(
      "open_corridor_cruise_mode_topic", "/mppi/open_corridor_cruise_mode");
    open_corridor_cruise_min_gear_ = declare_parameter<int>("open_corridor_cruise_min_gear", 4);
    open_corridor_cruise_jerk_mps3_ = declare_parameter<double>(
      "open_corridor_cruise_jerk_mps3", 0.1);
    if (open_corridor_cruise_jerk_mps3_ < 0.01) {
      open_corridor_cruise_jerk_mps3_ = 0.01;
    }
    open_corridor_cruise_accel_mps2_ = declare_parameter<double>(
      "open_corridor_cruise_accel_mps2", 0.5);
    if (open_corridor_cruise_accel_mps2_ < 0.05) {
      open_corridor_cruise_accel_mps2_ = 0.05;
    }
    open_corridor_cruise_front_half_deg_ = declare_parameter<double>(
      "open_corridor_cruise_front_half_deg", 30.0);
    if (open_corridor_cruise_front_half_deg_ < 0.0) {
      open_corridor_cruise_front_half_deg_ = 0.0;
    }
    open_corridor_cruise_exit_hold_frames_ = declare_parameter<int>(
      "open_corridor_cruise_exit_hold_frames", 20);
    open_corridor_cruise_pre_exit_jerk_mps3_ = declare_parameter<double>(
      "open_corridor_cruise_pre_exit_jerk_mps3", 0.2);
    if (open_corridor_cruise_pre_exit_jerk_mps3_ < 0.01) {
      open_corridor_cruise_pre_exit_jerk_mps3_ = 0.01;
    }
    open_corridor_cruise_pre_exit_decel_mps2_ = declare_parameter<double>(
      "open_corridor_cruise_pre_exit_decel_mps2", 0.4);
    if (open_corridor_cruise_pre_exit_decel_mps2_ < 0.05) {
      open_corridor_cruise_pre_exit_decel_mps2_ = 0.05;
    }
    open_corridor_cruise_pre_exit_exit_speed_kph_ = declare_parameter<double>(
      "open_corridor_cruise_pre_exit_exit_speed_kph", 2.0);
    if (open_corridor_cruise_pre_exit_exit_speed_kph_ < 0.0) {
      open_corridor_cruise_pre_exit_exit_speed_kph_ = 0.0;
    }
    open_corridor_cruise_pre_exit_mode_topic_ = declare_parameter<std::string>(
      "open_corridor_cruise_pre_exit_mode_topic",
      "/mppi/open_corridor_cruise_pre_exit_mode");
    open_corridor_cruise_exit_jerk_mps3_ = declare_parameter<double>(
      "open_corridor_cruise_exit_jerk_mps3", 0.2);
    if (open_corridor_cruise_exit_jerk_mps3_ < 0.01) {
      open_corridor_cruise_exit_jerk_mps3_ = 0.01;
    }
    open_corridor_cruise_exit_decel_mps2_ = declare_parameter<double>(
      "open_corridor_cruise_exit_decel_mps2", 0.8);
    if (open_corridor_cruise_exit_decel_mps2_ < 0.05) {
      open_corridor_cruise_exit_decel_mps2_ = 0.05;
    }
    open_corridor_cruise_exit_early_speed_kph_ = declare_parameter<double>(
      "open_corridor_cruise_exit_early_speed_kph", 2.0);
    if (open_corridor_cruise_exit_early_speed_kph_ < 0.0) {
      open_corridor_cruise_exit_early_speed_kph_ = 0.0;
    }
    open_corridor_cruise_exit_early_below_mppi_enable_ = declare_parameter<bool>(
      "open_corridor_cruise_exit_early_below_mppi_enable", true);
    open_corridor_cruise_mppi_handoff_enable_ = declare_parameter<bool>(
      "open_corridor_cruise_mppi_handoff_enable", true);
    open_corridor_cruise_mppi_handoff_jerk_mps3_ = declare_parameter<double>(
      "open_corridor_cruise_mppi_handoff_jerk_mps3", 0.2);
    if (open_corridor_cruise_mppi_handoff_jerk_mps3_ < 0.01) {
      open_corridor_cruise_mppi_handoff_jerk_mps3_ = 0.01;
    }
    open_corridor_cruise_mppi_handoff_accel_mps2_ = declare_parameter<double>(
      "open_corridor_cruise_mppi_handoff_accel_mps2", 0.5);
    if (open_corridor_cruise_mppi_handoff_accel_mps2_ < 0.05) {
      open_corridor_cruise_mppi_handoff_accel_mps2_ = 0.05;
    }
    open_corridor_cruise_mppi_handoff_max_sec_ = declare_parameter<double>(
      "open_corridor_cruise_mppi_handoff_max_sec", 0.6);
    if (open_corridor_cruise_mppi_handoff_max_sec_ < 0.05) {
      open_corridor_cruise_mppi_handoff_max_sec_ = 0.05;
    }
    open_corridor_cruise_mppi_handoff_vx_eps_ = declare_parameter<double>(
      "open_corridor_cruise_mppi_handoff_vx_eps", 0.08);
    if (open_corridor_cruise_mppi_handoff_vx_eps_ < 0.01) {
      open_corridor_cruise_mppi_handoff_vx_eps_ = 0.01;
    }
    proximity_speed_cap_during_cruise_exit_ = declare_parameter<bool>(
      "proximity_speed_cap_during_cruise_exit", true);
    const auto open_cruise_wz = declare_parameter<std::vector<double>>(
      "open_corridor_cruise_gear_wz_max_rps",
      std::vector<double>{0.55, 0.75, 0.95, 1.15});
    open_corridor_cruise_gear_wz_max_rps_.clear();
    open_corridor_cruise_gear_wz_max_rps_.reserve(open_cruise_wz.size());
    for (const double wz : open_cruise_wz) {
      open_corridor_cruise_gear_wz_max_rps_.push_back(std::max(0.0, wz));
    }
    if (open_corridor_cruise_gear_wz_max_rps_.empty()) {
      open_corridor_cruise_gear_wz_max_rps_ = {0.55, 0.75, 0.95, 1.15};
    }
    const auto open_cruise_kph = declare_parameter<std::vector<double>>(
      "open_corridor_cruise_gear_vx_max_kph",
      std::vector<double>{2.0, 3.0, 4.0, 6.0});
    open_corridor_cruise_gear_vx_max_mps_.clear();
    open_corridor_cruise_gear_vx_max_mps_.reserve(open_cruise_kph.size());
    for (const double kph : open_cruise_kph) {
      open_corridor_cruise_gear_vx_max_mps_.push_back(std::max(0.0, kph) / 3.6);
    }
    if (open_corridor_cruise_gear_vx_max_mps_.empty()) {
      open_corridor_cruise_gear_vx_max_mps_ = {2.0 / 3.6, 3.0 / 3.6, 4.0 / 3.6, 6.0 / 3.6};
    }

    if (narrow_gate_exit_ramp_sec_ < 0.05) {
      narrow_gate_exit_ramp_sec_ = 0.05;
    }
    if (narrow_gate_exit_ramp_vx_target_ < narrow_gate_vx_max_) {
      narrow_gate_exit_ramp_vx_target_ = narrow_gate_vx_max_;
    }

    side_speed_limit_enable_ = declare_parameter<bool>("side_speed_limit_enable", false);
    side_speed_limit_gear_only_ = declare_parameter<int>("side_speed_limit_gear_only", 1);
    side_speed_limit_body_half_width_m_ = declare_parameter<double>(
      "side_speed_limit_body_half_width_m", 0.34);
    side_speed_limit_x_min_m_ = declare_parameter<double>("side_speed_limit_x_min_m", 0.33);
    side_speed_limit_x_max_m_ = declare_parameter<double>("side_speed_limit_x_max_m", 1.00);
    side_speed_limit_dist_m_ = declare_parameter<std::vector<double>>(
      "side_speed_limit_dist_m", std::vector<double>{0.05, 0.10, 0.20, 0.40});
    side_speed_limit_vx_mps_ = declare_parameter<std::vector<double>>(
      "side_speed_limit_vx_mps", std::vector<double>{0.28, 0.60, 1.20, 2.40});
    side_speed_limit_decel_mps2_ = declare_parameter<double>(
      "side_speed_limit_decel_mps2", 0.60);
    side_speed_limit_wz_tighten_enable_ = declare_parameter<bool>(
      "side_speed_limit_wz_tighten_enable", false);
    if (side_speed_limit_dist_m_.size() != side_speed_limit_vx_mps_.size() ||
      side_speed_limit_dist_m_.size() < 2)
    {
      RCLCPP_WARN(
        get_logger(),
        "side_speed_limit dist/vx table invalid; using 5/10/20/40cm -> 0.28/0.60/1.20/2.40 m/s");
      side_speed_limit_dist_m_ = {0.05, 0.10, 0.20, 0.40};
      side_speed_limit_vx_mps_ = {0.28, 0.60, 1.20, 2.40};
    }
    for (size_t i = 1; i < side_speed_limit_dist_m_.size(); ++i) {
      if (side_speed_limit_dist_m_[i] <= side_speed_limit_dist_m_[i - 1]) {
        RCLCPP_WARN(
          get_logger(),
          "side_speed_limit_dist_m not strictly increasing; using defaults");
        side_speed_limit_dist_m_ = {0.05, 0.10, 0.20, 0.40};
        side_speed_limit_vx_mps_ = {0.28, 0.60, 1.20, 2.40};
        break;
      }
    }
    if (side_speed_limit_decel_mps2_ < 0.05) {
      side_speed_limit_decel_mps2_ = 0.05;
    }

    vx_accel_limit_enable_ = declare_parameter<bool>("vx_accel_limit_enable", true);
    vx_accel_max_mps2_ = declare_parameter<double>("vx_accel_max_mps2", 1.0);
    if (vx_accel_max_mps2_ < 0.05) {
      vx_accel_max_mps2_ = 0.05;
    }

    stick_vx_scale_enable_ = declare_parameter<bool>("stick_vx_scale_enable", true);
    stick_vx_min_ratio_ = declare_parameter<double>("stick_vx_min_ratio", 0.15);
    stick_wz_scale_enable_ = declare_parameter<bool>("stick_wz_scale_enable", true);
    stick_vx_min_ratio_ = std::clamp(stick_vx_min_ratio_, 0.0, 1.0);

    if (input_type_ == "twist") {
      twist_sub_ = create_subscription<Twist>(
        in_stamped_topic_, 10,
        std::bind(&CmdVelSafetyGateNode::onTwist, this, std::placeholders::_1));
    } else {
      twist_stamped_sub_ = create_subscription<TwistStamped>(
        in_stamped_topic_, 10,
        std::bind(&CmdVelSafetyGateNode::onTwistStamped, this, std::placeholders::_1));
    }

    handle_sub_ = create_subscription<Twist>(
      handle_topic_, 10,
      std::bind(&CmdVelSafetyGateNode::onHandle, this, std::placeholders::_1));

    joystick_sub_ = create_subscription<Joystick>(
      rocker_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CmdVelSafetyGateNode::onJoystick, this, std::placeholders::_1));

    car_info_sub_ = create_subscription<CarInfo>(
      car_info_topic_, 10,
      std::bind(&CmdVelSafetyGateNode::onCarInfo, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CmdVelSafetyGateNode::onCloud, this, std::placeholders::_1));

    narrow_gate_pass_mode_sub_ = create_subscription<Bool>(
      narrow_gate_pass_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onNarrowGatePassMode, this, std::placeholders::_1));

    narrow_gate_side_pass_mode_sub_ = create_subscription<Bool>(
      narrow_gate_side_pass_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onNarrowGateSidePassMode, this, std::placeholders::_1));

    open_corridor_cruise_mode_sub_ = create_subscription<Bool>(
      open_corridor_cruise_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onOpenCorridorCruiseMode, this, std::placeholders::_1));

    open_corridor_cruise_pre_exit_mode_sub_ = create_subscription<Bool>(
      open_corridor_cruise_pre_exit_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(
        &CmdVelSafetyGateNode::onOpenCorridorCruisePreExitMode, this,
        std::placeholders::_1));

    release_soft_stop_mode_sub_ = create_subscription<Bool>(
      release_soft_stop_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onReleaseSoftStopMode, this, std::placeholders::_1));

    stick_release_halt_sub_ = create_subscription<Bool>(
      stick_release_halt_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onStickReleaseHalt, this, std::placeholders::_1));

    out_pub_ = create_publisher<Twist>(out_topic_, 10);
    wz_tier_pub_ = create_publisher<UInt8>(
      std::string(get_name()) + "/wz_density_tier", 10);

    const double period = 1.0 / std::max(0.1, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period * 1000)),
      std::bind(&CmdVelSafetyGateNode::onTimer, this));

    last_pub_stamp_ = now();

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_safety_gate: gate_enable=%d bypass_source=%s in=%s handle=%s rocker=%s cloud=%s "
      "car_info=%s out=%s "
      "input_type=%s cmd_timeout=%.2f cmd_hold=%.2f rocker_deadband=%.1f proximity_cap=%d "
      "vx_reverse_max=%.2f release_soft_stop_topic=%s wz_density_cap=%d gear_vx_cap=%d "
      "gear_vx_kph=[%.0f,%.0f,%.0f,%.0f] narrow_gate_vx_max=%.2f "
      "narrow_gate_exit_ramp=%d(%.1fs->%.2fm/s) side_speed_limit=%d(gear_only=%d) "
      "stick_vx_scale=%d(min_ratio=%.2f wz=%d) vx_accel_limit=%d(%.1fm/s^2) side_pass_crawl_enable=%d "
      "side_pass_crawl=%.2fm/s/%.2frps forward_strips=%zu depth=%.0fmm",
      gate_enable_ ? 1 : 0, bypass_source_.c_str(),
      in_stamped_topic_.c_str(), handle_topic_.c_str(), rocker_topic_.c_str(),
      obstacle_topic_.c_str(), car_info_topic_.c_str(), out_topic_.c_str(),
      input_type_.c_str(),
      cmd_timeout_, cmd_hold_timeout_, rocker_deadband_,
      proximity_speed_cap_ ? 1 : 0, vx_reverse_max_,
      release_soft_stop_mode_topic_.c_str(),
      wz_density_cap_enable_ ? 1 : 0, gear_vx_cap_enable_ ? 1 : 0,
      gear_vx_max_mps_.size() > 0 ? gear_vx_max_mps_[0] * 3.6 : 0.0,
      gear_vx_max_mps_.size() > 1 ? gear_vx_max_mps_[1] * 3.6 : 0.0,
      gear_vx_max_mps_.size() > 2 ? gear_vx_max_mps_[2] * 3.6 : 0.0,
      gear_vx_max_mps_.size() > 3 ? gear_vx_max_mps_[3] * 3.6 : 0.0,
      narrow_gate_vx_max_,
      narrow_gate_exit_ramp_enable_ ? 1 : 0,
      narrow_gate_exit_ramp_sec_, narrow_gate_exit_ramp_vx_target_,
      side_speed_limit_enable_ ? 1 : 0, side_speed_limit_gear_only_,
      stick_vx_scale_enable_ ? 1 : 0, stick_vx_min_ratio_,
      stick_wz_scale_enable_ ? 1 : 0,
      vx_accel_limit_enable_ ? 1 : 0, vx_accel_max_mps2_,
      side_pass_crawl_enable_ ? 1 : 0,
      side_pass_crawl_vx_mps_, side_pass_crawl_wz_rps_,
      side_pass_crawl_forward_strip_x_.size(),
      side_pass_crawl_check_depth_m_ * 1000.0);
    for (size_t i = 0; i < side_pass_crawl_forward_strip_x_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(),
        "  side_pass forward strip[%zu]: x=[%.3f, %.3f] y=[%.3f, %.3f]",
        i,
        side_pass_crawl_forward_strip_x_[i],
        side_pass_crawl_forward_strip_x_[i] + side_pass_crawl_check_depth_m_,
        side_pass_crawl_forward_strip_y_min_[i],
        side_pass_crawl_forward_strip_y_max_[i]);
    }
    for (size_t i = 0; i < side_pass_crawl_rear_strip_x_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(),
        "  side_pass rear strip[%zu]: x=[%.3f, %.3f] y=[%.3f, %.3f]",
        i,
        side_pass_crawl_rear_strip_x_[i] - side_pass_crawl_check_depth_m_,
        side_pass_crawl_rear_strip_x_[i],
        side_pass_crawl_rear_strip_y_min_[i],
        side_pass_crawl_rear_strip_y_max_[i]);
    }
    for (size_t i = 0; i < side_pass_crawl_lateral_strip_x_min_.size(); ++i) {
      const double ye = side_pass_crawl_lateral_strip_y_edge_[i];
      const double depth = side_pass_crawl_check_depth_m_;
      RCLCPP_INFO(
        get_logger(),
        "  side_pass lateral strip[%zu]: x=[%.3f, %.3f] left_y=[%.3f, %.3f] "
        "right_y=[%.3f, %.3f]",
        i,
        side_pass_crawl_lateral_strip_x_min_[i],
        side_pass_crawl_lateral_strip_x_max_[i],
        ye, ye + depth, -(ye + depth), -ye);
    }
    RCLCPP_INFO(
      get_logger(),
      "rocker_sector_intent_filter=%d front_half=%.0fdeg side_span=%.0fdeg "
      "(left/right front each side_span wide)",
      rocker_sector_intent_filter_enable_ ? 1 : 0,
      rocker_intent_front_half_deg_, rocker_intent_side_span_deg_);
    if (left_turn_wz_trick_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "left_turn_wz_trick: +%.2f deg/s (%.4f rad/s) when |output wz|>%.3f",
        left_turn_wz_trick_offset_deg_s_, left_turn_wz_trick_offset_rps_, intent_eps_);
    }
    RCLCPP_INFO(
      get_logger(),
      "open_corridor_cruise: topic=%s gear>=%d front_half=%.0fdeg jerk=%.2f accel=%.2f pre_exit@jerk=%.2f decel=%.2f crawl_kph=%.1f exit=%d@jerk=%.2f decel=%.2f handoff=%d@jerk=%.2f accel=%.2f prox_exit=%d vx_kph=[%.0f,%.0f,%.0f,%.0f]",
      open_corridor_cruise_mode_topic_.c_str(), open_corridor_cruise_min_gear_,
      open_corridor_cruise_front_half_deg_, open_corridor_cruise_jerk_mps3_,
      open_corridor_cruise_accel_mps2_,
      open_corridor_cruise_pre_exit_jerk_mps3_, open_corridor_cruise_pre_exit_decel_mps2_,
      open_corridor_cruise_pre_exit_exit_speed_kph_,
      open_corridor_cruise_exit_hold_frames_, open_corridor_cruise_exit_jerk_mps3_,
      open_corridor_cruise_exit_decel_mps2_,
      open_corridor_cruise_mppi_handoff_enable_ ? 1 : 0,
      open_corridor_cruise_mppi_handoff_jerk_mps3_,
      open_corridor_cruise_mppi_handoff_accel_mps2_,
      proximity_speed_cap_during_cruise_exit_ ? 1 : 0,
      open_corridor_cruise_gear_vx_max_mps_.size() > 0 ?
      open_corridor_cruise_gear_vx_max_mps_[0] * 3.6 : 0.0,
      open_corridor_cruise_gear_vx_max_mps_.size() > 1 ?
      open_corridor_cruise_gear_vx_max_mps_[1] * 3.6 : 0.0,
      open_corridor_cruise_gear_vx_max_mps_.size() > 2 ?
      open_corridor_cruise_gear_vx_max_mps_[2] * 3.6 : 0.0,
      open_corridor_cruise_gear_vx_max_mps_.size() > 3 ?
      open_corridor_cruise_gear_vx_max_mps_[3] * 3.6 : 0.0);
  }

private:
  void onNarrowGatePassMode(const Bool::SharedPtr msg)
  {
    narrow_gate_pass_active_.store(msg->data, std::memory_order_relaxed);
  }

  void onOpenCorridorCruisePreExitMode(const Bool::SharedPtr msg)
  {
    open_corridor_cruise_pre_exit_active_.store(
      msg->data, std::memory_order_relaxed);
  }

  void onNarrowGateSidePassMode(const Bool::SharedPtr msg)
  {
    narrow_gate_side_pass_crawl_active_.store(msg->data, std::memory_order_relaxed);
  }

  void onOpenCorridorCruiseMode(const Bool::SharedPtr msg)
  {
    const bool active = msg->data;
    const bool was = open_corridor_cruise_active_.exchange(active, std::memory_order_relaxed);
    if (!was && active) {
      open_cruise_mppi_handoff_active_ = false;
      open_cruise_handoff_vx_ = 0.0;
      open_cruise_handoff_ax_ = 0.0;
      open_cruise_exiting_ = false;
      open_cruise_exit_frames_ = 0;
      open_cruise_vx_ = last_out_.linear.x;
      open_cruise_ax_ = 0.0;
    } else if (was && !active) {
      open_corridor_cruise_pre_exit_active_.store(false, std::memory_order_relaxed);
      open_cruise_exiting_ = true;
      open_cruise_exit_frames_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "open_corridor_cruise exit: hold %d frames @ jerk=%.2f decel=%.2fm/s^2",
        open_corridor_cruise_exit_hold_frames_, open_corridor_cruise_exit_jerk_mps3_,
        open_corridor_cruise_exit_decel_mps2_);
    }
  }

  void onReleaseSoftStopMode(const Bool::SharedPtr msg)
  {
    release_soft_stop_active_.store(msg->data, std::memory_order_relaxed);
  }

  void onStickReleaseHalt(const Bool::SharedPtr msg)
  {
    stick_release_halt_active_.store(msg->data, std::memory_order_relaxed);
  }

  void onTwist(const Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_twist_ = *msg;
    latest_input_stamp_ = now();
    have_input_ = true;
  }

  void onTwistStamped(const TwistStamped::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_twist_ = msg->twist;
    latest_input_stamp_ =
      (stamp.nanoseconds() == 0) ? now() : stamp;
    have_input_ = true;
  }

  void onHandle(const Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(handle_mutex_);
    latest_handle_ = *msg;
    have_handle_ = true;
  }

  void onJoystick(const Joystick::SharedPtr msg)
  {
    const double rx = applyRockerDeadbandAxis(static_cast<double>(msg->rocker_x));
    const double ry = applyRockerDeadbandAxis(static_cast<double>(msg->rocker_y));
    latest_rocker_x_.store(static_cast<int16_t>(std::lround(rx)), std::memory_order_relaxed);
    latest_rocker_y_.store(static_cast<int16_t>(std::lround(ry)), std::memory_order_relaxed);
    const bool at_origin = (std::abs(rx) <= 1e-6) && (std::abs(ry) <= 1e-6);
    rocker_at_origin_.store(at_origin, std::memory_order_relaxed);
    latest_rocker_stamp_ns_.store(now().nanoseconds(), std::memory_order_relaxed);
    have_rocker_.store(true, std::memory_order_relaxed);
  }

  void onCarInfo(const CarInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(gear_mutex_);
    latest_gear_ = static_cast<int>(msg->gear);
    have_gear_ = true;
  }

  static double pickGearTable(
    const std::vector<double> & table, int gear, int fallback_g)
  {
    if (table.empty()) {
      return 0.0;
    }
    if (gear >= 1 && gear <= static_cast<int>(table.size())) {
      return table[static_cast<size_t>(gear - 1)];
    }
    const int fb = std::max(1, std::min(fallback_g, static_cast<int>(table.size())));
    return table[static_cast<size_t>(fb - 1)];
  }

  double gearVxMaxMps() const
  {
    int gear = gear_default_index_;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      if (have_gear_ && latest_gear_ >= 1) {
        gear = latest_gear_;
      }
    }
    return pickGearTable(gear_vx_max_mps_, gear, gear_default_index_);
  }

  void applyGearVxCap(Twist & out) const
  {
    if (!gear_vx_cap_enable_) {
      return;
    }
    const double cap = gearVxMaxMps();
    if (cap <= 0.0) {
      return;
    }
    if (out.linear.x > 0.0) {
      out.linear.x = std::min(out.linear.x, cap);
    } else if (out.linear.x < 0.0) {
      out.linear.x = std::max(out.linear.x, -cap);
    }
  }

  void scaleWzWithLinearSpeedReduction(Twist & out, double original_vx) const
  {
    constexpr double eps = 1e-4;
    if (std::abs(original_vx) <= intent_eps_) {
      return;
    }

    const double new_vx = out.linear.x;
    const bool reduced =
      (original_vx > intent_eps_ && new_vx < original_vx - eps) ||
      (original_vx < -intent_eps_ && new_vx > original_vx + eps);
    if (!reduced) {
      return;
    }

    out.angular.z *= (new_vx / original_vx);
  }

  static double tighterSideLimitWz(double scaled_from_mppi, double upstream_capped)
  {
    constexpr double eps = 1e-4;
    if (std::abs(scaled_from_mppi) <= eps) {
      return scaled_from_mppi;
    }
    if (std::abs(upstream_capped) <= eps) {
      return upstream_capped;
    }
    if (scaled_from_mppi > eps && upstream_capped > eps) {
      return std::min(scaled_from_mppi, upstream_capped);
    }
    if (scaled_from_mppi < -eps && upstream_capped < -eps) {
      return std::max(scaled_from_mppi, upstream_capped);
    }
    return std::abs(scaled_from_mppi) <= std::abs(upstream_capped)
      ? scaled_from_mppi : upstream_capped;
  }

  double minSideObstacleClearanceM() const
  {
    double best = std::numeric_limits<double>::infinity();
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto & p : cloud_pts_) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x < side_speed_limit_x_min_m_ || x > side_speed_limit_x_max_m_) {
        continue;
      }
      const double edge_clearance = std::abs(y) - side_speed_limit_body_half_width_m_;
      if (edge_clearance < best) {
        best = edge_clearance;
      }
    }
    return best;
  }

  double sideObstacleTargetVx(double clearance_m) const
  {
    if (!std::isfinite(clearance_m) ||
      clearance_m > side_speed_limit_dist_m_.back())
    {
      return std::numeric_limits<double>::infinity();
    }

    const double d = std::max(0.0, clearance_m);
    if (d <= side_speed_limit_dist_m_.front()) {
      return side_speed_limit_vx_mps_.front();
    }

    for (size_t i = 1; i < side_speed_limit_dist_m_.size(); ++i) {
      if (d <= side_speed_limit_dist_m_[i]) {
        const double span = side_speed_limit_dist_m_[i] - side_speed_limit_dist_m_[i - 1];
        const double t = (d - side_speed_limit_dist_m_[i - 1]) / span;
        return side_speed_limit_vx_mps_[i - 1] +
          t * (side_speed_limit_vx_mps_[i] - side_speed_limit_vx_mps_[i - 1]);
      }
    }
    return side_speed_limit_vx_mps_.back();
  }

  void applySideObstacleSpeedLimit(
    Twist & out, double dt, bool cloud_ok, double mppi_raw_vx, double mppi_raw_wz)
  {
    if (!side_speed_limit_enable_ || !cloud_ok || dt <= 0.0 ||
      out.linear.x <= intent_eps_)
    {
      return;
    }
    if (side_speed_limit_gear_only_ > 0 &&
      currentGearIndex() != side_speed_limit_gear_only_)
    {
      return;
    }

    const double clearance = minSideObstacleClearanceM();
    const double v_cap = sideObstacleTargetVx(clearance);
    if (!std::isfinite(v_cap)) {
      return;
    }

    const double original_vx = out.linear.x;
    const double upstream_wz = out.angular.z;
    double desired = std::min(original_vx, v_cap);
    const double prev_vx = last_out_.linear.x;
    if (prev_vx > desired) {
      desired = std::max(desired, prev_vx - side_speed_limit_decel_mps2_ * dt);
    }
    if (desired < original_vx - 1e-4) {
      out.linear.x = desired;
      const double scaled_wz =
        std::abs(mppi_raw_vx) > intent_eps_
        ? mppi_raw_wz * (desired / mppi_raw_vx)
        : mppi_raw_wz;
      out.angular.z = side_speed_limit_wz_tighten_enable_
        ? tighterSideLimitWz(scaled_wz, upstream_wz)
        : scaled_wz;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "side_speed_limit: clearance=%.0fcm vx %.2f -> %.2f "
        "wz mppi=%.2f@vx=%.2f scaled=%.2f upstream=%.2f -> %.2f (cap=%.2f m/s)",
        clearance * 100.0, original_vx, desired,
        mppi_raw_wz, mppi_raw_vx, scaled_wz, upstream_wz, out.angular.z, v_cap);
    }
  }

  void applyLeftTurnWzTrick(Twist & out) const
  {
    if (!left_turn_wz_trick_enable_ ||
      std::abs(out.angular.z) <= intent_eps_)
    {
      return;
    }
    out.angular.z += left_turn_wz_trick_offset_rps_;
  }

  double applyRockerDeadbandAxis(double axis) const
  {
    return (std::abs(axis) <= rocker_deadband_) ? 0.0 : axis;
  }

  double stickDeflectionR(const rclcpp::Time & tnow) const
  {
    if (!rockerFresh(tnow)) {
      return 1.0;
    }
    const double rx = applyRockerDeadbandAxis(static_cast<double>(
      latest_rocker_x_.load(std::memory_order_relaxed)));
    const double ry = applyRockerDeadbandAxis(static_cast<double>(
      latest_rocker_y_.load(std::memory_order_relaxed)));
    return std::min(1.0, std::hypot(rx / 100.0, ry / 100.0));
  }

  double stickSpeedRatio(const rclcpp::Time & tnow) const
  {
    const double r = stickDeflectionR(tnow);
    return stick_vx_min_ratio_ + (1.0 - stick_vx_min_ratio_) * r;
  }

  double stickVxCapBase(bool narrow_gate_pass) const
  {
    double cap = gear_vx_cap_enable_ ? gearVxMaxMps() : std::numeric_limits<double>::infinity();
    if (narrow_gate_pass) {
      cap = std::min(cap, narrow_gate_vx_max_);
    }
    return cap;
  }

  void applyStickSpeedScale(Twist & out, const rclcpp::Time & tnow, bool narrow_gate_pass) const
  {
    if (!stick_vx_scale_enable_ || !rockerFresh(tnow)) {
      return;
    }
    const double ratio = stickSpeedRatio(tnow);
    if (ratio >= 1.0 - 1e-6) {
      return;
    }
    const double cap = stickVxCapBase(narrow_gate_pass);
    if (std::isfinite(cap) && cap > 0.0) {
      const double scaled_cap = cap * ratio;
      if (out.linear.x > 0.0) {
        out.linear.x = std::min(out.linear.x, scaled_cap);
      } else if (out.linear.x < 0.0) {
        out.linear.x = std::max(out.linear.x, -scaled_cap);
      }
    }
    if (stick_wz_scale_enable_) {
      out.angular.z *= ratio;
    }
  }

  bool rockerFresh(const rclcpp::Time & tnow) const
  {
    if (!have_rocker_.load(std::memory_order_relaxed)) {
      return false;
    }
    const auto age_ns = tnow.nanoseconds() -
      latest_rocker_stamp_ns_.load(std::memory_order_relaxed);
    return age_ns <= static_cast<int64_t>(rocker_timeout_ * 1e9);
  }

  bool isStickInOpenCruiseForwardSector(const rclcpp::Time & tnow) const
  {
    if (!rockerFresh(tnow)) {
      return false;
    }
    const double rx = static_cast<double>(
      latest_rocker_x_.load(std::memory_order_relaxed));
    const double ry = static_cast<double>(
      latest_rocker_y_.load(std::memory_order_relaxed));
    if (std::hypot(rx, ry) <= rocker_deadband_) {
      return false;
    }
    const double theta = std::atan2(ry, rx);
    const double half_rad = open_corridor_cruise_front_half_deg_ * M_PI / 180.0;
    return std::abs(theta) <= half_rad;
  }

  RockerForwardZone classifyRockerForwardZone(double rx, double ry) const
  {
    if (std::hypot(rx, ry) <= rocker_deadband_) {
      return RockerForwardZone::NONE;
    }

    const double theta = std::atan2(ry, rx);
    const double front_half_rad = rocker_intent_front_half_deg_ * M_PI / 180.0;
    const double side_span_rad = rocker_intent_side_span_deg_ * M_PI / 180.0;
    const double forward_limit_rad = front_half_rad + side_span_rad;

    if (std::abs(theta) <= front_half_rad) {
      return RockerForwardZone::FRONT;
    }
    if (theta > front_half_rad && theta <= forward_limit_rad) {
      return RockerForwardZone::LEFT_FRONT;
    }
    if (theta < -front_half_rad && theta >= -forward_limit_rad) {
      return RockerForwardZone::RIGHT_FRONT;
    }
    return RockerForwardZone::NONE;
  }

  void applyRockerSectorIntentFilter(Twist & out, const rclcpp::Time & tnow) const
  {
    if (!rocker_sector_intent_filter_enable_ || !rockerFresh(tnow)) {
      return;
    }

    const double rx = static_cast<double>(
      latest_rocker_x_.load(std::memory_order_relaxed));
    const double ry = static_cast<double>(
      latest_rocker_y_.load(std::memory_order_relaxed));

    switch (classifyRockerForwardZone(rx, ry)) {
      case RockerForwardZone::FRONT:
        if (out.linear.x < 0.0) {
          const double original_vx = out.linear.x;
          out.linear.x = 0.0;
          scaleWzWithLinearSpeedReduction(out, original_vx);
        }
        break;
      case RockerForwardZone::LEFT_FRONT:
        if (out.angular.z < 0.0) {
          out.angular.z = 0.0;
        }
        break;
      case RockerForwardZone::RIGHT_FRONT:
        if (out.angular.z > 0.0) {
          out.angular.z = 0.0;
        }
        break;
      case RockerForwardZone::NONE:
      default:
        break;
    }
  }

  void applyNarrowGateRockerIntentFilter(Twist & out, const rclcpp::Time & tnow) const
  {
    if (!narrow_gate_rocker_intent_filter_enable_ ||
      !narrow_gate_pass_active_.load(std::memory_order_relaxed) ||
      !rockerFresh(tnow))
    {
      return;
    }

    const double rx = static_cast<double>(
      latest_rocker_x_.load(std::memory_order_relaxed));
    const double ry = static_cast<double>(
      latest_rocker_y_.load(std::memory_order_relaxed));

    if (rx > narrow_gate_rocker_forward_min_) {
      if (out.linear.x < 0.0) {
        out.linear.x = 0.0;
      }
    } else if (rx < -narrow_gate_rocker_forward_min_) {
      if (out.linear.x > 0.0) {
        out.linear.x = 0.0;
      }
    }

    if (ry > narrow_gate_rocker_lateral_min_) {
      if (out.angular.z < 0.0) {
        out.angular.z = 0.0;
      }
    } else if (ry < -narrow_gate_rocker_lateral_min_) {
      if (out.angular.z > 0.0) {
        out.angular.z = 0.0;
      }
    }
  }

  void onCloud(const PointCloud2::SharedPtr msg)
  {
    std::vector<std::pair<float, float>> pts;
    pts.reserve(msg->width * std::max<uint32_t>(1, msg->height));
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    for (; ix != ix.end(); ++ix, ++iy) {
      const float x = *ix;
      const float y = *iy;
      if (std::isfinite(x) && std::isfinite(y)) {
        pts.emplace_back(x, y);
      }
    }
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud_pts_ = std::move(pts);
    latest_cloud_stamp_ = now();
    have_cloud_ = true;
  }

  int nearbyObstaclePointCountInRadius(double radius) const
  {
    const double r2 = radius * radius;
    int count = 0;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto & p : cloud_pts_) {
      if ((p.first * p.first + p.second * p.second) <= r2) {
        ++count;
      }
    }
    return count;
  }

  int nearbyObstaclePointCount() const
  {
    return nearbyObstaclePointCountInRadius(density_check_radius_);
  }

  int countObstaclePointsInBox(
    double x_min, double x_max, double y_min, double y_max) const
  {
    int count = 0;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto & p : cloud_pts_) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x >= x_min && x <= x_max && y >= y_min && y <= y_max) {
        ++count;
      }
    }
    return count;
  }

  bool sidePassRoiClear(
    double x_min, double x_max, double y_min, double y_max) const
  {
    return countObstaclePointsInBox(x_min, x_max, y_min, y_max) == 0;
  }

  bool sidePassForwardStripsClear() const
  {
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_forward_strip_x_.size(); ++i) {
      const double x0 = side_pass_crawl_forward_strip_x_[i];
      if (!sidePassRoiClear(
          x0, x0 + depth,
          side_pass_crawl_forward_strip_y_min_[i],
          side_pass_crawl_forward_strip_y_max_[i]))
      {
        return false;
      }
    }
    return true;
  }

  // Same left/front/right forward strips as joystick_local_path proximity_side_pass.
  bool mppiFailSoftStopForwardStripsClear() const
  {
    const double depth = mppi_fail_soft_stop_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_forward_strip_x_.size(); ++i) {
      const double x0 = side_pass_crawl_forward_strip_x_[i];
      if (!sidePassRoiClear(
          x0, x0 + depth,
          side_pass_crawl_forward_strip_y_min_[i],
          side_pass_crawl_forward_strip_y_max_[i]))
      {
        return false;
      }
    }
    return true;
  }

  // Left/right forward corner strips beyond check_depth, for mppi-fail wz avoid/decel.
  bool mppiFailSoftStopSideStripBlocked(bool left) const
  {
    if (side_pass_crawl_forward_strip_x_.size() < 3 ||
      side_pass_crawl_forward_strip_y_min_.size() < 3 ||
      side_pass_crawl_forward_strip_y_max_.size() < 3)
    {
      return false;
    }
    const size_t i = left ? 0 : 2;
    const double x0 =
      side_pass_crawl_forward_strip_x_[i] + mppi_fail_soft_stop_check_depth_m_;
    const double x1 = x0 + mppi_fail_soft_stop_side_probe_depth_m_;
    return !sidePassRoiClear(
      x0, x1,
      side_pass_crawl_forward_strip_y_min_[i],
      side_pass_crawl_forward_strip_y_max_[i]);
  }

  bool sidePassRearStripsClear() const
  {
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_rear_strip_x_.size(); ++i) {
      const double x0 = side_pass_crawl_rear_strip_x_[i];
      if (!sidePassRoiClear(
          x0 - depth, x0,
          side_pass_crawl_rear_strip_y_min_[i],
          side_pass_crawl_rear_strip_y_max_[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool sidePassLateralStripsClear(bool left) const
  {
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_lateral_strip_x_min_.size(); ++i) {
      const double ye = side_pass_crawl_lateral_strip_y_edge_[i];
      const double x0 = side_pass_crawl_lateral_strip_x_min_[i];
      const double x1 = side_pass_crawl_lateral_strip_x_max_[i];
      if (left) {
        if (!sidePassRoiClear(x0, x1, ye, ye + depth)) {
          return false;
        }
      } else if (!sidePassRoiClear(x0, x1, -(ye + depth), -ye)) {
        return false;
      }
    }
    return true;
  }

  Twist computeSidePassCrawlTwist(const rclcpp::Time & tnow) const
  {
    Twist out;
    if (!rockerFresh(tnow)) {
      return out;
    }

    const double rx = static_cast<double>(
      latest_rocker_x_.load(std::memory_order_relaxed));
    const double ry = static_cast<double>(
      latest_rocker_y_.load(std::memory_order_relaxed));

    if (rx > side_pass_crawl_rocker_forward_min_ &&
      sidePassForwardStripsClear())
    {
      out.linear.x = side_pass_crawl_vx_mps_;
    } else if (rx < -side_pass_crawl_rocker_forward_min_ &&
      sidePassRearStripsClear())
    {
      out.linear.x = -side_pass_crawl_vx_mps_;
    }

    if (ry > side_pass_crawl_rocker_lateral_min_ &&
      sidePassLateralStripsClear(true))
    {
      out.angular.z = side_pass_crawl_wz_rps_;
    } else if (ry < -side_pass_crawl_rocker_lateral_min_ &&
      sidePassLateralStripsClear(false))
    {
      out.angular.z = -side_pass_crawl_wz_rps_;
    }

    return out;
  }

  bool sidePassForwardRockerIntent(const rclcpp::Time & tnow) const
  {
    if (!rockerFresh(tnow)) {
      return false;
    }
    const double rx = static_cast<double>(
      latest_rocker_x_.load(std::memory_order_relaxed));
    return rx > side_pass_crawl_rocker_forward_min_;
  }

  bool sidePassAvoidTurnEligible() const
  {
    if (mppiFailSoftStopForwardStripsClear()) {
      return false;
    }
    const bool left_outer = mppiFailSoftStopSideStripBlocked(true);
    const bool right_outer = mppiFailSoftStopSideStripBlocked(false);
    return (left_outer && !right_outer) || (right_outer && !left_outer);
  }

  void clearSidePassAvoidTurn()
  {
    side_pass_avoid_turn_active_ = false;
    side_pass_avoid_turn_wz_ = 0.0;
    side_pass_avoid_turn_frame_count_ = 0;
  }

  void beginSidePassAvoidTurn(double wz0)
  {
    side_pass_avoid_turn_active_ = true;
    side_pass_avoid_turn_wz_ = wz0;
    side_pass_avoid_turn_frame_count_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "side_pass avoid turn: forward blocked, one-sided outer clear "
      "(probe=%.0fmm, max_frames=%d)",
      mppi_fail_soft_stop_side_probe_depth_m_ * 1000.0,
      mppi_fail_soft_stop_max_frames_);
  }

  void applyOneSidedOuterAvoidWzRamp(double & wz, double dt) const
  {
    const bool left_outer = mppiFailSoftStopSideStripBlocked(true);
    const bool right_outer = mppiFailSoftStopSideStripBlocked(false);
    const bool turning_left = wz > release_soft_stop_vel_eps_;
    const bool turning_right = wz < -release_soft_stop_vel_eps_;
    if ((right_outer && turning_right) || (left_outer && turning_left)) {
      wz = 0.0;
      return;
    }
    const double avoid_step =
      std::max(1e-6, mppi_fail_soft_stop_avoid_wz_accel_) * dt;
    const double wz_max = mppi_fail_soft_stop_avoid_wz_max_;
    if (right_outer && !left_outer) {
      wz = std::min(wz + avoid_step, wz_max);
    } else if (left_outer && !right_outer) {
      wz = std::max(wz - avoid_step, -wz_max);
    }
  }

  void tickSidePassAvoidTurn(Twist & out, const rclcpp::Time & tnow, bool cloud_ok)
  {
    const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
    const bool eligible =
      cloud_ok && sidePassForwardRockerIntent(tnow) && sidePassAvoidTurnEligible();

    if (side_pass_avoid_turn_active_) {
      ++side_pass_avoid_turn_frame_count_;
      const bool timeout = mppi_fail_soft_stop_max_frames_ > 0 &&
        side_pass_avoid_turn_frame_count_ >= mppi_fail_soft_stop_max_frames_;
      if (!eligible) {
        clearSidePassAvoidTurn();
        return;
      }
      if (sidePassForwardStripsClear()) {
        RCLCPP_INFO(get_logger(), "side_pass avoid turn done: forward clear");
        clearSidePassAvoidTurn();
        return;
      }
      if (timeout) {
        RCLCPP_INFO(
          get_logger(),
          "side_pass avoid turn timeout: max_frames=%d",
          mppi_fail_soft_stop_max_frames_);
        clearSidePassAvoidTurn();
        return;
      }
      applyOneSidedOuterAvoidWzRamp(side_pass_avoid_turn_wz_, dt);
      out.linear.x = 0.0;
      out.angular.z = side_pass_avoid_turn_wz_;
      return;
    }

    if (!eligible) {
      return;
    }
    if (std::abs(out.linear.x) > release_soft_stop_vel_eps_ ||
      std::abs(out.angular.z) > release_soft_stop_vel_eps_)
    {
      return;
    }
    beginSidePassAvoidTurn(out.angular.z);
    applyOneSidedOuterAvoidWzRamp(side_pass_avoid_turn_wz_, dt);
    out.linear.x = 0.0;
    out.angular.z = side_pass_avoid_turn_wz_;
  }

  bool sidePassHoldMotionStripsClear(const Twist & hold) const
  {
    const double eps = release_soft_stop_vel_eps_;
    if (hold.linear.x > eps && !sidePassForwardStripsClear()) {
      return false;
    }
    if (hold.linear.x < -eps && !sidePassRearStripsClear()) {
      return false;
    }
    if (hold.angular.z > eps && !sidePassLateralStripsClear(true)) {
      return false;
    }
    if (hold.angular.z < -eps && !sidePassLateralStripsClear(false)) {
      return false;
    }
    return true;
  }

  static double blendSignedMax(double a, double b)
  {
    if (a >= 0.0 && b >= 0.0) {
      return std::max(a, b);
    }
    if (a <= 0.0 && b <= 0.0) {
      return std::min(a, b);
    }
    return std::abs(a) >= std::abs(b) ? a : b;
  }

  void beginSidePassExitTransition()
  {
    if (!side_pass_exit_transition_enable_ || side_pass_crawl_enable_) {
      return;
    }
    side_pass_exit_transition_active_ = true;
    side_pass_exit_hold_active_ = false;
    side_pass_exit_transition_frame_count_ = 0;
    RCLCPP_INFO(
      get_logger(),
      "side_pass exit transition: max_frames=%d mppi_thresh=vx%.2f/wz%.2f (crawl)",
      side_pass_exit_transition_frames_,
      side_pass_crawl_vx_mps_, side_pass_crawl_wz_rps_);
  }

  void beginSidePassExitHold(const Twist & hold)
  {
    side_pass_exit_hold_active_ = true;
    side_pass_exit_hold_twist_ = hold;
    RCLCPP_INFO(
      get_logger(),
      "side_pass exit hold: vx=%.2f wz=%.2f until mppi |vx|>%.2f (strip depth=%.0fmm)",
      hold.linear.x, hold.angular.z, side_pass_crawl_vx_mps_,
      side_pass_crawl_check_depth_m_ * 1000.0);
  }

  void endSidePassExitTransition(const char * reason)
  {
    if (!side_pass_exit_transition_active_ && !side_pass_exit_hold_active_) {
      return;
    }
    side_pass_exit_transition_active_ = false;
    side_pass_exit_hold_active_ = false;
    side_pass_exit_transition_frame_count_ = 0;
    RCLCPP_INFO(get_logger(), "side_pass exit transition done: %s", reason);
  }

  void tickSidePassExitTransitionBlend(const Twist & mppi, bool input_ok, const Twist & blend_out)
  {
    if (side_pass_exit_hold_active_) {
      return;
    }
    ++side_pass_exit_transition_frame_count_;
    const double mppi_vx = input_ok ? mppi.linear.x : 0.0;
    const double mppi_wz = input_ok ? mppi.angular.z : 0.0;
    if (side_pass_exit_transition_frame_count_ >= side_pass_exit_transition_frames_) {
      beginSidePassExitHold(blend_out);
      return;
    }
    if (input_ok &&
      (std::abs(mppi_vx) > side_pass_crawl_vx_mps_ ||
      std::abs(mppi_wz) > side_pass_crawl_wz_rps_))
    {
      endSidePassExitTransition("mppi above threshold");
    }
  }

  void applySidePassExitTransitionFilters(
    Twist & out, const rclcpp::Time & tnow, double effective_wz_max,
    bool cloud_ok, double mppi_raw_vx, double mppi_raw_wz)
  {
    applyNarrowGateExitVxRamp(out, tnow);
    applyGearVxCap(out);
    applyStickSpeedScale(out, tnow, false);
    if (out.linear.x < 0.0) {
      out.linear.x = std::max(out.linear.x, -vx_reverse_max_);
    }
    out.angular.z = clamp(out.angular.z, -effective_wz_max, effective_wz_max);
    applyNarrowGateRockerIntentFilter(out, tnow);
    applyRockerSectorIntentFilter(out, tnow);
    const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
    applySideObstacleSpeedLimit(out, dt, cloud_ok, mppi_raw_vx, mppi_raw_wz);
    applyVxAccelLimit(out, dt);
  }

  double openCorridorCruiseVxMax(int gear) const
  {
    return pickGearTable(
      open_corridor_cruise_gear_vx_max_mps_, gear, gear_default_index_);
  }

  int currentGearIndex() const
  {
    int gear = gear_default_index_;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      if (have_gear_ && latest_gear_ >= 1) {
        gear = latest_gear_;
      }
    }
    return gear;
  }

  double openCorridorCruiseWzFromRocker(const rclcpp::Time & tnow, int gear) const
  {
    if (!rockerFresh(tnow)) {
      return 0.0;
    }
    const double ry = static_cast<double>(
      latest_rocker_y_.load(std::memory_order_relaxed));
    if (std::abs(ry) <= rocker_deadband_) {
      return 0.0;
    }
    const double norm_y = clamp(ry / 100.0, -1.0, 1.0);
    const double wz_max = pickGearTable(
      open_corridor_cruise_gear_wz_max_rps_, gear, gear_default_index_);
    return norm_y * wz_max;
  }

  double openCorridorCruiseDriveSign(
    const rclcpp::Time & tnow, const Twist & handle, bool handle_valid) const
  {
    if (!handle_valid) {
      return 0.0;
    }
    if (handle.linear.x > intent_eps_) {
      return 1.0;
    }
    if (handle.linear.x < -intent_eps_) {
      return -1.0;
    }
    if (rockerFresh(tnow)) {
      const double rx = static_cast<double>(
        latest_rocker_x_.load(std::memory_order_relaxed));
      if (rx > rocker_deadband_) {
        return 1.0;
      }
      if (rx < -rocker_deadband_) {
        return -1.0;
      }
    }
    return 0.0;
  }

  Twist applyOpenCruiseJerkTowardVx(
    const rclcpp::Time & tnow, double target_vx, double max_accel, double jerk,
    bool clamp_forward_nonnegative = false)
  {
    Twist out;
    const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
    const double max_da = jerk * dt;

    if (open_cruise_vx_ < target_vx - 1e-6) {
      open_cruise_ax_ = std::min(max_accel, open_cruise_ax_ + max_da);
    } else if (open_cruise_vx_ > target_vx + 1e-6) {
      open_cruise_ax_ = std::max(-max_accel, open_cruise_ax_ - max_da);
    } else if (open_cruise_ax_ > 0.0) {
      open_cruise_ax_ = std::max(0.0, open_cruise_ax_ - max_da);
    } else if (open_cruise_ax_ < 0.0) {
      open_cruise_ax_ = std::min(0.0, open_cruise_ax_ + max_da);
    } else {
      open_cruise_ax_ = 0.0;
    }

    open_cruise_vx_ += open_cruise_ax_ * dt;
    if (open_cruise_ax_ >= 0.0 && open_cruise_vx_ > target_vx) {
      open_cruise_vx_ = target_vx;
      open_cruise_ax_ = 0.0;
    } else if (open_cruise_ax_ <= 0.0 && open_cruise_vx_ < target_vx) {
      open_cruise_vx_ = target_vx;
      open_cruise_ax_ = 0.0;
    }

    if (clamp_forward_nonnegative && target_vx >= 0.0 && open_cruise_vx_ < 0.0) {
      open_cruise_vx_ = 0.0;
      open_cruise_ax_ = 0.0;
    }

    out.linear.x = open_cruise_vx_;
    out.angular.z = openCorridorCruiseWzFromRocker(tnow, currentGearIndex());
    return out;
  }

  Twist computeOpenCorridorCruiseTwist(
    const rclcpp::Time & tnow, const Twist & handle, bool handle_valid)
  {
    Twist out;
    if (!handle_valid) {
      return out;
    }

    const int gear = currentGearIndex();
    if (gear < open_corridor_cruise_min_gear_) {
      return out;
    }

    const double sign = openCorridorCruiseDriveSign(tnow, handle, handle_valid);
    if (sign == 0.0) {
      return out;
    }

    const double speed_ratio =
      stick_vx_scale_enable_ ? stickSpeedRatio(tnow) : 1.0;
    const double target_vx = sign * openCorridorCruiseVxMax(gear) * speed_ratio;
    const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
    const double max_accel = open_corridor_cruise_accel_mps2_;
    const double jerk = open_corridor_cruise_jerk_mps3_;
    const double max_da = jerk * dt;
    if (target_vx > open_cruise_vx_ + 1e-6) {
      // 匀 jerk 提速，加速度上限 max_accel
      open_cruise_ax_ = std::min(max_accel, open_cruise_ax_ + max_da);
      open_cruise_vx_ += open_cruise_ax_ * dt;
      if (open_cruise_vx_ >= target_vx) {
        open_cruise_vx_ = target_vx;
        open_cruise_ax_ = 0.0;
      }
    } else if (target_vx < open_cruise_vx_ - 1e-6) {
      // 降速仍用恒定减速度
      open_cruise_ax_ = 0.0;
      const double max_step = max_accel * dt;
      open_cruise_vx_ = std::max(target_vx, open_cruise_vx_ - max_step);
    } else {
      open_cruise_vx_ = target_vx;
      if (open_cruise_ax_ > 0.0) {
        open_cruise_ax_ = std::max(0.0, open_cruise_ax_ - max_da);
      } else {
        open_cruise_ax_ = 0.0;
      }
    }
    if (sign > 0.0 && open_cruise_vx_ < 0.0) {
      open_cruise_vx_ = 0.0;
      open_cruise_ax_ = 0.0;
    }
    out.linear.x = open_cruise_vx_;
    out.angular.z = openCorridorCruiseWzFromRocker(tnow, gear);
    return out;
  }

  Twist applyOpenCruiseJerkDecelTwist(
    const rclcpp::Time & tnow, double max_decel, double jerk)
  {
    Twist out;
    const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
    const double max_da = jerk * dt;
    if (open_cruise_vx_ > 1e-6) {
      const double desired_a = -max_decel;
      if (open_cruise_ax_ > desired_a) {
        open_cruise_ax_ = std::max(desired_a, open_cruise_ax_ - max_da);
      }
      open_cruise_vx_ += open_cruise_ax_ * dt;
      if (open_cruise_vx_ <= 0.0) {
        open_cruise_vx_ = 0.0;
        open_cruise_ax_ = 0.0;
      }
    } else if (open_cruise_vx_ < -1e-6) {
      const double desired_a = max_decel;
      if (open_cruise_ax_ < desired_a) {
        open_cruise_ax_ = std::min(desired_a, open_cruise_ax_ + max_da);
      }
      open_cruise_vx_ += open_cruise_ax_ * dt;
      if (open_cruise_vx_ >= 0.0) {
        open_cruise_vx_ = 0.0;
        open_cruise_ax_ = 0.0;
      }
    } else {
      open_cruise_vx_ = 0.0;
      open_cruise_ax_ = 0.0;
    }
    out.linear.x = open_cruise_vx_;
    out.angular.z = openCorridorCruiseWzFromRocker(tnow, currentGearIndex());
    return out;
  }

  Twist computeOpenCorridorCruiseExitTwist(const rclcpp::Time & tnow)
  {
    return applyOpenCruiseJerkDecelTwist(
      tnow, open_corridor_cruise_exit_decel_mps2_, open_corridor_cruise_exit_jerk_mps3_);
  }

  void endOpenCruiseMppiHandoff()
  {
    open_cruise_mppi_handoff_active_ = false;
    open_cruise_handoff_vx_ = 0.0;
    open_cruise_handoff_ax_ = 0.0;
  }

  void beginOpenCruiseMppiHandoff(const rclcpp::Time & tnow, double start_vx)
  {
    if (!open_corridor_cruise_mppi_handoff_enable_) {
      return;
    }
    open_cruise_mppi_handoff_active_ = true;
    open_cruise_handoff_vx_ = start_vx;
    open_cruise_handoff_ax_ = 0.0;
    open_cruise_handoff_start_ = tnow;
    RCLCPP_INFO(
      get_logger(),
      "open_corridor_cruise MPPI handoff: start_vx=%.2f jerk=%.2f accel=%.2f max=%.2fs",
      start_vx, open_corridor_cruise_mppi_handoff_jerk_mps3_,
      open_corridor_cruise_mppi_handoff_accel_mps2_,
      open_corridor_cruise_mppi_handoff_max_sec_);
  }

  Twist computeOpenCruiseMppiHandoffTwist(
    const rclcpp::Time & tnow, const Twist & mppi_target)
  {
    Twist out;
    const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
    const double target_vx = mppi_target.linear.x;
    const double max_accel = open_corridor_cruise_mppi_handoff_accel_mps2_;
    const double jerk = open_corridor_cruise_mppi_handoff_jerk_mps3_;
    const double max_da = jerk * dt;
    const double eps = open_corridor_cruise_mppi_handoff_vx_eps_;

    if (open_cruise_handoff_vx_ < target_vx - 1e-6) {
      open_cruise_handoff_ax_ = std::min(max_accel, open_cruise_handoff_ax_ + max_da);
    } else if (open_cruise_handoff_vx_ > target_vx + 1e-6) {
      open_cruise_handoff_ax_ = std::max(-max_accel, open_cruise_handoff_ax_ - max_da);
    } else if (open_cruise_handoff_ax_ > 0.0) {
      open_cruise_handoff_ax_ = std::max(0.0, open_cruise_handoff_ax_ - max_da);
    } else if (open_cruise_handoff_ax_ < 0.0) {
      open_cruise_handoff_ax_ = std::min(0.0, open_cruise_handoff_ax_ + max_da);
    } else {
      open_cruise_handoff_ax_ = 0.0;
    }

    open_cruise_handoff_vx_ += open_cruise_handoff_ax_ * dt;
    if (open_cruise_handoff_ax_ >= 0.0 && open_cruise_handoff_vx_ > target_vx) {
      open_cruise_handoff_vx_ = target_vx;
      open_cruise_handoff_ax_ = 0.0;
    } else if (open_cruise_handoff_ax_ <= 0.0 && open_cruise_handoff_vx_ < target_vx) {
      open_cruise_handoff_vx_ = target_vx;
      open_cruise_handoff_ax_ = 0.0;
    }

    out.linear.x = open_cruise_handoff_vx_;
    out.angular.z = mppi_target.angular.z;

    const double elapsed = (tnow - open_cruise_handoff_start_).seconds();
    if (std::abs(open_cruise_handoff_vx_ - target_vx) <= eps ||
      elapsed >= open_corridor_cruise_mppi_handoff_max_sec_)
    {
      RCLCPP_INFO(
        get_logger(),
        "open_corridor_cruise MPPI handoff complete (handoff_vx=%.2f mppi_vx=%.2f elapsed=%.2fs)",
        open_cruise_handoff_vx_, target_vx, elapsed);
      endOpenCruiseMppiHandoff();
    }
    return out;
  }

  void applyProximitySpeedCapTwist(
    Twist & out, int dir, double d, double overhang) const
  {
    if (dir == 0 || !std::isfinite(d)) {
      return;
    }
    const double clearance = d - overhang - standoff_;
    const double v_safe = (clearance <= 0.0)
      ? 0.0
      : std::sqrt(2.0 * std::max(1e-3, brake_decel_) * clearance);
    if (dir > 0) {
      out.linear.x = std::min(out.linear.x, v_safe);
    } else {
      out.linear.x = std::max(out.linear.x, -v_safe);
    }
  }

  bool shouldApplyCruiseExitProximityCap() const
  {
    return proximity_speed_cap_during_cruise_exit_ &&
      (open_cruise_exiting_ ||
      open_cruise_mppi_handoff_active_ ||
      open_corridor_cruise_pre_exit_active_.load(std::memory_order_relaxed));
  }

  Twist computeOpenCorridorCruisePreExitTwist(
    const rclcpp::Time & tnow, const Twist & handle, bool handle_valid)
  {
    const double sign = openCorridorCruiseDriveSign(tnow, handle, handle_valid);
    if (sign > 0.0) {
      const double crawl_mps = open_corridor_cruise_pre_exit_exit_speed_kph_ / 3.6;
      return applyOpenCruiseJerkTowardVx(
        tnow, crawl_mps, open_corridor_cruise_pre_exit_decel_mps2_,
        open_corridor_cruise_pre_exit_jerk_mps3_, true);
    }
    return applyOpenCruiseJerkDecelTwist(
      tnow, open_corridor_cruise_pre_exit_decel_mps2_,
      open_corridor_cruise_pre_exit_jerk_mps3_);
  }

  void beginNarrowGateExitRamp(const rclcpp::Time & tnow)
  {
    if (!narrow_gate_exit_ramp_enable_) {
      return;
    }
    narrow_gate_exit_ramp_active_ = true;
    narrow_gate_exit_ramp_start_ = tnow;
    RCLCPP_INFO(
      get_logger(),
      "narrow gate exit: vx ramp %.2f -> %.2f m/s over %.1fs",
      narrow_gate_vx_max_, narrow_gate_exit_ramp_vx_target_,
      narrow_gate_exit_ramp_sec_);
  }

  void endNarrowGateExitRamp()
  {
    narrow_gate_exit_ramp_active_ = false;
  }

  double narrowGateExitVxCap(const rclcpp::Time & tnow) const
  {
    if (!narrow_gate_exit_ramp_active_) {
      return std::numeric_limits<double>::infinity();
    }
    const double elapsed = (tnow - narrow_gate_exit_ramp_start_).seconds();
    if (elapsed >= narrow_gate_exit_ramp_sec_) {
      return std::numeric_limits<double>::infinity();
    }
    const double t = elapsed / narrow_gate_exit_ramp_sec_;
    return narrow_gate_vx_max_ +
      t * (narrow_gate_exit_ramp_vx_target_ - narrow_gate_vx_max_);
  }

  void applyNarrowGateExitVxRamp(Twist & out, const rclcpp::Time & tnow)
  {
    if (!narrow_gate_exit_ramp_active_) {
      return;
    }
    const double cap = narrowGateExitVxCap(tnow);
    if (!std::isfinite(cap)) {
      endNarrowGateExitRamp();
      return;
    }
    if (out.linear.x > cap) {
      out.linear.x = cap;
    }
    if ((tnow - narrow_gate_exit_ramp_start_).seconds() >= narrow_gate_exit_ramp_sec_) {
      endNarrowGateExitRamp();
    }
  }

  void applyMppiFailSoftStopVxJerkDecel(double dt)
  {
    const double max_da = mppi_fail_soft_stop_jerk_mps3_ * dt;
    const double max_decel = mppi_fail_soft_stop_decel_mps2_;

    if (std::abs(soft_stop_vx_) <= release_soft_stop_vel_eps_) {
      soft_stop_vx_ = 0.0;
      soft_stop_vx_ax_ = 0.0;
      return;
    }

    if (soft_stop_vx_ > release_soft_stop_vel_eps_) {
      const double desired_a = -max_decel;
      if (soft_stop_vx_ax_ > desired_a) {
        soft_stop_vx_ax_ = std::max(desired_a, soft_stop_vx_ax_ - max_da);
      }
      soft_stop_vx_ += soft_stop_vx_ax_ * dt;
      if (soft_stop_vx_ <= release_soft_stop_vel_eps_) {
        soft_stop_vx_ = 0.0;
        soft_stop_vx_ax_ = 0.0;
      }
    } else {
      const double desired_a = max_decel;
      if (soft_stop_vx_ax_ < desired_a) {
        soft_stop_vx_ax_ = std::min(desired_a, soft_stop_vx_ax_ + max_da);
      }
      soft_stop_vx_ += soft_stop_vx_ax_ * dt;
      if (soft_stop_vx_ >= -release_soft_stop_vel_eps_) {
        soft_stop_vx_ = 0.0;
        soft_stop_vx_ax_ = 0.0;
      }
    }
  }

  void applyMppiFailSoftStopWzRamp(double dt)
  {
    applyOneSidedOuterAvoidWzRamp(soft_stop_wz_, dt);
  }

  void applySoftStopRamp(Twist & out, double dt)
  {
    const double lin_decel = release_soft_stop_decel_;
    const double wz_decel = release_soft_stop_wz_decel_;
    const double lin_step = std::max(1e-6, lin_decel) * dt;
    const double wz_step = std::max(1e-6, wz_decel) * dt;

    if (soft_stop_ramp_legacy_) {
      if (std::abs(soft_stop_vx_) <= release_soft_stop_vel_eps_) {
        soft_stop_vx_ = 0.0;
      } else if (soft_stop_vx_ > 0.0) {
        soft_stop_vx_ = std::max(0.0, soft_stop_vx_ - lin_step);
      } else {
        soft_stop_vx_ = std::min(0.0, soft_stop_vx_ + lin_step);
      }
    } else {
      applyMppiFailSoftStopVxJerkDecel(dt);
    }

    if (soft_stop_ramp_legacy_) {
      if (std::abs(soft_stop_wz_) <= release_soft_stop_vel_eps_) {
        soft_stop_wz_ = 0.0;
      } else if (soft_stop_wz_ > 0.0) {
        soft_stop_wz_ = std::max(0.0, soft_stop_wz_ - wz_step);
      } else {
        soft_stop_wz_ = std::min(0.0, soft_stop_wz_ + wz_step);
      }
    } else {
      applyMppiFailSoftStopWzRamp(dt);
    }

    out.linear.x = soft_stop_vx_;
    out.angular.z = soft_stop_wz_;
    if (out.linear.x < 0.0) {
      out.linear.x = std::max(out.linear.x, -vx_reverse_max_);
    }
  }

  void applySoftStopLowPass(Twist & out, double dt)
  {
    const double valid_dt = std::max(1e-3, std::isfinite(dt) ? dt : 0.0);
    const double alpha = std::exp(
      -valid_dt / release_soft_stop_time_constant_s_);
    const double alpha_wz = std::exp(
      -valid_dt / release_soft_stop_angular_time_constant_s_);

    soft_stop_vx_ *= alpha;
    soft_stop_wz_ *= alpha_wz;

    if (std::abs(soft_stop_vx_) <= release_soft_stop_vel_eps_) {
      soft_stop_vx_ = 0.0;
    }
    if (std::abs(soft_stop_wz_) <= release_soft_stop_vel_eps_) {
      soft_stop_wz_ = 0.0;
    }

    out.linear.x = soft_stop_vx_;
    out.angular.z = soft_stop_wz_;
    if (out.linear.x < 0.0) {
      out.linear.x = std::max(out.linear.x, -vx_reverse_max_);
    }
  }

  void applyVxAccelLimit(Twist & out, double dt) const
  {
    if (!vx_accel_limit_enable_ || dt <= 0.0) {
      return;
    }

    const double prev_vx = last_out_.linear.x;
    const double desired_vx = out.linear.x;
    const double max_step = vx_accel_max_mps2_ * dt;
    constexpr double eps = 1e-4;

    const bool same_sign_forward = prev_vx >= -eps && desired_vx >= -eps;
    const bool same_sign_reverse = prev_vx <= eps && desired_vx <= eps;
    const bool accelerating_forward =
      same_sign_forward && desired_vx > prev_vx + eps;
    const bool accelerating_reverse =
      same_sign_reverse && desired_vx < prev_vx - eps;

    if (accelerating_forward) {
      out.linear.x = std::min(desired_vx, prev_vx + max_step);
    } else if (accelerating_reverse) {
      out.linear.x = std::max(desired_vx, prev_vx - max_step);
    }
  }

  bool canSoftStopOnRelease(bool cloud_ok) const
  {
    return release_soft_stop_enable_ && cloud_ok &&
           nearbyObstaclePointCountInRadius(release_soft_stop_radius_) == 0;
  }

  bool canSoftStopOnMppiFail(bool cloud_ok) const
  {
    return mppi_fail_soft_stop_enable_ && cloud_ok &&
           mppiFailSoftStopForwardStripsClear();
  }

  double minRangeInSectorHalf(double center_angle, double half_angle) const
  {
    double best = std::numeric_limits<double>::infinity();
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto & p : cloud_pts_) {
      const double ang = std::atan2(p.second, p.first);
      if (angleDiff(ang, center_angle) <= half_angle) {
        const double r = std::hypot(p.first, p.second);
        if (r < best) {
          best = r;
        }
      }
    }
    return best;
  }

  double softStopBrakeDistance(double vx) const
  {
    const double speed = std::abs(vx);
    if (speed <= release_soft_stop_vel_eps_) {
      return 0.0;
    }
    return speed * release_soft_stop_time_constant_s_;
  }

  bool softStopBrakeClearanceOk(double vx) const
  {
    if (!release_soft_stop_brake_check_enable_) {
      return true;
    }

    const double stop_dist = softStopBrakeDistance(vx);
    if (stop_dist <= 1e-6) {
      return true;
    }

    double overhang = front_overhang_;
    double center_angle = 0.0;
    if (vx > release_soft_stop_vel_eps_) {
      overhang = front_overhang_;
      center_angle = 0.0;
    } else if (vx < -release_soft_stop_vel_eps_) {
      overhang = rear_overhang_;
      center_angle = M_PI;
    } else {
      return true;
    }

    const double d = minRangeInSectorHalf(
      center_angle, release_soft_stop_brake_sector_half_angle_);
    if (!std::isfinite(d)) {
      return true;
    }

    const double clearance = d - overhang - release_soft_stop_standoff_;
    return stop_dist <= clearance;
  }

  bool canSoftStopOnReleaseLowPass(bool cloud_ok) const
  {
    return cloud_ok && softStopBrakeClearanceOk(last_out_.linear.x);
  }

  bool canContinueSoftStopLowPass() const
  {
    return softStopBrakeClearanceOk(soft_stop_vx_);
  }

  bool isTwistNearZero(const Twist & twist) const
  {
    return std::abs(twist.linear.x) <= release_soft_stop_vel_eps_ &&
           std::abs(twist.angular.z) <= release_soft_stop_vel_eps_;
  }

  void applyReleaseLikeStop(
    bool cloud_ok, bool & halted, std::string & reason, const char * context)
  {
    if (canSoftStopOnRelease(cloud_ok)) {
      beginSoftStopRampLegacy();
      return;
    }
    halted = true;
    endSoftStop();
    if (!cloud_ok) {
      reason = std::string(context) + " (cloud stale)";
    } else if (nearbyObstaclePointCountInRadius(release_soft_stop_radius_) > 0) {
      reason = std::string(context) + " (obstacles nearby)";
    } else {
      reason = std::string(context) + " (soft-stop brake clearance)";
    }
  }

  void applyMppiFailSoftStop(
    bool cloud_ok, bool & halted, std::string & reason, const char * context)
  {
    if (canSoftStopOnMppiFail(cloud_ok)) {
      beginSoftStopRampMppiFail();
      return;
    }
    halted = true;
    endSoftStop();
    if (!cloud_ok) {
      reason = std::string(context) + " (cloud stale)";
    } else {
      reason = std::string(context) + " (left/front/right strips blocked)";
    }
  }

  void readCmdVelInput(
    const rclcpp::Time & tnow, Twist & target, bool & input_ok,
    bool & halted, std::string & reason, bool halt_on_stale = true)
  {
    input_ok = false;
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!have_input_) {
      return;
    }
    const double age = (tnow - latest_input_stamp_).seconds();
    if (age <= cmd_hold_timeout_) {
      target = latest_twist_;
      input_ok = true;
      if (age > cmd_timeout_) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "cmd_vel hold (age=%.3fs, fresh<=%.2fs hold<=%.2fs)",
          age, cmd_timeout_, cmd_hold_timeout_);
      }
      return;
    }
    if (!halt_on_stale) {
      return;
    }
    halted = true;
    reason = "cmd_vel stale";
    clearMppiFailSoftStop();
    endSoftStop();
  }

  void tryRecoverFromMppiFailSoftStop(const Twist & target, bool input_ok)
  {
    if (!mppi_fail_soft_stop_ || !input_ok || isTwistNearZero(target)) {
      return;
    }
    clearMppiFailSoftStop();
    endSoftStop();
    RCLCPP_INFO(
      get_logger(),
      "mppi fail soft-stop recovered (cmd_vel vx=%.2f wz=%.2f)",
      target.linear.x, target.angular.z);
  }

  void beginSoftStopRampLegacy()
  {
    if (!soft_stopping_) {
      soft_stop_ramp_mode_ = true;
      soft_stop_ramp_legacy_ = true;
      soft_stop_vx_ = last_out_.linear.x;
      soft_stop_wz_ = last_out_.angular.z;
      soft_stopping_ = true;
      RCLCPP_INFO(
        get_logger(),
        "release soft-stop ramp: v0=%.2f wz0=%.2f (clear within %.1fm)",
        soft_stop_vx_, soft_stop_wz_, release_soft_stop_radius_);
    }
  }

  void beginSoftStopRampMppiFail()
  {
    if (!soft_stopping_) {
      soft_stop_ramp_mode_ = true;
      soft_stop_ramp_legacy_ = false;
      soft_stop_vx_ = last_out_.linear.x;
      soft_stop_wz_ = last_out_.angular.z;
      soft_stop_vx_ax_ = 0.0;
      soft_stopping_ = true;
      RCLCPP_INFO(
        get_logger(),
        "mppi fail soft-stop ramp: v0=%.2f wz0=%.2f jerk=%.2f decel=%.2f avoid_wz=%.2f/%.2f",
        soft_stop_vx_, soft_stop_wz_,
        mppi_fail_soft_stop_jerk_mps3_, mppi_fail_soft_stop_decel_mps2_,
        mppi_fail_soft_stop_avoid_wz_accel_, mppi_fail_soft_stop_avoid_wz_max_);
    }
  }

  void beginSoftStopLowPass()
  {
    if (!soft_stopping_) {
      soft_stop_ramp_mode_ = false;
      soft_stop_ramp_legacy_ = false;
      soft_stop_vx_ = last_out_.linear.x;
      soft_stop_wz_ = last_out_.angular.z;
      soft_stopping_ = true;
      RCLCPP_INFO(
        get_logger(),
        "release soft-stop lowpass: v0=%.2f wz0=%.2f tau=%.2fs/%.2fs",
        soft_stop_vx_, soft_stop_wz_,
        release_soft_stop_time_constant_s_,
        release_soft_stop_angular_time_constant_s_);
    }
  }

  void endSoftStop()
  {
    soft_stopping_ = false;
    soft_stop_ramp_mode_ = false;
    soft_stop_ramp_legacy_ = false;
    soft_stop_vx_ = 0.0;
    soft_stop_wz_ = 0.0;
    soft_stop_vx_ax_ = 0.0;
  }

  void clearMppiFailSoftStop()
  {
    mppi_fail_soft_stop_ = false;
    mppi_fail_soft_stop_frame_count_ = 0;
  }

  void finishSoftStopIfDecayed()
  {
    if (!soft_stopping_ || soft_stop_ramp_mode_) {
      return;
    }
    if (std::abs(soft_stop_vx_) <= release_soft_stop_vel_eps_ &&
      std::abs(soft_stop_wz_) <= release_soft_stop_vel_eps_)
    {
      endSoftStop();
    }
  }

  void updateWzDensityTier(int nearby_count, const rclcpp::Time & tnow)
  {
    if (!wz_density_cap_enable_) {
      return;
    }

    WzDensityTier desired = current_wz_tier_;
    switch (current_wz_tier_) {
      case WzDensityTier::OPEN:
        if (nearby_count >= obstacle_count_engage_moderate_) {
          desired = WzDensityTier::MODERATE;
        }
        break;
      case WzDensityTier::MODERATE:
        if (nearby_count >= obstacle_count_engage_dense_) {
          desired = WzDensityTier::DENSE;
        } else if (nearby_count < obstacle_count_release_open_) {
          desired = WzDensityTier::OPEN;
        }
        break;
      case WzDensityTier::DENSE:
        if (nearby_count < obstacle_count_release_moderate_) {
          desired = WzDensityTier::MODERATE;
        }
        break;
    }

    if (desired == current_wz_tier_) {
      return;
    }
    if ((tnow - last_wz_tier_change_stamp_).seconds() < wz_density_min_dwell_) {
      return;
    }
    current_wz_tier_ = desired;
    last_wz_tier_change_stamp_ = tnow;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "wz obstacle tier -> %s (nearby_points=%d)",
      desired == WzDensityTier::OPEN ? "OPEN" :
      (desired == WzDensityTier::MODERATE ? "MODERATE" : "DENSE"),
      nearby_count);
  }

  double wzDensityCap() const
  {
    if (!wz_density_cap_enable_ || wz_density_caps_rps_.empty()) {
      return wz_max_;
    }
    const size_t idx = static_cast<size_t>(current_wz_tier_);
    if (idx >= wz_density_caps_rps_.size()) {
      return wz_max_;
    }
    return std::min(wz_max_, wz_density_caps_rps_[idx]);
  }

  double minRangeInSector(double center_angle) const
  {
    double best = std::numeric_limits<double>::infinity();
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto & p : cloud_pts_) {
      const double ang = std::atan2(p.second, p.first);
      if (angleDiff(ang, center_angle) <= sector_half_angle_) {
        const double r = std::hypot(p.first, p.second);
        if (r < best) {
          best = r;
        }
      }
    }
    return best;
  }

  void onTimer()
  {
    const auto tnow = now();

    if (!gate_enable_) {
      publishBypass(tnow);
      return;
    }

    bool halted = false;
    std::string reason;
    bool rocker_stale = false;

    Twist handle;
    bool handle_valid;
    {
      std::lock_guard<std::mutex> lock(handle_mutex_);
      handle = latest_handle_;
      handle_valid = have_handle_;
    }

    bool cloud_ok;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_ok = have_cloud_ &&
        (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
    }

    const double intent_vx = handle_valid ? handle.linear.x : 0.0;
    const bool narrow_gate_pass =
      narrow_gate_pass_active_.load(std::memory_order_relaxed);
    if (narrow_gate_pass_prev_ && !narrow_gate_pass) {
      beginNarrowGateExitRamp(tnow);
    } else if (narrow_gate_pass) {
      endNarrowGateExitRamp();
    }
    narrow_gate_pass_prev_ = narrow_gate_pass;

    int dir = 0;
    double overhang = front_overhang_;
    double d = std::numeric_limits<double>::infinity();
    if (intent_vx > intent_eps_) {
      dir = 1;
      overhang = front_overhang_;
      d = minRangeInSector(0.0);
    } else if (intent_vx < -intent_eps_) {
      dir = -1;
      overhang = rear_overhang_;
      d = minRangeInSector(M_PI);
    }

    const bool release_soft_stop =
      release_soft_stop_active_.load(std::memory_order_relaxed);
    const bool stick_release_halt =
      stick_release_halt_active_.load(std::memory_order_relaxed);

    if (have_rocker_.load(std::memory_order_relaxed)) {
      const auto rocker_age_ns =
        tnow.nanoseconds() -
        latest_rocker_stamp_ns_.load(std::memory_order_relaxed);
      rocker_stale = rocker_age_ns >
        static_cast<int64_t>(rocker_timeout_ * 1e9);
      if (rocker_stale) {
        halted = true;
        reason = "joystick stale";
        endSoftStop();
      } else if (stick_release_halt) {
        clearMppiFailSoftStop();
        halted = true;
        endSoftStop();
        reason = "stick released (halt)";
      } else if (release_soft_stop) {
        clearMppiFailSoftStop();
        if (!soft_stopping_) {
          if (canSoftStopOnReleaseLowPass(cloud_ok)) {
            beginSoftStopLowPass();
          } else {
            halted = true;
            endSoftStop();
            reason = "stick released (soft-stop brake clearance)";
          }
        }
      } else if (!mppi_fail_soft_stop_) {
        endSoftStop();
      }
    }

    const bool side_pass_crawl =
      !side_pass_crawl_enable_ &&
      narrow_gate_side_pass_crawl_active_.load(std::memory_order_relaxed);
    if (side_pass_crawl_prev_ && !side_pass_crawl) {
      beginSidePassExitTransition();
      clearSidePassAvoidTurn();
    }
    side_pass_crawl_prev_ = side_pass_crawl;

    if (side_pass_crawl) {
      endSidePassExitTransition("side_pass re-entered");
      Twist out;
      if (!halted && !soft_stopping_ && cloud_ok && rockerFresh(tnow) &&
        !stick_release_halt && !release_soft_stop)
      {
        out = computeSidePassCrawlTwist(tnow);
        if (side_pass_avoid_turn_enable_ && mppi_fail_soft_stop_enable_) {
          tickSidePassAvoidTurn(out, tnow, cloud_ok);
        }
      } else if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "side_pass crawl halted: %s", reason.c_str());
      } else if (!cloud_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "side_pass crawl halted: cloud stale");
      }
      applyLeftTurnWzTrick(out);
      out_pub_->publish(out);
      last_pub_stamp_ = tnow;
      last_out_ = out;
      return;
    }

    if (side_pass_exit_transition_active_) {
      if (halted || soft_stopping_ || stick_release_halt || release_soft_stop) {
        endSidePassExitTransition("halt/soft-stop");
      } else {
        Twist target;
        bool input_ok = false;
        readCmdVelInput(tnow, target, input_ok, halted, reason, false);
        const double mppi_raw_vx = target.linear.x;
        const double mppi_raw_wz = target.angular.z;
        const double mppi_vx = input_ok ? target.linear.x : 0.0;
        const double mppi_wz = input_ok ? target.angular.z : 0.0;

        double effective_wz_max = wz_max_;
        if (wz_density_cap_enable_ && input_ok) {
          if (!cloud_ok) {
            current_wz_tier_ = WzDensityTier::DENSE;
          } else {
            updateWzDensityTier(nearbyObstaclePointCount(), tnow);
          }
          effective_wz_max = wzDensityCap();
        }

        Twist out;
        if (side_pass_exit_hold_active_) {
          if (!cloud_ok) {
            halted = true;
            reason = "side_pass exit hold (cloud stale)";
            endSidePassExitTransition("cloud stale");
          } else if (!sidePassHoldMotionStripsClear(side_pass_exit_hold_twist_)) {
            halted = true;
            reason = "side_pass exit hold (motion strips blocked)";
            endSidePassExitTransition("strips blocked");
          } else if (input_ok && std::abs(mppi_vx) > side_pass_crawl_vx_mps_) {
            endSidePassExitTransition("mppi vx ready");
            out = target;
          } else {
            out = side_pass_exit_hold_twist_;
          }
        } else {
          Twist crawl;
          if (cloud_ok && rockerFresh(tnow)) {
            crawl = computeSidePassCrawlTwist(tnow);
          }
          out.linear.x = blendSignedMax(crawl.linear.x, mppi_vx);
          out.angular.z = blendSignedMax(crawl.angular.z, mppi_wz);
          tickSidePassExitTransitionBlend(target, input_ok, out);
        }

        if (!halted && side_pass_exit_transition_active_) {
          applySidePassExitTransitionFilters(
            out, tnow, effective_wz_max, cloud_ok, mppi_raw_vx, mppi_raw_wz);
          applyLeftTurnWzTrick(out);
          out_pub_->publish(out);
          last_pub_stamp_ = tnow;
          last_out_ = out;

          UInt8 tier_msg;
          tier_msg.data = static_cast<uint8_t>(current_wz_tier_);
          wz_tier_pub_->publish(tier_msg);
          return;
        }
        if (!side_pass_exit_transition_active_ && !halted && input_ok) {
          applySidePassExitTransitionFilters(
            out, tnow, effective_wz_max, cloud_ok, mppi_raw_vx, mppi_raw_wz);
          applyLeftTurnWzTrick(out);
          out_pub_->publish(out);
          last_pub_stamp_ = tnow;
          last_out_ = out;

          UInt8 tier_msg;
          tier_msg.data = static_cast<uint8_t>(current_wz_tier_);
          wz_tier_pub_->publish(tier_msg);
          return;
        }
        if (halted) {
          out = Twist();
          applyLeftTurnWzTrick(out);
          out_pub_->publish(out);
          last_pub_stamp_ = tnow;
          last_out_ = out;
          return;
        }
      }
    }

    const bool open_corridor_cruise =
      open_corridor_cruise_active_.load(std::memory_order_relaxed);
    const bool cruise_drive_ok =
      openCorridorCruiseDriveSign(tnow, handle, handle_valid) != 0.0;
    if (open_corridor_cruise || open_cruise_exiting_) {
      Twist out;
      if (halted || soft_stopping_) {
        open_cruise_exiting_ = false;
        open_cruise_exit_frames_ = 0;
        open_cruise_vx_ = 0.0;
        open_cruise_ax_ = 0.0;
        endOpenCruiseMppiHandoff();
      } else if (open_cruise_exiting_) {
        out = computeOpenCorridorCruiseExitTwist(tnow);
        ++open_cruise_exit_frames_;
        const double early_exit_mps =
          open_corridor_cruise_exit_early_speed_kph_ / 3.6;
        const bool speed_low =
          std::abs(open_cruise_vx_) <= early_exit_mps + 1e-6 ||
          std::abs(last_out_.linear.x) <= early_exit_mps + 1e-6;
        bool below_mppi = false;
        double mppi_vx = 0.0;
        if (open_corridor_cruise_exit_early_below_mppi_enable_) {
          Twist mppi_target;
          bool mppi_ok = false;
          bool halted_tmp = false;
          std::string reason_tmp;
          readCmdVelInput(tnow, mppi_target, mppi_ok, halted_tmp, reason_tmp);
          if (mppi_ok) {
            mppi_vx = mppi_target.linear.x;
            if (open_cruise_vx_ >= 0.0 && mppi_vx > 0.0) {
              below_mppi = open_cruise_vx_ < mppi_vx - 1e-6;
            } else if (open_cruise_vx_ <= 0.0 && mppi_vx < 0.0) {
              below_mppi = open_cruise_vx_ > mppi_vx + 1e-6;
            }
          }
        }
        if (open_cruise_exit_frames_ >= open_corridor_cruise_exit_hold_frames_ ||
          speed_low || below_mppi)
        {
          const double handoff_start_vx = open_cruise_vx_;
          open_cruise_exiting_ = false;
          open_cruise_exit_frames_ = 0;
          open_cruise_vx_ = 0.0;
          open_cruise_ax_ = 0.0;
          beginOpenCruiseMppiHandoff(tnow, handoff_start_vx);
          const char * exit_reason = "hold frames elapsed";
          if (below_mppi) {
            exit_reason = "vx below mppi cmd";
          } else if (speed_low) {
            exit_reason = "speed<=early threshold";
          }
          RCLCPP_INFO(
            get_logger(),
            "open_corridor_cruise exit complete (%s, cruise_vx=%.2f mppi_vx=%.2f)",
            exit_reason, handoff_start_vx, mppi_vx);
        }
      } else if (open_corridor_cruise_pre_exit_active_.load(std::memory_order_relaxed) &&
        cloud_ok && cruise_drive_ok)
      {
        out = computeOpenCorridorCruisePreExitTwist(tnow, handle, handle_valid);
      } else if (cloud_ok && handle_valid && !stick_release_halt &&
        !release_soft_stop && cruise_drive_ok) {
        out = computeOpenCorridorCruiseTwist(tnow, handle, handle_valid);
      } else if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "open_corridor_cruise halted: %s", reason.c_str());
      } else if (!cloud_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "open_corridor_cruise halted: cloud stale");
      } else if (open_corridor_cruise && !cruise_drive_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "open_corridor_cruise idle: no forward drive intent (handle/rocker)");
        open_cruise_vx_ = 0.0;
        open_cruise_ax_ = 0.0;
      } else {
        open_cruise_vx_ = out.linear.x;
        open_cruise_ax_ = 0.0;
      }
      if (shouldApplyCruiseExitProximityCap() && cloud_ok && dir != 0 &&
        std::isfinite(d))
      {
        applyProximitySpeedCapTwist(out, dir, d, overhang);
      }
      applyLeftTurnWzTrick(out);
      out_pub_->publish(out);
      last_pub_stamp_ = tnow;
      last_out_ = out;
      return;
    }

    if (open_cruise_mppi_handoff_active_) {
      Twist mppi_target;
      bool mppi_ok = false;
      readCmdVelInput(tnow, mppi_target, mppi_ok, halted, reason);
      Twist out;
      if (halted || soft_stopping_ || !mppi_ok) {
        endOpenCruiseMppiHandoff();
        if (halted || !mppi_ok) {
          applyLeftTurnWzTrick(out);
          out_pub_->publish(out);
          last_pub_stamp_ = tnow;
          last_out_ = out;
          return;
        }
      } else {
        out = computeOpenCruiseMppiHandoffTwist(tnow, mppi_target);
        if (shouldApplyCruiseExitProximityCap() && cloud_ok && dir != 0 &&
          std::isfinite(d))
        {
          applyProximitySpeedCapTwist(out, dir, d, overhang);
        }
        applyGearVxCap(out);
        applyLeftTurnWzTrick(out);
        out_pub_->publish(out);
        last_pub_stamp_ = tnow;
        last_out_ = out;
        if (open_cruise_mppi_handoff_active_) {
          return;
        }
      }
    }

    Twist target;
    bool input_ok = false;
    readCmdVelInput(tnow, target, input_ok, halted, reason);
    const double mppi_raw_vx = target.linear.x;
    const double mppi_raw_wz = target.angular.z;
    tryRecoverFromMppiFailSoftStop(target, input_ok);

    if (mppi_fail_soft_stop_enable_ && !side_pass_crawl && !halted && !soft_stopping_ &&
      input_ok && have_rocker_.load(std::memory_order_relaxed) &&
      !rocker_stale && !stick_release_halt && !release_soft_stop &&
      isTwistNearZero(target))
    {
      const bool was_moving =
        std::abs(last_out_.linear.x) > release_soft_stop_vel_eps_ ||
        std::abs(last_out_.angular.z) > release_soft_stop_vel_eps_;
      if (was_moving) {
        applyMppiFailSoftStop(cloud_ok, halted, reason, "mppi fail");
        if (!halted) {
          mppi_fail_soft_stop_ = true;
          mppi_fail_soft_stop_frame_count_ = 0;
          RCLCPP_INFO(
            get_logger(),
            "mppi fail: release soft-stop (rocker active, cmd_vel=0, forward strips depth=%.0fmm)",
            mppi_fail_soft_stop_check_depth_m_ * 1000.0);
        }
      }
    }

    if (soft_stopping_ && !halted) {
      if (mppi_fail_soft_stop_) {
        if (mppi_fail_soft_stop_max_frames_ > 0) {
          ++mppi_fail_soft_stop_frame_count_;
          if (mppi_fail_soft_stop_frame_count_ >= mppi_fail_soft_stop_max_frames_) {
            RCLCPP_INFO(
              get_logger(),
              "mppi fail soft-stop timeout: max_frames=%d -> output 0",
              mppi_fail_soft_stop_max_frames_);
            clearMppiFailSoftStop();
            endSoftStop();
          }
        }
        if (mppi_fail_soft_stop_ &&
          (!cloud_ok || !mppiFailSoftStopForwardStripsClear()))
        {
          halted = true;
          clearMppiFailSoftStop();
          endSoftStop();
          reason = !cloud_ok ?
            "mppi fail (cloud stale)" : "mppi fail (left/front/right strips blocked)";
        }
      } else if (soft_stop_ramp_mode_ && soft_stop_ramp_legacy_) {
        if (!cloud_ok ||
          nearbyObstaclePointCountInRadius(release_soft_stop_radius_) > 0)
        {
          halted = true;
          endSoftStop();
          reason = !cloud_ok ?
            "release soft-stop (cloud stale)" : "release soft-stop (obstacles nearby)";
        }
      } else if (!cloud_ok || stick_release_halt || !canContinueSoftStopLowPass()) {
        halted = true;
        endSoftStop();
        if (!cloud_ok) {
          reason = "soft-stop aborted (cloud stale)";
        } else if (stick_release_halt) {
          reason = "soft-stop aborted (stick release halt)";
        } else {
          reason = "soft-stop aborted (brake clearance)";
        }
      }
    }

    double effective_wz_max = wz_max_;
    if (wz_density_cap_enable_ && input_ok && !halted && !soft_stopping_) {
      if (!cloud_ok) {
        current_wz_tier_ = WzDensityTier::DENSE;
      } else {
        updateWzDensityTier(nearbyObstaclePointCount(), tnow);
      }
      effective_wz_max = wzDensityCap();
    }

    if (proximity_speed_cap_ && !narrow_gate_pass && input_ok && !halted &&
      !soft_stopping_ && dir != 0 && std::isfinite(d))
    {
      const double clearance = d - overhang - standoff_;
      const double v_safe = (clearance <= 0.0)
        ? 0.0
        : std::sqrt(2.0 * std::max(1e-3, brake_decel_) * clearance);
      if (dir > 0) {
        target.linear.x = std::min(target.linear.x, v_safe);
      } else {
        target.linear.x = std::max(target.linear.x, -v_safe);
      }
    }

    Twist out;
    if (soft_stopping_ && !halted) {
      const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
      if (soft_stop_ramp_mode_) {
        applySoftStopRamp(out, dt);
      } else {
        applySoftStopLowPass(out, dt);
        finishSoftStopIfDecayed();
      }
      applyGearVxCap(out);
    } else if (!halted && input_ok) {
      out = target;
      if (narrow_gate_pass) {
        if (out.linear.x > 0.0) {
          out.linear.x = std::min(out.linear.x, narrow_gate_vx_max_);
        } else if (out.linear.x < 0.0) {
          out.linear.x = std::max(out.linear.x, -narrow_gate_vx_max_);
        }
      } else {
        applyNarrowGateExitVxRamp(out, tnow);
      }
      applyGearVxCap(out);
      applyStickSpeedScale(out, tnow, narrow_gate_pass);
      if (!halted && out.linear.x < 0.0) {
        out.linear.x = std::max(out.linear.x, -vx_reverse_max_);
      }
      if (!halted) {
        out.angular.z = clamp(out.angular.z, -effective_wz_max, effective_wz_max);
      }
      applyNarrowGateRockerIntentFilter(out, tnow);
      applyRockerSectorIntentFilter(out, tnow);
      if (!soft_stopping_) {
        const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
        applySideObstacleSpeedLimit(out, dt, cloud_ok, mppi_raw_vx, mppi_raw_wz);
        applyVxAccelLimit(out, dt);
      }
    } else {
      clearMppiFailSoftStop();
      last_out_ = Twist();
      if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "cmd_vel_safety_gate halting: %s", reason.c_str());
      }
    }

    applyLeftTurnWzTrick(out);
    out_pub_->publish(out);
    last_pub_stamp_ = tnow;
    last_out_ = out;

    UInt8 tier_msg;
    tier_msg.data = static_cast<uint8_t>(current_wz_tier_);
    wz_tier_pub_->publish(tier_msg);
  }

  void publishBypass(const rclcpp::Time & tnow)
  {
    Twist out;
    if (bypass_source_ == "cmd_vel") {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (have_input_) {
        out = latest_twist_;
      }
    } else {
      std::lock_guard<std::mutex> lock(handle_mutex_);
      if (have_handle_) {
        out = latest_handle_;
      }
    }

    out_pub_->publish(out);
    last_pub_stamp_ = tnow;
    last_out_ = out;

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "gate_enable=false: bypass %s -> %s",
      bypass_source_.c_str(), out_topic_.c_str());
  }

  // Parameters
  std::string in_stamped_topic_;
  std::string out_topic_;
  std::string handle_topic_;
  std::string obstacle_topic_;
  std::string input_type_;
  std::string rocker_topic_;
  double rocker_deadband_{5.0};
  double rocker_timeout_{0.30};
  bool rocker_sector_intent_filter_enable_{true};
  double rocker_intent_front_half_deg_{30.0};
  double rocker_intent_side_span_deg_{90.0};
  bool gate_enable_{true};
  std::string bypass_source_{"handle"};
  double cmd_timeout_{0.30};
  double cmd_hold_timeout_{0.50};
  double publish_rate_hz_{50.0};
  double sector_half_angle_{0.785};
  double obstacle_timeout_{0.5};
  double intent_eps_{0.03};
  double front_overhang_{0.935};
  double rear_overhang_{0.2};
  double standoff_{0.4};
  double brake_decel_{1.2};
  bool proximity_speed_cap_{true};
  double wz_max_{1.00};
  double vx_reverse_max_{0.5};
  bool left_turn_wz_trick_enable_{false};
  double left_turn_wz_trick_offset_deg_s_{2.0};
  double left_turn_wz_trick_offset_rps_{0.03490658503988659};
  std::string car_info_topic_;
  bool gear_vx_cap_enable_{true};
  int gear_default_index_{3};
  std::vector<double> gear_vx_max_mps_;
  bool wz_density_cap_enable_{true};
  double density_check_radius_{1.2};
  std::vector<double> wz_density_caps_rps_{1.30, 0.785, 0.524};
  int obstacle_count_engage_moderate_{12};
  int obstacle_count_release_open_{6};
  int obstacle_count_engage_dense_{40};
  int obstacle_count_release_moderate_{28};
  double wz_density_min_dwell_{0.4};
  bool release_soft_stop_enable_{true};
  double release_soft_stop_radius_{0.8};
  double release_soft_stop_decel_{1.0};
  double release_soft_stop_wz_decel_{2.0};
  double release_soft_stop_time_constant_s_{0.30};
  double release_soft_stop_angular_time_constant_s_{0.30};
  double release_soft_stop_vel_eps_{0.02};
  bool release_soft_stop_brake_check_enable_{true};
  double release_soft_stop_standoff_{0.08};
  double release_soft_stop_brake_sector_half_angle_{0.30};
  double mppi_fail_soft_stop_jerk_mps3_{0.3};
  double mppi_fail_soft_stop_decel_mps2_{1.0};
  double mppi_fail_soft_stop_check_depth_m_{0.10};
  double mppi_fail_soft_stop_side_probe_depth_m_{0.20};
  double mppi_fail_soft_stop_avoid_wz_accel_{0.50};
  double mppi_fail_soft_stop_avoid_wz_max_{0.35};
  int mppi_fail_soft_stop_max_frames_{50};
  std::string release_soft_stop_mode_topic_;
  std::string stick_release_halt_topic_;
  bool mppi_fail_soft_stop_enable_{true};

  std::string narrow_gate_pass_mode_topic_;
  std::string narrow_gate_side_pass_mode_topic_;
  double narrow_gate_vx_max_{0.20};
  bool narrow_gate_rocker_intent_filter_enable_{true};
  double narrow_gate_rocker_forward_min_{5.0};
  double narrow_gate_rocker_lateral_min_{5.0};
  bool narrow_gate_exit_ramp_enable_{true};
  double narrow_gate_exit_ramp_sec_{2.0};
  double narrow_gate_exit_ramp_vx_target_{1.50};
  bool side_pass_crawl_enable_{false};
  double side_pass_crawl_check_depth_m_{0.06};
  std::vector<double> side_pass_crawl_forward_strip_x_{0.685, 0.935, 0.685};
  std::vector<double> side_pass_crawl_forward_strip_y_min_{0.2, -0.2, -0.34};
  std::vector<double> side_pass_crawl_forward_strip_y_max_{0.34, 0.2, -0.2};
  std::vector<double> side_pass_crawl_rear_strip_x_{-0.2, -0.2, -0.2};
  std::vector<double> side_pass_crawl_rear_strip_y_min_{0.2, -0.2, -0.34};
  std::vector<double> side_pass_crawl_rear_strip_y_max_{0.34, 0.2, -0.2};
  std::vector<double> side_pass_crawl_lateral_strip_x_min_{-0.2, 0.685, 0.7};
  std::vector<double> side_pass_crawl_lateral_strip_x_max_{0.685, 0.7, 0.935};
  std::vector<double> side_pass_crawl_lateral_strip_y_edge_{0.33, 0.34, 0.20};
  double side_pass_crawl_vx_mps_{0.10};
  double side_pass_crawl_wz_rps_{0.10};
  double side_pass_crawl_rocker_forward_min_{5.0};
  double side_pass_crawl_rocker_lateral_min_{5.0};
  bool side_pass_exit_transition_enable_{true};
  int side_pass_exit_transition_frames_{10};
  bool side_pass_avoid_turn_enable_{true};
  bool side_pass_exit_transition_active_{false};
  bool side_pass_exit_hold_active_{false};
  Twist side_pass_exit_hold_twist_;
  int side_pass_exit_transition_frame_count_{0};
  bool side_pass_avoid_turn_active_{false};
  double side_pass_avoid_turn_wz_{0.0};
  int side_pass_avoid_turn_frame_count_{0};
  bool side_pass_crawl_prev_{false};
  std::string open_corridor_cruise_mode_topic_;
  int open_corridor_cruise_min_gear_{4};
  double open_corridor_cruise_jerk_mps3_{0.1};
  double open_corridor_cruise_accel_mps2_{0.5};
  double open_corridor_cruise_front_half_deg_{30.0};
  int open_corridor_cruise_exit_hold_frames_{20};
  double open_corridor_cruise_pre_exit_jerk_mps3_{0.2};
  double open_corridor_cruise_pre_exit_decel_mps2_{0.4};
  double open_corridor_cruise_pre_exit_exit_speed_kph_{2.0};
  std::string open_corridor_cruise_pre_exit_mode_topic_;
  double open_corridor_cruise_exit_jerk_mps3_{0.2};
  double open_corridor_cruise_exit_decel_mps2_{0.8};
  double open_corridor_cruise_exit_early_speed_kph_{2.0};
  bool open_corridor_cruise_exit_early_below_mppi_enable_{true};
  bool open_corridor_cruise_mppi_handoff_enable_{true};
  double open_corridor_cruise_mppi_handoff_jerk_mps3_{0.2};
  double open_corridor_cruise_mppi_handoff_accel_mps2_{0.5};
  double open_corridor_cruise_mppi_handoff_max_sec_{0.6};
  double open_corridor_cruise_mppi_handoff_vx_eps_{0.08};
  bool proximity_speed_cap_during_cruise_exit_{true};
  bool open_cruise_mppi_handoff_active_{false};
  double open_cruise_handoff_vx_{0.0};
  double open_cruise_handoff_ax_{0.0};
  rclcpp::Time open_cruise_handoff_start_{0, 0, RCL_ROS_TIME};
  std::vector<double> open_corridor_cruise_gear_wz_max_rps_;
  std::vector<double> open_corridor_cruise_gear_vx_max_mps_;
  double open_cruise_vx_{0.0};
  double open_cruise_ax_{0.0};
  bool open_cruise_exiting_{false};
  int open_cruise_exit_frames_{0};
  bool vx_accel_limit_enable_{true};
  double vx_accel_max_mps2_{1.0};
  bool stick_vx_scale_enable_{true};
  double stick_vx_min_ratio_{0.15};
  bool stick_wz_scale_enable_{true};
  bool side_speed_limit_enable_{false};
  int side_speed_limit_gear_only_{1};
  double side_speed_limit_body_half_width_m_{0.34};
  double side_speed_limit_x_min_m_{0.33};
  double side_speed_limit_x_max_m_{1.00};
  std::vector<double> side_speed_limit_dist_m_{0.05, 0.10, 0.20, 0.40};
  std::vector<double> side_speed_limit_vx_mps_{0.28, 0.60, 1.20, 2.40};
  double side_speed_limit_decel_mps2_{0.60};
  bool side_speed_limit_wz_tighten_enable_{false};
  std::atomic<bool> narrow_gate_pass_active_{false};
  std::atomic<bool> narrow_gate_side_pass_crawl_active_{false};
  std::atomic<bool> open_corridor_cruise_active_{false};
  std::atomic<bool> open_corridor_cruise_pre_exit_active_{false};
  std::atomic<bool> release_soft_stop_active_{false};
  std::atomic<bool> stick_release_halt_active_{false};
  bool narrow_gate_pass_prev_{false};
  bool narrow_gate_exit_ramp_active_{false};
  rclcpp::Time narrow_gate_exit_ramp_start_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<Twist>::SharedPtr handle_sub_;
  rclcpp::Subscription<Joystick>::SharedPtr joystick_sub_;
  rclcpp::Subscription<CarInfo>::SharedPtr car_info_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Bool>::SharedPtr narrow_gate_pass_mode_sub_;
  rclcpp::Subscription<Bool>::SharedPtr narrow_gate_side_pass_mode_sub_;
  rclcpp::Subscription<Bool>::SharedPtr open_corridor_cruise_mode_sub_;
  rclcpp::Subscription<Bool>::SharedPtr open_corridor_cruise_pre_exit_mode_sub_;
  rclcpp::Subscription<Bool>::SharedPtr release_soft_stop_mode_sub_;
  rclcpp::Subscription<Bool>::SharedPtr stick_release_halt_sub_;
  rclcpp::Publisher<Twist>::SharedPtr out_pub_;
  rclcpp::Publisher<UInt8>::SharedPtr wz_tier_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex input_mutex_;
  Twist latest_twist_;
  rclcpp::Time latest_input_stamp_{0, 0, RCL_ROS_TIME};
  bool have_input_{false};

  std::mutex handle_mutex_;
  Twist latest_handle_;
  bool have_handle_{false};

  mutable std::mutex gear_mutex_;
  int latest_gear_{3};
  bool have_gear_{false};

  std::atomic<bool> rocker_at_origin_{true};
  std::atomic<int16_t> latest_rocker_x_{0};
  std::atomic<int16_t> latest_rocker_y_{0};
  std::atomic<int64_t> latest_rocker_stamp_ns_{0};
  std::atomic<bool> have_rocker_{false};

  mutable std::mutex cloud_mutex_;
  std::vector<std::pair<float, float>> cloud_pts_;
  rclcpp::Time latest_cloud_stamp_{0, 0, RCL_ROS_TIME};
  bool have_cloud_{false};

  WzDensityTier current_wz_tier_{WzDensityTier::OPEN};
  rclcpp::Time last_wz_tier_change_stamp_{0, 0, RCL_ROS_TIME};

  Twist last_out_;
  rclcpp::Time last_pub_stamp_{0, 0, RCL_ROS_TIME};
  bool soft_stopping_{false};
  bool soft_stop_ramp_mode_{false};
  bool soft_stop_ramp_legacy_{false};
  double soft_stop_vx_{0.0};
  double soft_stop_wz_{0.0};
  double soft_stop_vx_ax_{0.0};
  bool mppi_fail_soft_stop_{false};
  int mppi_fail_soft_stop_frame_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
