// Joystick / handle intent -> short local path -> Nav2 FollowPath (MPPI).
//
// spatial_range: stick polar -> body-frame goal on concentric arcs (per gear).
//   Forward sector: goal on arc, path yaw lerps toward stick_theta.
//   Backward sector (60 deg + hysteresis): Cartesian goal behind vehicle,
//   path yaw fixed at current heading -> MPPI drives vx<0 (reverse avoid, no U-turn).
//   Optional backward_reverse_lateral_invert flips stick lateral for reverse crawl goals.
//   Pure lateral (|rx| deadbanded) -> in-place turn.
//   Forward + narrow gate (head-on): bilateral cloud scan along body +x only;
//     oblique/side crossing uses side_pass crawl (this node publishes /cmd_vel, skip MPPI).
//     proximity side_pass (no separate enable): enter on left/right/front + slow; exit shell -> hot MPPI.
//     pass_mode enters ONLY when |odom|<=gate_creep_speed_max_mps; once active, keeps until
//     body clear (latch default); intent/front_to_gate exits optional via yaml.
//     gate distance refreshed from odom anchor + cloud; zero footprint padding (all gears
//     normally use 1 cm padding via gear_footprint_padding_m outside pass/side_pass);
//     side_pass memory match: center from odom memory, width from live bilateral cloud slice.
//
// MPPI warm-start reset (stick enters dead zone only):
//   1. cancel FollowPath (handle or cancel_all_goals)
//   2. wait mppi_reset_quiet_sec (> controller FollowPath.reset_period)
//   3. sendGoal(..., cancel_active=true)
//
// velocity_integration: legacy /handle/control (v,w) integration.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "interface/msg/car_info.hpp"
#include "interface/msg/joystick.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/action/follow_path.hpp"

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  quat.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

double clampNorm(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double lerpAngle(double a, double b, double t)
{
  const double diff = normalizeAngle(b - a);
  return normalizeAngle(a + t * diff);
}

struct BodyPoint
{
  double x{0.0};
  double y{0.0};
};

BodyPoint hermitePoint(
  double u, double x0, double y0, double x1, double y1,
  double mx0, double my0, double mx1, double my1)
{
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
  const double h10 = u3 - 2.0 * u2 + u;
  const double h01 = -2.0 * u3 + 3.0 * u2;
  const double h11 = u3 - u2;
  return BodyPoint{
    h00 * x0 + h10 * mx0 + h01 * x1 + h11 * mx1,
    h00 * y0 + h10 * my0 + h01 * y1 + h11 * my1};
}

}  // namespace

struct NarrowGateInfo
{
  double distance_m{0.0};
  double center_y_m{0.0};
  double width_m{0.0};
  double axis_yaw_body{0.0};
  bool confirmed{false};
};

struct GateMemoryEntry
{
  double center_x_odom{0.0};
  double center_y_odom{0.0};
  double left_x_odom{0.0};
  double left_y_odom{0.0};
  double right_x_odom{0.0};
  double right_y_odom{0.0};
  int pass_count{0};
};

enum class DriveSector
{
  Forward,
  Backward
};

class JoystickLocalPathNode : public rclcpp::Node
{
public:
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using Path = nav_msgs::msg::Path;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Joystick = interface::msg::Joystick;
  using CarInfo = interface::msg::CarInfo;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  JoystickLocalPathNode()
  : rclcpp::Node("joystick_local_path_node")
  {
    path_generation_mode_ = declare_parameter<std::string>(
      "path_generation_mode", "spatial_range");

    control_topic_ = declare_parameter<std::string>("control_topic", "/handle/control");
    joystick_topic_ = declare_parameter<std::string>("joystick_topic", "/sensor/joystick");
    car_info_topic_ = declare_parameter<std::string>("car_info_topic", "/vlago/car/info");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom/fused");
    intent_path_topic_ = declare_parameter<std::string>(
      "intent_path_topic", "/mppi/local_intent_path");
    action_name_ = declare_parameter<std::string>("follow_path_action", "/follow_path");

    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    controller_id_ = declare_parameter<std::string>("controller_id", "FollowPath");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");

    // velocity_integration mode
    lin_deadband_ = declare_parameter<double>("lin_deadband", 0.02);
    ang_deadband_ = declare_parameter<double>("ang_deadband", 0.02);
    turn_assist_vx_ = declare_parameter<double>("turn_assist_vx", 0.08);
    control_timeout_ = declare_parameter<double>("control_timeout", 0.30);

    // spatial_range mode — rocker in [-100,100]; chassis: +x forward, +y right on stick,
    // base_link: +x forward, +y left.
    rocker_deadband_ = declare_parameter<double>("rocker_deadband", 5.0);
    joystick_timeout_ = declare_parameter<double>("joystick_timeout", 0.30);
    gear_forward_range_m_ = declare_parameter<std::vector<double>>(
      "gear_forward_range_m", {0.50, 1.00, 1.50, 2.00});
    gear_back_range_m_ = declare_parameter<std::vector<double>>(
      "gear_back_range_m", {0.15, 0.25, 0.35, 0.50});
    gear_lateral_range_m_ = declare_parameter<std::vector<double>>(
      "gear_lateral_range_m", {0.60, 0.80, 1.00, 1.50});
    gear_lateral_wz_max_ = declare_parameter<std::vector<double>>(
      "gear_lateral_wz_max", {0.55, 0.75, 0.95, 1.15});
    forward_sector_deg_ = declare_parameter<double>("forward_sector_deg", 300.0);
    backward_sector_deg_ = declare_parameter<double>("backward_sector_deg", 60.0);
    sector_hysteresis_deg_ = declare_parameter<double>("sector_hysteresis_deg", 10.0);
    backward_reverse_path_enable_ = declare_parameter<bool>("backward_reverse_path_enable", true);
    backward_reverse_lateral_invert_ = declare_parameter<bool>(
      "backward_reverse_lateral_invert", false);
    gear_default_index_ = declare_parameter<int>("gear_default_index", 3);
    path_interpolation_steps_ = declare_parameter<int>("path_interpolation_steps", 20);
    min_goal_distance_m_ = declare_parameter<double>("min_goal_distance_m", 0.12);
    goal_resend_xy_delta_m_ = declare_parameter<double>("goal_resend_xy_delta_m", 0.05);
    goal_resend_yaw_delta_ = declare_parameter<double>("goal_resend_yaw_delta", 0.08);
    // Must exceed FollowPath.reset_period so Optimizer clears warm-start after cancel.
    mppi_reset_quiet_sec_ = declare_parameter<double>("mppi_reset_quiet_sec", 0.40);

    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.30);

    horizon_ = declare_parameter<double>("horizon", 1.5);
    dt_ = declare_parameter<double>("dt", 0.10);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 5.0);

    footprint_publish_enable_ = declare_parameter<bool>("footprint_publish_enable", true);
    footprint_topic_ = declare_parameter<std::string>(
      "footprint_topic", "/local_costmap/footprint");
    footprint_base_ = declare_parameter<std::string>(
      "footprint_base",
      "[[0.935, 0.34], [0.935, -0.34], [-0.2, -0.34], [-0.2, 0.34]]");
    gear_footprint_padding_m_ = declare_parameter<std::vector<double>>(
      "gear_footprint_padding_m", {0.01, 0.02, 0.03, 0.04});
    path_obstacle_limit_enable_ = declare_parameter<bool>("path_obstacle_limit_enable", true);
    path_obstacle_stop_margin_m_ = declare_parameter<double>("path_obstacle_stop_margin_m", 0.20);
    path_obstacle_min_keep_m_ = declare_parameter<double>("path_obstacle_min_keep_m", 0.12);
    path_obstacle_limit_skip_pass_mode_ = declare_parameter<bool>(
      "path_obstacle_limit_skip_pass_mode", true);
    path_obstacle_limit_skip_in_place_ = declare_parameter<bool>(
      "path_obstacle_limit_skip_in_place", true);
    path_open_cruise_enable_ = declare_parameter<bool>("path_open_cruise_enable", true);
    path_open_cruise_min_gear_ = declare_parameter<int>("path_open_cruise_min_gear", 3);
    path_open_cruise_gear_front_half_deg_ = declare_parameter<std::vector<double>>(
      "path_open_cruise_gear_front_half_deg", {45.0, 45.0, 45.0, 30.0});
    path_open_cruise_gear_extra_padding_m_ = declare_parameter<std::vector<double>>(
      "path_open_cruise_gear_extra_padding_m", {0.0, 0.0, 0.0, 0.05});
    path_open_cruise_outer_far_m_ = declare_parameter<double>(
      "path_open_cruise_outer_far_m", 0.5);
    if (path_open_cruise_outer_far_m_ < 0.0) {
      path_open_cruise_outer_far_m_ = 0.0;
    }
    path_open_cruise_pre_exit_confirm_frames_ = declare_parameter<int>(
      "path_open_cruise_pre_exit_confirm_frames", 2);
    if (path_open_cruise_pre_exit_confirm_frames_ < 1) {
      path_open_cruise_pre_exit_confirm_frames_ = 1;
    }
    path_open_cruise_pre_exit_exit_speed_kph_ = declare_parameter<double>(
      "path_open_cruise_pre_exit_exit_speed_kph", 2.0);
    if (path_open_cruise_pre_exit_exit_speed_kph_ < 0.0) {
      path_open_cruise_pre_exit_exit_speed_kph_ = 0.0;
    }
    path_open_cruise_stall_exit_frames_ = declare_parameter<int>(
      "path_open_cruise_stall_exit_frames", 30);
    if (path_open_cruise_stall_exit_frames_ < 1) {
      path_open_cruise_stall_exit_frames_ = 1;
    }
    path_open_cruise_confirm_frames_ = declare_parameter<int>(
      "path_open_cruise_confirm_frames", 10);
    path_open_cruise_enter_min_speed_kph_ = declare_parameter<double>(
      "path_open_cruise_enter_min_speed_kph", 3.0);
    if (path_open_cruise_enter_min_speed_kph_ < 0.0) {
      path_open_cruise_enter_min_speed_kph_ = 0.0;
    }
    path_open_cruise_exit_confirm_frames_ = declare_parameter<int>(
      "path_open_cruise_exit_confirm_frames", 3);
    if (path_open_cruise_exit_confirm_frames_ < 1) {
      path_open_cruise_exit_confirm_frames_ = 1;
    }
    path_open_cruise_reenter_cooldown_frames_ = declare_parameter<int>(
      "path_open_cruise_reenter_cooldown_frames", 30);
    if (path_open_cruise_reenter_cooldown_frames_ < 0) {
      path_open_cruise_reenter_cooldown_frames_ = 0;
    }
    path_open_cruise_topic_ = declare_parameter<std::string>(
      "path_open_cruise_topic", "/mppi/open_corridor_cruise_mode");
    path_open_cruise_pre_exit_topic_ = declare_parameter<std::string>(
      "path_open_cruise_pre_exit_topic", "/mppi/open_corridor_cruise_pre_exit_mode");

    release_soft_stop_enable_ = declare_parameter<bool>("release_soft_stop_enable", true);
    release_soft_stop_radius_ = declare_parameter<double>("release_soft_stop_radius", 0.8);
    release_soft_stop_topic_ = declare_parameter<std::string>(
      "release_soft_stop_topic", "/mppi/release_soft_stop_mode");
    stick_release_halt_topic_ = declare_parameter<std::string>(
      "stick_release_halt_topic", "/mppi/stick_release_halt");

    rocker_sector_vx_min_enable_ = declare_parameter<bool>(
      "rocker_sector_vx_min_enable", true);
    rocker_forward_sector_topic_ = declare_parameter<std::string>(
      "rocker_forward_sector_topic", "/mppi/rocker_forward_sector_mode");
    rocker_intent_zone_topic_ = declare_parameter<std::string>(
      "rocker_intent_zone_topic", "/mppi/rocker_intent_zone");
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

    wz_density_cap_enable_ = declare_parameter<bool>("wz_density_cap_enable", true);
    wz_density_cap_topic_ = declare_parameter<std::string>(
      "wz_density_cap_topic", "/mppi/wz_density_cap_rps");
    wz_density_tier_topic_ = declare_parameter<std::string>(
      "wz_density_tier_topic", "/mppi/wz_density_tier");
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
    if (wz_density_caps_rps_.empty()) {
      wz_density_caps_rps_ = {1.30, 0.785, 0.524};
    }

    gear_vx_cap_enable_ = declare_parameter<bool>("gear_vx_cap_enable", true);
    gear_vx_cap_topic_ = declare_parameter<std::string>(
      "gear_vx_cap_topic", "/mppi/gear_vx_max_mps");
    const auto gear_vx_max_kph = declare_parameter<std::vector<double>>(
      "gear_vx_max_kph", std::vector<double>{2.0, 3.0, 5.0, 6.0});
    gear_vx_max_mps_.clear();
    gear_vx_max_mps_.reserve(gear_vx_max_kph.size());
    for (const double kph : gear_vx_max_kph) {
      gear_vx_max_mps_.push_back(std::max(0.0, kph) / 3.6);
    }
    if (gear_vx_max_mps_.empty()) {
      gear_vx_max_mps_ = {2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6, 6.0 / 3.6};
    }

    gate_path_enable_ = declare_parameter<bool>("gate_path_enable", true);
    obstacle_points_topic_ = declare_parameter<std::string>(
      "obstacle_points_topic", "/perception/obstacle_points");
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.50);
    gate_width_min_m_ = declare_parameter<double>("gate_width_min_m", 0.75);
    gate_width_max_m_ = declare_parameter<double>("gate_width_max_m", 1.30);
    gate_local_radius_m_ = declare_parameter<double>("gate_local_radius_m", 1.40);
    gate_detect_mode_ = declare_parameter<std::string>("gate_detect_mode", "bilateral");
    gate_scan_dist_min_m_ = declare_parameter<double>("gate_scan_dist_min_m", 0.15);
    gate_scan_dist_step_m_ = declare_parameter<double>("gate_scan_dist_step_m", 0.05);
    gate_opening_depth_m_ = declare_parameter<double>("gate_opening_depth_m", 0.25);
    gate_pillar_mode_enable_ = declare_parameter<bool>("gate_pillar_mode_enable", true);
    gate_min_side_boundary_m_ = declare_parameter<double>("gate_min_side_boundary_m", 0.38);
    gate_pillar_symmetry_ratio_min_ = declare_parameter<double>(
      "gate_pillar_symmetry_ratio_min", 0.55);
    if (gate_pillar_symmetry_ratio_min_ < 0.0) {
      gate_pillar_symmetry_ratio_min_ = 0.0;
    } else if (gate_pillar_symmetry_ratio_min_ > 1.0) {
      gate_pillar_symmetry_ratio_min_ = 1.0;
    }
    gate_min_side_points_ = declare_parameter<int>("gate_min_side_points", 1);
    gate_center_clear_half_width_m_ = declare_parameter<double>(
      "gate_center_clear_half_width_m", 0.12);
    gate_approach_corridor_half_width_m_ = declare_parameter<double>(
      "gate_approach_corridor_half_width_m", 0.25);
    gate_approach_corridor_min_depth_m_ = declare_parameter<double>(
      "gate_approach_corridor_min_depth_m", 0.5);
    gate_approach_corridor_min_points_ = declare_parameter<int>(
      "gate_approach_corridor_min_points", 2);
    if (gate_approach_corridor_min_depth_m_ < 0.05) {
      gate_approach_corridor_min_depth_m_ = 0.05;
    }
    if (gate_approach_corridor_min_points_ < 1) {
      gate_approach_corridor_min_points_ = 1;
    }
    gate_side_wall_y_threshold_m_ = declare_parameter<double>(
      "gate_side_wall_y_threshold_m", 0.32);
    gate_side_wall_min_points_ = declare_parameter<int>("gate_side_wall_min_points", 4);
    gate_side_wall_min_span_m_ = declare_parameter<double>(
      "gate_side_wall_min_span_m", 0.45);
    if (gate_side_wall_min_points_ < 1) {
      gate_side_wall_min_points_ = 1;
    }
    gate_lateral_stick_forward_min_ = declare_parameter<double>(
      "gate_lateral_stick_forward_min", 0.08);
    gate_stick_forward_min_ = declare_parameter<double>("gate_stick_forward_min", 0.20);
    gate_center_y_deadband_m_ = declare_parameter<double>("gate_center_y_deadband_m", 0.18);
    gate_stick_lateral_min_ = declare_parameter<double>("gate_stick_lateral_min", 0.08);
    gate_intent_lateral_ratio_min_ = declare_parameter<double>(
      "gate_intent_lateral_ratio_min", 0.20);
    gate_intent_cross_slack_m_ = declare_parameter<double>("gate_intent_cross_slack_m", 0.18);
    gate_intent_angle_delta_max_ = declare_parameter<double>(
      "gate_intent_angle_delta_max", 0.45);
    gate_creep_speed_max_mps_ = declare_parameter<double>("gate_creep_speed_max_mps", 0.20);
    gate_detect_speed_max_mps_ = declare_parameter<double>("gate_detect_speed_max_mps", 0.55);
    gate_detect_confirm_sec_ = declare_parameter<double>("gate_detect_confirm_sec", 0.80);
    gate_detect_hold_sec_ = declare_parameter<double>("gate_detect_hold_sec", 0.60);
    gate_creep_stick_forward_min_ = declare_parameter<double>(
      "gate_creep_stick_forward_min", 0.08);
    gate_creep_stick_min_ = declare_parameter<double>("gate_creep_stick_min", 0.10);
    gate_creep_intent_angle_max_ = declare_parameter<double>(
      "gate_creep_intent_angle_max", 0.95);
    gate_creep_intent_cross_slack_m_ = declare_parameter<double>(
      "gate_creep_intent_cross_slack_m", 0.35);
    gate_creep_intent_angle_delta_max_ = declare_parameter<double>(
      "gate_creep_intent_angle_delta_max", 0.95);
    gate_path_tail_m_ = declare_parameter<double>("gate_path_tail_m", 0.50);
    gate_hold_sec_ = declare_parameter<double>("gate_hold_sec", 0.25);
    gate_confirm_sec_ = declare_parameter<double>("gate_confirm_sec", 0.50);
    gate_confirmed_hold_sec_ = declare_parameter<double>("gate_confirmed_hold_sec", 0.60);
    gate_ema_alpha_ = declare_parameter<double>("gate_ema_alpha", 0.35);
    gate_lock_distance_delta_m_ = declare_parameter<double>("gate_lock_distance_delta_m", 0.18);
    gate_lock_center_y_delta_m_ = declare_parameter<double>("gate_lock_center_y_delta_m", 0.15);
    gate_memory_max_count_ = declare_parameter<int>("gate_memory_max_count", 5);
    gate_memory_max_count_ = std::max(1, gate_memory_max_count_);
    gate_memory_merge_radius_m_ = declare_parameter<double>("gate_memory_merge_radius_m", 1.00);
    gate_memory_lateral_band_m_ = declare_parameter<double>("gate_memory_lateral_band_m", 0.20);
    gate_memory_min_speed_mps_ = declare_parameter<double>("gate_memory_min_speed_mps", 0.30);
    gate_memory_confirm_frames_ = declare_parameter<int>("gate_memory_confirm_frames", 5);
    gate_memory_body_x_min_m_ = declare_parameter<double>("gate_memory_body_x_min_m", -1.0);
    gate_memory_body_x_max_m_ = declare_parameter<double>("gate_memory_body_x_max_m", -1.0);
    gate_memory_half_width_m_ = declare_parameter<double>("gate_memory_half_width_m", -1.0);
    gate_memory_min_side_points_ = declare_parameter<int>("gate_memory_min_side_points", 1);
    gate_memory_rearm_frames_ = declare_parameter<int>("gate_memory_rearm_frames", 3);
    gate_memory_update_alpha_ = declare_parameter<double>("gate_memory_update_alpha", 0.0);
    gate_side_pass_enable_ = declare_parameter<bool>("gate_side_pass_enable", true);
    gate_side_pass_min_pass_count_ = declare_parameter<int>("gate_side_pass_min_pass_count", 2);
    gate_side_pass_trigger_dist_m_ = declare_parameter<double>(
      "gate_side_pass_trigger_dist_m", 0.20);
    gate_side_pass_stick_forward_min_ = declare_parameter<double>(
      "gate_side_pass_stick_forward_min", 0.08);
    gate_side_pass_stick_lateral_min_ = declare_parameter<double>(
      "gate_side_pass_stick_lateral_min", 0.08);
    gate_side_pass_stick_angle_max_ = declare_parameter<double>(
      "gate_side_pass_stick_angle_max", 0.95);
    gate_side_pass_max_speed_mps_ = declare_parameter<double>(
      "gate_side_pass_max_speed_mps", 0.10);
    gate_side_pass_body_clear_m_ = declare_parameter<double>("gate_side_pass_body_clear_m", 0.35);
    gate_side_pass_width_min_m_ = declare_parameter<double>("gate_side_pass_width_min_m", 0.25);
    gate_side_pass_enter_forward_cloud_enable_ = declare_parameter<bool>(
      "gate_side_pass_enter_forward_cloud_enable", true);
    gate_side_pass_enter_check_depth_m_ = declare_parameter<double>(
      "gate_side_pass_enter_check_depth_m", 0.15);
    gate_side_pass_enter_forward_strip_x_ = declare_parameter<std::vector<double>>(
      "gate_side_pass_enter_forward_strip_x", {0.685, 0.935, 0.685});
    gate_side_pass_enter_forward_strip_y_min_ = declare_parameter<std::vector<double>>(
      "gate_side_pass_enter_forward_strip_y_min", {0.2, -0.2, -0.34});
    gate_side_pass_enter_forward_strip_y_max_ = declare_parameter<std::vector<double>>(
      "gate_side_pass_enter_forward_strip_y_max", {0.34, 0.2, -0.2});
    if (gate_side_pass_enter_forward_strip_x_.size() !=
      gate_side_pass_enter_forward_strip_y_min_.size() ||
      gate_side_pass_enter_forward_strip_x_.size() !=
      gate_side_pass_enter_forward_strip_y_max_.size())
    {
      RCLCPP_WARN(
        get_logger(),
        "gate_side_pass_enter forward strip array size mismatch; using defaults");
      gate_side_pass_enter_forward_strip_x_ = {0.685, 0.935, 0.685};
      gate_side_pass_enter_forward_strip_y_min_ = {0.2, -0.2, -0.34};
      gate_side_pass_enter_forward_strip_y_max_ = {0.34, 0.2, -0.2};
    }
    gate_side_pass_exit_forward_clear_enable_ = declare_parameter<bool>(
      "gate_side_pass_exit_forward_clear_enable", true);
    gate_side_pass_exit_check_depth_m_ = declare_parameter<double>(
      "gate_side_pass_exit_check_depth_m", 0.10);
    gate_side_pass_exit_forward_strip_x_ = declare_parameter<std::vector<double>>(
      "gate_side_pass_exit_forward_strip_x", {0.685, 0.685});
    gate_side_pass_exit_forward_strip_y_min_ = declare_parameter<std::vector<double>>(
      "gate_side_pass_exit_forward_strip_y_min", {0.2, -0.34});
    gate_side_pass_exit_forward_strip_y_max_ = declare_parameter<std::vector<double>>(
      "gate_side_pass_exit_forward_strip_y_max", {0.34, -0.2});
    if (gate_side_pass_exit_forward_strip_x_.size() !=
      gate_side_pass_exit_forward_strip_y_min_.size() ||
      gate_side_pass_exit_forward_strip_x_.size() !=
      gate_side_pass_exit_forward_strip_y_max_.size())
    {
      RCLCPP_WARN(
        get_logger(),
        "gate_side_pass_exit forward strip array size mismatch; using defaults");
      gate_side_pass_exit_forward_strip_x_ = {0.685, 0.685};
      gate_side_pass_exit_forward_strip_y_min_ = {0.2, -0.34};
      gate_side_pass_exit_forward_strip_y_max_ = {0.34, -0.2};
    }
    proximity_side_pass_enable_ = declare_parameter<bool>(
      "proximity_side_pass_enable", true);
    proximity_side_pass_outer_probe_depth_m_ = declare_parameter<double>(
      "proximity_side_pass_outer_probe_depth_m", 0.20);
    proximity_side_pass_exit_margin_m_ = declare_parameter<double>(
      "proximity_side_pass_exit_margin_m", 0.15);
    proximity_side_pass_max_speed_mps_ = declare_parameter<double>(
      "proximity_side_pass_max_speed_mps", 0.10);
    proximity_side_pass_max_wz_rps_ = declare_parameter<double>(
      "proximity_side_pass_max_wz_rps", 0.20);
    if (proximity_side_pass_max_wz_rps_ < 0.0) {
      proximity_side_pass_max_wz_rps_ = 0.0;
    }
    proximity_side_pass_confirm_frames_ = declare_parameter<int>(
      "proximity_side_pass_confirm_frames", 10);
    proximity_side_pass_exit_confirm_frames_ = declare_parameter<int>(
      "proximity_side_pass_exit_confirm_frames", 5);
    gate_pass_mode_enter_m_ = declare_parameter<double>("gate_pass_mode_enter_m", 0.80);
    gate_pass_front_center_x_m_ = declare_parameter<double>("gate_pass_front_center_x_m", 0.85);
    gate_pass_mode_exit_m_ = declare_parameter<double>("gate_pass_mode_exit_m", 0.95);
    gate_pass_mode_body_clear_x_m_ = declare_parameter<double>(
      "gate_pass_mode_body_clear_x_m", -0.55);
    gate_pass_mode_hold_sec_ = declare_parameter<double>("gate_pass_mode_hold_sec", 0.25);
    gate_pass_mode_latch_until_body_clear_ = declare_parameter<bool>(
      "gate_pass_mode_latch_until_body_clear", true);
    gate_pass_mode_intent_timeout_enable_ = declare_parameter<bool>(
      "gate_pass_mode_intent_timeout_enable", false);
    gate_pass_mode_front_to_gate_exit_enable_ = declare_parameter<bool>(
      "gate_pass_mode_front_to_gate_exit_enable", false);
    narrow_gate_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_pass_mode_topic", "/mppi/narrow_gate_pass_mode");
    narrow_gate_side_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_side_pass_mode_topic", "/mppi/narrow_gate_side_pass_mode");
    side_pass_crawl_enable_ = declare_parameter<bool>("side_pass_crawl_enable", false);
    side_pass_crawl_cmd_vel_topic_ = declare_parameter<std::string>(
      "side_pass_crawl_cmd_vel_topic", "/cmd_vel");
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
        "side_pass_crawl forward strip array size mismatch; using defaults");
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
        "side_pass_crawl rear strip array size mismatch; using defaults");
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
        "side_pass_crawl lateral strip array size mismatch; using defaults");
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
    gate_marker_enable_ = declare_parameter<bool>("gate_marker_enable", true);
    gate_marker_topic_ = declare_parameter<std::string>(
      "gate_marker_topic", "/mppi/gate_markers");
    gate_memory_publish_enable_ = declare_parameter<bool>("gate_memory_publish_enable", true);
    gate_memory_topic_ = declare_parameter<std::string>(
      "gate_memory_topic", "/mppi/gate_memory");
    gate_memory_state_topic_ = declare_parameter<std::string>(
      "gate_memory_state_topic", "/mppi/gate_memory/state");
    use_spatial_range_ = (path_generation_mode_ == "spatial_range");

    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, 10,
      std::bind(&JoystickLocalPathNode::onOdom, this, std::placeholders::_1));

    if (use_spatial_range_) {
      joystick_sub_ = create_subscription<Joystick>(
        joystick_topic_, 10,
        std::bind(&JoystickLocalPathNode::onJoystick, this, std::placeholders::_1));
      if (gate_path_enable_ || path_open_cruise_enable_ || release_soft_stop_enable_) {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          obstacle_points_topic_, rclcpp::SensorDataQoS(),
          std::bind(&JoystickLocalPathNode::onCloud, this, std::placeholders::_1));
      }
    } else {
      control_sub_ = create_subscription<Twist>(
        control_topic_, 10,
        std::bind(&JoystickLocalPathNode::onControl, this, std::placeholders::_1));
    }

    if (footprint_publish_enable_ || use_spatial_range_ || gear_vx_cap_enable_) {
      car_info_sub_ = create_subscription<CarInfo>(
        car_info_topic_, 10,
        std::bind(&JoystickLocalPathNode::onCarInfo, this, std::placeholders::_1));
    }

    path_pub_ = create_publisher<Path>(intent_path_topic_, 1);
    narrow_gate_pass_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      narrow_gate_pass_mode_topic_, rclcpp::QoS(1).transient_local());
    narrow_gate_side_pass_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      narrow_gate_side_pass_mode_topic_, rclcpp::QoS(1).transient_local());
    side_pass_crawl_cmd_vel_pub_ = create_publisher<Twist>(
      side_pass_crawl_cmd_vel_topic_, 10);
    open_corridor_cruise_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      path_open_cruise_topic_, rclcpp::QoS(1).transient_local());
    open_corridor_cruise_pre_exit_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      path_open_cruise_pre_exit_topic_, rclcpp::QoS(1).transient_local());
    release_soft_stop_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      release_soft_stop_topic_, rclcpp::QoS(1).transient_local());
    stick_release_halt_pub_ = create_publisher<std_msgs::msg::Bool>(
      stick_release_halt_topic_, rclcpp::QoS(1).transient_local());
    if (rocker_sector_vx_min_enable_) {
      rocker_forward_sector_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
        rocker_forward_sector_topic_, rclcpp::QoS(1).transient_local());
      rocker_intent_zone_pub_ = create_publisher<std_msgs::msg::UInt8>(
        rocker_intent_zone_topic_, rclcpp::QoS(1).transient_local());
    }
    if (wz_density_cap_enable_) {
      wz_density_cap_pub_ = create_publisher<std_msgs::msg::Float64>(
        wz_density_cap_topic_, rclcpp::QoS(1).transient_local());
      wz_density_tier_pub_ = create_publisher<std_msgs::msg::UInt8>(
        wz_density_tier_topic_, rclcpp::QoS(1).transient_local());
    }
    if (gear_vx_cap_enable_) {
      gear_vx_cap_pub_ = create_publisher<std_msgs::msg::Float64>(
        gear_vx_cap_topic_, rclcpp::QoS(1).transient_local());
    }
    if (gate_marker_enable_ && gate_path_enable_) {
      gate_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        gate_marker_topic_, rclcpp::QoS(1).transient_local());
    }
    if (gate_memory_publish_enable_ && gate_path_enable_) {
      const auto bag_qos = rclcpp::QoS(10).reliable();
      gate_memory_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        gate_memory_topic_, bag_qos);
      gate_memory_state_pub_ = create_publisher<std_msgs::msg::String>(
        gate_memory_state_topic_, bag_qos);
    }
    if (footprint_publish_enable_) {
      if (!nav2_costmap_2d::makeFootprintFromString(footprint_base_, footprint_base_pts_)) {
        RCLCPP_ERROR(get_logger(), "Invalid footprint_base: %s", footprint_base_.c_str());
        footprint_publish_enable_ = false;
      } else {
        footprint_pub_ = create_publisher<geometry_msgs::msg::Polygon>(footprint_topic_, 1);
        publishFootprintForGear(gear_default_index_);
      }
    }
    initGateMemoryBodyBoundsFromFootprint();
    std_msgs::msg::Bool pass_mode_init;
    pass_mode_init.data = false;
    narrow_gate_pass_mode_pub_->publish(pass_mode_init);
    narrow_gate_side_pass_mode_pub_->publish(pass_mode_init);
    open_corridor_cruise_mode_pub_->publish(pass_mode_init);
    open_corridor_cruise_pre_exit_mode_pub_->publish(pass_mode_init);
    release_soft_stop_mode_pub_->publish(pass_mode_init);
    stick_release_halt_pub_->publish(pass_mode_init);
    if (rocker_forward_sector_mode_pub_) {
      std_msgs::msg::Bool forward_sector_init;
      forward_sector_init.data = true;
      rocker_forward_sector_mode_pub_->publish(forward_sector_init);
      rocker_forward_sector_active_ = true;
    }
    if (rocker_intent_zone_pub_) {
      std_msgs::msg::UInt8 zone_init;
      zone_init.data = static_cast<uint8_t>(RockerIntentZone::NONE);
      rocker_intent_zone_pub_->publish(zone_init);
      rocker_intent_zone_active_ = RockerIntentZone::NONE;
    }
    if (wz_density_cap_pub_) {
      publishWzDensityCap(wzDensityCapRps());
      std_msgs::msg::UInt8 tier_init;
      tier_init.data = static_cast<uint8_t>(current_wz_tier_);
      wz_density_tier_pub_->publish(tier_init);
    }
    if (gear_vx_cap_pub_) {
      publishGearVxCap(gearVxMaxMps());
    }
    action_client_ = rclcpp_action::create_client<FollowPath>(this, action_name_);

    const double period = 1.0 / std::max(0.1, send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period * 1000)),
      std::bind(&JoystickLocalPathNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "joystick_local_path_node: mode=%s odom=%s action=%s frame=%s send_rate=%.1f",
      path_generation_mode_.c_str(), odom_topic_.c_str(), action_name_.c_str(),
      global_frame_.c_str(), send_rate_hz_);
    if (use_spatial_range_) {
      const double half_back = backward_sector_deg_ * 0.5;
      RCLCPP_INFO(
        get_logger(),
        "spatial_range: joystick=%s forward=%.0fdeg back=%.0fdeg hysteresis=%.0fdeg "
        "boundary=%.0fdeg steps=%d backward_reverse=%s lateral_invert=%s",
        joystick_topic_.c_str(), forward_sector_deg_, backward_sector_deg_,
        sector_hysteresis_deg_, 180.0 - half_back, path_interpolation_steps_,
        backward_reverse_path_enable_ ? "on" : "off",
        backward_reverse_lateral_invert_ ? "on" : "off");
      if (gate_path_enable_) {
        RCLCPP_INFO(
          get_logger(),
          "gate_path: cloud=%s detect=%s local_r=%.2fm detect_v<=%.2f creep_v<=%.2f "
          "width=[%.2f,%.2f]m memory_max=%d merge_r=%.2fm commit_frames=%d",
          obstacle_points_topic_.c_str(), gate_detect_mode_.c_str(),
          gate_local_radius_m_, gate_detect_speed_max_mps_, gate_creep_speed_max_mps_,
          gate_width_min_m_, gate_width_max_m_, gate_memory_max_count_,
          gate_memory_merge_radius_m_, gate_memory_confirm_frames_);
      }
      if (gate_memory_publish_enable_ && gate_path_enable_) {
        RCLCPP_INFO(
          get_logger(),
          "gate_memory: body_x=[%.3f, %.3f] half_w=%.3f band=%.2fm speed>%.2f confirm=%d "
          "max=%d merge_r=%.2f",
          gate_memory_body_x_min_m_, gate_memory_body_x_max_m_, gate_memory_half_width_m_,
          gate_memory_lateral_band_m_, gate_memory_min_speed_mps_,
          gate_memory_confirm_frames_, gate_memory_max_count_,
          gate_memory_merge_radius_m_);
      }
    }
    if (footprint_publish_enable_) {
      RCLCPP_INFO(
        get_logger(), "footprint publish: topic=%s gear_padding_cm=[%.0f,%.0f,%.0f,%.0f]",
        footprint_topic_.c_str(),
        gear_footprint_padding_m_.size() > 0 ? gear_footprint_padding_m_[0] * 100.0 : 0.0,
        gear_footprint_padding_m_.size() > 1 ? gear_footprint_padding_m_[1] * 100.0 : 0.0,
        gear_footprint_padding_m_.size() > 2 ? gear_footprint_padding_m_[2] * 100.0 : 0.0,
        gear_footprint_padding_m_.size() > 3 ? gear_footprint_padding_m_[3] * 100.0 : 0.0);
    }
    if (path_obstacle_limit_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "path_obstacle_limit: stop_margin=%.2fm min_keep=%.2fm skip_pass_mode=%s skip_in_place=%s",
        path_obstacle_stop_margin_m_, path_obstacle_min_keep_m_,
        path_obstacle_limit_skip_pass_mode_ ? "yes" : "no",
        path_obstacle_limit_skip_in_place_ ? "yes" : "no");
    }
    if (path_open_cruise_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "path_open_cruise: gear>=%d front_half_deg=[%.0f,%.0f,%.0f,%.0f] extra_pad_cm=[%.0f,%.0f,%.0f,%.0f] outer_far=%.2fm pre_exit=%d pre_exit_exit_kph=%.1f enter=%d enter_min_kph=%.1f exit_inner=%d reenter_cd=%d topic=%s pre_exit_topic=%s",
        path_open_cruise_min_gear_,
        path_open_cruise_gear_front_half_deg_.size() > 0 ?
        path_open_cruise_gear_front_half_deg_[0] : 0.0,
        path_open_cruise_gear_front_half_deg_.size() > 1 ?
        path_open_cruise_gear_front_half_deg_[1] : 0.0,
        path_open_cruise_gear_front_half_deg_.size() > 2 ?
        path_open_cruise_gear_front_half_deg_[2] : 0.0,
        path_open_cruise_gear_front_half_deg_.size() > 3 ?
        path_open_cruise_gear_front_half_deg_[3] : 0.0,
        path_open_cruise_gear_extra_padding_m_.size() > 0 ?
        path_open_cruise_gear_extra_padding_m_[0] * 100.0 : 0.0,
        path_open_cruise_gear_extra_padding_m_.size() > 1 ?
        path_open_cruise_gear_extra_padding_m_[1] * 100.0 : 0.0,
        path_open_cruise_gear_extra_padding_m_.size() > 2 ?
        path_open_cruise_gear_extra_padding_m_[2] * 100.0 : 0.0,
        path_open_cruise_gear_extra_padding_m_.size() > 3 ?
        path_open_cruise_gear_extra_padding_m_[3] * 100.0 : 0.0,
        path_open_cruise_outer_far_m_, path_open_cruise_pre_exit_confirm_frames_,
        path_open_cruise_pre_exit_exit_speed_kph_,
        path_open_cruise_confirm_frames_, path_open_cruise_enter_min_speed_kph_,
        path_open_cruise_exit_confirm_frames_,
        path_open_cruise_reenter_cooldown_frames_, path_open_cruise_topic_.c_str(),
        path_open_cruise_pre_exit_topic_.c_str());
    }
    if (release_soft_stop_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "release_soft_stop: radius=%.2fm topic=%s halt_topic=%s (skip MPPI, gate ramps)",
        release_soft_stop_radius_, release_soft_stop_topic_.c_str(),
        stick_release_halt_topic_.c_str());
    }
    if (rocker_sector_vx_min_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "rocker intent zone: front_half=%.0fdeg side_span=%.0fdeg "
        "forward_topic=%s zone_topic=%s",
        rocker_intent_front_half_deg_, rocker_intent_side_span_deg_,
        rocker_forward_sector_topic_.c_str(), rocker_intent_zone_topic_.c_str());
    }
    if (wz_density_cap_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "wz density cap: radius=%.2fm caps=[%.2f, %.2f, %.2f] cap_topic=%s tier_topic=%s",
        density_check_radius_,
        wz_density_caps_rps_.size() > 0 ? wz_density_caps_rps_[0] : 0.0,
        wz_density_caps_rps_.size() > 1 ? wz_density_caps_rps_[1] : 0.0,
        wz_density_caps_rps_.size() > 2 ? wz_density_caps_rps_[2] : 0.0,
        wz_density_cap_topic_.c_str(), wz_density_tier_topic_.c_str());
    }
    if (gear_vx_cap_enable_) {
      RCLCPP_INFO(
        get_logger(),
        "gear vx cap: kph=[%.0f, %.0f, %.0f, %.0f] topic=%s",
        gear_vx_max_mps_.size() > 0 ? gear_vx_max_mps_[0] * 3.6 : 0.0,
        gear_vx_max_mps_.size() > 1 ? gear_vx_max_mps_[1] * 3.6 : 0.0,
        gear_vx_max_mps_.size() > 2 ? gear_vx_max_mps_[2] * 3.6 : 0.0,
        gear_vx_max_mps_.size() > 3 ? gear_vx_max_mps_[3] * 3.6 : 0.0,
        gear_vx_cap_topic_.c_str());
    }
  }

private:
  enum class WzDensityTier : uint8_t { OPEN = 0, MODERATE = 1, DENSE = 2 };
  enum class RockerIntentZone : uint8_t { NONE = 0, FRONT = 1, LEFT_FRONT = 2, RIGHT_FRONT = 3 };

  RockerIntentZone classifyRockerIntentZone(double rx, double ry) const
  {
    if (std::hypot(rx, ry) <= rocker_deadband_) {
      return RockerIntentZone::NONE;
    }

    const double theta = std::atan2(ry, rx);
    const double front_half_rad = rocker_intent_front_half_deg_ * M_PI / 180.0;
    const double side_span_rad = rocker_intent_side_span_deg_ * M_PI / 180.0;
    const double forward_limit_rad = front_half_rad + side_span_rad;

    if (std::abs(theta) <= front_half_rad) {
      return RockerIntentZone::FRONT;
    }
    if (theta > front_half_rad && theta <= forward_limit_rad) {
      return RockerIntentZone::LEFT_FRONT;
    }
    if (theta < -front_half_rad && theta >= -forward_limit_rad) {
      return RockerIntentZone::RIGHT_FRONT;
    }
    return RockerIntentZone::NONE;
  }

  struct RockerIntentState
  {
    RockerIntentZone zone{RockerIntentZone::NONE};
    bool forward_sector{true};
  };

  RockerIntentState computeRockerIntentState(const rclcpp::Time & tnow)
  {
    Joystick::SharedPtr joy;
    rclcpp::Time joy_stamp;
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      joy = latest_joystick_;
      joy_stamp = latest_joystick_stamp_;
    }
    if (!joy || (tnow - joy_stamp).seconds() > joystick_timeout_) {
      return {RockerIntentZone::NONE, true};
    }

    const double rx = applyRockerDeadbandAxis(static_cast<double>(joy->rocker_x));
    const double ry = applyRockerDeadbandAxis(static_cast<double>(joy->rocker_y));
    if (std::hypot(rx, ry) <= rocker_deadband_) {
      return {RockerIntentZone::NONE, true};
    }

    const RockerIntentZone zone = classifyRockerIntentZone(rx, ry);
    return {zone, zone == RockerIntentZone::FRONT};
  }

  void publishRockerIntentZoneState(const rclcpp::Time & tnow)
  {
    if (!rocker_forward_sector_mode_pub_ && !rocker_intent_zone_pub_) {
      return;
    }

    const RockerIntentState state = computeRockerIntentState(tnow);

    if (rocker_intent_zone_pub_) {
      const auto zone = static_cast<uint8_t>(state.zone);
      std_msgs::msg::UInt8 zone_msg;
      zone_msg.data = zone;
      rocker_intent_zone_pub_->publish(zone_msg);
      rocker_intent_zone_active_ = state.zone;
    }

    if (rocker_forward_sector_mode_pub_ &&
      state.forward_sector != rocker_forward_sector_active_)
    {
      rocker_forward_sector_active_ = state.forward_sector;
      std_msgs::msg::Bool forward_msg;
      forward_msg.data = state.forward_sector;
      rocker_forward_sector_mode_pub_->publish(forward_msg);
    }
  }

  void onControl(const Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(ctrl_mutex_);
    latest_ctrl_ = msg;
    latest_ctrl_stamp_ = now();
  }

  void onJoystick(const Joystick::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    latest_joystick_ = msg;
    latest_joystick_stamp_ = now();
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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
    updateProximitySidePassCrawlModeFromCloud();
  }

  void onCarInfo(const CarInfo::SharedPtr msg)
  {
    int gear = static_cast<int>(msg->gear);
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      latest_gear_ = gear;
      have_gear_ = true;
    }
    if (gear_vx_cap_enable_) {
      publishGearVxCap(gearVxMaxMps());
    }
    if (footprint_publish_enable_) {
      publishFootprintForGear(gear);
    }
  }

  void publishFootprintWithPadding(double padding)
  {
    if (!footprint_pub_ || footprint_base_pts_.empty()) {
      return;
    }

    if (have_published_footprint_ && std::abs(padding - last_footprint_padding_) < 1e-6) {
      return;
    }

    auto pts = footprint_base_pts_;
    nav2_costmap_2d::padFootprint(pts, padding);

    geometry_msgs::msg::Polygon poly = nav2_costmap_2d::toPolygon(pts);
    footprint_pub_->publish(poly);
    last_footprint_padding_ = padding;
    have_published_footprint_ = true;
  }

  void publishFootprintForGear(int gear)
  {
    if (narrow_gate_pass_active_ || narrow_gate_side_pass_crawl_active_) {
      publishFootprintWithPadding(0.0);
      return;
    }
    const double padding = gearFootprintPadding(gear);
    publishFootprintWithPadding(padding);
    RCLCPP_INFO(
      get_logger(), "local_costmap footprint gear=%d padding=%.0fcm",
      gear, padding * 100.0);
  }

  void setNarrowGatePassMode(bool active, const rclcpp::Time & tnow)
  {
    if (active == narrow_gate_pass_active_) {
      return;
    }
    narrow_gate_pass_active_ = active;
    narrow_gate_pass_last_intent_stamp_ = tnow;
    have_published_footprint_ = false;
    pass_mode_gate_odom_valid_ = false;

    std_msgs::msg::Bool msg;
    msg.data = active;
    narrow_gate_pass_mode_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(), "narrow_gate_pass_mode -> %s",
      active ? "ACTIVE" : "inactive");
  }

  enum class SidePassSource { None, MemoryGate, Proximity };

  void setSidePassCrawlMode(
    bool active, const rclcpp::Time & tnow, int memory_idx = -1,
    SidePassSource source = SidePassSource::MemoryGate)
  {
    if (active == narrow_gate_side_pass_crawl_active_ &&
      (!active || side_pass_source_ == source))
    {
      return;
    }
    narrow_gate_side_pass_crawl_active_ = active;
    narrow_gate_side_pass_last_intent_stamp_ = tnow;
    have_published_footprint_ = false;
    if (active) {
      side_pass_source_ = source;
      side_pass_memory_gate_idx_ = memory_idx;
      proximity_side_pass_enter_streak_ = 0;
      proximity_side_pass_exit_streak_ = 0;
      cancelActiveGoal();
    } else {
      side_pass_source_ = SidePassSource::None;
      side_pass_memory_gate_idx_ = -1;
      proximity_side_pass_enter_streak_ = 0;
      proximity_side_pass_exit_streak_ = 0;
      scheduleMppiReset("side_pass exit");
    }

    std_msgs::msg::Bool msg;
    msg.data = active;
    narrow_gate_side_pass_mode_pub_->publish(msg);

    const char * src =
      source == SidePassSource::Proximity ? "proximity" :
      source == SidePassSource::MemoryGate ? "memory_gate" : "none";
    RCLCPP_INFO(
      get_logger(), "narrow_gate_side_pass_mode -> %s (%s)",
      active ? "ACTIVE (crawl)" : "inactive", src);
  }

  void publishOpenCorridorCruisePreExitModeState()
  {
    std_msgs::msg::Bool msg;
    msg.data = open_corridor_cruise_pre_exit_active_;
    open_corridor_cruise_pre_exit_mode_pub_->publish(msg);
  }

  void resetOpenCruisePreExitState()
  {
    path_open_cruise_pre_exit_streak_ = 0;
    path_open_cruise_stall_streak_ = 0;
  }

  void setOpenCorridorCruisePreExitMode(bool active)
  {
    if (active == open_corridor_cruise_pre_exit_active_) {
      return;
    }
    open_corridor_cruise_pre_exit_active_ = active;
    publishOpenCorridorCruisePreExitModeState();
    RCLCPP_INFO(
      get_logger(), "open_corridor_cruise_pre_exit -> %s",
      active ? "ACTIVE (jerk decel)" : "inactive");
  }

  void publishOpenCorridorCruiseModeState()
  {
    std_msgs::msg::Bool msg;
    msg.data = open_corridor_cruise_active_;
    open_corridor_cruise_mode_pub_->publish(msg);
  }

  void setOpenCorridorCruiseMode(bool active, const rclcpp::Time & tnow)
  {
    (void)tnow;
    if (active == open_corridor_cruise_active_) {
      return;
    }
    open_corridor_cruise_active_ = active;
    publishOpenCorridorCruiseModeState();
    if (active) {
      cancelActiveGoal();
      resetOpenCruisePreExitState();
      RCLCPP_INFO(
        get_logger(),
        "open_corridor_cruise -> ACTIVE (skip MPPI, gear cruise, confirm=%d frames)",
        path_open_cruise_confirm_frames_);
    } else {
      setOpenCorridorCruisePreExitMode(false);
      resetOpenCruisePreExitState();
      RCLCPP_INFO(get_logger(), "open_corridor_cruise -> inactive (MPPI resume)");
      path_open_cruise_enter_streak_ = 0;
      path_open_cruise_exit_streak_ = 0;
      path_open_cruise_frames_since_exit_ = 0;
      open_cruise_mppi_resume_ = true;
    }
  }

  void publishReleaseSoftStopModeState()
  {
    std_msgs::msg::Bool msg;
    msg.data = release_soft_stop_active_;
    release_soft_stop_mode_pub_->publish(msg);
  }

  void publishStickReleaseHaltState()
  {
    std_msgs::msg::Bool msg;
    msg.data = stick_release_halt_active_;
    stick_release_halt_pub_->publish(msg);
  }

  void setReleaseSoftStopMode(bool active)
  {
    if (active == release_soft_stop_active_) {
      return;
    }
    release_soft_stop_active_ = active;
    publishReleaseSoftStopModeState();
    if (active) {
      cancelActiveGoal();
      RCLCPP_INFO(
        get_logger(),
        "release_soft_stop -> ACTIVE (skip MPPI, clear within %.2fm)",
        release_soft_stop_radius_);
    } else {
      RCLCPP_INFO(get_logger(), "release_soft_stop -> inactive");
    }
  }

  void setStickReleaseHalt(bool active)
  {
    if (active == stick_release_halt_active_) {
      return;
    }
    stick_release_halt_active_ = active;
    publishStickReleaseHaltState();
    if (active) {
      cancelActiveGoal();
      RCLCPP_INFO(get_logger(), "stick_release_halt -> ACTIVE (skip MPPI, gate halt)");
    } else {
      RCLCPP_INFO(get_logger(), "stick_release_halt -> inactive");
    }
  }

  int nearbyObstaclePointCountInRadius(
    double radius_m, const rclcpp::Time & tnow) const
  {
    if (radius_m <= 0.0) {
      return 0;
    }
    std::vector<std::pair<float, float>> pts;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!have_cloud_ || (tnow - latest_cloud_stamp_).seconds() > obstacle_timeout_) {
        return -1;
      }
      pts = cloud_pts_;
    }
    const double r2 = radius_m * radius_m;
    int count = 0;
    for (const auto & p : pts) {
      const double d2 = static_cast<double>(p.first) * p.first +
        static_cast<double>(p.second) * p.second;
      if (d2 <= r2) {
        ++count;
      }
    }
    return count;
  }

  int nearbyObstaclePointCount(const rclcpp::Time & tnow) const
  {
    return nearbyObstaclePointCountInRadius(density_check_radius_, tnow);
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

  double wzDensityCapRps() const
  {
    if (!wz_density_cap_enable_ || wz_density_caps_rps_.empty()) {
      return wz_density_caps_rps_.empty() ? 1.30 : wz_density_caps_rps_.front();
    }
    const size_t idx = static_cast<size_t>(current_wz_tier_);
    if (idx >= wz_density_caps_rps_.size()) {
      return wz_density_caps_rps_.back();
    }
    return wz_density_caps_rps_[idx];
  }

  void publishWzDensityCap(double cap_rps)
  {
    if (!wz_density_cap_pub_) {
      return;
    }
    std_msgs::msg::Float64 msg;
    msg.data = cap_rps;
    wz_density_cap_pub_->publish(msg);
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

  double gearVxMaxMps() const
  {
    return pickGearTable(gear_vx_max_mps_, currentGearIndex(), gear_default_index_);
  }

  void publishGearVxCap(double cap_mps)
  {
    if (!gear_vx_cap_pub_) {
      return;
    }
    std_msgs::msg::Float64 msg;
    msg.data = cap_mps;
    gear_vx_cap_pub_->publish(msg);
  }

  void updateWzDensityCap(const rclcpp::Time & tnow)
  {
    if (!wz_density_cap_enable_ || !wz_density_cap_pub_) {
      return;
    }

    const int nearby = nearbyObstaclePointCount(tnow);
    if (nearby < 0) {
      current_wz_tier_ = WzDensityTier::DENSE;
    } else {
      updateWzDensityTier(nearby, tnow);
    }

    publishWzDensityCap(wzDensityCapRps());
    if (wz_density_tier_pub_) {
      std_msgs::msg::UInt8 tier_msg;
      tier_msg.data = static_cast<uint8_t>(current_wz_tier_);
      wz_density_tier_pub_->publish(tier_msg);
    }
    if (gear_vx_cap_enable_) {
      publishGearVxCap(gearVxMaxMps());
    }
  }

  void updateReleaseSoftStopOnStickIdle(const rclcpp::Time & tnow)
  {
    if (!release_soft_stop_enable_) {
      setReleaseSoftStopMode(false);
      setStickReleaseHalt(true);
      return;
    }
    const int nearby = nearbyObstaclePointCountInRadius(release_soft_stop_radius_, tnow);
    if (nearby < 0) {
      setReleaseSoftStopMode(false);
      setStickReleaseHalt(true);
      return;
    }
    if (nearby == 0) {
      setReleaseSoftStopMode(true);
      setStickReleaseHalt(false);
    } else {
      setReleaseSoftStopMode(false);
      setStickReleaseHalt(true);
    }
  }

  void clearReleaseSoftStopModes()
  {
    setReleaseSoftStopMode(false);
    setStickReleaseHalt(false);
  }

  void publishSidePassCrawlModeState()
  {
    std_msgs::msg::Bool msg;
    msg.data = narrow_gate_side_pass_crawl_active_;
    narrow_gate_side_pass_mode_pub_->publish(msg);
  }

  void clearSidePassCrawlMode(const rclcpp::Time & tnow)
  {
    setSidePassCrawlMode(false, tnow);
  }

  bool cloudFresh(const rclcpp::Time & tnow) const
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return have_cloud_ &&
      (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
  }

  int countObstaclePointsInBox(
    double x_min, double x_max, double y_min, double y_max)
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

  bool sidePassCrawlRoiClear(
    double x_min, double x_max, double y_min, double y_max)
  {
    return countObstaclePointsInBox(x_min, x_max, y_min, y_max) == 0;
  }

  bool sidePassCrawlForwardStripsClear()
  {
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_forward_strip_x_.size(); ++i) {
      const double x0 = side_pass_crawl_forward_strip_x_[i];
      if (!sidePassCrawlRoiClear(
          x0, x0 + depth,
          side_pass_crawl_forward_strip_y_min_[i],
          side_pass_crawl_forward_strip_y_max_[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool sidePassCrawlRearStripsClear()
  {
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_rear_strip_x_.size(); ++i) {
      const double x0 = side_pass_crawl_rear_strip_x_[i];
      if (!sidePassCrawlRoiClear(
          x0 - depth, x0,
          side_pass_crawl_rear_strip_y_min_[i],
          side_pass_crawl_rear_strip_y_max_[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool sidePassCrawlLateralStripsClear(bool left)
  {
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_lateral_strip_x_min_.size(); ++i) {
      const double ye = side_pass_crawl_lateral_strip_y_edge_[i];
      const double x0 = side_pass_crawl_lateral_strip_x_min_[i];
      const double x1 = side_pass_crawl_lateral_strip_x_max_[i];
      if (left) {
        if (!sidePassCrawlRoiClear(x0, x1, ye, ye + depth)) {
          return false;
        }
      } else if (!sidePassCrawlRoiClear(x0, x1, -(ye + depth), -ye)) {
        return false;
      }
    }
    return true;
  }

  Twist computeSidePassCrawlTwist(const rclcpp::Time & tnow)
  {
    Twist out;
    Joystick::SharedPtr joy;
    rclcpp::Time joy_stamp;
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      joy = latest_joystick_;
      joy_stamp = latest_joystick_stamp_;
    }
    if (!joy || (tnow - joy_stamp).seconds() > joystick_timeout_) {
      return out;
    }

    const double rx = static_cast<double>(joy->rocker_x);
    const double ry = static_cast<double>(joy->rocker_y);

    if (rx > side_pass_crawl_rocker_forward_min_ &&
      sidePassCrawlForwardStripsClear())
    {
      out.linear.x = side_pass_crawl_vx_mps_;
    } else if (rx < -side_pass_crawl_rocker_forward_min_ &&
      sidePassCrawlRearStripsClear())
    {
      out.linear.x = -side_pass_crawl_vx_mps_;
    }

    if (ry > side_pass_crawl_rocker_lateral_min_ &&
      sidePassCrawlLateralStripsClear(true))
    {
      out.angular.z = side_pass_crawl_wz_rps_;
    } else if (ry < -side_pass_crawl_rocker_lateral_min_ &&
      sidePassCrawlLateralStripsClear(false))
    {
      out.angular.z = -side_pass_crawl_wz_rps_;
    }

    return out;
  }

  void publishSidePassCrawlCmdVel(const rclcpp::Time & tnow)
  {
    Twist out;
    if (cloudFresh(tnow) &&
      !release_soft_stop_active_ && !stick_release_halt_active_)
    {
      out = computeSidePassCrawlTwist(tnow);
    } else if (!cloudFresh(tnow)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "side_pass crawl halted: cloud stale");
    }
    side_pass_crawl_cmd_vel_pub_->publish(out);
  }

  // Forward-strip ROI helpers (footprint_base x/y; no gear padding in cloud checks).
  // Enter/exit depths differ: side_pass 15/10 cm, proximity 10 cm, crawl 6 cm.
  bool forwardCornerStripHasPoint(
    const std::vector<std::pair<float, float>> & local_pts,
    double x0, double depth, double y0, double y1) const
  {
    const double x1 = x0 + depth;
    for (const auto & p : local_pts) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
        return true;
      }
    }
    return false;
  }

  bool sidePassEnterForwardRegionHasCloud(
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    if (!gate_side_pass_enter_forward_cloud_enable_) {
      return true;
    }
    if (local_pts.empty()) {
      return false;
    }

    const double depth = gate_side_pass_enter_check_depth_m_;
    for (size_t i = 0; i < gate_side_pass_enter_forward_strip_x_.size(); ++i) {
      if (forwardCornerStripHasPoint(
          local_pts,
          gate_side_pass_enter_forward_strip_x_[i], depth,
          gate_side_pass_enter_forward_strip_y_min_[i],
          gate_side_pass_enter_forward_strip_y_max_[i]))
      {
        return true;
      }
    }
    return false;
  }

  bool sidePassExitForwardCornersClear(
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    if (!gate_side_pass_exit_forward_clear_enable_) {
      return true;
    }
    if (local_pts.empty()) {
      return true;
    }

    const double depth = gate_side_pass_exit_check_depth_m_;
    for (size_t i = 0; i < gate_side_pass_exit_forward_strip_x_.size(); ++i) {
      const double x0 = gate_side_pass_exit_forward_strip_x_[i];
      const double x1 = x0 + depth;
      const double y0 = gate_side_pass_exit_forward_strip_y_min_[i];
      const double y1 = gate_side_pass_exit_forward_strip_y_max_[i];
      for (const auto & p : local_pts) {
        const double x = static_cast<double>(p.first);
        const double y = static_cast<double>(p.second);
        if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
          return false;
        }
      }
    }
    return true;
  }

  bool proximitySidePassInnerForwardClear(
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    if (local_pts.empty()) {
      return false;
    }
    const double depth = side_pass_crawl_check_depth_m_;
    for (size_t i = 0; i < side_pass_crawl_forward_strip_x_.size(); ++i) {
      if (forwardCornerStripHasPoint(
          local_pts,
          side_pass_crawl_forward_strip_x_[i], depth,
          side_pass_crawl_forward_strip_y_min_[i],
          side_pass_crawl_forward_strip_y_max_[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool proximitySidePassForwardOuterStripBlocked(
    const std::vector<std::pair<float, float>> & local_pts, bool left) const
  {
    if (local_pts.empty() ||
      side_pass_crawl_forward_strip_x_.size() < 3 ||
      side_pass_crawl_forward_strip_y_min_.size() < 3 ||
      side_pass_crawl_forward_strip_y_max_.size() < 3)
    {
      return false;
    }
    const size_t i = left ? 0 : 2;
    const double x0 =
      side_pass_crawl_forward_strip_x_[i] + side_pass_crawl_check_depth_m_;
    const double x1 = x0 + proximity_side_pass_outer_probe_depth_m_;
    return forwardCornerStripHasPoint(
      local_pts, x0, x1 - x0,
      side_pass_crawl_forward_strip_y_min_[i],
      side_pass_crawl_forward_strip_y_max_[i]);
  }

  bool proximitySidePassEnterConditionsMet(
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    if (local_pts.empty()) {
      return false;
    }
    const bool inner_clear = proximitySidePassInnerForwardClear(local_pts);
    const bool outer_blocked =
      proximitySidePassForwardOuterStripBlocked(local_pts, true) ||
      proximitySidePassForwardOuterStripBlocked(local_pts, false);
    return inner_clear && outer_blocked;
  }

  bool proximitySidePassSurroundingClear(
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    if (local_pts.empty()) {
      return false;
    }
    const double m = proximity_side_pass_exit_margin_m_;
    const double x0 = footprint_body_x_min_ - m;
    const double x1 = footprint_body_x_max_ + m;
    const double y0 = footprint_body_y_min_ - m;
    const double y1 = footprint_body_y_max_ + m;
    for (const auto & p : local_pts) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
        return false;
      }
    }
    return true;
  }

  void updateProximitySidePassCrawlModeFromCloud()
  {
    if (!proximity_side_pass_enable_) {
      return;
    }
    if (narrow_gate_pass_active_) {
      proximity_side_pass_enter_streak_ = 0;
      return;
    }
    if (narrow_gate_side_pass_crawl_active_ &&
      side_pass_source_ == SidePassSource::MemoryGate)
    {
      return;
    }

    const rclcpp::Time tnow = now();
    Odometry::SharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = latest_odom_;
    }
    if (!odom) {
      proximity_side_pass_enter_streak_ = 0;
      return;
    }

    const auto local_pts = filterLocalCloudPoints(cloud_pts_);

    if (narrow_gate_side_pass_crawl_active_ &&
      side_pass_source_ == SidePassSource::Proximity)
    {
      if (proximitySidePassSurroundingClear(local_pts)) {
        ++proximity_side_pass_exit_streak_;
        if (proximity_side_pass_exit_streak_ >= proximity_side_pass_exit_confirm_frames_) {
          setSidePassCrawlMode(false, tnow);
          RCLCPP_INFO(
            get_logger(),
            "proximity side_pass crawl exit: surrounding %.0fcm clear",
            proximity_side_pass_exit_margin_m_ * 100.0);
        }
      } else {
        proximity_side_pass_exit_streak_ = 0;
      }
      return;
    }

    const bool speed_ok =
      vehiclePlanarSpeedMps(*odom) < proximity_side_pass_max_speed_mps_;
    const bool wz_ok =
      std::abs(odom->twist.twist.angular.z) < proximity_side_pass_max_wz_rps_;
    const bool enter_ok = proximitySidePassEnterConditionsMet(local_pts);
    const bool rocker_ok = isRockerOutOfDeadband(tnow);
    if (!speed_ok || !wz_ok || !enter_ok || !rocker_ok) {
      proximity_side_pass_enter_streak_ = 0;
      return;
    }

    ++proximity_side_pass_enter_streak_;
    if (proximity_side_pass_enter_streak_ >= proximity_side_pass_confirm_frames_) {
      setSidePassCrawlMode(true, tnow, -1, SidePassSource::Proximity);
      RCLCPP_INFO(
        get_logger(),
        "proximity side_pass crawl enter: |v|<%.2f |wz|<%.2f inner %.0fmm clear "
        "outer %.0fmm blocked x%d frames",
        proximity_side_pass_max_speed_mps_,
        proximity_side_pass_max_wz_rps_,
        side_pass_crawl_check_depth_m_ * 100.0,
        proximity_side_pass_outer_probe_depth_m_ * 100.0,
        proximity_side_pass_confirm_frames_);
      proximity_side_pass_enter_streak_ = 0;
    }
  }

  void updateSidePassCrawlMode(
    const NarrowGateInfo & gate, double norm_x, double norm_y,
    const Odometry & odom, const rclcpp::Time & tnow, int memory_idx,
    const std::vector<std::pair<float, float>> & local_pts)
  {
    const double border_dist = bodyBorderDistanceToPoint(
      gate.distance_m, gate.center_y_m);
    const bool forward_corners_clear = sidePassExitForwardCornersClear(local_pts);

    if (gateSidePassBodyCleared(gate)) {
      if (forward_corners_clear) {
        setSidePassCrawlMode(false, tnow);
        RCLCPP_INFO(get_logger(), "side_pass crawl exit: cleared along bisector");
        return;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "side_pass crawl exit held: forward corner ROI blocked (bisector cleared)");
    }

    if (border_dist > gate_side_pass_trigger_dist_m_) {
      if (narrow_gate_side_pass_crawl_active_) {
        if (forward_corners_clear) {
          setSidePassCrawlMode(false, tnow);
          RCLCPP_INFO(
            get_logger(),
            "side_pass crawl exit: border_dist=%.2fm > %.2fm",
            border_dist, gate_side_pass_trigger_dist_m_);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "side_pass crawl exit held: forward corner ROI blocked (border_dist=%.2fm)",
            border_dist);
        }
      }
      return;
    }

    const bool intent_ok = stickPointsTowardSidePass(gate, norm_x, norm_y);
    const bool speed_ok = vehiclePlanarSpeedMps(odom) < gate_side_pass_max_speed_mps_;
    const bool forward_region_blocked =
      sidePassEnterForwardRegionHasCloud(local_pts);
    if (!narrow_gate_side_pass_crawl_active_) {
      if (intent_ok && speed_ok && forward_region_blocked) {
        setSidePassCrawlMode(true, tnow, memory_idx);
        RCLCPP_INFO(
          get_logger(),
          "side_pass crawl enter: border_dist=%.2fm gate body=(%.2f, %.2f) idx=%d",
          border_dist, gate.distance_m, gate.center_y_m, memory_idx);
      } else if (intent_ok && speed_ok) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "side_pass crawl enter held: no cloud in left/right/front forward ROI");
      }
      return;
    }

    if (intent_ok) {
      narrow_gate_side_pass_last_intent_stamp_ = tnow;
    } else if ((tnow - narrow_gate_side_pass_last_intent_stamp_).seconds() >
      gate_pass_mode_hold_sec_)
    {
      setSidePassCrawlMode(false, tnow);
      RCLCPP_INFO(
        get_logger(), "side_pass crawl exit: intent timeout (%.2fs)",
        (tnow - narrow_gate_side_pass_last_intent_stamp_).seconds());
    }
  }

  void publishNarrowGatePassModeState()
  {
    std_msgs::msg::Bool msg;
    msg.data = narrow_gate_pass_active_;
    narrow_gate_pass_mode_pub_->publish(msg);
  }

  void publishGateMarkers(const Odometry & odom)
  {
    if (!gate_marker_pub_) {
      return;
    }

    const rclcpp::Time stamp = now();
    const rclcpp::Duration marker_lifetime(0, 0);
    visualization_msgs::msg::MarkerArray markers;

    auto makeDelete = [&](const std::string & ns, int id) {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = global_frame_;
      del.header.stamp = stamp;
      del.ns = ns;
      del.id = id;
      del.action = visualization_msgs::msg::Marker::DELETE;
      return del;
    };

    std::lock_guard<std::mutex> lock(gate_mutex_);

    for (size_t i = 0; i < memorized_gates_.size(); ++i) {
      const auto & stored = memorized_gates_[i];
      visualization_msgs::msg::Marker line;
      line.header.frame_id = global_frame_;
      line.header.stamp = stamp;
      line.ns = "gate_memory_frame";
      line.id = static_cast<int>(i);
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.04;
      line.color.r = 0.25F;
      line.color.g = 0.65F;
      line.color.b = 1.0F;
      line.color.a = 0.75F;
      line.lifetime = marker_lifetime;
      geometry_msgs::msg::Point pl;
      pl.x = stored.left_x_odom;
      pl.y = stored.left_y_odom;
      pl.z = 0.08;
      geometry_msgs::msg::Point pr;
      pr.x = stored.right_x_odom;
      pr.y = stored.right_y_odom;
      pr.z = 0.08;
      line.points.push_back(pl);
      line.points.push_back(pr);
      markers.markers.push_back(line);

      visualization_msgs::msg::Marker mem;
      mem.header = line.header;
      mem.ns = "gate_memory";
      mem.id = static_cast<int>(i);
      mem.type = visualization_msgs::msg::Marker::SPHERE;
      mem.action = visualization_msgs::msg::Marker::ADD;
      mem.pose.position.x = stored.center_x_odom;
      mem.pose.position.y = stored.center_y_odom;
      mem.pose.position.z = 0.08;
      mem.pose.orientation.w = 1.0;
      mem.scale.x = 0.12;
      mem.scale.y = 0.12;
      mem.scale.z = 0.12;
      mem.color.r = 0.25F;
      mem.color.g = 0.65F;
      mem.color.b = 1.0F;
      mem.color.a = 0.55F;
      mem.lifetime = marker_lifetime;
      markers.markers.push_back(mem);

      visualization_msgs::msg::Marker label;
      label.header = mem.header;
      label.ns = "gate_memory_label";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = mem.pose.position;
      label.pose.position.z = 0.35;
      label.scale.z = 0.18;
      label.color.r = 0.8F;
      label.color.g = 0.9F;
      label.color.b = 1.0F;
      label.color.a = 0.9F;
      label.text = "M" + std::to_string(i + 1) + " n=" +
        std::to_string(stored.pass_count);
      label.lifetime = mem.lifetime;
      markers.markers.push_back(label);
    }

    for (int i = static_cast<int>(memorized_gates_.size());
      i < gate_memory_max_count_; ++i)
    {
      markers.markers.push_back(makeDelete("gate_memory_frame", i));
      markers.markers.push_back(makeDelete("gate_memory", i));
      markers.markers.push_back(makeDelete("gate_memory_label", i));
    }

    if (have_last_gate_) {
      const double half_w = 0.5 * last_gate_.width_m;
      double ax_o = 0.0;
      double ay_o = 0.0;
      double bx_o = 0.0;
      double by_o = 0.0;
      double cx_o = 0.0;
      double cy_o = 0.0;
      bodyCenterToOdom(
        odom, last_gate_.distance_m, last_gate_.center_y_m - half_w, ax_o, ay_o);
      bodyCenterToOdom(
        odom, last_gate_.distance_m, last_gate_.center_y_m + half_w, bx_o, by_o);
      bodyCenterToOdom(
        odom, last_gate_.distance_m, last_gate_.center_y_m, cx_o, cy_o);

      visualization_msgs::msg::Marker line;
      line.header.frame_id = global_frame_;
      line.header.stamp = stamp;
      line.ns = "gate_active";
      line.id = 0;
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = narrow_gate_pass_active_ ? 0.06 : 0.04;
      if (narrow_gate_pass_active_) {
        line.color.r = 1.0F;
        line.color.g = 0.15F;
        line.color.b = 1.0F;
      } else if (last_gate_.confirmed) {
        line.color.r = 0.1F;
        line.color.g = 1.0F;
        line.color.b = 0.2F;
      } else {
        line.color.r = 1.0F;
        line.color.g = 0.85F;
        line.color.b = 0.0F;
      }
      line.color.a = 0.95F;
      line.lifetime = marker_lifetime;

      geometry_msgs::msg::Point pa;
      pa.x = ax_o;
      pa.y = ay_o;
      pa.z = 0.10;
      geometry_msgs::msg::Point pb;
      pb.x = bx_o;
      pb.y = by_o;
      pb.z = 0.10;
      line.points.push_back(pa);
      line.points.push_back(pb);
      markers.markers.push_back(line);

      visualization_msgs::msg::Marker center;
      center.header = line.header;
      center.ns = "gate_active";
      center.id = 1;
      center.type = visualization_msgs::msg::Marker::SPHERE;
      center.action = visualization_msgs::msg::Marker::ADD;
      center.pose.position.x = cx_o;
      center.pose.position.y = cy_o;
      center.pose.position.z = 0.10;
      center.pose.orientation.w = 1.0;
      center.scale.x = 0.10;
      center.scale.y = 0.10;
      center.scale.z = 0.10;
      center.color = line.color;
      center.color.a = 0.85F;
      center.lifetime = line.lifetime;
      markers.markers.push_back(center);

      visualization_msgs::msg::Marker label;
      label.header = line.header;
      label.ns = "gate_active";
      label.id = 2;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = cx_o;
      label.pose.position.y = cy_o;
      label.pose.position.z = 0.35;
      label.scale.z = 0.18;
      label.color = line.color;
      label.color.a = 0.95F;
      label.text = std::string(narrow_gate_pass_active_ ? "PASS " : "") +
        (last_gate_.confirmed ? "confirmed " : "track ") +
        "x=" + std::to_string(static_cast<int>(last_gate_.distance_m * 100.0)) + "cm " +
        "y=" + std::to_string(static_cast<int>(last_gate_.center_y_m * 100.0)) + "cm";
      label.lifetime = line.lifetime;
      markers.markers.push_back(label);
    } else {
      markers.markers.push_back(makeDelete("gate_active", 0));
      markers.markers.push_back(makeDelete("gate_active", 1));
      markers.markers.push_back(makeDelete("gate_active", 2));
    }

    gate_marker_pub_->publish(markers);
  }

  void publishGateMemorySnapshotUnlocked(const rclcpp::Time & stamp)
  {
    if (!gate_memory_pub_ && !gate_memory_state_pub_) {
      return;
    }

    const rclcpp::Duration marker_lifetime(0, 0);
    visualization_msgs::msg::MarkerArray markers;

    auto makeDelete = [&](const std::string & ns, int id) {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = global_frame_;
      del.header.stamp = stamp;
      del.ns = ns;
      del.id = id;
      del.action = visualization_msgs::msg::Marker::DELETE;
      return del;
    };

    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    json << "{\"frame_id\":\"" << global_frame_ << "\","
         << "\"stamp_sec\":" << stamp.seconds() << ",\"gates\":[";

    for (size_t i = 0; i < memorized_gates_.size(); ++i) {
      const auto & stored = memorized_gates_[i];
      if (i > 0) {
        json << ',';
      }
      json << "{\"id\":" << i
           << ",\"center\":{\"x\":" << stored.center_x_odom
           << ",\"y\":" << stored.center_y_odom << "}"
           << ",\"left\":{\"x\":" << stored.left_x_odom
           << ",\"y\":" << stored.left_y_odom << "}"
           << ",\"right\":{\"x\":" << stored.right_x_odom
           << ",\"y\":" << stored.right_y_odom << "}"
           << ",\"pass_count\":" << stored.pass_count << "}";

      visualization_msgs::msg::Marker line;
      line.header.frame_id = global_frame_;
      line.header.stamp = stamp;
      line.ns = "gate_memory_frame";
      line.id = static_cast<int>(i);
      line.type = visualization_msgs::msg::Marker::LINE_LIST;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.05;
      line.color.r = 0.25F;
      line.color.g = 0.65F;
      line.color.b = 1.0F;
      line.color.a = 0.9F;
      line.lifetime = marker_lifetime;
      geometry_msgs::msg::Point pl;
      pl.x = stored.left_x_odom;
      pl.y = stored.left_y_odom;
      pl.z = 0.08;
      geometry_msgs::msg::Point pr;
      pr.x = stored.right_x_odom;
      pr.y = stored.right_y_odom;
      pr.z = 0.08;
      line.points.push_back(pl);
      line.points.push_back(pr);
      markers.markers.push_back(line);

      visualization_msgs::msg::Marker mem;
      mem.header = line.header;
      mem.ns = "gate_memory";
      mem.id = static_cast<int>(i);
      mem.type = visualization_msgs::msg::Marker::SPHERE;
      mem.action = visualization_msgs::msg::Marker::ADD;
      mem.pose.position.x = stored.center_x_odom;
      mem.pose.position.y = stored.center_y_odom;
      mem.pose.position.z = 0.10;
      mem.pose.orientation.w = 1.0;
      mem.scale.x = 0.12;
      mem.scale.y = 0.12;
      mem.scale.z = 0.12;
      mem.color.r = 0.25F;
      mem.color.g = 0.65F;
      mem.color.b = 1.0F;
      mem.color.a = 0.85F;
      mem.lifetime = marker_lifetime;
      markers.markers.push_back(mem);

      visualization_msgs::msg::Marker label;
      label.header = mem.header;
      label.ns = "gate_memory_label";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = mem.pose.position;
      label.pose.position.z = 0.35;
      label.scale.z = 0.18;
      label.color.r = 0.8F;
      label.color.g = 0.9F;
      label.color.b = 1.0F;
      label.color.a = 0.9F;
      label.text = "M" + std::to_string(i + 1) + " n=" +
        std::to_string(stored.pass_count);
      label.lifetime = mem.lifetime;
      markers.markers.push_back(label);
    }
    json << "]}";

    for (int i = static_cast<int>(memorized_gates_.size());
      i < gate_memory_max_count_; ++i)
    {
      markers.markers.push_back(makeDelete("gate_memory_frame", i));
      markers.markers.push_back(makeDelete("gate_memory", i));
      markers.markers.push_back(makeDelete("gate_memory_label", i));
    }

    if (gate_memory_pub_) {
      gate_memory_pub_->publish(markers);
    }
    if (gate_memory_state_pub_) {
      std_msgs::msg::String state;
      state.data = json.str();
      gate_memory_state_pub_->publish(state);
    }
  }

  void publishGateMemorySnapshot()
  {
    if (!gate_memory_pub_ && !gate_memory_state_pub_) {
      return;
    }
    const rclcpp::Time stamp = now();
    std::lock_guard<std::mutex> lock(gate_mutex_);
    publishGateMemorySnapshotUnlocked(stamp);
  }

  double gateFrontCenterToGateM(const NarrowGateInfo & gate) const
  {
    const double dx = gate.distance_m - gate_pass_front_center_x_m_;
    const double dy = gate.center_y_m;
    return std::hypot(dx, dy);
  }

  bool gateBodyCleared(const NarrowGateInfo & gate) const
  {
    return gate.distance_m <= gate_pass_mode_body_clear_x_m_;
  }

  bool gateSidePassBodyCleared(const NarrowGateInfo & gate) const
  {
    const double cx = gate.distance_m;
    const double cy = gate.center_y_m;
    const double ex = std::cos(gate.axis_yaw_body);
    const double ey = std::sin(gate.axis_yaw_body);
    const double along = (-cx) * ex + (-cy) * ey;
    return along >= gate_side_pass_body_clear_m_;
  }

  void updateNarrowGatePassMode(
    const NarrowGateInfo & gate, double norm_x, double norm_y,
    const Odometry & odom, const rclcpp::Time & tnow)
  {
    const bool intent_ok = intentMatchesGate(gate, norm_x, norm_y, odom) &&
      intentMatchesTrackDirection(norm_x, norm_y, odom, &gate);
    const double front_to_gate = gateFrontCenterToGateM(gate);

    if (gateBodyCleared(gate)) {
      setNarrowGatePassMode(false, tnow);
      RCLCPP_INFO(
        get_logger(),
        "narrow_gate_pass_mode exit: body cleared gate x=%.2fm",
        gate.distance_m);
      return;
    }

    if (narrow_gate_pass_active_) {
      if (intent_ok) {
        narrow_gate_pass_last_intent_stamp_ = tnow;
      } else if (gate_pass_mode_intent_timeout_enable_ &&
        !gate_pass_mode_latch_until_body_clear_ &&
        (tnow - narrow_gate_pass_last_intent_stamp_).seconds() >
        gate_pass_mode_hold_sec_)
      {
        setNarrowGatePassMode(false, tnow);
        RCLCPP_INFO(
          get_logger(),
          "narrow_gate_pass_mode exit: intent timeout (%.2fs > %.2fs)",
          (tnow - narrow_gate_pass_last_intent_stamp_).seconds(),
          gate_pass_mode_hold_sec_);
        return;
      }
      if (gate_pass_mode_front_to_gate_exit_enable_ &&
        !gate_pass_mode_latch_until_body_clear_ &&
        front_to_gate > gate_pass_mode_exit_m_)
      {
        setNarrowGatePassMode(false, tnow);
        RCLCPP_INFO(
          get_logger(),
          "narrow_gate_pass_mode exit: front_to_gate=%.2fm > %.2fm",
          front_to_gate, gate_pass_mode_exit_m_);
      }
      return;
    }

    if (!isVehicleCreepOrStopped(odom)) {
      return;
    }

    if (intent_ok && front_to_gate <= gate_pass_mode_enter_m_) {
      if (!gate.confirmed) {
        return;
      }
      setNarrowGatePassMode(true, tnow);
      RCLCPP_INFO(
        get_logger(),
        "narrow_gate_pass_mode enter: front_to_gate=%.2fm gate=(%.2f, %.2f)",
        front_to_gate, gate.distance_m, gate.center_y_m);
    }
  }

  void clearNarrowGatePassMode(const rclcpp::Time & tnow)
  {
    setNarrowGatePassMode(false, tnow);
  }

  void onOdom(const Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
  }

  void onTimer()
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "FollowPath action server not ready, waiting...");
      return;
    }

    Odometry::SharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom = latest_odom_;
    }

    const auto tnow = now();
    updateWzDensityCap(tnow);

    if (!isOdomFresh(odom, tnow)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "odom stale or missing, cancelling FollowPath goal");
      cancelActiveGoal();
      publishEmptyPath();
      setOpenCorridorCruiseMode(false, tnow);
      clearReleaseSoftStopModes();
      return;
    }

    Path path;
    bool stick_idle = false;
    if (use_spatial_range_) {
      if (!buildSpatialPath(odom, tnow, path, stick_idle)) {
        if (stick_idle) {
          updateReleaseSoftStopOnStickIdle(tnow);
          if (!was_stick_idle_) {
            scheduleMppiReset("stick center");
            publishEmptyPath();
            was_stick_idle_ = true;
          }
          resetGateTracking();
          clearNarrowGatePassMode(tnow);
          clearSidePassCrawlMode(tnow);
          setOpenCorridorCruiseMode(false, tnow);
          cancelActiveGoal();
          publishEmptyPath();
          publishGateMarkers(*odom);
          return;
        }
        clearReleaseSoftStopModes();
        was_stick_idle_ = false;
        cancelActiveGoal();
        publishEmptyPath();
        setOpenCorridorCruiseMode(false, tnow);
        publishGateMarkers(*odom);
        return;
      }
      clearReleaseSoftStopModes();
      was_stick_idle_ = false;
    } else {
      if (!buildVelocityPath(odom, tnow, path)) {
        cancelActiveGoal();
        publishEmptyPath();
        setOpenCorridorCruiseMode(false, tnow);
        publishGateMarkers(*odom);
        return;
      }
    }

    int gear = gear_default_index_;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      if (have_gear_ && latest_gear_ >= 1) {
        gear = latest_gear_;
      }
    }
    applyPathObstacleLimit(path, *odom, gear, tnow);

    bool open_cruise_base_ok = false;
    if (path_open_cruise_enable_ && use_spatial_range_ &&
      gear >= path_open_cruise_min_gear_ &&
      drive_sector_ == DriveSector::Forward &&
      isStickInOpenCruiseForwardSector(gear, tnow) &&
      !narrow_gate_pass_active_ && !narrow_gate_side_pass_crawl_active_)
    {
      open_cruise_base_ok = true;
    }

    bool open_cruise_conditions = false;
    const double cruise_speed_kph = vehiclePlanarSpeedMps(*odom) * 3.6;
    if (open_cruise_base_ok) {
      if (open_corridor_cruise_active_) {
        const auto inner = checkOpenCruiseForwardRegionClear(
          gear, tnow, openCruiseInnerCorridorHalfWidth(gear));
        open_cruise_conditions = inner.valid && inner.clear;
      } else if (cruise_speed_kph > path_open_cruise_enter_min_speed_kph_) {
        const auto full = checkOpenCruiseForwardRegionClear(
          gear, tnow, openCruiseCorridorHalfWidth(gear), path_open_cruise_outer_far_m_);
        open_cruise_conditions = full.valid && full.clear;
      }
    }

    if (open_corridor_cruise_active_ && open_cruise_base_ok) {
      if (isOpenCruiseOuterRegionOccupied(gear, tnow)) {
        ++path_open_cruise_pre_exit_streak_;
      } else {
        path_open_cruise_pre_exit_streak_ = 0;
      }

      bool pre_exit =
        path_open_cruise_pre_exit_streak_ >= path_open_cruise_pre_exit_confirm_frames_;

      if (pre_exit) {
        const double speed_kph = vehiclePlanarSpeedMps(*odom) * 3.6;
        const bool outer_occupied = isOpenCruiseOuterRegionOccupied(gear, tnow);
        const auto inner = checkOpenCruiseForwardRegionClear(
          gear, tnow, openCruiseInnerCorridorHalfWidth(gear));
        const bool inner_clear = inner.valid && inner.clear;
        if (!inner_clear) {
          path_open_cruise_pre_exit_streak_ = 0;
          pre_exit = false;
          path_open_cruise_exit_streak_ = path_open_cruise_exit_confirm_frames_;
          RCLCPP_INFO(
            get_logger(),
            "open_corridor_cruise: pre_exit inner occupied (speed %.2f kph) -> formal exit (MPPI resume)",
            speed_kph);
        } else if (speed_kph <= path_open_cruise_pre_exit_exit_speed_kph_) {
          if (!outer_occupied) {
            path_open_cruise_pre_exit_streak_ = 0;
            pre_exit = false;
            RCLCPP_INFO(
              get_logger(),
              "open_corridor_cruise: pre_exit speed %.2f kph <= %.2f, inner+outer clear -> resume cruise",
              speed_kph, path_open_cruise_pre_exit_exit_speed_kph_);
          } else {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "open_corridor_cruise: pre_exit speed %.2f kph <= %.2f, inner clear outer occupied -> hold pre_exit",
              speed_kph, path_open_cruise_pre_exit_exit_speed_kph_);
          }
        }
      }

      if (!pre_exit) {
        const double speed_kph = vehiclePlanarSpeedMps(*odom) * 3.6;
        if (isStickInOpenCruiseForwardSector(gear, tnow) &&
          speed_kph <= path_open_cruise_pre_exit_exit_speed_kph_)
        {
          ++path_open_cruise_stall_streak_;
        } else {
          path_open_cruise_stall_streak_ = 0;
        }
        if (path_open_cruise_stall_streak_ >= path_open_cruise_stall_exit_frames_) {
          path_open_cruise_exit_streak_ = path_open_cruise_exit_confirm_frames_;
          path_open_cruise_stall_streak_ = 0;
          RCLCPP_INFO(
            get_logger(),
            "open_corridor_cruise: forward stall %.2f kph <= %.2f for %d frames -> formal exit",
            speed_kph, path_open_cruise_pre_exit_exit_speed_kph_,
            path_open_cruise_stall_exit_frames_);
        }
      } else {
        path_open_cruise_stall_streak_ = 0;
      }

      setOpenCorridorCruisePreExitMode(pre_exit);
    } else {
      resetOpenCruisePreExitState();
      setOpenCorridorCruisePreExitMode(false);
    }

    if (!open_corridor_cruise_active_) {
      if (path_open_cruise_frames_since_exit_ <
        path_open_cruise_reenter_cooldown_frames_ + 1)
      {
        ++path_open_cruise_frames_since_exit_;
      }
    }

    if (open_cruise_conditions) {
      path_open_cruise_exit_streak_ = 0;
      if (!open_corridor_cruise_active_ &&
        path_open_cruise_frames_since_exit_ > path_open_cruise_reenter_cooldown_frames_)
      {
        ++path_open_cruise_enter_streak_;
      }
    } else {
      path_open_cruise_enter_streak_ = 0;
      if (open_corridor_cruise_active_) {
        ++path_open_cruise_exit_streak_;
      }
    }

    bool open_cruise = false;
    if (open_corridor_cruise_active_) {
      open_cruise =
        path_open_cruise_exit_streak_ < path_open_cruise_exit_confirm_frames_;
    } else {
      open_cruise = open_cruise_conditions &&
        path_open_cruise_enter_streak_ >= path_open_cruise_confirm_frames_ &&
        path_open_cruise_frames_since_exit_ > path_open_cruise_reenter_cooldown_frames_ &&
        cruise_speed_kph > path_open_cruise_enter_min_speed_kph_;
    }
    setOpenCorridorCruiseMode(open_cruise, tnow);

    path_pub_->publish(path);
    publishGateMarkers(*odom);
    publishGateMemorySnapshot();
    if (gate_path_enable_) {
      updateGateMemory(*odom, tnow);
    }
    if (rocker_sector_vx_min_enable_) {
      publishRockerIntentZoneState(tnow);
    }
    if (use_spatial_range_ && gate_path_enable_) {
      publishNarrowGatePassModeState();
      publishSidePassCrawlModeState();
      publishOpenCorridorCruiseModeState();
      publishOpenCorridorCruisePreExitModeState();
      publishReleaseSoftStopModeState();
      publishStickReleaseHaltState();
      if (footprint_publish_enable_) {
        int gear = gear_default_index_;
        {
          std::lock_guard<std::mutex> lock(gear_mutex_);
          if (have_gear_ && latest_gear_ >= 1) {
            gear = latest_gear_;
          }
        }
        publishFootprintForGear(gear);
      }
    }

    if (narrow_gate_side_pass_crawl_active_) {
      cancelActiveGoal();
      publishEmptyPath();
      if (side_pass_crawl_enable_) {
        publishSidePassCrawlCmdVel(tnow);
      }
      return;
    }

    if (open_corridor_cruise_active_) {
      return;
    }

    if (release_soft_stop_active_ || stick_release_halt_active_) {
      return;
    }

    if (open_cruise_mppi_resume_) {
      resend_after_reset_ = false;
      sendGoal(path, true);
      open_cruise_mppi_resume_ = false;
      return;
    }

    if (!mppiResetQuietElapsed()) {
      return;
    }

    if (resend_after_reset_) {
      resend_after_reset_ = false;
      sendGoal(path, true);
    } else {
      sendGoalIfChanged(path);
    }
  }

  bool isOdomFresh(const Odometry::SharedPtr & odom, const rclcpp::Time & tnow) const
  {
    const rclcpp::Time odom_stamp = odom ? rclcpp::Time(odom->header.stamp)
                                         : rclcpp::Time(0, 0, RCL_ROS_TIME);
    const double odom_age =
      (odom_stamp.nanoseconds() == 0) ? 0.0 : (tnow - odom_stamp).seconds();
    return odom && odom_age <= odom_timeout_;
  }

  bool buildVelocityPath(const Odometry::SharedPtr & odom, const rclcpp::Time & tnow, Path & path)
  {
    Twist::SharedPtr ctrl;
    rclcpp::Time ctrl_recv_stamp;
    {
      std::lock_guard<std::mutex> lock(ctrl_mutex_);
      ctrl = latest_ctrl_;
      ctrl_recv_stamp = latest_ctrl_stamp_;
    }

    if (!ctrl || (tnow - ctrl_recv_stamp).seconds() > control_timeout_) {
      return false;
    }

    double v_user = ctrl->linear.x;
    double w_user = ctrl->angular.z;

    const bool zero_intent =
      (std::abs(v_user) <= lin_deadband_) && (std::abs(w_user) <= ang_deadband_);
    if (zero_intent) {
      return false;
    }

    if (std::abs(w_user) > 1e-3 && std::abs(v_user) < turn_assist_vx_) {
      v_user = std::copysign(turn_assist_vx_, (v_user != 0.0) ? v_user : 1.0);
    }

    path = buildPathFromVelocity(*odom, v_user, w_user);
    return true;
  }

  bool buildSpatialPath(
    const Odometry::SharedPtr & odom, const rclcpp::Time & tnow, Path & path,
    bool & stick_idle)
  {
    stick_idle = false;

    Joystick::SharedPtr joy;
    rclcpp::Time joy_stamp;
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      joy = latest_joystick_;
      joy_stamp = latest_joystick_stamp_;
    }

    if (!joy || (tnow - joy_stamp).seconds() > joystick_timeout_) {
      return false;
    }

    const double rx_raw = static_cast<double>(joy->rocker_x);
    const double ry_raw = static_cast<double>(joy->rocker_y);
    const double rx = applyRockerDeadbandAxis(rx_raw);
    const double ry = applyRockerDeadbandAxis(ry_raw);
    if (std::abs(rx) < 1e-6 && std::abs(ry) < 1e-6) {
      stick_idle = true;
      return false;
    }

    const DriveSector prev_sector = drive_sector_;
    const DriveSector sector = updateDriveSector(rx, ry);
    if (backward_reverse_path_enable_ && sector != prev_sector) {
      scheduleMppiReset(
        sector == DriveSector::Backward ? "backward sector" : "forward sector");
    }

    double max_forward = 0.0;
    double max_back = 0.0;
    double max_lateral = 0.0;
    gearRanges(max_forward, max_back, max_lateral);

    const double norm_x = rx / 100.0;
    const double norm_y = ry / 100.0;

    int gear = gear_default_index_;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      if (have_gear_ && latest_gear_ >= 1) {
        gear = latest_gear_;
      }
    }

    const double stick_theta = std::atan2(norm_y, norm_x);

    // Pure lateral: no forward/back on stick -> spin in place (same xy, yaw only).
    if (std::abs(rx) < 1e-6) {
      const double wz = clampNorm(norm_y, -1.0, 1.0) * gearLateralWzMax(gear);
      path = buildPathInPlaceTurn(*odom, wz);
      return true;
    }

    const double stick_r = std::min(1.0, std::hypot(norm_x, norm_y));
    if (stick_r < 1e-6) {
      return false;
    }

    double dx_body = 0.0;
    double dy_body = 0.0;
    if (sector == DriveSector::Forward ||
      (sector == DriveSector::Backward && !backward_reverse_path_enable_))
    {
      const double R =
        (sector == DriveSector::Forward) ? max_forward : max_back;
      dx_body = stick_r * std::cos(stick_theta) * R;
      dy_body = stick_r * std::sin(stick_theta) * R;
      const double goal_dist = std::hypot(dx_body, dy_body);
      if (goal_dist < min_goal_distance_m_) {
        const double scale = min_goal_distance_m_ / goal_dist;
        dx_body *= scale;
        dy_body *= scale;
      }
      path = buildPathToBodyOffset(*odom, dx_body, dy_body, stick_theta);
      if (gate_path_enable_ && sector == DriveSector::Forward) {
        const auto side_pass_local_pts =
          gate_side_pass_enable_ ? fetchLocalCloudPoints(tnow) :
          std::vector<std::pair<float, float>>{};
        if (gate_side_pass_enable_ &&
          side_pass_source_ != SidePassSource::Proximity)
        {
          std::lock_guard<std::mutex> lock(gate_mutex_);
          if (const auto mem = findSidePassMemorizedGateUnlocked(
              *odom, norm_x, norm_y, side_pass_local_pts))
          {
            last_gate_ = mem->first;
            have_last_gate_ = true;
            updateSidePassCrawlMode(
              last_gate_, norm_x, norm_y, *odom, tnow, mem->second,
              side_pass_local_pts);
            if (narrow_gate_side_pass_crawl_active_) {
              refreshSidePassMemoryGateUnlocked(
                *odom, norm_x, norm_y, side_pass_local_pts);
            }
          } else if (narrow_gate_side_pass_crawl_active_ &&
            side_pass_source_ == SidePassSource::MemoryGate &&
            side_pass_memory_gate_idx_ >= 0 &&
            side_pass_memory_gate_idx_ <
            static_cast<int>(memorized_gates_.size()))
          {
            const auto & entry =
              memorized_gates_[static_cast<size_t>(side_pass_memory_gate_idx_)];
            if (const auto live_gate = narrowGateFromMemoryEntryUnlocked(
                entry, *odom, norm_x, norm_y, side_pass_local_pts))
            {
              last_gate_ = *live_gate;
              have_last_gate_ = true;
              updateSidePassCrawlMode(
                *live_gate, norm_x, norm_y, *odom, tnow, side_pass_memory_gate_idx_,
                side_pass_local_pts);
            } else {
              clearSidePassCrawlMode(tnow);
            }
          } else if (narrow_gate_side_pass_crawl_active_ &&
            side_pass_source_ == SidePassSource::MemoryGate)
          {
            clearSidePassCrawlMode(tnow);
          }
        } else if (narrow_gate_side_pass_crawl_active_ &&
          side_pass_source_ == SidePassSource::MemoryGate)
        {
          clearSidePassCrawlMode(tnow);
        }

        if (!narrow_gate_side_pass_crawl_active_ && narrow_gate_pass_active_) {
          std::lock_guard<std::mutex> lock(gate_mutex_);
          const bool refresh_ok = have_last_gate_ &&
            refreshPassModeGateUnlocked(tnow, *odom, norm_x, norm_y);
          if (refresh_ok ||
            (gate_pass_mode_latch_until_body_clear_ && have_last_gate_ &&
            !gateBodyCleared(last_gate_)))
          {
            updateNarrowGatePassMode(last_gate_, norm_x, norm_y, *odom, tnow);
            if (narrow_gate_pass_active_) {
              NarrowGateInfo path_gate = last_gate_;
              path_gate.confirmed = true;
              path = buildGateCenterlinePath(*odom, path_gate, dx_body);
              RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "gate_path confirmed [pass_mode]: dist=%.2fm front_to_gate=%.2fm width=%.2fm center_y=%.2fm",
                last_gate_.distance_m, gateFrontCenterToGateM(last_gate_),
                last_gate_.width_m, last_gate_.center_y_m);
            }
          } else {
            clearNarrowGatePassMode(tnow);
          }
        } else if (!narrow_gate_side_pass_crawl_active_ &&
          isVehicleGateDetectSpeed(*odom) &&
          !isStickLateralIntent(norm_x, norm_y))
        {
          const auto gate = detectNarrowGate(tnow, *odom, norm_x, norm_y);
          if (gate) {
            if (isVehicleCreepOrStopped(*odom)) {
              updateNarrowGatePassMode(*gate, norm_x, norm_y, *odom, tnow);
            }
            NarrowGateInfo path_gate = *gate;
            if (narrow_gate_pass_active_) {
              path_gate.confirmed = true;
            }
            path = buildGateCenterlinePath(*odom, path_gate, dx_body);
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "gate_path %s%s: dist=%.2fm front_to_gate=%.2fm width=%.2fm center_y=%.2fm",
              path_gate.confirmed ? "confirmed" : "tracking",
              narrow_gate_pass_active_ ? " [pass_mode]" : "",
              gate->distance_m, gateFrontCenterToGateM(*gate),
              gate->width_m, gate->center_y_m);
          } else if (!isVehicleGateDetectSpeed(*odom)) {
            resetGateTracking();
            clearNarrowGatePassMode(tnow);
          } else {
            clearNarrowGatePassMode(tnow);
          }
        } else if (!narrow_gate_side_pass_crawl_active_) {
          if (!isVehicleGateDetectSpeed(*odom)) {
            resetGateTracking();
            clearNarrowGatePassMode(tnow);
          }
        }
      } else if (gate_path_enable_) {
        clearNarrowGatePassMode(tnow);
      }
    } else {
      const double norm_x_clamped = clampNorm(norm_x, -1.0, 1.0);
      const double norm_y_clamped = clampNorm(norm_y, -1.0, 1.0);
      dx_body = std::min(0.0, norm_x_clamped) * max_back;
      dy_body = norm_y_clamped * max_lateral;
      if (backward_reverse_lateral_invert_) {
        dy_body = -dy_body;
      }
      const double goal_dist = std::hypot(dx_body, dy_body);
      if (goal_dist < 1e-4) {
        return false;
      }
      if (goal_dist < min_goal_distance_m_) {
        const double scale = min_goal_distance_m_ / goal_dist;
        dx_body *= scale;
        dy_body *= scale;
      }
      path = buildPathToBodyOffsetConstantHeading(*odom, dx_body, dy_body);
    }
    return true;
  }

  double applyRockerDeadbandAxis(double axis) const
  {
    return (std::abs(axis) <= rocker_deadband_) ? 0.0 : axis;
  }

  double pathOpenCruiseFrontHalfDeg(int gear) const
  {
    return pickGearTable(path_open_cruise_gear_front_half_deg_, gear, gear_default_index_);
  }

  bool isStickInOpenCruiseForwardSector(int gear, const rclcpp::Time & tnow)
  {
    Joystick::SharedPtr joy;
    rclcpp::Time joy_stamp;
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      joy = latest_joystick_;
      joy_stamp = latest_joystick_stamp_;
    }
    if (!joy || (tnow - joy_stamp).seconds() > joystick_timeout_) {
      return false;
    }
    const double rx = applyRockerDeadbandAxis(static_cast<double>(joy->rocker_x));
    const double ry = applyRockerDeadbandAxis(static_cast<double>(joy->rocker_y));
    if (std::abs(rx) < 1e-6 && std::abs(ry) < 1e-6) {
      return false;
    }
    const double theta = std::atan2(ry, rx);
    const double half_rad = pathOpenCruiseFrontHalfDeg(gear) * M_PI / 180.0;
    return std::abs(theta) <= half_rad;
  }

  bool isRockerOutOfDeadband(const rclcpp::Time & tnow)
  {
    Joystick::SharedPtr joy;
    rclcpp::Time joy_stamp;
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      joy = latest_joystick_;
      joy_stamp = latest_joystick_stamp_;
    }
    if (!joy || (tnow - joy_stamp).seconds() > joystick_timeout_) {
      return false;
    }
    const double rx = applyRockerDeadbandAxis(static_cast<double>(joy->rocker_x));
    const double ry = applyRockerDeadbandAxis(static_cast<double>(joy->rocker_y));
    return std::abs(rx) >= 1e-6 || std::abs(ry) >= 1e-6;
  }

  // Stick angle atan2(ry, rx): 0 = forward (+x). Forward 300 deg <=> |angle| <= 150 deg;
  // backward 60 deg <=> |angle| > 150 deg. Hysteresis +/- sector_hysteresis_deg at boundary.
  DriveSector updateDriveSector(double rx, double ry)
  {
    const double theta = std::atan2(ry, rx);
    const double abs_theta = std::abs(theta);
    const double half_back_rad = (backward_sector_deg_ * 0.5) * M_PI / 180.0;
    const double boundary_rad = M_PI - half_back_rad;
    const double hyst_rad = sector_hysteresis_deg_ * M_PI / 180.0;
    const double enter_back_rad = boundary_rad + hyst_rad;
    const double enter_forward_rad = boundary_rad - hyst_rad;

    if (drive_sector_ == DriveSector::Forward) {
      if (abs_theta > enter_back_rad) {
        drive_sector_ = DriveSector::Backward;
      }
    } else if (abs_theta < enter_forward_rad) {
      drive_sector_ = DriveSector::Forward;
    }
    return drive_sector_;
  }

  double gearFootprintPadding(int gear) const
  {
    return pickGearTable(gear_footprint_padding_m_, gear, gear_default_index_);
  }

  double openCruiseInnerCorridorHalfWidth(int gear) const
  {
    return footprintHalfWidthM() + gearFootprintPadding(gear);
  }

  double openCruiseExtraPadding(int gear) const
  {
    return pickGearTable(path_open_cruise_gear_extra_padding_m_, gear, gear_default_index_);
  }

  double openCruiseCorridorHalfWidth(int gear) const
  {
    return openCruiseInnerCorridorHalfWidth(gear) + openCruiseExtraPadding(gear);
  }

  double footprintHalfWidthM() const
  {
    if (!footprint_base_pts_.empty()) {
      double hw = 0.0;
      for (const auto & p : footprint_base_pts_) {
        hw = std::max(hw, std::abs(p.y));
      }
      return hw;
    }
    return footprint_body_y_max_;
  }

  struct OpenCruiseForwardRegionResult
  {
    bool valid{false};
    bool clear{false};
  };

  OpenCruiseForwardRegionResult checkOpenCruiseForwardRegionClear(
    int gear, const rclcpp::Time & tnow, double corridor_half,
    double forward_far_extra_m = 0.0) const
  {
    OpenCruiseForwardRegionResult result;
    const double forward_range_m =
      pickGearTable(gear_forward_range_m_, gear, gear_default_index_) + forward_far_extra_m;
    if (forward_range_m <= 1e-6) {
      return result;
    }
    if (corridor_half <= 0.0) {
      return result;
    }

    std::vector<std::pair<float, float>> cloud_body;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!have_cloud_ || (tnow - latest_cloud_stamp_).seconds() > obstacle_timeout_) {
        return result;
      }
      cloud_body = cloud_pts_;
    }

    result.valid = true;
    if (cloud_body.empty()) {
      result.clear = true;
      return result;
    }

    const double half_rad = pathOpenCruiseFrontHalfDeg(gear) * M_PI / 180.0;
    for (const auto & bp : cloud_body) {
      const double px = static_cast<double>(bp.first);
      const double py = static_cast<double>(bp.second);
      if (px < 0.0 || px > forward_range_m) {
        continue;
      }
      if (std::abs(py) > corridor_half) {
        continue;
      }
      if (std::abs(std::atan2(py, px)) > half_rad + 1e-6) {
        continue;
      }
      result.clear = false;
      return result;
    }
    result.clear = true;
    return result;
  }

  bool isOpenCruiseOuterRegionOccupied(int gear, const rclcpp::Time & tnow) const
  {
    const double forward_range_m =
      pickGearTable(gear_forward_range_m_, gear, gear_default_index_);
    if (forward_range_m <= 1e-6) {
      return false;
    }
    const double inner_half = openCruiseInnerCorridorHalfWidth(gear);
    const double extra = openCruiseExtraPadding(gear);
    const double outer_half = inner_half + extra;
    if (extra <= 1e-6 && path_open_cruise_outer_far_m_ <= 1e-6) {
      return false;
    }

    std::vector<std::pair<float, float>> cloud_body;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!have_cloud_ || (tnow - latest_cloud_stamp_).seconds() > obstacle_timeout_) {
        return false;
      }
      cloud_body = cloud_pts_;
    }
    if (cloud_body.empty()) {
      return false;
    }

    const double half_rad = pathOpenCruiseFrontHalfDeg(gear) * M_PI / 180.0;
    const double far_end = forward_range_m + path_open_cruise_outer_far_m_;
    for (const auto & bp : cloud_body) {
      const double px = static_cast<double>(bp.first);
      const double py = static_cast<double>(bp.second);
      if (px < 0.0 || px > far_end) {
        continue;
      }
      if (std::abs(std::atan2(py, px)) > half_rad + 1e-6) {
        continue;
      }
      const double abs_py = std::abs(py);
      if (extra > 1e-6 && px <= forward_range_m + 1e-6 &&
        abs_py > inner_half + 1e-6 && abs_py <= outer_half + 1e-6)
      {
        return true;
      }
      if (path_open_cruise_outer_far_m_ > 1e-6 && px > forward_range_m + 1e-6 &&
        abs_py <= outer_half + 1e-6)
      {
        return true;
      }
    }
    return false;
  }

  void truncatePathAtArcLength(Path & path, double max_arc_m) const
  {
    if (path.poses.size() < 2 || max_arc_m <= 0.0) {
      return;
    }

    Path out;
    out.header = path.header;
    out.poses.push_back(path.poses.front());

    double s = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & prev = path.poses[i - 1].pose.position;
      const auto & cur = path.poses[i].pose.position;
      const double seg = std::hypot(cur.x - prev.x, cur.y - prev.y);
      if (seg < 1e-9) {
        if (s <= max_arc_m + 1e-6) {
          out.poses.push_back(path.poses[i]);
        }
        continue;
      }
      if (s + seg <= max_arc_m + 1e-6) {
        s += seg;
        out.poses.push_back(path.poses[i]);
        continue;
      }

      const double remain = max_arc_m - s;
      const double t = std::clamp(remain / seg, 0.0, 1.0);
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = prev.x + t * (cur.x - prev.x);
      p.pose.position.y = prev.y + t * (cur.y - prev.y);
      p.pose.position.z = prev.z + t * (cur.z - prev.z);
      const double yaw0 = yawFromQuaternion(path.poses[i - 1].pose.orientation);
      const double yaw1 = yawFromQuaternion(path.poses[i].pose.orientation);
      p.pose.orientation = quaternionFromYaw(lerpAngle(yaw0, yaw1, t));
      out.poses.push_back(p);
      break;
    }

    if (out.poses.size() >= 2) {
      path = std::move(out);
    }
  }

  struct PathCorridorObstacleResult
  {
    bool valid{false};
    double full_len{0.0};
    double nearest_block_s{std::numeric_limits<double>::infinity()};
  };

  PathCorridorObstacleResult computePathCorridorObstacle(
    const Path & path, const Odometry & odom, int gear, const rclcpp::Time & tnow) const
  {
    PathCorridorObstacleResult result;
    if (path.poses.size() < 2) {
      return result;
    }
    if (path_obstacle_limit_skip_pass_mode_ &&
      (narrow_gate_pass_active_ || narrow_gate_side_pass_crawl_active_))
    {
      return result;
    }

    if (path_obstacle_limit_skip_in_place_) {
      const auto & a = path.poses.front().pose.position;
      const auto & b = path.poses.back().pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) < min_goal_distance_m_ * 0.5) {
        return result;
      }
    }

    std::vector<std::pair<float, float>> cloud_body;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!have_cloud_ || (tnow - latest_cloud_stamp_).seconds() > obstacle_timeout_) {
        return result;
      }
      cloud_body = cloud_pts_;
    }

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    const double cos_y = std::cos(yaw0);
    const double sin_y = std::sin(yaw0);

    std::vector<std::pair<double, double>> cloud_odom;
    cloud_odom.reserve(cloud_body.size());
    for (const auto & bp : cloud_body) {
      const double bx = static_cast<double>(bp.first);
      const double by = static_cast<double>(bp.second);
      cloud_odom.emplace_back(
        x0 + cos_y * bx - sin_y * by,
        y0 + sin_y * bx + cos_y * by);
    }

    const double corridor_half = footprintHalfWidthM() + gearFootprintPadding(gear);
    if (corridor_half <= 0.0) {
      return result;
    }

    std::vector<double> arc_s(path.poses.size(), 0.0);
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & prev = path.poses[i - 1].pose.position;
      const auto & cur = path.poses[i].pose.position;
      arc_s[i] = arc_s[i - 1] + std::hypot(cur.x - prev.x, cur.y - prev.y);
    }
    result.full_len = arc_s.back();
    if (result.full_len < path_obstacle_min_keep_m_) {
      return result;
    }

    result.valid = true;
    if (cloud_odom.empty()) {
      return result;
    }

    double min_block_s = std::numeric_limits<double>::infinity();
    for (const auto & obs : cloud_odom) {
      const double ox = obs.first;
      const double oy = obs.second;
      for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
        const auto & A = path.poses[i].pose.position;
        const auto & B = path.poses[i + 1].pose.position;
        const double dx = B.x - A.x;
        const double dy = B.y - A.y;
        const double seg_len = std::hypot(dx, dy);
        if (seg_len < 1e-6) {
          continue;
        }
        const double tx = dx / seg_len;
        const double ty = dy / seg_len;
        const double ax = ox - A.x;
        const double ay = oy - A.y;
        const double proj = ax * tx + ay * ty;
        const double lat = std::abs(-ty * ax + tx * ay);
        if (lat > corridor_half) {
          continue;
        }
        const double proj_clamped = std::clamp(proj, 0.0, seg_len);
        const double s_block = std::max(
          arc_s[i] + proj_clamped - path_obstacle_stop_margin_m_,
          path_obstacle_min_keep_m_);
        min_block_s = std::min(min_block_s, s_block);
      }
    }
    result.nearest_block_s = min_block_s;
    return result;
  }

  void applyPathObstacleLimit(
    Path & path, const Odometry & odom, int gear, const rclcpp::Time & tnow)
  {
    if (!path_obstacle_limit_enable_) {
      return;
    }
    const auto corridor = computePathCorridorObstacle(path, odom, gear, tnow);
    if (!corridor.valid ||
      corridor.nearest_block_s >= corridor.full_len - 1e-3)
    {
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "path_obstacle_limit: truncate %.2fm -> %.2fm gear=%d",
      corridor.full_len, corridor.nearest_block_s, gear);

    truncatePathAtArcLength(path, corridor.nearest_block_s);
  }

  double gearLateralWzMax(int gear) const
  {
    return pickGearTable(gear_lateral_wz_max_, gear, gear_default_index_);
  }

  static double pickGearTable(
    const std::vector<double> & table, int g, int fallback_g)
  {
    if (table.empty()) {
      return 0.0;
    }
    if (g >= 1 && g <= static_cast<int>(table.size())) {
      return table[static_cast<size_t>(g - 1)];
    }
    const int fb = std::max(1, std::min(fallback_g, static_cast<int>(table.size())));
    return table[static_cast<size_t>(fb - 1)];
  }

  void gearRanges(double & max_forward, double & max_back, double & max_lateral)
  {
    int gear = gear_default_index_;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      if (have_gear_ && latest_gear_ >= 1) {
        gear = latest_gear_;
      }
    }
    max_forward = pickGearTable(gear_forward_range_m_, gear, gear_default_index_);
    max_back = pickGearTable(gear_back_range_m_, gear, gear_default_index_);
    max_lateral = pickGearTable(gear_lateral_range_m_, gear, gear_default_index_);
  }

  std::vector<std::pair<float, float>> filterLocalCloudPoints(
    const std::vector<std::pair<float, float>> & pts) const
  {
    std::vector<std::pair<float, float>> local;
    local.reserve(pts.size());
    const double r2 = gate_local_radius_m_ * gate_local_radius_m_;
    for (const auto & p : pts) {
      const double d2 = static_cast<double>(p.first) * p.first +
        static_cast<double>(p.second) * p.second;
      if (d2 <= r2) {
        local.push_back(p);
      }
    }
    return local;
  }

  std::vector<std::pair<float, float>> fetchLocalCloudPoints(
    const rclcpp::Time & tnow) const
  {
    bool cloud_ok = false;
    std::vector<std::pair<float, float>> pts;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_ok = have_cloud_ &&
        (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
      if (cloud_ok) {
        pts = cloud_pts_;
      }
    }
    if (!cloud_ok) {
      return {};
    }
    return filterLocalCloudPoints(pts);
  }

  bool isStickLateralIntent(double norm_x, double norm_y) const
  {
    return std::abs(norm_y) / std::max(std::abs(norm_x), 1e-3) >=
           gate_intent_lateral_ratio_min_;
  }

  double vehiclePlanarSpeedMps(const Odometry & odom) const
  {
    const double vx = odom.twist.twist.linear.x;
    const double vy = odom.twist.twist.linear.y;
    return std::hypot(vx, vy);
  }

  bool isVehicleCreepOrStopped(const Odometry & odom) const
  {
    return vehiclePlanarSpeedMps(odom) <= gate_creep_speed_max_mps_;
  }

  bool isVehicleGateDetectSpeed(const Odometry & odom) const
  {
    return vehiclePlanarSpeedMps(odom) <= gate_detect_speed_max_mps_;
  }

  bool isGateCreepSpeed(const Odometry & odom) const
  {
    return isVehicleCreepOrStopped(odom);
  }

  bool useCreepGateIntent(const Odometry & odom) const
  {
    return narrow_gate_pass_active_ || isGateCreepSpeed(odom);
  }

  double effectiveStickForwardMin(double norm_x, double norm_y, const Odometry & odom) const
  {
    (void)norm_x;
    (void)norm_y;
    return useCreepGateIntent(odom) ?
      gate_creep_stick_forward_min_ : gate_stick_forward_min_;
  }

  bool intentForwardTowardGate(
    const NarrowGateInfo & gate, double norm_x, double norm_y,
    const Odometry & odom) const
  {
    if (isStickLateralIntent(norm_x, norm_y)) {
      return false;
    }
    const double fwd_min = useCreepGateIntent(odom) ?
      gate_creep_stick_forward_min_ : gate_stick_forward_min_;
    if (norm_x < fwd_min) {
      return false;
    }
    const double stick_r = std::hypot(norm_x, norm_y);
    const double stick_min = useCreepGateIntent(odom) ? gate_creep_stick_min_ : 1e-3;
    if (stick_r < stick_min) {
      return false;
    }
    if (gate.distance_m < gate_scan_dist_min_m_ ||
      gate.distance_m > gate_local_radius_m_ + 1e-6)
    {
      return false;
    }

    const double slack = useCreepGateIntent(odom) ?
      gate_creep_intent_cross_slack_m_ : gate_intent_cross_slack_m_;
    const double half_w = 0.5 * gate.width_m + slack;
    if (std::abs(gate.center_y_m) > half_w) {
      return false;
    }

    const double max_angle = useCreepGateIntent(odom) ?
      gate_creep_intent_angle_max_ : gate_intent_angle_delta_max_;
    const double stick_angle = std::atan2(norm_y, norm_x);
    const double gate_angle = std::atan2(
      gate.center_y_m, std::max(gate.distance_m, 0.05));
    if (std::abs(normalizeAngle(stick_angle - gate_angle)) > max_angle) {
      return false;
    }

    const double ux = norm_x / stick_r;
    const double uy = norm_y / stick_r;
    return gate.distance_m * ux + gate.center_y_m * uy >= 0.05;
  }

  bool intentMatchesGate(
    const NarrowGateInfo & gate, double norm_x, double norm_y,
    const Odometry & odom) const
  {
    return intentForwardTowardGate(gate, norm_x, norm_y, odom);
  }

  bool stickPointsTowardSidePass(
    const NarrowGateInfo & gate, double norm_x, double norm_y) const
  {
    const double stick_r = std::hypot(norm_x, norm_y);
    if (stick_r < 1e-3) {
      return false;
    }

    const double cx = gate.distance_m;
    const double cy = gate.center_y_m;

    if (norm_x >= gate_side_pass_stick_forward_min_ && cx > 0.05) {
      const double stick_angle = std::atan2(norm_y, norm_x);
      const double gate_angle = std::atan2(cy, std::max(cx, 0.05));
      if (std::abs(normalizeAngle(stick_angle - gate_angle)) <=
        gate_side_pass_stick_angle_max_)
      {
        return true;
      }
    }

    if (norm_y >= gate_side_pass_stick_lateral_min_ &&
      cy > gate_center_y_deadband_m_)
    {
      return true;
    }
    if (norm_y <= -gate_side_pass_stick_lateral_min_ &&
      cy < -gate_center_y_deadband_m_)
    {
      return true;
    }
    return false;
  }

  bool openingSliceValid(
    float left_y, float right_y, int left_count, int right_count, double width,
    double width_min, double width_max) const
  {
    if (left_count < gate_min_side_points_ || right_count < gate_min_side_points_) {
      return false;
    }
    if (!gate_pillar_mode_enable_) {
      if (left_y < gate_min_side_boundary_m_ || (-right_y) < gate_min_side_boundary_m_) {
        return false;
      }
    } else {
      const double left_in = static_cast<double>(left_y);
      const double right_in = -static_cast<double>(right_y);
      const double min_in = std::min(left_in, right_in);
      const double max_in = std::max(left_in, right_in);
      if (max_in < 1e-3 ||
        min_in / max_in < gate_pillar_symmetry_ratio_min_)
      {
        return false;
      }
    }
    return width >= width_min && width <= width_max;
  }

  bool openingSliceValid(
    float left_y, float right_y, int left_count, int right_count, double width) const
  {
    return openingSliceValid(
      left_y, right_y, left_count, right_count, width,
      gate_width_min_m_, gate_width_max_m_);
  }

  struct OpeningSliceSample
  {
    double distance_m{0.0};
    double center_y_m{0.0};
    double width_m{0.0};
    float left_y{0.0f};
    float right_y{0.0f};
  };

  bool openingApproachCorridorClear(
    const std::vector<std::pair<float, float>> & local_pts,
    double dist, double center_y) const
  {
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);
    const double x_start = gate_scan_dist_min_m_;
    const double x_end = dist - dist_step * 0.5;
    if (x_end <= x_start) {
      return true;
    }

    const double approach_len = x_end - x_start;
    const double depth_req = std::min(gate_approach_corridor_min_depth_m_, approach_len);
    const double check_x_start = x_end - depth_req;

    const double half_w = gate_approach_corridor_half_width_m_;
    int block_count = 0;
    for (const auto & p : local_pts) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x < check_x_start || x > x_end) {
        continue;
      }
      if (std::abs(y - center_y) <= half_w) {
        ++block_count;
        if (block_count >= gate_approach_corridor_min_points_) {
          return false;
        }
      }
    }
    return true;
  }

  bool openingApproachHasOneSidedWall(
    const std::vector<std::pair<float, float>> & local_pts,
    double dist) const
  {
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);
    const double x_start = gate_scan_dist_min_m_;
    const double x_end = dist - dist_step * 0.5;
    if (x_end <= x_start) {
      return false;
    }

    auto detect_wall = [this, &local_pts, x_start, x_end](bool left_side) {
      int count = 0;
      double x_min = std::numeric_limits<double>::infinity();
      double x_max = -std::numeric_limits<double>::infinity();
      for (const auto & p : local_pts) {
        const double x = static_cast<double>(p.first);
        const double y = static_cast<double>(p.second);
        if (x < x_start || x > x_end) {
          continue;
        }
        const bool hit = left_side ?
          (y >= gate_side_wall_y_threshold_m_) : (y <= -gate_side_wall_y_threshold_m_);
        if (!hit) {
          continue;
        }
        ++count;
        x_min = std::min(x_min, x);
        x_max = std::max(x_max, x);
      }
      if (count < gate_side_wall_min_points_) {
        return false;
      }
      return (x_max - x_min) >= gate_side_wall_min_span_m_;
    };

    return detect_wall(true) != detect_wall(false);
  }

  std::optional<OpeningSliceSample> sampleOpeningSliceAtDistance(
    const std::vector<std::pair<float, float>> & local_pts,
    double dist, double width_min, double width_max) const
  {
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);
    const double bin_half = dist_step * 0.5;
    const double center_clear = std::max(0.05, gate_center_clear_half_width_m_);

    float left_y = std::numeric_limits<float>::infinity();
    float right_y = -std::numeric_limits<float>::infinity();
    float left_x_at = 0.0f;
    float right_x_at = 0.0f;
    int left_count = 0;
    int right_count = 0;

    for (const auto & p : local_pts) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x < dist - bin_half || x > dist + bin_half) {
        continue;
      }
      if (y > center_clear) {
        ++left_count;
        if (y < left_y) {
          left_y = static_cast<float>(y);
          left_x_at = p.first;
        }
      } else if (y < -center_clear) {
        ++right_count;
        if (y > right_y) {
          right_y = static_cast<float>(y);
          right_x_at = p.first;
        }
      }
    }

    if (!std::isfinite(left_y) || !std::isfinite(right_y)) {
      return std::nullopt;
    }

    if (std::abs(static_cast<double>(left_x_at) - static_cast<double>(right_x_at)) >
      gate_opening_depth_m_)
    {
      return std::nullopt;
    }

    const double width = static_cast<double>(left_y - right_y);
    if (!openingSliceValid(left_y, right_y, left_count, right_count, width, width_min, width_max)) {
      return std::nullopt;
    }

    for (const auto & p : local_pts) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x < dist - bin_half - 0.04 || x > dist + bin_half + 0.04) {
        continue;
      }
      if (std::abs(y) <= center_clear) {
        return std::nullopt;
      }
    }

    const double center_y = 0.5 * (static_cast<double>(left_y) + static_cast<double>(right_y));
    if (!openingApproachCorridorClear(local_pts, dist, center_y) ||
      openingApproachHasOneSidedWall(local_pts, dist))
    {
      return std::nullopt;
    }

    OpeningSliceSample sample;
    sample.distance_m = dist;
    sample.center_y_m = center_y;
    sample.width_m = width;
    sample.left_y = left_y;
    sample.right_y = right_y;
    return sample;
  }

  std::optional<OpeningSliceSample> sampleOpeningBilateralNearDistance(
    const std::vector<std::pair<float, float>> & local_pts,
    double target_dist, double width_min, double width_max) const
  {
    const double tol = gate_lock_distance_delta_m_;
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);
    const double dist_min = std::max(gate_scan_dist_min_m_, target_dist - tol);
    const double dist_max = std::min(gate_local_radius_m_, target_dist + tol);

    std::optional<OpeningSliceSample> best;
    double best_dist_err = std::numeric_limits<double>::infinity();

    for (double dist = dist_min; dist <= dist_max + 1e-6; dist += dist_step) {
      const auto slice = sampleOpeningSliceAtDistance(local_pts, dist, width_min, width_max);
      if (!slice) {
        continue;
      }
      const double err = std::abs(slice->distance_m - target_dist);
      if (err < best_dist_err) {
        best_dist_err = err;
        best = slice;
      }
    }
    return best;
  }

  // Head-on gate detect: bilateral cloud slices along body +x; intent via intentForwardTowardGate.
  std::optional<NarrowGateInfo> scanOpeningBilateral(
    const std::vector<std::pair<float, float>> & local_pts,
    double norm_x, double norm_y, const Odometry & odom) const
  {
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);

    std::optional<NarrowGateInfo> best;
    double best_score = std::numeric_limits<double>::infinity();

    for (
      double dist = gate_scan_dist_min_m_;
      dist <= gate_local_radius_m_ + 1e-6;
      dist += dist_step)
    {
      const auto slice = sampleOpeningSliceAtDistance(
        local_pts, dist, gate_width_min_m_, gate_width_max_m_);
      if (!slice) {
        continue;
      }

      NarrowGateInfo gate;
      gate.distance_m = slice->distance_m;
      gate.center_y_m = slice->center_y_m;
      gate.width_m = slice->width_m;
      if (!intentMatchesGate(gate, norm_x, norm_y, odom)) {
        continue;
      }

      const double score = openingIntentScore(
        slice->distance_m, slice->center_y_m, norm_x, norm_y);
      if (score < best_score) {
        best_score = score;
        best = gate;
      }
    }
    return best;
  }

  bool intentMatchesTrackDirection(
    double norm_x, double norm_y, const Odometry & odom,
    const NarrowGateInfo * live_gate = nullptr) const
  {
    if (!gate_tracking_active_ && !live_gate) {
      return true;
    }
    const double max_delta = useCreepGateIntent(odom) ?
      gate_creep_intent_angle_delta_max_ : gate_intent_angle_delta_max_;
    const double cur = std::atan2(norm_y, norm_x);
    const double track_x = live_gate ? live_gate->distance_m : gate_track_distance_m_;
    const double track_y = live_gate ? live_gate->center_y_m : gate_track_center_y_m_;
    const double tracked = std::atan2(
      track_y, std::max(track_x, 1e-3));
    const double delta = std::abs(normalizeAngle(cur - tracked));
    return delta <= max_delta;
  }

  double openingIntentScore(
    double center_x, double center_y, double norm_x, double norm_y) const
  {
    const double stick_r = std::hypot(norm_x, norm_y);
    if (stick_r < 1e-3) {
      return std::numeric_limits<double>::infinity();
    }
    const double ux = norm_x / stick_r;
    const double uy = norm_y / stick_r;
    const double t = center_x * ux + center_y * uy;
    const double px = t * ux;
    const double py = t * uy;
    return std::hypot(center_x - px, center_y - py) + 0.35 * t;
  }

  double computeGateAxisYaw(const NarrowGateInfo & gate, double norm_y) const
  {
    (void)gate;
    (void)norm_y;
    return 0.0;
  }

  void bodyCenterToOdom(
    const Odometry & odom, double center_x_body, double center_y_body,
    double & center_x_odom, double & center_y_odom) const
  {
    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    const double cos_y = std::cos(yaw0);
    const double sin_y = std::sin(yaw0);
    center_x_odom = x0 + cos_y * center_x_body - sin_y * center_y_body;
    center_y_odom = y0 + sin_y * center_x_body + cos_y * center_y_body;
  }

  void odomCenterToBody(
    const Odometry & odom, double center_x_odom, double center_y_odom,
    double & center_x_body, double & center_y_body) const
  {
    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    const double cos_y = std::cos(yaw0);
    const double sin_y = std::sin(yaw0);
    const double dx = center_x_odom - x0;
    const double dy = center_y_odom - y0;
    center_x_body = cos_y * dx + sin_y * dy;
    center_y_body = -sin_y * dx + cos_y * dy;
  }

  void initGateMemoryBodyBoundsFromFootprint()
  {
    if (footprint_base_pts_.empty()) {
      if (gate_memory_body_x_min_m_ < 0.0) {
        gate_memory_body_x_min_m_ = -0.2;
      }
      if (gate_memory_body_x_max_m_ < 0.0) {
        gate_memory_body_x_max_m_ = 0.935;
      }
      if (gate_memory_half_width_m_ <= 0.0) {
        gate_memory_half_width_m_ = 0.34;
      }
      footprint_body_x_min_ = gate_memory_body_x_min_m_;
      footprint_body_x_max_ = gate_memory_body_x_max_m_;
      footprint_body_y_min_ = -gate_memory_half_width_m_;
      footprint_body_y_max_ = gate_memory_half_width_m_;
      return;
    }

    double fp_x_min = footprint_base_pts_.front().x;
    double fp_x_max = fp_x_min;
    double fp_hw = 0.0;
    for (const auto & p : footprint_base_pts_) {
      fp_x_min = std::min(fp_x_min, p.x);
      fp_x_max = std::max(fp_x_max, p.x);
      fp_hw = std::max(fp_hw, std::abs(p.y));
    }
    if (gate_memory_body_x_min_m_ < 0.0) {
      gate_memory_body_x_min_m_ = fp_x_min;
    }
    if (gate_memory_body_x_max_m_ < 0.0) {
      gate_memory_body_x_max_m_ = fp_x_max;
    }
    if (gate_memory_half_width_m_ <= 0.0) {
      gate_memory_half_width_m_ = fp_hw;
    }
    footprint_body_x_min_ = fp_x_min;
    footprint_body_x_max_ = fp_x_max;
    footprint_body_y_min_ = -fp_hw;
    footprint_body_y_max_ = fp_hw;
  }

  double bodyBorderDistanceToPoint(double bx, double by) const
  {
    double dx = 0.0;
    if (bx < footprint_body_x_min_) {
      dx = footprint_body_x_min_ - bx;
    } else if (bx > footprint_body_x_max_) {
      dx = bx - footprint_body_x_max_;
    }
    double dy = 0.0;
    if (by < footprint_body_y_min_) {
      dy = footprint_body_y_min_ - by;
    } else if (by > footprint_body_y_max_) {
      dy = by - footprint_body_y_max_;
    }
    return std::hypot(dx, dy);
  }

  struct GateMemorySideHit
  {
    bool ok{false};
    double left_x_body{0.0};
    double left_y_body{0.0};
    double right_x_body{0.0};
    double right_y_body{0.0};
  };

  GateMemorySideHit sampleGateMemoryAtBodySides(
    const std::vector<std::pair<float, float>> & pts) const
  {
    GateMemorySideHit hit;
    double left_x_sum = 0.0;
    double left_y_sum = 0.0;
    int left_n = 0;
    double right_x_sum = 0.0;
    double right_y_sum = 0.0;
    int right_n = 0;

    const double hw = gate_memory_half_width_m_;
    const double band = gate_memory_lateral_band_m_;

    for (const auto & p : pts) {
      const double x = static_cast<double>(p.first);
      const double y = static_cast<double>(p.second);
      if (x < gate_memory_body_x_min_m_ || x > gate_memory_body_x_max_m_) {
        continue;
      }
      if (y >= hw && y <= hw + band) {
        left_x_sum += x;
        left_y_sum += y;
        ++left_n;
      } else if (y <= -hw && y >= -hw - band) {
        right_x_sum += x;
        right_y_sum += y;
        ++right_n;
      }
    }

    if (left_n < gate_memory_min_side_points_ || right_n < gate_memory_min_side_points_) {
      return hit;
    }

    hit.ok = true;
    hit.left_x_body = left_x_sum / static_cast<double>(left_n);
    hit.left_y_body = left_y_sum / static_cast<double>(left_n);
    hit.right_x_body = right_x_sum / static_cast<double>(right_n);
    hit.right_y_body = right_y_sum / static_cast<double>(right_n);
    return hit;
  }

  int findGateMemoryNearOdomUnlocked(double ox, double oy) const
  {
    int best_idx = -1;
    double best_dist = gate_memory_merge_radius_m_;
    for (int i = 0; i < static_cast<int>(memorized_gates_.size()); ++i) {
      const auto & stored = memorized_gates_[static_cast<size_t>(i)];
      const double dist = std::hypot(stored.center_x_odom - ox, stored.center_y_odom - oy);
      if (dist <= gate_memory_merge_radius_m_ && dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
    return best_idx;
  }

  std::optional<NarrowGateInfo> narrowGateFromMemoryEntryUnlocked(
    const GateMemoryEntry & entry, const Odometry & odom,
    double norm_x, double norm_y,
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    double lx = 0.0;
    double ly = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    odomCenterToBody(odom, entry.left_x_odom, entry.left_y_odom, lx, ly);
    odomCenterToBody(odom, entry.right_x_odom, entry.right_y_odom, rx, ry);
    odomCenterToBody(odom, entry.center_x_odom, entry.center_y_odom, cx, cy);

    double width = 0.0;
    double gate_line_yaw = 0.0;
    bool have_geometry = false;

    if (!local_pts.empty()) {
      if (const auto slice = sampleOpeningBilateralNearDistance(
          local_pts, cx, gate_side_pass_width_min_m_, gate_width_max_m_))
      {
        cx = slice->distance_m;
        cy = slice->center_y_m;
        width = slice->width_m;
        gate_line_yaw = std::atan2(
          static_cast<double>(slice->right_y) - static_cast<double>(slice->left_y),
          0.0);
        have_geometry = true;
      }
    }

    if (!have_geometry) {
      width = std::hypot(rx - lx, ry - ly);
      if (width < gate_side_pass_width_min_m_ - 1e-3 ||
        width > gate_width_max_m_ + 1e-3)
      {
        return std::nullopt;
      }
      gate_line_yaw = std::atan2(ry - ly, rx - lx);
    }

    const double bisector_a = normalizeAngle(gate_line_yaw + M_PI_2);
    const double bisector_b = normalizeAngle(gate_line_yaw - M_PI_2);
    const double stick_yaw = std::atan2(norm_y, norm_x);
    const double axis_yaw = std::abs(normalizeAngle(bisector_a - stick_yaw)) <=
      std::abs(normalizeAngle(bisector_b - stick_yaw)) ?
      bisector_a : bisector_b;

    NarrowGateInfo gate;
    gate.distance_m = cx;
    gate.center_y_m = cy;
    gate.width_m = width;
    gate.axis_yaw_body = axis_yaw;
    gate.confirmed = true;
    return gate;
  }

  std::optional<std::pair<NarrowGateInfo, int>> findSidePassMemorizedGateUnlocked(
    const Odometry & odom, double norm_x, double norm_y,
    const std::vector<std::pair<float, float>> & local_pts) const
  {
    if (!gate_side_pass_enable_) {
      return std::nullopt;
    }
    if (vehiclePlanarSpeedMps(odom) >= gate_side_pass_max_speed_mps_) {
      return std::nullopt;
    }

    std::optional<std::pair<NarrowGateInfo, int>> best;
    double best_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(memorized_gates_.size()); ++i) {
      const auto & entry = memorized_gates_[static_cast<size_t>(i)];
      if (entry.pass_count <= gate_side_pass_min_pass_count_) {
        continue;
      }

      const auto gate = narrowGateFromMemoryEntryUnlocked(
        entry, odom, norm_x, norm_y, local_pts);
      if (!gate) {
        continue;
      }

      const double border_dist = bodyBorderDistanceToPoint(
        gate->distance_m, gate->center_y_m);
      if (border_dist > gate_side_pass_trigger_dist_m_) {
        continue;
      }
      if (!stickPointsTowardSidePass(*gate, norm_x, norm_y)) {
        continue;
      }

      if (border_dist < best_dist) {
        best_dist = border_dist;
        best = std::make_pair(*gate, i);
      }
    }
    return best;
  }

  bool refreshSidePassMemoryGateUnlocked(
    const Odometry & odom, double norm_x, double norm_y,
    const std::vector<std::pair<float, float>> & local_pts)
  {
    if (side_pass_memory_gate_idx_ < 0 ||
      side_pass_memory_gate_idx_ >= static_cast<int>(memorized_gates_.size()))
    {
      return false;
    }

    const auto & entry = memorized_gates_[static_cast<size_t>(side_pass_memory_gate_idx_)];
    const auto gate = narrowGateFromMemoryEntryUnlocked(
      entry, odom, norm_x, norm_y, local_pts);
    if (!gate) {
      return false;
    }

    pass_mode_gate_odom_x_ = entry.center_x_odom;
    pass_mode_gate_odom_y_ = entry.center_y_odom;
    pass_mode_gate_width_m_ = gate->width_m;
    pass_mode_gate_odom_valid_ = true;

    last_gate_ = *gate;
    have_last_gate_ = true;
    gate_confirmed_ = true;
    confirmed_gate_ = *gate;
    return true;
  }

  void bodyPointToOdom(
    const Odometry & odom, double bx, double by, double & ox, double & oy) const
  {
    bodyCenterToOdom(odom, bx, by, ox, oy);
  }

  double gateMemoryUpdateAlpha(int pass_count) const
  {
    if (pass_count <= 0) {
      return 1.0;
    }
    if (gate_memory_update_alpha_ > 0.0) {
      return clampNorm(gate_memory_update_alpha_, 0.01, 1.0);
    }
    return 1.0 / static_cast<double>(pass_count);
  }

  void applyGateMemoryWeightedUpdate(
    GateMemoryEntry & stored,
    double lx_o, double ly_o, double rx_o, double ry_o)
  {
    const double alpha = gateMemoryUpdateAlpha(stored.pass_count);
    const auto blend = [alpha](double old_v, double new_v) {
      return (1.0 - alpha) * old_v + alpha * new_v;
    };
    stored.left_x_odom = blend(stored.left_x_odom, lx_o);
    stored.left_y_odom = blend(stored.left_y_odom, ly_o);
    stored.right_x_odom = blend(stored.right_x_odom, rx_o);
    stored.right_y_odom = blend(stored.right_y_odom, ry_o);
    stored.center_x_odom = 0.5 * (stored.left_x_odom + stored.right_x_odom);
    stored.center_y_odom = 0.5 * (stored.left_y_odom + stored.right_y_odom);
  }

  void commitGateMemoryUnlocked(
    const Odometry & odom, const GateMemorySideHit & hit, const rclcpp::Time & tnow)
  {
    double lx_o = 0.0;
    double ly_o = 0.0;
    double rx_o = 0.0;
    double ry_o = 0.0;
    bodyPointToOdom(odom, hit.left_x_body, hit.left_y_body, lx_o, ly_o);
    bodyPointToOdom(odom, hit.right_x_body, hit.right_y_body, rx_o, ry_o);
    const double cx_body = 0.5 * (hit.left_x_body + hit.right_x_body);
    const double cy_body = 0.5 * (hit.left_y_body + hit.right_y_body);
    double cx_o = 0.0;
    double cy_o = 0.0;
    bodyPointToOdom(odom, cx_body, cy_body, cx_o, cy_o);

    const int idx = findGateMemoryNearOdomUnlocked(cx_o, cy_o);
    if (idx >= 0) {
      auto & stored = memorized_gates_[static_cast<size_t>(idx)];
      stored.pass_count = std::max(1, stored.pass_count + 1);
      applyGateMemoryWeightedUpdate(stored, lx_o, ly_o, rx_o, ry_o);
      publishGateMemorySnapshotUnlocked(tnow);
      RCLCPP_INFO(
        get_logger(),
        "gate_memory pass #%d (alpha=%.2f): center odom=(%.2f, %.2f) left=(%.2f, %.2f) right=(%.2f, %.2f)",
        stored.pass_count, gateMemoryUpdateAlpha(stored.pass_count),
        stored.center_x_odom, stored.center_y_odom,
        stored.left_x_odom, stored.left_y_odom, stored.right_x_odom, stored.right_y_odom);
      return;
    }

    if (static_cast<int>(memorized_gates_.size()) >= gate_memory_max_count_) {
      const auto & evicted = memorized_gates_.front();
      RCLCPP_INFO(
        get_logger(),
        "gate_memory evict oldest: center odom=(%.2f, %.2f) pass=%d",
        evicted.center_x_odom, evicted.center_y_odom, evicted.pass_count);
      memorized_gates_.erase(memorized_gates_.begin());
    }

    GateMemoryEntry stored;
    stored.left_x_odom = lx_o;
    stored.left_y_odom = ly_o;
    stored.right_x_odom = rx_o;
    stored.right_y_odom = ry_o;
    stored.center_x_odom = 0.5 * (lx_o + rx_o);
    stored.center_y_odom = 0.5 * (ly_o + ry_o);
    stored.pass_count = 1;
    memorized_gates_.push_back(stored);
    publishGateMemorySnapshotUnlocked(tnow);
    RCLCPP_INFO(
      get_logger(),
      "gate_memory saved #%zu: center odom=(%.2f, %.2f) left=(%.2f, %.2f) right=(%.2f, %.2f)",
      memorized_gates_.size(), cx_o, cy_o, lx_o, ly_o, rx_o, ry_o);
  }

  void updateGateMemory(const Odometry & odom, const rclcpp::Time & tnow)
  {
    if (!gate_memory_publish_enable_ || !gate_path_enable_) {
      return;
    }

    const bool cloud_ok = have_cloud_ &&
      (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
    if (!cloud_ok) {
      return;
    }

    std::vector<std::pair<float, float>> pts;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      pts = cloud_pts_;
    }

    const GateMemorySideHit hit = sampleGateMemoryAtBodySides(pts);
    const bool speed_ok = vehiclePlanarSpeedMps(odom) > gate_memory_min_speed_mps_;
    const bool detect_ok = hit.ok && speed_ok;

    std::lock_guard<std::mutex> lock(gate_mutex_);

    if (gate_memory_cooldown_frames_ > 0) {
      --gate_memory_cooldown_frames_;
    }

    if (!detect_ok) {
      if (!hit.ok) {
        gate_memory_rearm_left_frames_ = gate_memory_rearm_frames_;
      }
      gate_memory_streak_frames_ = 0;
      return;
    }

    if (gate_memory_rearm_left_frames_ > 0) {
      --gate_memory_rearm_left_frames_;
      gate_memory_streak_frames_ = 0;
      return;
    }

    if (gate_memory_cooldown_frames_ > 0) {
      return;
    }

    ++gate_memory_streak_frames_;
    if (gate_memory_streak_frames_ < gate_memory_confirm_frames_) {
      return;
    }

    commitGateMemoryUnlocked(odom, hit, tnow);
    gate_memory_streak_frames_ = 0;
    gate_memory_cooldown_frames_ = gate_memory_confirm_frames_;
    gate_memory_rearm_left_frames_ = gate_memory_rearm_frames_;
  }

  void resetGateTrackingUnlocked()
  {
    gate_tracking_active_ = false;
    gate_confirmed_ = false;
    gate_match_count_ = 0;
    have_last_gate_ = false;
  }

  void resetGateTracking()
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    resetGateTrackingUnlocked();
    clearNarrowGatePassMode(now());
  }

  bool gateCandidateMatchesTrack(const NarrowGateInfo & gate) const
  {
    if (!gate_tracking_active_) {
      return false;
    }
    return std::abs(gate.distance_m - gate_track_distance_m_) <= gate_lock_distance_delta_m_ &&
           std::abs(gate.center_y_m - gate_track_center_y_m_) <= gate_lock_center_y_delta_m_;
  }

  void beginGateTrack(
    const NarrowGateInfo & gate, const rclcpp::Time & tnow, bool from_cloud)
  {
    gate_tracking_active_ = true;
    gate_confirmed_ = false;
    gate_match_count_ = 1;
    gate_first_match_stamp_ = tnow;
    gate_track_from_cloud_ = from_cloud;
    gate_track_distance_m_ = gate.distance_m;
    gate_track_center_y_m_ = gate.center_y_m;
    gate_track_width_m_ = gate.width_m;
    confirmed_gate_ = gate;
    confirmed_gate_.confirmed = false;
  }

  void updateGateTrackMeasurement(const NarrowGateInfo & gate)
  {
    const double alpha = clampNorm(gate_ema_alpha_, 0.05, 0.95);
    gate_track_distance_m_ = alpha * gate.distance_m + (1.0 - alpha) * gate_track_distance_m_;
    gate_track_center_y_m_ = alpha * gate.center_y_m + (1.0 - alpha) * gate_track_center_y_m_;
    gate_track_width_m_ = alpha * gate.width_m + (1.0 - alpha) * gate_track_width_m_;
    ++gate_match_count_;
  }

  void maybeConfirmGateTrack(
    const rclcpp::Time & tnow, double norm_y, const Odometry & odom)
  {
    if (gate_confirmed_) {
      return;
    }
    const double confirm_sec = isGateCreepSpeed(odom) ?
      gate_confirm_sec_ : gate_detect_confirm_sec_;
    if ((tnow - gate_first_match_stamp_).seconds() < confirm_sec) {
      return;
    }
    gate_confirmed_ = true;
    confirmed_gate_.distance_m = gate_track_distance_m_;
    confirmed_gate_.center_y_m = gate_track_center_y_m_;
    confirmed_gate_.width_m = gate_track_width_m_;
    confirmed_gate_.axis_yaw_body = computeGateAxisYaw(confirmed_gate_, norm_y);
    confirmed_gate_.confirmed = true;
  }

  NarrowGateInfo makeActiveGate(double norm_y) const
  {
    NarrowGateInfo gate;
    if (gate_confirmed_) {
      gate = confirmed_gate_;
    } else {
      gate.distance_m = gate_track_distance_m_;
      gate.center_y_m = gate_track_center_y_m_;
      gate.width_m = gate_track_width_m_;
      gate.confirmed = false;
      gate.axis_yaw_body = computeGateAxisYaw(gate, norm_y);
    }
    return gate;
  }

  std::optional<NarrowGateInfo> scanNarrowGateFromCloud(
    const rclcpp::Time & tnow, const Odometry & odom,
    double norm_x, double norm_y) const
  {
    if (isStickLateralIntent(norm_x, norm_y)) {
      return std::nullopt;
    }
    if (norm_x < effectiveStickForwardMin(norm_x, norm_y, odom)) {
      return std::nullopt;
    }
    const double stick_r = std::hypot(norm_x, norm_y);
    if (stick_r < 1e-3) {
      return std::nullopt;
    }

    bool cloud_ok = false;
    std::vector<std::pair<float, float>> pts;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_ok = have_cloud_ &&
        (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
      if (cloud_ok) {
        pts = cloud_pts_;
      }
    }
    if (!cloud_ok) {
      return std::nullopt;
    }

    const auto local_pts = filterLocalCloudPoints(pts);
    if (local_pts.empty()) {
      return std::nullopt;
    }

    return scanOpeningBilateral(local_pts, norm_x, norm_y, odom);
  }

  std::optional<NarrowGateInfo> scanPassModeGateFromCloud(
    const rclcpp::Time & tnow, const Odometry & odom,
    double norm_x, double norm_y) const
  {
    if (isStickLateralIntent(norm_x, norm_y)) {
      return std::nullopt;
    }
    if (norm_x < gate_creep_stick_forward_min_) {
      return std::nullopt;
    }
    const double stick_r = std::hypot(norm_x, norm_y);
    if (stick_r < gate_creep_stick_min_) {
      return std::nullopt;
    }

    bool cloud_ok = false;
    std::vector<std::pair<float, float>> pts;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_ok = have_cloud_ &&
        (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
      if (cloud_ok) {
        pts = cloud_pts_;
      }
    }
    if (!cloud_ok) {
      return std::nullopt;
    }

    const auto local_pts = filterLocalCloudPoints(pts);
    if (local_pts.empty()) {
      return std::nullopt;
    }

    return scanOpeningBilateral(local_pts, norm_x, norm_y, odom);
  }

  bool refreshPassModeGateUnlocked(
    const rclcpp::Time & tnow, const Odometry & odom,
    double norm_x, double norm_y)
  {
    if (!have_last_gate_) {
      return false;
    }

    if (!pass_mode_gate_odom_valid_) {
      bodyCenterToOdom(
        odom, last_gate_.distance_m, last_gate_.center_y_m,
        pass_mode_gate_odom_x_, pass_mode_gate_odom_y_);
      pass_mode_gate_width_m_ = last_gate_.width_m;
      pass_mode_gate_odom_valid_ = true;
    }

    double measured_x = 0.0;
    double measured_y = 0.0;
    odomCenterToBody(
      odom, pass_mode_gate_odom_x_, pass_mode_gate_odom_y_,
      measured_x, measured_y);

    double measured_w = pass_mode_gate_width_m_;

    if (const auto cloud_gate = scanPassModeGateFromCloud(tnow, odom, norm_x, norm_y)) {
      const double pass_dist_tol = gate_lock_distance_delta_m_ * 2.5;
      const double pass_y_tol = gate_lock_center_y_delta_m_ * 2.0;
      if (std::abs(cloud_gate->distance_m - measured_x) <= pass_dist_tol &&
        std::abs(cloud_gate->center_y_m - measured_y) <= pass_y_tol)
      {
        const double alpha = clampNorm(gate_ema_alpha_, 0.05, 0.95);
        measured_x = alpha * cloud_gate->distance_m + (1.0 - alpha) * measured_x;
        measured_y = alpha * cloud_gate->center_y_m + (1.0 - alpha) * measured_y;
        measured_w = alpha * cloud_gate->width_m + (1.0 - alpha) * measured_w;
        bodyCenterToOdom(
          odom, measured_x, measured_y,
          pass_mode_gate_odom_x_, pass_mode_gate_odom_y_);
        pass_mode_gate_width_m_ = measured_w;
      }
    }

    gate_track_distance_m_ = measured_x;
    gate_track_center_y_m_ = measured_y;
    gate_track_width_m_ = measured_w;
    gate_confirmed_ = true;
    confirmed_gate_.distance_m = measured_x;
    confirmed_gate_.center_y_m = measured_y;
    confirmed_gate_.width_m = measured_w;
    confirmed_gate_.axis_yaw_body = computeGateAxisYaw(confirmed_gate_, norm_y);
    confirmed_gate_.confirmed = true;

    last_gate_ = confirmed_gate_;
    last_gate_stamp_ = tnow;
    return true;
  }

  std::optional<NarrowGateInfo> detectNarrowGate(
    const rclcpp::Time & tnow, const Odometry & odom, double norm_x, double norm_y)
  {
    if (isStickLateralIntent(norm_x, norm_y)) {
      resetGateTracking();
      return std::nullopt;
    }
    if (norm_x < effectiveStickForwardMin(norm_x, norm_y, odom)) {
      resetGateTracking();
      return std::nullopt;
    }

    auto cloud_candidate = scanNarrowGateFromCloud(tnow, odom, norm_x, norm_y);
    std::lock_guard<std::mutex> lock(gate_mutex_);

    const auto candidate = cloud_candidate;
    const bool intent_ok = candidate &&
      intentMatchesGate(*candidate, norm_x, norm_y, odom) &&
      intentMatchesTrackDirection(norm_x, norm_y, odom, &(*candidate));

    if (intent_ok) {
      if (!gate_tracking_active_ || !gateCandidateMatchesTrack(*candidate)) {
        beginGateTrack(*candidate, tnow, cloud_candidate.has_value());
      } else {
        updateGateTrackMeasurement(*candidate);
      }
      maybeConfirmGateTrack(tnow, norm_y, odom);
      last_gate_ = makeActiveGate(norm_y);
      last_gate_stamp_ = tnow;
      have_last_gate_ = true;
      return last_gate_;
    }

    if (gate_tracking_active_ && have_last_gate_ &&
      intentMatchesGate(last_gate_, norm_x, norm_y, odom) &&
      intentMatchesTrackDirection(norm_x, norm_y, odom, &last_gate_))
    {
      const double hold_limit = gate_confirmed_ ? gate_confirmed_hold_sec_ :
        (isGateCreepSpeed(odom) ? gate_hold_sec_ : gate_detect_hold_sec_);
      if ((tnow - last_gate_stamp_).seconds() <= hold_limit) {
        ++gate_match_count_;
        maybeConfirmGateTrack(tnow, norm_y, odom);
        last_gate_ = makeActiveGate(norm_y);
        last_gate_stamp_ = tnow;
        return last_gate_;
      }
    }

    resetGateTrackingUnlocked();
    return std::nullopt;
  }

  Path buildGateCenterlinePath(
    const Odometry & odom, const NarrowGateInfo & gate, double goal_x_body) const
  {
    return buildForwardGateCenterlinePath(odom, gate, goal_x_body);
  }

  Path buildForwardGateCenterlinePath(
    const Odometry & odom, const NarrowGateInfo & gate, double goal_x_body) const
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);
    const double cos_y = std::cos(yaw0);
    const double sin_y = std::sin(yaw0);

    const double x_end = gate.distance_m;
    const double y_end = gate.center_y_m;
    const double x_tail = std::max(
      x_end + gate_path_tail_m_, std::max(x_end, goal_x_body));

    const double tangent_len = std::max(0.25, 0.65 * x_end);
    const double mx0 = tangent_len;
    const double my0 = 0.0;
    const double mx1 = tangent_len;
    const double my1 = 0.0;

    std::vector<BodyPoint> body_pts;
    body_pts.reserve(static_cast<size_t>(path_interpolation_steps_) + 8);
    body_pts.push_back({0.0, 0.0});

    const int hermite_steps = std::max(8, path_interpolation_steps_ / 2);
    for (int i = 1; i <= hermite_steps; ++i) {
      const double u = static_cast<double>(i) / static_cast<double>(hermite_steps);
      body_pts.push_back(hermitePoint(u, 0.0, 0.0, x_end, y_end, mx0, my0, mx1, my1));
    }

    const double tail_step = 0.08;
    for (double x = x_end + tail_step; x <= x_tail + 1e-6; x += tail_step) {
      body_pts.push_back({x, y_end});
    }

    for (size_t i = 0; i < body_pts.size(); ++i) {
      const auto & bp = body_pts[i];
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x0 + cos_y * bp.x - sin_y * bp.y;
      p.pose.position.y = y0 + sin_y * bp.x + cos_y * bp.y;
      p.pose.position.z = 0.0;
      // Keep heading near current yaw so the vehicle enters the gate perpendicular
      // (along +x body), only xy bends toward the opening center.
      p.pose.orientation = quaternionFromYaw(yaw0);
      path.poses.push_back(p);
    }

    return path;
  }

  Path buildPathToBodyOffsetConstantHeading(
    const Odometry & odom, double dx_body, double dy_body) const
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);

    const double cos_y = std::cos(yaw0);
    const double sin_y = std::sin(yaw0);
    const double goal_x = x0 + cos_y * dx_body - sin_y * dy_body;
    const double goal_y = y0 + sin_y * dx_body + cos_y * dy_body;

    const int steps = std::max(2, path_interpolation_steps_);

    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x0 + t * (goal_x - x0);
      p.pose.position.y = y0 + t * (goal_y - y0);
      p.pose.position.z = 0.0;
      p.pose.orientation = quaternionFromYaw(yaw0);
      path.poses.push_back(p);
    }

    return path;
  }

  Path buildPathToBodyOffset(
    const Odometry & odom, double dx_body, double dy_body, double yaw_offset_body) const
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);

    const double cos_y = std::cos(yaw0);
    const double sin_y = std::sin(yaw0);
    const double goal_x = x0 + cos_y * dx_body - sin_y * dy_body;
    const double goal_y = y0 + sin_y * dx_body + cos_y * dy_body;
    const double yaw_goal = normalizeAngle(yaw0 + yaw_offset_body);

    const int steps = std::max(2, path_interpolation_steps_);

    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x0 + t * (goal_x - x0);
      p.pose.position.y = y0 + t * (goal_y - y0);
      p.pose.position.z = 0.0;
      p.pose.orientation = quaternionFromYaw(lerpAngle(yaw0, yaw_goal, t));
      path.poses.push_back(p);
    }

    return path;
  }

  Path buildPathInPlaceTurn(const Odometry & odom, double wz) const
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double yaw0 = yawFromQuaternion(odom.pose.pose.orientation);

    const int steps = std::max(2, path_interpolation_steps_);
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x0;
      p.pose.position.y = y0;
      p.pose.position.z = 0.0;
      p.pose.orientation = quaternionFromYaw(normalizeAngle(yaw0 + wz * t * horizon_));
      path.poses.push_back(p);
    }

    return path;
  }

  Path buildPathFromVelocity(const Odometry & odom, double v_user, double w_user) const
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;

    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double yaw = yawFromQuaternion(odom.pose.pose.orientation);

    const int N = std::max(1, static_cast<int>(std::round(horizon_ / dt_)));

    PoseStamped pose;
    pose.header = path.header;
    pose.pose = odom.pose.pose;
    path.poses.push_back(pose);

    for (int i = 1; i <= N; ++i) {
      x += v_user * std::cos(yaw) * dt_;
      y += v_user * std::sin(yaw) * dt_;
      yaw += w_user * dt_;

      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.position.z = 0.0;
      p.pose.orientation = quaternionFromYaw(yaw);
      path.poses.push_back(p);
    }

    return path;
  }

  bool goalChangedEnough(const Path & path) const
  {
    if (path.poses.empty()) {
      return false;
    }
    if (!have_last_sent_goal_) {
      return true;
    }

    const auto & goal = path.poses.back().pose;
    const double dx = goal.position.x - last_sent_goal_x_;
    const double dy = goal.position.y - last_sent_goal_y_;
    const double dyaw = std::abs(
      yawFromQuaternion(goal.orientation) - last_sent_goal_yaw_);
    const double dyaw_wrapped = std::min(dyaw, 2.0 * M_PI - dyaw);

    return std::hypot(dx, dy) >= goal_resend_xy_delta_m_ ||
           dyaw_wrapped >= goal_resend_yaw_delta_;
  }

  void sendGoalIfChanged(const Path & path)
  {
    if (!goalChangedEnough(path)) {
      return;
    }
    sendGoal(path);
  }

  void sendGoal(const Path & path, bool cancel_active = false)
  {
    if (path.poses.empty()) {
      return;
    }

    const auto & goal = path.poses.back().pose;
    last_sent_goal_x_ = goal.position.x;
    last_sent_goal_y_ = goal.position.y;
    last_sent_goal_yaw_ = yawFromQuaternion(goal.orientation);
    have_last_sent_goal_ = true;

    auto goal_msg = FollowPath::Goal();
    goal_msg.path = path;
    goal_msg.controller_id = controller_id_;
    goal_msg.goal_checker_id = goal_checker_id_;

    auto send_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = goal_handle;
      };
    send_options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(get_logger(), "FollowPath result: SUCCEEDED");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_DEBUG(get_logger(), "FollowPath result: ABORTED");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_DEBUG(get_logger(), "FollowPath result: CANCELED");
            break;
          default:
            break;
        }
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_.reset();
      };

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (cancel_active) {
        cancelAllFollowPathGoalsUnlocked();
      }
    }

    await_mppi_reset_before_send_ = false;
    action_client_->async_send_goal(goal_msg, send_options);
  }

  void cancelAllFollowPathGoalsUnlocked()
  {
    if (current_goal_handle_) {
      action_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
    } else {
      action_client_->async_cancel_all_goals();
    }
  }

  void cancelAllFollowPathGoals()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    cancelAllFollowPathGoalsUnlocked();
  }

  void scheduleMppiReset(const char * reason)
  {
    have_last_sent_goal_ = false;
    follow_path_cancel_time_ = now();
    await_mppi_reset_before_send_ = true;
    resend_after_reset_ = true;
    cancelAllFollowPathGoals();
    RCLCPP_INFO(
      get_logger(), "FollowPath cancel (%s), wait %.2fs before resend", reason,
      mppi_reset_quiet_sec_);
  }

  bool mppiResetQuietElapsed() const
  {
    if (!await_mppi_reset_before_send_) {
      return true;
    }
    return (now() - follow_path_cancel_time_).seconds() >= mppi_reset_quiet_sec_;
  }

  void cancelActiveGoal()
  {
    have_last_sent_goal_ = false;
    was_stick_idle_ = false;
    await_mppi_reset_before_send_ = false;
    resend_after_reset_ = false;
    drive_sector_ = DriveSector::Forward;
    cancelAllFollowPathGoals();
  }

  void publishEmptyPath()
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;
    path_pub_->publish(path);
  }

  // Parameters
  std::string path_generation_mode_;
  bool use_spatial_range_{true};
  std::string control_topic_;
  std::string joystick_topic_;
  std::string car_info_topic_;
  std::string odom_topic_;
  std::string intent_path_topic_;
  std::string action_name_;
  std::string global_frame_;
  std::string controller_id_;
  std::string goal_checker_id_;

  double lin_deadband_{0.02};
  double ang_deadband_{0.02};
  double turn_assist_vx_{0.08};
  double control_timeout_{0.30};
  double rocker_deadband_{5.0};
  double joystick_timeout_{0.30};
  std::vector<double> gear_forward_range_m_;
  std::vector<double> gear_back_range_m_;
  std::vector<double> gear_lateral_range_m_;
  std::vector<double> gear_lateral_wz_max_;
  double forward_sector_deg_{300.0};
  double backward_sector_deg_{60.0};
  double sector_hysteresis_deg_{10.0};
  bool backward_reverse_path_enable_{true};
  bool backward_reverse_lateral_invert_{false};
  int gear_default_index_{3};
  int path_interpolation_steps_{20};
  double min_goal_distance_m_{0.12};
  double goal_resend_xy_delta_m_{0.05};
  double goal_resend_yaw_delta_{0.08};
  double mppi_reset_quiet_sec_{0.40};
  double odom_timeout_{0.30};

  double horizon_{1.5};
  double dt_{0.10};
  double send_rate_hz_{5.0};

  bool footprint_publish_enable_{true};
  std::string footprint_topic_;
  std::string footprint_base_;
  std::vector<double> gear_footprint_padding_m_;
  bool path_obstacle_limit_enable_{true};
  double path_obstacle_stop_margin_m_{0.20};
  double path_obstacle_min_keep_m_{0.12};
  bool path_obstacle_limit_skip_pass_mode_{true};
  bool path_obstacle_limit_skip_in_place_{true};
  bool path_open_cruise_enable_{true};
  int path_open_cruise_min_gear_{3};
  std::vector<double> path_open_cruise_gear_front_half_deg_;
  std::vector<double> path_open_cruise_gear_extra_padding_m_;
  double path_open_cruise_outer_far_m_{0.5};
  int path_open_cruise_pre_exit_confirm_frames_{2};
  double path_open_cruise_pre_exit_exit_speed_kph_{2.0};
  int path_open_cruise_stall_exit_frames_{30};
  int path_open_cruise_pre_exit_streak_{0};
  int path_open_cruise_stall_streak_{0};
  bool open_corridor_cruise_pre_exit_active_{false};
  std::string path_open_cruise_pre_exit_topic_;
  int path_open_cruise_confirm_frames_{10};
  double path_open_cruise_enter_min_speed_kph_{3.0};
  int path_open_cruise_exit_confirm_frames_{3};
  int path_open_cruise_reenter_cooldown_frames_{30};
  int path_open_cruise_enter_streak_{0};
  int path_open_cruise_exit_streak_{0};
  int path_open_cruise_frames_since_exit_{1000000};
  bool open_cruise_mppi_resume_{false};
  std::string path_open_cruise_topic_;
  bool open_corridor_cruise_active_{false};
  bool release_soft_stop_enable_{true};
  double release_soft_stop_radius_{0.8};
  std::string release_soft_stop_topic_;
  std::string stick_release_halt_topic_;
  bool rocker_sector_vx_min_enable_{true};
  std::string rocker_forward_sector_topic_;
  std::string rocker_intent_zone_topic_;
  double rocker_intent_front_half_deg_{30.0};
  double rocker_intent_side_span_deg_{90.0};
  bool rocker_forward_sector_active_{true};
  RockerIntentZone rocker_intent_zone_active_{RockerIntentZone::NONE};
  bool wz_density_cap_enable_{true};
  std::string wz_density_cap_topic_;
  std::string wz_density_tier_topic_;
  double density_check_radius_{1.2};
  std::vector<double> wz_density_caps_rps_{1.30, 0.785, 0.524};
  bool gear_vx_cap_enable_{true};
  std::string gear_vx_cap_topic_;
  std::vector<double> gear_vx_max_mps_{2.0 / 3.6, 3.0 / 3.6, 5.0 / 3.6, 6.0 / 3.6};
  int obstacle_count_engage_moderate_{12};
  int obstacle_count_release_open_{6};
  int obstacle_count_engage_dense_{40};
  int obstacle_count_release_moderate_{28};
  double wz_density_min_dwell_{0.4};
  WzDensityTier current_wz_tier_{WzDensityTier::OPEN};
  rclcpp::Time last_wz_tier_change_stamp_{0, 0, RCL_ROS_TIME};
  bool release_soft_stop_active_{false};
  bool stick_release_halt_active_{false};
  std::vector<geometry_msgs::msg::Point> footprint_base_pts_;
  double last_footprint_padding_{-1.0};
  bool have_published_footprint_{false};

  bool gate_path_enable_{true};
  std::string obstacle_points_topic_;
  double obstacle_timeout_{0.50};
  double gate_width_min_m_{0.75};
  double gate_width_max_m_{1.30};
  double gate_local_radius_m_{1.40};
  std::string gate_detect_mode_{"bilateral"};
  double gate_scan_dist_min_m_{0.15};
  double gate_scan_dist_step_m_{0.05};
  double gate_opening_depth_m_{0.25};
  bool gate_pillar_mode_enable_{true};
  double gate_min_side_boundary_m_{0.38};
  double gate_pillar_symmetry_ratio_min_{0.55};
  int gate_min_side_points_{1};
  double gate_center_clear_half_width_m_{0.12};
  double gate_approach_corridor_half_width_m_{0.25};
  double gate_approach_corridor_min_depth_m_{0.5};
  int gate_approach_corridor_min_points_{2};
  double gate_side_wall_y_threshold_m_{0.32};
  int gate_side_wall_min_points_{4};
  double gate_side_wall_min_span_m_{0.45};
  double gate_stick_forward_min_{0.20};
  double gate_lateral_stick_forward_min_{0.08};
  double gate_center_y_deadband_m_{0.18};
  double gate_stick_lateral_min_{0.08};
  double gate_intent_lateral_ratio_min_{0.20};
  double gate_intent_cross_slack_m_{0.18};
  double gate_intent_angle_delta_max_{0.45};
  double gate_creep_speed_max_mps_{0.20};
  double gate_detect_speed_max_mps_{0.55};
  double gate_detect_confirm_sec_{0.80};
  double gate_detect_hold_sec_{0.60};
  double gate_creep_stick_forward_min_{0.08};
  double gate_creep_stick_min_{0.10};
  double gate_creep_intent_angle_max_{0.95};
  double gate_creep_intent_cross_slack_m_{0.35};
  double gate_creep_intent_angle_delta_max_{0.95};
  double gate_path_tail_m_{0.50};
  double gate_hold_sec_{0.25};
  double gate_confirm_sec_{0.50};
  double gate_confirmed_hold_sec_{0.60};
  double gate_ema_alpha_{0.35};
  double gate_lock_distance_delta_m_{0.18};
  double gate_lock_center_y_delta_m_{0.15};
  int gate_memory_confirm_frames_{5};
  int gate_memory_max_count_{5};
  double gate_memory_merge_radius_m_{1.00};
  double gate_memory_lateral_band_m_{0.20};
  double gate_memory_min_speed_mps_{0.30};
  double gate_memory_body_x_min_m_{-0.2};
  double gate_memory_body_x_max_m_{0.935};
  double gate_memory_half_width_m_{0.34};
  int gate_memory_min_side_points_{1};
  int gate_memory_rearm_frames_{3};
  double gate_memory_update_alpha_{0.0};
  bool gate_side_pass_enable_{true};
  bool side_pass_crawl_enable_{false};
  int gate_side_pass_min_pass_count_{2};
  double gate_side_pass_trigger_dist_m_{0.20};
  double gate_side_pass_stick_forward_min_{0.08};
  double gate_side_pass_stick_lateral_min_{0.08};
  double gate_side_pass_stick_angle_max_{0.95};
  double gate_side_pass_max_speed_mps_{0.10};
  double gate_side_pass_body_clear_m_{0.35};
  double gate_side_pass_width_min_m_{0.25};
  bool gate_side_pass_enter_forward_cloud_enable_{true};
  double gate_side_pass_enter_check_depth_m_{0.15};
  std::vector<double> gate_side_pass_enter_forward_strip_x_{0.685, 0.935, 0.685};
  std::vector<double> gate_side_pass_enter_forward_strip_y_min_{0.2, -0.2, -0.34};
  std::vector<double> gate_side_pass_enter_forward_strip_y_max_{0.34, 0.2, -0.2};
  bool gate_side_pass_exit_forward_clear_enable_{true};
  double gate_side_pass_exit_check_depth_m_{0.10};
  std::vector<double> gate_side_pass_exit_forward_strip_x_{0.685, 0.685};
  std::vector<double> gate_side_pass_exit_forward_strip_y_min_{0.2, -0.34};
  std::vector<double> gate_side_pass_exit_forward_strip_y_max_{0.34, -0.2};
  bool proximity_side_pass_enable_{true};
  double proximity_side_pass_outer_probe_depth_m_{0.20};
  double proximity_side_pass_exit_margin_m_{0.15};
  double proximity_side_pass_max_speed_mps_{0.10};
  double proximity_side_pass_max_wz_rps_{0.20};
  int proximity_side_pass_confirm_frames_{10};
  int proximity_side_pass_exit_confirm_frames_{5};
  int proximity_side_pass_enter_streak_{0};
  int proximity_side_pass_exit_streak_{0};
  SidePassSource side_pass_source_{SidePassSource::None};
  double footprint_body_x_min_{-0.2};
  double footprint_body_x_max_{0.935};
  double footprint_body_y_min_{-0.34};
  double footprint_body_y_max_{0.34};
  double gate_pass_mode_enter_m_{0.80};
  double gate_pass_front_center_x_m_{0.85};
  double gate_pass_mode_exit_m_{0.95};
  double gate_pass_mode_body_clear_x_m_{-0.55};
  double gate_pass_mode_hold_sec_{0.25};
  bool gate_pass_mode_latch_until_body_clear_{true};
  bool gate_pass_mode_intent_timeout_enable_{false};
  bool gate_pass_mode_front_to_gate_exit_enable_{false};
  std::string narrow_gate_pass_mode_topic_;
  std::string narrow_gate_side_pass_mode_topic_;
  std::string side_pass_crawl_cmd_vel_topic_;
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
  bool gate_marker_enable_{true};
  std::string gate_marker_topic_;
  bool gate_memory_publish_enable_{true};
  std::string gate_memory_topic_;
  std::string gate_memory_state_topic_;
  bool narrow_gate_pass_active_{false};
  bool narrow_gate_side_pass_crawl_active_{false};
  int side_pass_memory_gate_idx_{-1};
  rclcpp::Time narrow_gate_pass_last_intent_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time narrow_gate_side_pass_last_intent_stamp_{0, 0, RCL_ROS_TIME};
  bool pass_mode_gate_odom_valid_{false};
  double pass_mode_gate_odom_x_{0.0};
  double pass_mode_gate_odom_y_{0.0};
  double pass_mode_gate_width_m_{0.95};

  mutable std::mutex cloud_mutex_;
  std::vector<std::pair<float, float>> cloud_pts_;
  rclcpp::Time latest_cloud_stamp_{0, 0, RCL_ROS_TIME};
  bool have_cloud_{false};

  mutable std::mutex gate_mutex_;
  mutable NarrowGateInfo last_gate_;
  mutable NarrowGateInfo confirmed_gate_;
  mutable rclcpp::Time last_gate_stamp_{0, 0, RCL_ROS_TIME};
  mutable rclcpp::Time gate_first_match_stamp_{0, 0, RCL_ROS_TIME};
  mutable bool have_last_gate_{false};
  mutable bool gate_tracking_active_{false};
  mutable bool gate_confirmed_{false};
  mutable bool gate_track_from_cloud_{false};
  mutable int gate_match_count_{0};
  int gate_memory_streak_frames_{0};
  int gate_memory_cooldown_frames_{0};
  int gate_memory_rearm_left_frames_{0};
  std::vector<GateMemoryEntry> memorized_gates_;
  mutable double gate_track_distance_m_{0.0};
  mutable double gate_track_center_y_m_{0.0};
  mutable double gate_track_width_m_{0.0};

  DriveSector drive_sector_{DriveSector::Forward};
  bool was_stick_idle_{false};
  bool await_mppi_reset_before_send_{false};
  bool resend_after_reset_{false};
  rclcpp::Time follow_path_cancel_time_{0, 0, RCL_ROS_TIME};

  bool have_last_sent_goal_{false};
  double last_sent_goal_x_{0.0};
  double last_sent_goal_y_{0.0};
  double last_sent_goal_yaw_{0.0};

  rclcpp::Subscription<Twist>::SharedPtr control_sub_;
  rclcpp::Subscription<Joystick>::SharedPtr joystick_sub_;
  rclcpp::Subscription<CarInfo>::SharedPtr car_info_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr narrow_gate_pass_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr narrow_gate_side_pass_mode_pub_;
  rclcpp::Publisher<Twist>::SharedPtr side_pass_crawl_cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr open_corridor_cruise_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr open_corridor_cruise_pre_exit_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr release_soft_stop_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stick_release_halt_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rocker_forward_sector_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rocker_intent_zone_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wz_density_cap_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr wz_density_tier_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gear_vx_cap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gate_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gate_memory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_memory_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_pub_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex ctrl_mutex_;
  Twist::SharedPtr latest_ctrl_;
  rclcpp::Time latest_ctrl_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex joystick_mutex_;
  Joystick::SharedPtr latest_joystick_;
  rclcpp::Time latest_joystick_stamp_{0, 0, RCL_ROS_TIME};

  mutable std::mutex gear_mutex_;
  int latest_gear_{3};
  bool have_gear_{false};

  std::mutex odom_mutex_;
  Odometry::SharedPtr latest_odom_;

  std::mutex goal_mutex_;
  GoalHandle::SharedPtr current_goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickLocalPathNode>());
  rclcpp::shutdown();
  return 0;
}
