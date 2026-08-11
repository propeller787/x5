// Joystick / handle intent -> short local path -> Nav2 FollowPath (MPPI).
//
// spatial_range: stick polar -> body-frame goal on concentric arcs (per gear).
//   Forward sector: goal on arc, path yaw lerps toward stick_theta.
//   Backward sector (60 deg + hysteresis): Cartesian goal behind vehicle,
//   path yaw fixed at current heading -> MPPI drives vx<0 (reverse avoid, no U-turn).
//   Pure lateral (|rx| deadbanded) -> in-place turn.
//   Forward + narrow gate: detect/track up to gate_detect_speed_max_mps while approaching;
//     pass_mode enters ONLY when |odom|<=gate_creep_speed_max_mps; once active, keeps until
//     body clear / intent timeout / front_to_gate (not odom overspeed). In pass_mode:
//     gate distance refreshed from odom anchor + cloud; zero footprint padding;
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
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
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

struct MemorizedGateCenter
{
  double center_x_odom{0.0};
  double center_y_odom{0.0};
  double width_m{0.95};
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

    gate_path_enable_ = declare_parameter<bool>("gate_path_enable", true);
    obstacle_points_topic_ = declare_parameter<std::string>(
      "obstacle_points_topic", "/perception/obstacle_points");
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.50);
    gate_width_min_m_ = declare_parameter<double>("gate_width_min_m", 0.75);
    gate_width_max_m_ = declare_parameter<double>("gate_width_max_m", 1.00);
    gate_local_radius_m_ = declare_parameter<double>("gate_local_radius_m", 1.20);
    gate_detect_mode_ = declare_parameter<std::string>("gate_detect_mode", "bilateral");
    gate_scan_dist_min_m_ = declare_parameter<double>("gate_scan_dist_min_m", 0.25);
    gate_scan_dist_step_m_ = declare_parameter<double>("gate_scan_dist_step_m", 0.05);
    gate_slice_half_width_m_ = declare_parameter<double>("gate_slice_half_width_m", 0.06);
    gate_opening_depth_m_ = declare_parameter<double>("gate_opening_depth_m", 0.25);
    gate_pillar_mode_enable_ = declare_parameter<bool>("gate_pillar_mode_enable", true);
    gate_min_side_boundary_m_ = declare_parameter<double>("gate_min_side_boundary_m", 0.38);
    gate_min_side_points_ = declare_parameter<int>("gate_min_side_points", 1);
    gate_center_clear_half_width_m_ = declare_parameter<double>(
      "gate_center_clear_half_width_m", 0.12);
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
    gate_creep_scan_body_forward_ = declare_parameter<bool>(
      "gate_creep_scan_body_forward", true);
    gate_path_tail_m_ = declare_parameter<double>("gate_path_tail_m", 0.50);
    gate_hold_sec_ = declare_parameter<double>("gate_hold_sec", 0.25);
    gate_confirm_sec_ = declare_parameter<double>("gate_confirm_sec", 0.50);
    gate_confirmed_hold_sec_ = declare_parameter<double>("gate_confirmed_hold_sec", 0.60);
    gate_ema_alpha_ = declare_parameter<double>("gate_ema_alpha", 0.35);
    gate_lock_distance_delta_m_ = declare_parameter<double>("gate_lock_distance_delta_m", 0.18);
    gate_lock_center_y_delta_m_ = declare_parameter<double>("gate_lock_center_y_delta_m", 0.15);
    gate_memory_commit_frames_ = declare_parameter<int>("gate_memory_commit_frames", 50);
    gate_memory_max_count_ = declare_parameter<int>("gate_memory_max_count", 1);
    gate_memory_max_count_ = std::max(1, gate_memory_max_count_);
    gate_memory_merge_radius_m_ = declare_parameter<double>("gate_memory_merge_radius_m", 2.00);
    gate_pass_mode_enter_m_ = declare_parameter<double>("gate_pass_mode_enter_m", 0.80);
    gate_pass_front_center_x_m_ = declare_parameter<double>("gate_pass_front_center_x_m", 0.85);
    gate_pass_mode_exit_m_ = declare_parameter<double>("gate_pass_mode_exit_m", 0.95);
    gate_pass_mode_body_clear_x_m_ = declare_parameter<double>(
      "gate_pass_mode_body_clear_x_m", -0.45);
    gate_pass_mode_hold_sec_ = declare_parameter<double>("gate_pass_mode_hold_sec", 0.25);
    narrow_gate_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_pass_mode_topic", "/mppi/narrow_gate_pass_mode");
    gate_marker_enable_ = declare_parameter<bool>("gate_marker_enable", true);
    gate_marker_topic_ = declare_parameter<std::string>(
      "gate_marker_topic", "/mppi/gate_markers");
    use_spatial_range_ = (path_generation_mode_ == "spatial_range");

    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, 10,
      std::bind(&JoystickLocalPathNode::onOdom, this, std::placeholders::_1));

    if (use_spatial_range_) {
      joystick_sub_ = create_subscription<Joystick>(
        joystick_topic_, 10,
        std::bind(&JoystickLocalPathNode::onJoystick, this, std::placeholders::_1));
      if (gate_path_enable_) {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          obstacle_points_topic_, rclcpp::SensorDataQoS(),
          std::bind(&JoystickLocalPathNode::onCloud, this, std::placeholders::_1));
      }
    } else {
      control_sub_ = create_subscription<Twist>(
        control_topic_, 10,
        std::bind(&JoystickLocalPathNode::onControl, this, std::placeholders::_1));
    }

    if (footprint_publish_enable_ || use_spatial_range_) {
      car_info_sub_ = create_subscription<CarInfo>(
        car_info_topic_, 10,
        std::bind(&JoystickLocalPathNode::onCarInfo, this, std::placeholders::_1));
    }

    path_pub_ = create_publisher<Path>(intent_path_topic_, 1);
    narrow_gate_pass_mode_pub_ = create_publisher<std_msgs::msg::Bool>(
      narrow_gate_pass_mode_topic_, rclcpp::QoS(1).transient_local());
    if (gate_marker_enable_ && gate_path_enable_) {
      gate_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        gate_marker_topic_, rclcpp::QoS(1).transient_local());
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
    std_msgs::msg::Bool pass_mode_init;
    pass_mode_init.data = false;
    narrow_gate_pass_mode_pub_->publish(pass_mode_init);
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
        "boundary=%.0fdeg steps=%d backward_reverse=%s",
        joystick_topic_.c_str(), forward_sector_deg_, backward_sector_deg_,
        sector_hysteresis_deg_, 180.0 - half_back, path_interpolation_steps_,
        backward_reverse_path_enable_ ? "on" : "off");
      if (gate_path_enable_) {
        RCLCPP_INFO(
          get_logger(),
          "gate_path: cloud=%s detect=%s local_r=%.2fm detect_v<=%.2f creep_v<=%.2f "
          "width=[%.2f,%.2f]m memory_max=%d merge_r=%.2fm commit_frames=%d",
          obstacle_points_topic_.c_str(), gate_detect_mode_.c_str(),
          gate_local_radius_m_, gate_detect_speed_max_mps_, gate_creep_speed_max_mps_,
          gate_width_min_m_, gate_width_max_m_, gate_memory_max_count_,
          gate_memory_merge_radius_m_, gate_memory_commit_frames_);
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
  }

private:
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
  }

  void onCarInfo(const CarInfo::SharedPtr msg)
  {
    int gear = static_cast<int>(msg->gear);
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      latest_gear_ = gear;
      have_gear_ = true;
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
    if (narrow_gate_pass_active_) {
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
      visualization_msgs::msg::Marker mem;
      mem.header.frame_id = global_frame_;
      mem.header.stamp = stamp;
      mem.ns = "gate_memory";
      mem.id = static_cast<int>(i);
      mem.type = visualization_msgs::msg::Marker::SPHERE;
      mem.action = visualization_msgs::msg::Marker::ADD;
      mem.pose.position.x = stored.center_x_odom;
      mem.pose.position.y = stored.center_y_odom;
      mem.pose.position.z = 0.08;
      mem.pose.orientation.w = 1.0;
      mem.scale.x = std::max(0.12, stored.width_m);
      mem.scale.y = std::max(0.12, stored.width_m);
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
      label.text = "M" + std::to_string(i + 1) + " w=" +
        std::to_string(static_cast<int>(stored.width_m * 100.0)) + "cm";
      label.lifetime = mem.lifetime;
      markers.markers.push_back(label);
    }

    for (int i = static_cast<int>(memorized_gates_.size());
      i < gate_memory_max_count_; ++i)
    {
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

  void updateNarrowGatePassMode(
    const NarrowGateInfo & gate, double norm_x, double norm_y,
    const Odometry & odom, const rclcpp::Time & tnow)
  {
    const bool intent_ok = intentMatchesGate(gate, norm_x, norm_y, odom) &&
      intentMatchesTrackDirection(norm_x, norm_y, odom);
    const double front_to_gate = gateFrontCenterToGateM(gate);

    if (gateBodyCleared(gate)) {
      setNarrowGatePassMode(false, tnow);
      RCLCPP_INFO(
        get_logger(), "narrow_gate_pass_mode exit: body cleared gate x=%.2fm",
        gate.distance_m);
      return;
    }

    if (narrow_gate_pass_active_) {
      if (intent_ok) {
        narrow_gate_pass_last_intent_stamp_ = tnow;
      } else if ((tnow - narrow_gate_pass_last_intent_stamp_).seconds() >
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
      if (front_to_gate > gate_pass_mode_exit_m_) {
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

    if (!isOdomFresh(odom, tnow)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "odom stale or missing, cancelling FollowPath goal");
      cancelActiveGoal();
      publishEmptyPath();
      return;
    }

    Path path;
    bool stick_idle = false;
    if (use_spatial_range_) {
      if (!buildSpatialPath(odom, tnow, path, stick_idle)) {
        if (stick_idle) {
          if (!was_stick_idle_) {
            scheduleMppiReset("stick center");
            publishEmptyPath();
            was_stick_idle_ = true;
          }
          resetGateTracking();
          clearNarrowGatePassMode(tnow);
          publishGateMarkers(*odom);
          return;
        }
        was_stick_idle_ = false;
        cancelActiveGoal();
        publishEmptyPath();
        publishGateMarkers(*odom);
        return;
      }
      was_stick_idle_ = false;
    } else {
      if (!buildVelocityPath(odom, tnow, path)) {
        cancelActiveGoal();
        publishEmptyPath();
        publishGateMarkers(*odom);
        return;
      }
    }

    path_pub_->publish(path);
    if (gate_marker_pub_) {
      publishGateMarkers(*odom);
    }
    if (use_spatial_range_ && gate_path_enable_) {
      publishNarrowGatePassModeState();
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
        if (narrow_gate_pass_active_) {
          std::lock_guard<std::mutex> lock(gate_mutex_);
          if (have_last_gate_ && refreshPassModeGateUnlocked(tnow, *odom, norm_x, norm_y)) {
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
          } else if (narrow_gate_pass_active_) {
            clearNarrowGatePassMode(tnow);
          }
        } else if (isVehicleGateDetectSpeed(*odom)) {
          const auto gate = detectNarrowGate(tnow, *odom, norm_x, norm_y);
          if (gate) {
            updateNarrowGatePassMode(*gate, norm_x, norm_y, *odom, tnow);
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
          } else {
            clearNarrowGatePassMode(tnow);
          }
        } else {
          resetGateTracking();
          clearNarrowGatePassMode(tnow);
        }
      } else if (gate_path_enable_) {
        clearNarrowGatePassMode(tnow);
      }
    } else {
      const double norm_x_clamped = clampNorm(norm_x, -1.0, 1.0);
      const double norm_y_clamped = clampNorm(norm_y, -1.0, 1.0);
      dx_body = std::min(0.0, norm_x_clamped) * max_back;
      dy_body = norm_y_clamped * max_lateral;
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

  double effectiveStickForwardMin(const Odometry & odom) const
  {
    return isGateCreepSpeed(odom) ? gate_creep_stick_forward_min_ : gate_stick_forward_min_;
  }

  bool intentTrendTowardOpening(
    const NarrowGateInfo & gate, double norm_x, double norm_y) const
  {
    if (norm_x < gate_creep_stick_forward_min_) {
      return false;
    }
    const double stick_r = std::hypot(norm_x, norm_y);
    if (stick_r < gate_creep_stick_min_) {
      return false;
    }
    if (gate.distance_m < gate_scan_dist_min_m_ ||
      gate.distance_m > gate_local_radius_m_ + 1e-6)
    {
      return false;
    }

    const double half_w = 0.5 * gate.width_m + gate_creep_intent_cross_slack_m_;
    if (std::abs(gate.center_y_m) > half_w) {
      return false;
    }

    const double stick_angle = std::atan2(norm_y, norm_x);
    const double gate_angle = std::atan2(
      gate.center_y_m, std::max(gate.distance_m, 0.05));
    const double delta = std::abs(normalizeAngle(stick_angle - gate_angle));
    if (delta > gate_creep_intent_angle_max_) {
      return false;
    }

    const double ux = norm_x / stick_r;
    const double uy = norm_y / stick_r;
    const double along = gate.distance_m * ux + gate.center_y_m * uy;
    return along >= 0.05;
  }

  bool intentMatchesGate(
    const NarrowGateInfo & gate, double norm_x, double norm_y,
    const Odometry & odom) const
  {
    if (isGateCreepSpeed(odom)) {
      return intentTrendTowardOpening(gate, norm_x, norm_y);
    }
    return intentCrossesOpening(gate, norm_x, norm_y);
  }

  bool openingSliceValid(
    float left_y, float right_y, int left_count, int right_count, double width) const
  {
    if (left_count < gate_min_side_points_ || right_count < gate_min_side_points_) {
      return false;
    }
    if (!gate_pillar_mode_enable_) {
      if (left_y < gate_min_side_boundary_m_ || (-right_y) < gate_min_side_boundary_m_) {
        return false;
      }
    }
    return width >= gate_width_min_m_ && width <= gate_width_max_m_;
  }

  std::optional<NarrowGateInfo> scanOpeningAlongRay(
    const std::vector<std::pair<float, float>> & local_pts,
    double theta, double norm_x, double norm_y, const Odometry & odom) const
  {
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    const double slice_half = std::max(0.03, gate_slice_half_width_m_);
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);
    const double center_clear = std::max(0.05, gate_center_clear_half_width_m_);

    std::optional<NarrowGateInfo> best;
    double best_score = std::numeric_limits<double>::infinity();

    for (
      double dist = gate_scan_dist_min_m_;
      dist <= gate_local_radius_m_ + 1e-6;
      dist += dist_step)
    {
      float left_y = std::numeric_limits<float>::infinity();
      float right_y = -std::numeric_limits<float>::infinity();
      int left_count = 0;
      int right_count = 0;

      for (const auto & p : local_pts) {
        const double xs = static_cast<double>(p.first) * cos_t +
          static_cast<double>(p.second) * sin_t;
        const double ys = -static_cast<double>(p.first) * sin_t +
          static_cast<double>(p.second) * cos_t;
        if (std::abs(xs - dist) > slice_half) {
          continue;
        }
        if (ys > center_clear) {
          ++left_count;
          left_y = std::min(left_y, static_cast<float>(ys));
        } else if (ys < -center_clear) {
          ++right_count;
          right_y = std::max(right_y, static_cast<float>(ys));
        }
      }

      if (!std::isfinite(left_y) || !std::isfinite(right_y)) {
        continue;
      }

      const double width = static_cast<double>(left_y - right_y);
      if (!openingSliceValid(left_y, right_y, left_count, right_count, width)) {
        continue;
      }

      bool center_blocked = false;
      for (const auto & p : local_pts) {
        const double xs = static_cast<double>(p.first) * cos_t +
          static_cast<double>(p.second) * sin_t;
        const double ys = -static_cast<double>(p.first) * sin_t +
          static_cast<double>(p.second) * cos_t;
        if (std::abs(xs - dist) > slice_half + 0.04) {
          continue;
        }
        if (std::abs(ys) <= center_clear) {
          center_blocked = true;
          break;
        }
      }
      if (center_blocked) {
        continue;
      }

      const double center_x = dist * cos_t;
      const double center_y = dist * sin_t;

      NarrowGateInfo gate;
      gate.distance_m = center_x;
      gate.center_y_m = center_y;
      gate.width_m = width;
      if (!intentMatchesGate(gate, norm_x, norm_y, odom)) {
        continue;
      }

      const double score = openingIntentScore(center_x, center_y, norm_x, norm_y);
      if (score < best_score) {
        best_score = score;
        best = gate;
      }
    }
    return best;
  }

  std::optional<NarrowGateInfo> scanOpeningBilateral(
    const std::vector<std::pair<float, float>> & local_pts,
    double norm_x, double norm_y, const Odometry & odom) const
  {
    const double dist_step = std::max(0.03, gate_scan_dist_step_m_);
    const double bin_half = dist_step * 0.5;
    const double center_clear = std::max(0.05, gate_center_clear_half_width_m_);

    std::optional<NarrowGateInfo> best;
    double best_score = std::numeric_limits<double>::infinity();

    for (
      double dist = gate_scan_dist_min_m_;
      dist <= gate_local_radius_m_ + 1e-6;
      dist += dist_step)
    {
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
        continue;
      }

      if (std::abs(static_cast<double>(left_x_at) - static_cast<double>(right_x_at)) >
        gate_opening_depth_m_)
      {
        continue;
      }

      const double width = static_cast<double>(left_y - right_y);
      if (!openingSliceValid(left_y, right_y, left_count, right_count, width)) {
        continue;
      }

      bool center_blocked = false;
      for (const auto & p : local_pts) {
        const double x = static_cast<double>(p.first);
        const double y = static_cast<double>(p.second);
        if (x < dist - bin_half - 0.04 || x > dist + bin_half + 0.04) {
          continue;
        }
        if (std::abs(y) <= center_clear) {
          center_blocked = true;
          break;
        }
      }
      if (center_blocked) {
        continue;
      }

      const double center_x = dist;
      const double center_y = 0.5 * (static_cast<double>(left_y) + static_cast<double>(right_y));

      NarrowGateInfo gate;
      gate.distance_m = center_x;
      gate.center_y_m = center_y;
      gate.width_m = width;
      if (!intentMatchesGate(gate, norm_x, norm_y, odom)) {
        continue;
      }

      const double score = openingIntentScore(center_x, center_y, norm_x, norm_y);
      if (score < best_score) {
        best_score = score;
        best = gate;
      }
    }
    return best;
  }

  std::optional<NarrowGateInfo> scanOpeningFromLocalCloud(
    const std::vector<std::pair<float, float>> & local_pts,
    double norm_x, double norm_y, const Odometry & odom,
    bool always_try_body_forward) const
  {
    if (gate_detect_mode_ == "bilateral") {
      return scanOpeningBilateral(local_pts, norm_x, norm_y, odom);
    }

    const double stick_theta = std::atan2(norm_y, norm_x);
    std::optional<NarrowGateInfo> best;
    double best_score = std::numeric_limits<double>::infinity();

    auto try_theta = [&](double theta) {
      if (const auto gate = scanOpeningAlongRay(local_pts, theta, norm_x, norm_y, odom)) {
        const double score = openingIntentScore(
          gate->distance_m, gate->center_y_m, norm_x, norm_y);
        if (score < best_score) {
          best_score = score;
          best = gate;
        }
      }
    };

    try_theta(stick_theta);
    if (always_try_body_forward) {
      try_theta(0.0);
    } else if (gate_creep_scan_body_forward_ && isGateCreepSpeed(odom) &&
      std::abs(normalizeAngle(stick_theta)) > 0.05)
    {
      try_theta(0.0);
    }
    return best;
  }

  bool intentCrossesOpening(
    const NarrowGateInfo & gate, double norm_x, double norm_y) const
  {
    if (norm_x < gate_stick_forward_min_) {
      return false;
    }
    const double stick_r = std::hypot(norm_x, norm_y);
    if (stick_r < 1e-3) {
      return false;
    }
    const double ux = norm_x / stick_r;
    const double uy = norm_y / stick_r;

    const double half_w = 0.5 * gate.width_m + gate_intent_cross_slack_m_;
    const double half_d = 0.5 * gate_opening_depth_m_;
    const double cx = gate.distance_m;
    const double cy = gate.center_y_m;

    if (std::abs(cy) <= gate_center_y_deadband_m_) {
      if (ux <= 1e-3) {
        return false;
      }
      const double t = cx / ux;
      if (t < 0.0 || t > gate_local_radius_m_) {
        return false;
      }
      const double y_at = t * uy;
      return std::abs(y_at - cy) <= half_w;
    }

    if (std::abs(norm_y) < gate_stick_lateral_min_) {
      return false;
    }
    if (cy * norm_y < 0.0) {
      return false;
    }
    const double lateral_ratio =
      std::abs(norm_y) / std::max(std::abs(norm_x), 1e-3);
    if (lateral_ratio < gate_intent_lateral_ratio_min_) {
      return false;
    }

    if (std::abs(ux) > 1e-3) {
      const double t = cx / ux;
      if (t >= 0.0 && t <= gate_local_radius_m_) {
        const double y_at = t * uy;
        if (std::abs(y_at - cy) <= half_w) {
          return true;
        }
      }
    }

    if (std::abs(uy) > 1e-3) {
      const double t = cy / uy;
      if (t >= 0.0 && t <= gate_local_radius_m_) {
        const double x_at = t * ux;
        if (std::abs(x_at - cx) <= half_d) {
          return true;
        }
      }
    }
    return false;
  }

  bool intentMatchesTrackDirection(
    double norm_x, double norm_y, const Odometry & odom) const
  {
    if (!gate_tracking_active_) {
      return true;
    }
    const double max_delta = isGateCreepSpeed(odom) ?
      gate_creep_intent_angle_delta_max_ : gate_intent_angle_delta_max_;
    const double cur = std::atan2(norm_y, norm_x);
    const double tracked = std::atan2(
      gate_track_center_y_m_, std::max(gate_track_distance_m_, 1e-3));
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
    if (std::abs(gate.center_y_m) <= gate_center_y_deadband_m_) {
      return 0.0;
    }
    const double side_sign =
      (std::abs(gate.center_y_m) > 1e-3) ? ((gate.center_y_m > 0.0) ? 1.0 : -1.0) :
      ((norm_y >= 0.0) ? 1.0 : -1.0);
    return side_sign * (M_PI / 2.0);
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

  int findMemorizedGateNearOdomUnlocked(double ox, double oy) const
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

  void consolidateMemorizedGatesUnlocked()
  {
    for (size_t i = 0; i < memorized_gates_.size(); ++i) {
      for (size_t j = memorized_gates_.size(); j-- > i;) {
        const double dist = std::hypot(
          memorized_gates_[i].center_x_odom - memorized_gates_[j].center_x_odom,
          memorized_gates_[i].center_y_odom - memorized_gates_[j].center_y_odom);
        if (dist > gate_memory_merge_radius_m_) {
          continue;
        }
        memorized_gates_[i].center_x_odom =
          0.5 * (memorized_gates_[i].center_x_odom + memorized_gates_[j].center_x_odom);
        memorized_gates_[i].center_y_odom =
          0.5 * (memorized_gates_[i].center_y_odom + memorized_gates_[j].center_y_odom);
        memorized_gates_[i].width_m =
          0.5 * (memorized_gates_[i].width_m + memorized_gates_[j].width_m);
        memorized_gates_.erase(memorized_gates_.begin() + static_cast<std::ptrdiff_t>(j));
        RCLCPP_INFO(
          get_logger(),
          "gate_memory consolidated duplicate -> odom=(%.2f, %.2f) width=%.2fm total=%zu",
          memorized_gates_[i].center_x_odom, memorized_gates_[i].center_y_odom,
          memorized_gates_[i].width_m, memorized_gates_.size());
      }
    }
  }

  bool snapGateToMemorizedIfNearUnlocked(const Odometry & odom, NarrowGateInfo & gate) const
  {
    double ox = 0.0;
    double oy = 0.0;
    bodyCenterToOdom(odom, gate.distance_m, gate.center_y_m, ox, oy);
    const int idx = findMemorizedGateNearOdomUnlocked(ox, oy);
    if (idx < 0) {
      return false;
    }
    const auto & stored = memorized_gates_[static_cast<size_t>(idx)];
    odomCenterToBody(
      odom, stored.center_x_odom, stored.center_y_odom,
      gate.distance_m, gate.center_y_m);
    gate.width_m = stored.width_m;
    gate.confirmed = true;
    return true;
  }

  void resetGateMemoryAccumUnlocked()
  {
    gate_memory_accum_active_ = false;
    gate_memory_frame_count_ = 0;
    gate_memory_sum_x_odom_ = 0.0;
    gate_memory_sum_y_odom_ = 0.0;
    gate_memory_sum_width_m_ = 0.0;
    gate_memory_track_committed_ = false;
  }

  void updateGateMemoryAccumUnlocked(
    const Odometry & odom, const NarrowGateInfo & gate)
  {
    if (gate_memory_track_committed_) {
      return;
    }
    double ox = 0.0;
    double oy = 0.0;
    bodyCenterToOdom(odom, gate.distance_m, gate.center_y_m, ox, oy);
    if (!gate_memory_accum_active_) {
      gate_memory_accum_active_ = true;
      gate_memory_frame_count_ = 0;
      gate_memory_sum_x_odom_ = 0.0;
      gate_memory_sum_y_odom_ = 0.0;
      gate_memory_sum_width_m_ = 0.0;
    }
    ++gate_memory_frame_count_;
    gate_memory_sum_x_odom_ += ox;
    gate_memory_sum_y_odom_ += oy;
    gate_memory_sum_width_m_ += gate.width_m;
  }

  void maybeCommitMemorizedGateUnlocked()
  {
    if (gate_memory_track_committed_ || !gate_memory_accum_active_) {
      return;
    }
    if (gate_memory_frame_count_ < gate_memory_commit_frames_) {
      return;
    }

    const double mean_x = gate_memory_sum_x_odom_ /
      static_cast<double>(gate_memory_frame_count_);
    const double mean_y = gate_memory_sum_y_odom_ /
      static_cast<double>(gate_memory_frame_count_);
    const double mean_w = gate_memory_sum_width_m_ /
      static_cast<double>(gate_memory_frame_count_);

    for (auto & stored : memorized_gates_) {
      if (std::hypot(stored.center_x_odom - mean_x, stored.center_y_odom - mean_y) <=
        gate_memory_merge_radius_m_)
      {
        stored.center_x_odom = 0.7 * stored.center_x_odom + 0.3 * mean_x;
        stored.center_y_odom = 0.7 * stored.center_y_odom + 0.3 * mean_y;
        stored.width_m = 0.7 * stored.width_m + 0.3 * mean_w;
        gate_memory_track_committed_ = true;
        consolidateMemorizedGatesUnlocked();
        RCLCPP_INFO(
          get_logger(),
          "gate_memory merge same door: odom=(%.2f, %.2f) width=%.2fm frames=%d total=%zu",
          stored.center_x_odom, stored.center_y_odom, stored.width_m,
          gate_memory_frame_count_, memorized_gates_.size());
        return;
      }
    }

    if (static_cast<int>(memorized_gates_.size()) >= gate_memory_max_count_) {
      const auto & evicted = memorized_gates_.front();
      RCLCPP_INFO(
        get_logger(),
        "gate_memory evict oldest: odom=(%.2f, %.2f) width=%.2fm",
        evicted.center_x_odom, evicted.center_y_odom, evicted.width_m);
      memorized_gates_.erase(memorized_gates_.begin());
    }

    MemorizedGateCenter stored;
    stored.center_x_odom = mean_x;
    stored.center_y_odom = mean_y;
    stored.width_m = mean_w;
    memorized_gates_.push_back(stored);
    gate_memory_track_committed_ = true;
    consolidateMemorizedGatesUnlocked();
    RCLCPP_INFO(
      get_logger(),
      "gate_memory saved #%zu: odom=(%.2f, %.2f) width=%.2fm frames=%d",
      memorized_gates_.size(), mean_x, mean_y, mean_w, gate_memory_frame_count_);
  }

  std::optional<NarrowGateInfo> scanMemorizedGate(
    const Odometry & odom, double norm_x, double norm_y) const
  {
    if (memorized_gates_.empty()) {
      return std::nullopt;
    }

    std::optional<NarrowGateInfo> best;
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto & stored : memorized_gates_) {
      double bx = 0.0;
      double by = 0.0;
      odomCenterToBody(odom, stored.center_x_odom, stored.center_y_odom, bx, by);
      if (std::hypot(bx, by) > gate_local_radius_m_) {
        continue;
      }

      NarrowGateInfo gate;
      gate.distance_m = bx;
      gate.center_y_m = by;
      gate.width_m = stored.width_m;
      gate.confirmed = true;
      gate.axis_yaw_body = computeGateAxisYaw(gate, norm_y);
      if (!intentMatchesGate(gate, norm_x, norm_y, odom)) {
        continue;
      }

      const double score = openingIntentScore(bx, by, norm_x, norm_y);
      if (score < best_score) {
        best_score = score;
        best = gate;
      }
    }
    return best;
  }

  void resetGateTrackingUnlocked()
  {
    gate_tracking_active_ = false;
    gate_confirmed_ = false;
    gate_match_count_ = 0;
    have_last_gate_ = false;
    resetGateMemoryAccumUnlocked();
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
    resetGateMemoryAccumUnlocked();
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
    if (norm_x < effectiveStickForwardMin(odom)) {
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

    return scanOpeningFromLocalCloud(local_pts, norm_x, norm_y, odom, false);
  }

  std::optional<NarrowGateInfo> scanPassModeGateFromCloud(
    const rclcpp::Time & tnow, const Odometry & odom,
    double norm_x, double norm_y) const
  {
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

    return scanOpeningFromLocalCloud(local_pts, norm_x, norm_y, odom, true);
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
    if (norm_x < effectiveStickForwardMin(odom)) {
      resetGateTracking();
      return std::nullopt;
    }

    auto cloud_candidate = scanNarrowGateFromCloud(tnow, odom, norm_x, norm_y);
    std::lock_guard<std::mutex> lock(gate_mutex_);

    auto candidate = cloud_candidate;
    if (candidate && snapGateToMemorizedIfNearUnlocked(odom, *candidate)) {
      RCLCPP_DEBUG(
        get_logger(), "gate_path snap cloud detect to memorized gate body=(%.2f, %.2f)",
        candidate->distance_m, candidate->center_y_m);
    }
    if (!candidate) {
      candidate = scanMemorizedGate(odom, norm_x, norm_y);
    }
    const bool intent_ok = candidate &&
      intentMatchesGate(*candidate, norm_x, norm_y, odom) &&
      intentMatchesTrackDirection(norm_x, norm_y, odom);

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
      if (gate_track_from_cloud_) {
        updateGateMemoryAccumUnlocked(odom, last_gate_);
        maybeCommitMemorizedGateUnlocked();
      }
      return last_gate_;
    }

    if (gate_tracking_active_ && have_last_gate_ &&
      intentMatchesGate(last_gate_, norm_x, norm_y, odom) &&
      intentMatchesTrackDirection(norm_x, norm_y, odom))
    {
      const double hold_limit = gate_confirmed_ ? gate_confirmed_hold_sec_ :
        (isGateCreepSpeed(odom) ? gate_hold_sec_ : gate_detect_hold_sec_);
      if ((tnow - last_gate_stamp_).seconds() <= hold_limit) {
        ++gate_match_count_;
        maybeConfirmGateTrack(tnow, norm_y, odom);
        last_gate_ = makeActiveGate(norm_y);
        last_gate_stamp_ = tnow;
        if (gate_track_from_cloud_) {
          updateGateMemoryAccumUnlocked(odom, last_gate_);
          maybeCommitMemorizedGateUnlocked();
        }
        return last_gate_;
      }
    }

    resetGateTrackingUnlocked();
    return std::nullopt;
  }

  Path buildGateCenterlinePath(
    const Odometry & odom, const NarrowGateInfo & gate, double goal_x_body) const
  {
    if (gate.confirmed) {
      return buildConfirmedGateCenterlinePath(odom, gate, goal_x_body);
    }
    return buildTentativeGateCenterlinePath(odom, gate, goal_x_body);
  }

  Path buildTentativeGateCenterlinePath(
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

  Path buildConfirmedGateCenterlinePath(
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
    const double axis_yaw = gate.axis_yaw_body;
    const double yaw_entry = normalizeAngle(yaw0 + axis_yaw);
    const double side_sign = (y_end >= 0.0) ? 1.0 : -1.0;

    const double turn_lead_m = std::clamp(
      0.30 + 0.45 * std::abs(y_end), 0.35, std::max(0.40, x_end - 0.10));
    const double x_turn = std::clamp(
      x_end - turn_lead_m, 0.15, std::max(0.15, x_end - 0.08));

    const double curve_len = std::hypot(x_end - x_turn, y_end);
    const double tangent_len0 = std::max(0.20, 0.75 * curve_len);
    const double tangent_len1 = std::max(0.20, 0.75 * curve_len);
    const double mx0 = tangent_len0;
    const double my0 = 0.0;
    const double mx1 = tangent_len1 * std::cos(axis_yaw);
    const double my1 = tangent_len1 * std::sin(axis_yaw);

    std::vector<BodyPoint> body_pts;
    body_pts.push_back({0.0, 0.0});

    const double approach_step = 0.08;
    for (double x = approach_step; x < x_turn - 1e-6; x += approach_step) {
      body_pts.push_back({x, 0.0});
    }
    if (body_pts.back().x < x_turn - 1e-6) {
      body_pts.push_back({x_turn, 0.0});
    }

    const int hermite_steps = std::max(10, path_interpolation_steps_ / 2);
    for (int i = 1; i <= hermite_steps; ++i) {
      const double u = static_cast<double>(i) / static_cast<double>(hermite_steps);
      body_pts.push_back(
        hermitePoint(u, x_turn, 0.0, x_end, y_end, mx0, my0, mx1, my1));
    }

    const double tail_m = std::max(
      gate_path_tail_m_,
      std::clamp(0.45 + 0.25 * std::abs(y_end), 0.45, 0.80));
    const double tail_step = 0.08;
    if (std::abs(axis_yaw) < 1e-3) {
      const double x_tail = std::max(x_end + tail_m, std::max(x_end, goal_x_body));
      for (double x = x_end + tail_step; x <= x_tail + 1e-6; x += tail_step) {
        body_pts.push_back({x, y_end});
      }
    } else {
      for (double s = tail_step; s <= tail_m + 1e-6; s += tail_step) {
        body_pts.push_back({x_end, y_end + side_sign * s});
      }
    }

    for (const auto & bp : body_pts) {
      PoseStamped p;
      p.header = path.header;
      p.pose.position.x = x0 + cos_y * bp.x - sin_y * bp.y;
      p.pose.position.y = y0 + sin_y * bp.x + cos_y * bp.y;
      p.pose.position.z = 0.0;

      double yaw_path = yaw0;
      if (bp.x >= x_turn - 1e-6) {
        if (bp.x >= x_end - 1e-6 && std::abs(axis_yaw) > 1e-3) {
          yaw_path = yaw_entry;
        } else {
          const double t = std::clamp(
            (bp.x - x_turn) / std::max(x_end - x_turn, 1e-3), 0.0, 1.0);
          yaw_path = lerpAngle(yaw0, yaw_entry, t);
        }
      }
      p.pose.orientation = quaternionFromYaw(yaw_path);
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
  std::vector<geometry_msgs::msg::Point> footprint_base_pts_;
  double last_footprint_padding_{-1.0};
  bool have_published_footprint_{false};

  bool gate_path_enable_{true};
  std::string obstacle_points_topic_;
  double obstacle_timeout_{0.50};
  double gate_width_min_m_{0.75};
  double gate_width_max_m_{1.00};
  double gate_local_radius_m_{1.20};
  std::string gate_detect_mode_{"bilateral"};
  double gate_scan_dist_min_m_{0.25};
  double gate_scan_dist_step_m_{0.05};
  double gate_slice_half_width_m_{0.06};
  double gate_opening_depth_m_{0.25};
  bool gate_pillar_mode_enable_{true};
  double gate_min_side_boundary_m_{0.38};
  int gate_min_side_points_{1};
  double gate_center_clear_half_width_m_{0.12};
  double gate_stick_forward_min_{0.20};
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
  bool gate_creep_scan_body_forward_{true};
  double gate_path_tail_m_{0.50};
  double gate_hold_sec_{0.25};
  double gate_confirm_sec_{0.50};
  double gate_confirmed_hold_sec_{0.60};
  double gate_ema_alpha_{0.35};
  double gate_lock_distance_delta_m_{0.18};
  double gate_lock_center_y_delta_m_{0.15};
  int gate_memory_commit_frames_{50};
  int gate_memory_max_count_{1};
  double gate_memory_merge_radius_m_{2.00};
  double gate_pass_mode_enter_m_{0.80};
  double gate_pass_front_center_x_m_{0.85};
  double gate_pass_mode_exit_m_{0.95};
  double gate_pass_mode_body_clear_x_m_{-0.45};
  double gate_pass_mode_hold_sec_{0.25};
  std::string narrow_gate_pass_mode_topic_;
  bool gate_marker_enable_{true};
  std::string gate_marker_topic_;
  bool narrow_gate_pass_active_{false};
  rclcpp::Time narrow_gate_pass_last_intent_stamp_{0, 0, RCL_ROS_TIME};
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
  mutable int gate_memory_frame_count_{0};
  mutable bool gate_memory_accum_active_{false};
  mutable bool gate_memory_track_committed_{false};
  mutable double gate_memory_sum_x_odom_{0.0};
  mutable double gate_memory_sum_y_odom_{0.0};
  mutable double gate_memory_sum_width_m_{0.0};
  mutable std::vector<MemorizedGateCenter> memorized_gates_;
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gate_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr footprint_pub_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex ctrl_mutex_;
  Twist::SharedPtr latest_ctrl_;
  rclcpp::Time latest_ctrl_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex joystick_mutex_;
  Joystick::SharedPtr latest_joystick_;
  rclcpp::Time latest_joystick_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex gear_mutex_;
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
