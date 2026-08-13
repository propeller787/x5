// Safety gate between Nav2 MPPI (/cmd_vel) and the wheelchair CAN adapter (/nav/control).
//
// Always forwards MPPI output (with optional gate_enable bypass for testing).
// Retained safety layers:
//   - cmd_hold_timeout: hold last /cmd_vel when MPPI is slower than publish_rate_hz
//   - joystick release / stale -> stop (hard if obstacles nearby or cloud stale;
//     soft ramp when clear within release_soft_stop_radius)
//   - proximity speed cap from obstacle cloud + travel intent
//   - tiered wz cap from nearby obstacle-point density
//   - gear vx cap from /vlago/car/info (1-4 gear -> configurable kph limits)
//   - narrow gate pass mode (/mppi/narrow_gate_pass_mode): vx cap; rocker-intent filter
//   - side pass crawl (/mppi/narrow_gate_side_pass_mode): skip MPPI; footprint 5cm forward/lateral strips + crawl
//     (forward -> no reverse, backward -> no forward; lateral -> no opposite turn); stop on release
//   - after narrow gate exit: linear vx ramp-up to avoid sudden acceleration
//   - vx accel limit: max 1 m/s^2 when speeding up forward/reverse; decel/brake unchanged

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
    release_soft_stop_radius_ = declare_parameter<double>("release_soft_stop_radius", 1.2);
    release_soft_stop_decel_ = declare_parameter<double>("release_soft_stop_decel", 1.5);
    release_soft_stop_wz_decel_ = declare_parameter<double>("release_soft_stop_wz_decel", 2.0);
    release_soft_stop_vel_eps_ = declare_parameter<double>("release_soft_stop_vel_eps", 0.02);

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
    side_pass_crawl_check_depth_m_ = declare_parameter<double>(
      "side_pass_crawl_check_depth_m", 0.05);
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

    if (narrow_gate_exit_ramp_sec_ < 0.05) {
      narrow_gate_exit_ramp_sec_ = 0.05;
    }
    if (narrow_gate_exit_ramp_vx_target_ < narrow_gate_vx_max_) {
      narrow_gate_exit_ramp_vx_target_ = narrow_gate_vx_max_;
    }

    vx_accel_limit_enable_ = declare_parameter<bool>("vx_accel_limit_enable", true);
    vx_accel_max_mps2_ = declare_parameter<double>("vx_accel_max_mps2", 1.0);
    if (vx_accel_max_mps2_ < 0.05) {
      vx_accel_max_mps2_ = 0.05;
    }

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
      "vx_reverse_max=%.2f release_soft_stop=%d(r=%.1fm) wz_density_cap=%d gear_vx_cap=%d "
      "gear_vx_kph=[%.0f,%.0f,%.0f,%.0f] narrow_gate_vx_max=%.2f "
      "narrow_gate_exit_ramp=%d(%.1fs->%.2fm/s) vx_accel_limit=%d(%.1fm/s^2) "
      "side_pass_crawl=%.2fm/s/%.2frps forward_strips=%zu depth=%.0fmm",
      gate_enable_ ? 1 : 0, bypass_source_.c_str(),
      in_stamped_topic_.c_str(), handle_topic_.c_str(), rocker_topic_.c_str(),
      obstacle_topic_.c_str(), car_info_topic_.c_str(), out_topic_.c_str(),
      input_type_.c_str(),
      cmd_timeout_, cmd_hold_timeout_, rocker_deadband_,
      proximity_speed_cap_ ? 1 : 0, vx_reverse_max_,
      release_soft_stop_enable_ ? 1 : 0, release_soft_stop_radius_,
      wz_density_cap_enable_ ? 1 : 0, gear_vx_cap_enable_ ? 1 : 0,
      gear_vx_max_mps_.size() > 0 ? gear_vx_max_mps_[0] * 3.6 : 0.0,
      gear_vx_max_mps_.size() > 1 ? gear_vx_max_mps_[1] * 3.6 : 0.0,
      gear_vx_max_mps_.size() > 2 ? gear_vx_max_mps_[2] * 3.6 : 0.0,
      gear_vx_max_mps_.size() > 3 ? gear_vx_max_mps_[3] * 3.6 : 0.0,
      narrow_gate_vx_max_,
      narrow_gate_exit_ramp_enable_ ? 1 : 0,
      narrow_gate_exit_ramp_sec_, narrow_gate_exit_ramp_vx_target_,
      vx_accel_limit_enable_ ? 1 : 0, vx_accel_max_mps2_,
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
  }

private:
  void onNarrowGatePassMode(const Bool::SharedPtr msg)
  {
    narrow_gate_pass_active_.store(msg->data, std::memory_order_relaxed);
  }

  void onNarrowGateSidePassMode(const Bool::SharedPtr msg)
  {
    narrow_gate_side_pass_crawl_active_.store(msg->data, std::memory_order_relaxed);
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

  double applyRockerDeadbandAxis(double axis) const
  {
    return (std::abs(axis) <= rocker_deadband_) ? 0.0 : axis;
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

  void applySoftStopRamp(Twist & out, double dt)
  {
    const double lin_step = std::max(1e-6, release_soft_stop_decel_) * dt;
    const double wz_step = std::max(1e-6, release_soft_stop_wz_decel_) * dt;

    if (std::abs(soft_stop_vx_) <= release_soft_stop_vel_eps_) {
      soft_stop_vx_ = 0.0;
    } else if (soft_stop_vx_ > 0.0) {
      soft_stop_vx_ = std::max(0.0, soft_stop_vx_ - lin_step);
    } else {
      soft_stop_vx_ = std::min(0.0, soft_stop_vx_ + lin_step);
    }

    if (std::abs(soft_stop_wz_) <= release_soft_stop_vel_eps_) {
      soft_stop_wz_ = 0.0;
    } else if (soft_stop_wz_ > 0.0) {
      soft_stop_wz_ = std::max(0.0, soft_stop_wz_ - wz_step);
    } else {
      soft_stop_wz_ = std::min(0.0, soft_stop_wz_ + wz_step);
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

  void beginSoftStop()
  {
    if (!soft_stopping_) {
      soft_stop_vx_ = last_out_.linear.x;
      soft_stop_wz_ = last_out_.angular.z;
      soft_stopping_ = true;
      RCLCPP_INFO(
        get_logger(),
        "release soft-stop: v0=%.2f wz0=%.2f (clear within %.1fm)",
        soft_stop_vx_, soft_stop_wz_, release_soft_stop_radius_);
    }
  }

  void endSoftStop()
  {
    soft_stopping_ = false;
    soft_stop_vx_ = 0.0;
    soft_stop_wz_ = 0.0;
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
    bool rocker_released = false;
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

    if (have_rocker_.load(std::memory_order_relaxed)) {
      const auto rocker_age_ns =
        tnow.nanoseconds() -
        latest_rocker_stamp_ns_.load(std::memory_order_relaxed);
      rocker_stale = rocker_age_ns >
        static_cast<int64_t>(rocker_timeout_ * 1e9);
      rocker_released = !rocker_stale &&
        rocker_at_origin_.load(std::memory_order_relaxed);
      if (rocker_stale) {
        halted = true;
        reason = "joystick stale";
        endSoftStop();
      } else if (rocker_released) {
        if (canSoftStopOnRelease(cloud_ok)) {
          beginSoftStop();
        } else {
          halted = true;
          endSoftStop();
          reason = cloud_ok ?
            "joystick released (obstacles nearby)" :
            "joystick released (cloud stale)";
        }
      } else {
        endSoftStop();
      }
    }

    const bool side_pass_crawl =
      narrow_gate_side_pass_crawl_active_.load(std::memory_order_relaxed);
    if (side_pass_crawl) {
      Twist out;
      if (!halted && !soft_stopping_ && cloud_ok && rockerFresh(tnow) &&
        !rocker_released)
      {
        out = computeSidePassCrawlTwist(tnow);
      } else if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "side_pass crawl halted: %s", reason.c_str());
      } else if (!cloud_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "side_pass crawl halted: cloud stale");
      }
      out_pub_->publish(out);
      last_pub_stamp_ = tnow;
      last_out_ = out;
      return;
    }

    Twist target;
    bool input_ok = false;
    if (!soft_stopping_) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (have_input_) {
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
        } else {
          halted = true;
          reason = "cmd_vel stale";
          endSoftStop();
        }
      }
    }

    if (soft_stopping_ && (!cloud_ok ||
      nearbyObstaclePointCountInRadius(release_soft_stop_radius_) > 0))
    {
      halted = true;
      endSoftStop();
      reason = cloud_ok ?
        "soft-stop aborted (obstacles appeared)" :
        "soft-stop aborted (cloud stale)";
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

    double effective_wz_max = wz_max_;
    if (wz_density_cap_enable_ && input_ok && !halted && !soft_stopping_) {
      if (!cloud_ok) {
        current_wz_tier_ = WzDensityTier::DENSE;
      } else {
        updateWzDensityTier(nearbyObstaclePointCount(), tnow);
      }
      effective_wz_max = wzDensityCap();
    }

    Twist out;
    if (soft_stopping_ && !halted) {
      const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
      applySoftStopRamp(out, dt);
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
      if (!halted && out.linear.x < 0.0) {
        out.linear.x = std::max(out.linear.x, -vx_reverse_max_);
      }
      if (!halted) {
        out.angular.z = clamp(out.angular.z, -effective_wz_max, effective_wz_max);
      }
      applyNarrowGateRockerIntentFilter(out, tnow);
      if (!soft_stopping_) {
        const double dt = 1.0 / std::max(0.1, publish_rate_hz_);
        applyVxAccelLimit(out, dt);
      }
    } else {
      last_out_ = Twist();
      if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "cmd_vel_safety_gate halting: %s", reason.c_str());
      }
    }

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
  double release_soft_stop_radius_{1.2};
  double release_soft_stop_decel_{1.5};
  double release_soft_stop_wz_decel_{2.0};
  double release_soft_stop_vel_eps_{0.02};

  std::string narrow_gate_pass_mode_topic_;
  std::string narrow_gate_side_pass_mode_topic_;
  double narrow_gate_vx_max_{0.20};
  bool narrow_gate_rocker_intent_filter_enable_{true};
  double narrow_gate_rocker_forward_min_{5.0};
  double narrow_gate_rocker_lateral_min_{5.0};
  bool narrow_gate_exit_ramp_enable_{true};
  double narrow_gate_exit_ramp_sec_{2.0};
  double narrow_gate_exit_ramp_vx_target_{1.50};
  double side_pass_crawl_check_depth_m_{0.05};
  std::vector<double> side_pass_crawl_forward_strip_x_{0.685, 0.935, 0.685};
  std::vector<double> side_pass_crawl_forward_strip_y_min_{0.2, -0.2, -0.34};
  std::vector<double> side_pass_crawl_forward_strip_y_max_{0.34, 0.2, -0.2};
  std::vector<double> side_pass_crawl_lateral_strip_x_min_{-0.2, 0.685, 0.7};
  std::vector<double> side_pass_crawl_lateral_strip_x_max_{0.685, 0.7, 0.935};
  std::vector<double> side_pass_crawl_lateral_strip_y_edge_{0.33, 0.34, 0.20};
  double side_pass_crawl_vx_mps_{0.10};
  double side_pass_crawl_wz_rps_{0.10};
  double side_pass_crawl_rocker_forward_min_{5.0};
  double side_pass_crawl_rocker_lateral_min_{5.0};
  bool vx_accel_limit_enable_{true};
  double vx_accel_max_mps2_{1.0};
  std::atomic<bool> narrow_gate_pass_active_{false};
  std::atomic<bool> narrow_gate_side_pass_crawl_active_{false};
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
  double soft_stop_vx_{0.0};
  double soft_stop_wz_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
