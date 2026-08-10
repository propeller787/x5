// Safety gate between Nav2 MPPI (/cmd_vel) and the wheelchair CAN adapter (/nav/control).
//
// Always forwards MPPI output (with optional gate_enable bypass for testing).
// Retained safety layers:
//   - cmd_hold_timeout: hold last /cmd_vel when MPPI is slower than publish_rate_hz
//   - joystick release / stale -> stop (hard if obstacles nearby or cloud stale;
//     soft ramp when clear within release_soft_stop_radius)
//   - proximity speed cap from obstacle cloud + travel intent
//   - tiered wz cap from nearby obstacle-point density
//   - narrow gate pass mode (/mppi/narrow_gate_pass_mode): vx cap; rocker-intent filter
//     (forward -> no reverse, backward -> no forward; lateral -> no opposite turn); stop on release

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

    cloud_sub_ = create_subscription<PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CmdVelSafetyGateNode::onCloud, this, std::placeholders::_1));

    narrow_gate_pass_mode_sub_ = create_subscription<Bool>(
      narrow_gate_pass_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onNarrowGatePassMode, this, std::placeholders::_1));

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
      "cmd_vel_safety_gate: gate_enable=%d bypass_source=%s in=%s handle=%s rocker=%s cloud=%s out=%s "
      "input_type=%s cmd_timeout=%.2f cmd_hold=%.2f rocker_deadband=%.1f proximity_cap=%d "
      "vx_reverse_max=%.2f release_soft_stop=%d(r=%.1fm) wz_density_cap=%d narrow_gate_vx_max=%.2f",
      gate_enable_ ? 1 : 0, bypass_source_.c_str(),
      in_stamped_topic_.c_str(), handle_topic_.c_str(), rocker_topic_.c_str(),
      obstacle_topic_.c_str(), out_topic_.c_str(), input_type_.c_str(),
      cmd_timeout_, cmd_hold_timeout_, rocker_deadband_,
      proximity_speed_cap_ ? 1 : 0, vx_reverse_max_,
      release_soft_stop_enable_ ? 1 : 0, release_soft_stop_radius_,
      wz_density_cap_enable_ ? 1 : 0, narrow_gate_vx_max_);
  }

private:
  void onNarrowGatePassMode(const Bool::SharedPtr msg)
  {
    narrow_gate_pass_active_.store(msg->data, std::memory_order_relaxed);
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
    } else if (!halted && input_ok) {
      out = target;
      if (narrow_gate_pass) {
        if (out.linear.x > 0.0) {
          out.linear.x = std::min(out.linear.x, narrow_gate_vx_max_);
        } else if (out.linear.x < 0.0) {
          out.linear.x = std::max(out.linear.x, -narrow_gate_vx_max_);
        }
      }
      if (!halted && out.linear.x < 0.0) {
        out.linear.x = std::max(out.linear.x, -vx_reverse_max_);
      }
      if (!halted) {
        out.angular.z = clamp(out.angular.z, -effective_wz_max, effective_wz_max);
      }
      applyNarrowGateRockerIntentFilter(out, tnow);
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
  double narrow_gate_vx_max_{0.20};
  bool narrow_gate_rocker_intent_filter_enable_{true};
  double narrow_gate_rocker_forward_min_{5.0};
  double narrow_gate_rocker_lateral_min_{5.0};
  std::atomic<bool> narrow_gate_pass_active_{false};

  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<Twist>::SharedPtr handle_sub_;
  rclcpp::Subscription<Joystick>::SharedPtr joystick_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<Bool>::SharedPtr narrow_gate_pass_mode_sub_;
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
