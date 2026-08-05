// Arbitrating safety gate between the joystick intent, the Nav2 MPPI controller
// and the wheelchair CAN adapter.
//
// The wheelchair CanAdapter drives the motors from geometry_msgs/Twist on
// /nav/control, and this node is the single writer of that topic. It arbitrates
// between two velocity sources and then applies safety gating:
//
//   sources:
//     - PASSTHROUGH: /handle/control (geometry_msgs/Twist) -- the gear-scaled
//       joystick velocity straight from the chassis firmware. Used in open
//       space where no avoidance action is needed (lowest latency, exact feel).
//     - AVOID: /cmd_vel -- the Nav2 MPPI controller output. Used when an
//       obstacle is within the intended direction of travel.
//
//   arbitration (obstacle-gated, asymmetric hysteresis):
//     - obstacle distance d = nearest /perception/obstacle_points range inside a
//       sector centred on the intended travel direction (forward if the
//       joystick commands forward, rear if it commands reverse).
//     - PASSTHROUGH -> AVOID  : immediate when d < d_engage (fast to protect).
//     - AVOID -> PASSTHROUGH  : only when d > d_release AND held in AVOID for
//                               >= min_dwell (slow to relax; avoids chatter).
//     - fail-safe: if the obstacle cloud is stale/missing, or passthrough is
//       disabled, force AVOID (never passthrough on blind perception).
//
//   safety gating (applied to whichever source is selected):
//     - drops commands older than cmd_hold_timeout (AVOID: holds last /cmd_vel
//       until then to avoid zero pulses when MPPI publishes slower than 50 Hz)
//     - halts when /safety_state (std_msgs/UInt8) reports a non-zero fault code
//       (only enforced when require_safety_state is false; otherwise the gate
//       stays operational even if no safety_state producer is running)
//     - clamps linear/angular velocity to vx_max/vx_min/vy_max/wz_max
//     - clamps acceleration to ax_max/az_max using the previous accepted command
//       (this also keeps PASSTHROUGH<->AVOID switches jerk-free)
//     - publishes zero to cmd_vel_out whenever input is stale, faulted, or empty
//
// Output is always geometry_msgs/Twist on cmd_vel_out (remapped to /nav/control
// in the launch file). The current arbitration mode is published on
// <node>/mode (std_msgs/UInt8: 0=PASSTHROUGH, 1=AVOID) for debugging.
//
// gate_enable=false bypasses all arbitration and safety gating for testing;
// bypass_source selects handle (default) or cmd_vel as the direct output.

#include <algorithm>
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
#include <std_msgs/msg/u_int8.hpp>
#include <interface/msg/car_info.hpp>
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

// Smallest absolute difference between two angles, wrapped to [0, pi].
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
  using CarInfo = interface::msg::CarInfo;
  using Joystick = interface::msg::Joystick;

  enum class Mode : uint8_t { PASSTHROUGH = 0, AVOID = 1 };
  enum class WzDensityTier : uint8_t { OPEN = 0, MODERATE = 1, DENSE = 2 };

  CmdVelSafetyGateNode()
  : rclcpp::Node("cmd_vel_safety_gate_node")
  {
    in_stamped_topic_ = declare_parameter<std::string>("cmd_vel_in", "/cmd_vel");
    out_topic_ = declare_parameter<std::string>("cmd_vel_out", "/nav/control");
    handle_topic_ = declare_parameter<std::string>("handle_control_topic", "/handle/control");
    obstacle_topic_ = declare_parameter<std::string>("obstacle_points_topic", "/perception/obstacle_points");
    safety_state_topic_ = declare_parameter<std::string>("safety_state_topic", "/safety_state");
    input_type_ = declare_parameter<std::string>("input_type", "twist");
    require_safety_state_ = declare_parameter<bool>("require_safety_state", false);
    safety_fault_threshold_ = declare_parameter<int>("safety_fault_threshold", 1);

    gate_enable_ = declare_parameter<bool>("gate_enable", true);
    bypass_source_ = declare_parameter<std::string>("bypass_source", "handle");
    if (bypass_source_ != "handle" && bypass_source_ != "cmd_vel") {
      RCLCPP_WARN(
        get_logger(),
        "bypass_source='%s' invalid; use 'handle' or 'cmd_vel', defaulting to handle",
        bypass_source_.c_str());
      bypass_source_ = "handle";
    }

    // Raw rocker (interface/msg/Joystick, rocker_x/y in [-100, 100]) from
    // handle_can_adapter. Used to trigger an immediate stop the moment the
    // user releases the stick -- /handle/control lags because ODrive applies
    // an internal ramp/filter to top_input_vel (0x220).
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

    // Arbitration (obstacle-gated passthrough).
    passthrough_enable_ = declare_parameter<bool>("passthrough_enable", true);
    d_engage_ = declare_parameter<double>("d_engage", 1.2);
    d_release_ = declare_parameter<double>("d_release", 1.8);
    min_dwell_ = declare_parameter<double>("min_dwell", 0.5);
    sector_half_angle_ = declare_parameter<double>("sector_half_angle", 0.785);
    obstacle_timeout_ = declare_parameter<double>("obstacle_timeout", 0.5);
    intent_eps_ = declare_parameter<double>("intent_eps", 0.03);

    // Speed-dynamic engagement + proximity speed cap (needed for high gears).
    dynamic_d_engage_ = declare_parameter<bool>("dynamic_d_engage", true);
    front_overhang_ = declare_parameter<double>("front_overhang", 0.935);
    rear_overhang_ = declare_parameter<double>("rear_overhang", 0.2);
    standoff_ = declare_parameter<double>("standoff", 0.4);
    reaction_time_ = declare_parameter<double>("reaction_time", 0.2);
    brake_decel_ = declare_parameter<double>("brake_decel", 1.2);
    release_gap_ = declare_parameter<double>("release_gap", 0.6);
    proximity_speed_cap_ = declare_parameter<bool>("proximity_speed_cap", true);

    vx_max_ = declare_parameter<double>("vx_max", 0.50);
    vx_min_ = declare_parameter<double>("vx_min", -0.20);
    vy_max_ = declare_parameter<double>("vy_max", 0.0);
    wz_max_ = declare_parameter<double>("wz_max", 1.00);
    ax_max_ = declare_parameter<double>("ax_max", 0.60);
    ax_min_ = declare_parameter<double>("ax_min", -0.80);
    az_max_ = declare_parameter<double>("az_max", 1.20);

    // Gear-based max-speed cap on the arbitrated (MPPI) output. The current gear
    // (挡位, 1..3) is reported on /vlago/car/info (interface/msg/CarInfo). Each
    // gear caps |linear.x| so MPPI never drives faster than the selected gear
    // (default: gear1=2km/h, gear2=4km/h, gear3=6km/h).
    gear_speed_cap_enable_ = declare_parameter<bool>("gear_speed_cap_enable", true);
    gear_speed_caps_mps_ = declare_parameter<std::vector<double>>(
      "gear_speed_caps_mps", std::vector<double>{0.5556, 1.1111, 1.6667});
    gear_default_mps_ = declare_parameter<double>("gear_default_mps", 1.6667);
    car_info_topic_ = declare_parameter<std::string>("car_info_topic", "/vlago/car/info");

    // Obstacle-point tiered wz cap (open / moderate / dense) with hysteresis.
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

    if (d_release_ <= d_engage_) {
      RCLCPP_WARN(
        get_logger(),
        "d_release (%.2f) <= d_engage (%.2f); hysteresis disabled, expect mode chatter",
        d_release_, d_engage_);
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

    // Raw rocker: SensorDataQoS to match the ~50 Hz publisher in
    // handle_can_adapter, no latch so we always see the latest value.
    joystick_sub_ = create_subscription<Joystick>(
      rocker_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CmdVelSafetyGateNode::onJoystick, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CmdVelSafetyGateNode::onCloud, this, std::placeholders::_1));

    safety_sub_ = create_subscription<UInt8>(
      safety_state_topic_, rclcpp::QoS(10).transient_local(),
      std::bind(&CmdVelSafetyGateNode::onSafetyState, this, std::placeholders::_1));

    car_info_sub_ = create_subscription<CarInfo>(
      car_info_topic_, 10,
      std::bind(&CmdVelSafetyGateNode::onCarInfo, this, std::placeholders::_1));

    out_pub_ = create_publisher<Twist>(out_topic_, 10);
    mode_pub_ = create_publisher<UInt8>(std::string(get_name()) + "/mode", 10);
    wz_tier_pub_ = create_publisher<UInt8>(
      std::string(get_name()) + "/wz_density_tier", 10);

    const double period = 1.0 / std::max(0.1, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period * 1000)),
      std::bind(&CmdVelSafetyGateNode::onTimer, this));

    last_pub_stamp_ = now();
    last_mode_change_stamp_ = now();

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_safety_gate: gate_enable=%d bypass_source=%s in=%s handle=%s rocker=%s cloud=%s out=%s "
      "input_type=%s passthrough=%d d_engage=%.2f d_release=%.2f min_dwell=%.2f "
      "sector_half=%.2f cmd_timeout=%.2f cmd_hold=%.2f rocker_deadband=%.1f vx=[%.2f,%.2f] wz_max=%.2f "
      "wz_density_cap=%d",
      gate_enable_ ? 1 : 0, bypass_source_.c_str(),
      in_stamped_topic_.c_str(), handle_topic_.c_str(), rocker_topic_.c_str(),
      obstacle_topic_.c_str(), out_topic_.c_str(), input_type_.c_str(),
      passthrough_enable_ ? 1 : 0,
      d_engage_, d_release_, min_dwell_, sector_half_angle_, cmd_timeout_,
      cmd_hold_timeout_,
      rocker_deadband_, vx_min_, vx_max_, wz_max_,
      wz_density_cap_enable_ ? 1 : 0);
  }

private:
  void onTwist(const Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_twist_ = *msg;
    latest_input_stamp_ = now();
    have_input_ = true;
  }

  void onTwistStamped(const TwistStamped::SharedPtr msg)
  {
    // Fall back to reception time when the publisher leaves header.stamp unset
    // (or its clock source differs from ours): otherwise the age check would
    // treat every command as stale and the gate would latch to zero output.
    const rclcpp::Time stamp(msg->header.stamp);
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_twist_ = msg->twist;
    latest_input_stamp_ =
      (stamp.nanoseconds() == 0) ? now() : stamp;
    have_input_ = true;
  }

  void onHandle(const Twist::SharedPtr msg)
  {
    // /handle/control is an unstamped Twist, so use reception time for freshness.
    std::lock_guard<std::mutex> lock(handle_mutex_);
    latest_handle_ = *msg;
    latest_handle_stamp_ = now();
    have_handle_ = true;
  }

  void onJoystick(const Joystick::SharedPtr msg)
  {
    // Raw rocker (CAN 0x226, ~50 Hz). We only use this to detect "rocker
    // released" -- the actual commanded velocity still comes from
    // /handle/control. Keeping the read here tiny (no locks) so the ~50 Hz
    // callback never blocks the timer thread.
    const bool at_origin =
      (std::abs(msg->rocker_x) <= rocker_deadband_) &&
      (std::abs(msg->rocker_y) <= rocker_deadband_);
    rocker_at_origin_.store(at_origin, std::memory_order_relaxed);
    latest_rocker_stamp_ns_.store(now().nanoseconds(), std::memory_order_relaxed);
    have_rocker_.store(true, std::memory_order_relaxed);
  }

  void onCloud(const PointCloud2::SharedPtr msg)
  {
    // Cache the obstacle points as (x, y) in the cloud frame (base_link for the
    // merged obstacle cloud). Distance is evaluated per-cycle against the
    // current travel direction. The upstream filter keeps this cloud small.
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

  void onSafetyState(const UInt8::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(safety_mutex_);
    latest_safety_ = msg->data;
    have_safety_ = true;
  }

  void onCarInfo(const CarInfo::SharedPtr msg)
  {
    // Current gear (挡位) feedback from the chassis (~1 Hz). Drives the
    // gear-based output speed cap below.
    std::lock_guard<std::mutex> lock(gear_mutex_);
    latest_gear_ = msg->gear;
    have_gear_ = true;
  }

  // Count obstacle points within density_check_radius_ of base_link origin.
  int nearbyObstaclePointCount() const
  {
    const double r2 = density_check_radius_ * density_check_radius_;
    int count = 0;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    for (const auto & p : cloud_pts_) {
      if ((p.first * p.first + p.second * p.second) <= r2) {
        ++count;
      }
    }
    return count;
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

  // Max |linear.x| allowed by the currently selected gear. Returns +inf when the
  // cap is disabled, or when the gear is unknown / out of range and no positive
  // gear_default_mps is configured.
  double gearSpeedCap() const
  {
    if (!gear_speed_cap_enable_ || gear_speed_caps_mps_.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    int g;
    bool ok;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      g = latest_gear_;
      ok = have_gear_;
    }
    if (ok && g >= 1 && g <= static_cast<int>(gear_speed_caps_mps_.size())) {
      return gear_speed_caps_mps_[g - 1];
    }
    return (gear_default_mps_ > 0.0)
      ? gear_default_mps_ : std::numeric_limits<double>::infinity();
  }

  // Nearest obstacle range inside a sector centred on center_angle (radians,
  // base_link: 0 = forward, pi = rear). Returns +inf if no point falls in the
  // sector. dir_sign selects which half of x the travel is heading into so a
  // forward intent ignores obstacles strictly behind and vice versa.
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

  // Asymmetric-hysteresis mode decision. The threat distance d and the engage/
  // release thresholds are computed by the caller (they depend on current speed
  // when dynamic engagement is enabled).
  Mode decideMode(double d, double d_eng, double d_rel, bool cloud_ok,
                  const rclcpp::Time & tnow)
  {
    // Fail-safe: passthrough disabled or no fresh obstacle cloud -> AVOID.
    if (!passthrough_enable_ || !cloud_ok) {
      switchMode(Mode::AVOID, tnow);
      return current_mode_;
    }
    if (current_mode_ == Mode::PASSTHROUGH) {
      if (d < d_eng) {
        switchMode(Mode::AVOID, tnow);          // fast to protect
      }
    } else {  // AVOID
      const bool dwell_ok = (tnow - last_mode_change_stamp_).seconds() >= min_dwell_;
      if (d > d_rel && dwell_ok) {
        switchMode(Mode::PASSTHROUGH, tnow);     // slow to relax
      }
    }
    return current_mode_;
  }

  void switchMode(Mode m, const rclcpp::Time & tnow)
  {
    if (m != current_mode_) {
      current_mode_ = m;
      last_mode_change_stamp_ = tnow;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "arbitration mode -> %s", m == Mode::AVOID ? "AVOID" : "PASSTHROUGH");
    }
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

    // Safety state check.
    {
      std::lock_guard<std::mutex> lock(safety_mutex_);
      if (require_safety_state_ && !have_safety_) {
        halted = true;
        reason = "no safety_state received and require_safety_state=true";
      } else if (have_safety_ && latest_safety_ >= safety_fault_threshold_) {
        halted = true;
        reason = "safety_state fault";
      }
    }

    // Snapshot the joystick intent (also drives the arbitration direction).
    Twist handle;
    rclcpp::Time handle_stamp;
    bool handle_valid;
    {
      std::lock_guard<std::mutex> lock(handle_mutex_);
      handle = latest_handle_;
      handle_stamp = latest_handle_stamp_;
      handle_valid = have_handle_;
    }

    // --- Threat assessment: nearest obstacle in the intended travel direction ---
    bool cloud_ok;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_ok = have_cloud_ &&
        (tnow - latest_cloud_stamp_).seconds() <= obstacle_timeout_;
    }
    const double intent_vx = handle_valid ? handle.linear.x : 0.0;
    // Reference speed for braking: the larger of what we are currently doing and
    // what the user is asking for, so a fresh high-gear push engages early.
    const double v_ref = std::max(std::abs(last_out_.linear.x), std::abs(intent_vx));

    int dir = 0;                    // +1 forward, -1 reverse, 0 not translating
    double overhang = front_overhang_;
    double d = std::numeric_limits<double>::infinity();
    if (intent_vx > intent_eps_) {
      dir = 1; overhang = front_overhang_; d = minRangeInSector(0.0);
    } else if (intent_vx < -intent_eps_) {
      dir = -1; overhang = rear_overhang_; d = minRangeInSector(M_PI);
    }

    // Engage/release thresholds (from base_link origin). Dynamic: leave enough
    // room to brake from v_ref with the standoff margin.
    double d_eng = d_engage_, d_rel = d_release_;
    if (dynamic_d_engage_) {
      const double stop_dist =
        v_ref * reaction_time_ + (v_ref * v_ref) / (2.0 * std::max(1e-3, brake_decel_));
      d_eng = overhang + standoff_ + stop_dist;
      d_rel = d_eng + release_gap_;
    }

    const Mode mode = decideMode(d, d_eng, d_rel, cloud_ok, tnow);

    // Rocker released -> immediate stop, regardless of mode. We watch the raw
    // rocker (/sensor/joystick, CAN 0x226) instead of /handle/control because
    // ODrive applies an internal ramp/filter to top_input_vel (0x220), which
    // makes /handle/control.linear.x lag the physical stick by 100~500 ms. The
    // raw rocker reflects the user's actual intent within one CAN frame. Stale
    // or missing rocker data also triggers the halt (fail-safe).
    if (have_rocker_.load(std::memory_order_relaxed)) {
      const auto rocker_age_ns =
        tnow.nanoseconds() -
        latest_rocker_stamp_ns_.load(std::memory_order_relaxed);
      const bool rocker_stale = rocker_age_ns >
        static_cast<int64_t>(rocker_timeout_ * 1e9);
      if (rocker_stale) {
        halted = true;
        reason = "joystick stale";
      } else if (rocker_at_origin_.load(std::memory_order_relaxed)) {
        halted = true;
        reason = "joystick released (rocker at origin)";
      }
    }

    // Select the velocity source for this mode and check its freshness.
    Twist target;
    bool input_ok = false;
    if (mode == Mode::PASSTHROUGH) {
      if (handle_valid && (tnow - handle_stamp).seconds() <= cmd_timeout_) {
        target = handle;
        input_ok = true;
      } else if (handle_valid) {
        halted = true;
        reason = "handle_control stale";
      }
    } else {  // AVOID -> MPPI /cmd_vel
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
        }
      }
    }

    // --- Proximity speed cap (both modes): bound the forward/reverse speed to
    // what can still brake to a stop 'standoff' short of the nearest obstacle.
    // This is the safety net when the full braking distance exceeds the sensor
    // range at high gears -- the chair slows as the obstacle enters range. ---
    if (proximity_speed_cap_ && input_ok && !halted && dir != 0 && std::isfinite(d)) {
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

    // --- Gear-based speed cap (挡位): bound |vx| to the selected gear's max, so
    // the arbitrated (MPPI) output never exceeds e.g. gear1=2km/h, gear2=4km/h,
    // gear3=6km/h regardless of what MPPI or the joystick asks for. ---
    if (input_ok && !halted) {
      const double gcap = gearSpeedCap();
      if (std::isfinite(gcap)) {
        target.linear.x = clamp(target.linear.x, -gcap, gcap);
      }
    }

    // --- Obstacle-point tiered wz cap (hysteresis): slow in-place turns near
    // obstacles; full wz_max in open space. Reuses obstacle_points_topic. ---
    double effective_wz_max = wz_max_;
    if (wz_density_cap_enable_ && input_ok && !halted) {
      if (!cloud_ok) {
        current_wz_tier_ = WzDensityTier::DENSE;
      } else {
        updateWzDensityTier(nearbyObstaclePointCount(), tnow);
      }
      effective_wz_max = wzDensityCap();
    }

    Twist out;
    if (!halted && input_ok) {
      out = applyLimits(target, tnow, effective_wz_max);
    } else {
      // Halting: zero output, also reset last command so accel limit does not
      // fight the deceleration when commands resume.
      last_out_ = Twist();
      if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "cmd_vel_safety_gate halting: %s", reason.c_str());
      }
    }

    out_pub_->publish(out);
    last_pub_stamp_ = tnow;
    last_out_ = out;

    UInt8 mode_msg;
    mode_msg.data = static_cast<uint8_t>(mode);
    mode_pub_->publish(mode_msg);

    UInt8 tier_msg;
    tier_msg.data = static_cast<uint8_t>(current_wz_tier_);
    wz_tier_pub_->publish(tier_msg);
  }

  Twist applyLimits(const Twist & target, const rclcpp::Time & tnow, double wz_cap)
  {
    Twist out;

    const double dt = std::max(1e-3, (tnow - last_pub_stamp_).seconds());

    // Clamp to velocity envelope first.
    const double vx_des = clamp(target.linear.x, vx_min_, vx_max_);
    const double vy_des = clamp(target.linear.y, -vy_max_, vy_max_);
    const double wz_des = clamp(target.angular.z, -wz_cap, wz_cap);

    // Then clamp the per-cycle delta to the acceleration envelope, using
    // symmetric bounds derived from ax_min/ax_max and az_max.
    const double ax_lo = -std::abs(ax_min_) * dt;
    const double ax_hi = std::abs(ax_max_) * dt;
    const double az_hi = std::abs(az_max_) * dt;

    out.linear.x = last_out_.linear.x + clamp(vx_des - last_out_.linear.x, ax_lo, ax_hi);
    out.linear.y = last_out_.linear.y + clamp(vy_des - last_out_.linear.y, ax_lo, ax_hi);
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = last_out_.angular.z + clamp(wz_des - last_out_.angular.z, -az_hi, az_hi);
    return out;
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

    UInt8 mode_msg;
    mode_msg.data = static_cast<uint8_t>(Mode::PASSTHROUGH);
    mode_pub_->publish(mode_msg);
  }

  // Parameters
  std::string in_stamped_topic_;
  std::string out_topic_;
  std::string handle_topic_;
  std::string obstacle_topic_;
  std::string safety_state_topic_;
  std::string input_type_;
  std::string rocker_topic_;
  double rocker_deadband_{5.0};
  double rocker_timeout_{0.30};
  bool require_safety_state_{false};
  int safety_fault_threshold_{1};
  bool gate_enable_{true};
  std::string bypass_source_{"handle"};
  double cmd_timeout_{0.30};
  double cmd_hold_timeout_{0.50};
  double publish_rate_hz_{50.0};
  bool passthrough_enable_{true};
  double d_engage_{1.2};
  double d_release_{1.8};
  double min_dwell_{0.5};
  double sector_half_angle_{0.785};
  double obstacle_timeout_{0.5};
  double intent_eps_{0.03};
  bool dynamic_d_engage_{true};
  double front_overhang_{0.935};
  double rear_overhang_{0.2};
  double standoff_{0.4};
  double reaction_time_{0.2};
  double brake_decel_{1.2};
  double release_gap_{0.6};
  bool proximity_speed_cap_{true};
  double vx_max_{0.50};
  double vx_min_{-0.20};
  double vy_max_{0.0};
  double wz_max_{1.00};
  double ax_max_{0.60};
  double ax_min_{-0.80};
  double az_max_{1.20};
  bool gear_speed_cap_enable_{true};
  std::vector<double> gear_speed_caps_mps_{0.5556, 1.1111, 1.6667};
  double gear_default_mps_{1.6667};
  std::string car_info_topic_{"/vlago/car/info"};
  bool wz_density_cap_enable_{true};
  double density_check_radius_{1.2};
  std::vector<double> wz_density_caps_rps_{1.30, 0.785, 0.524};
  int obstacle_count_engage_moderate_{12};
  int obstacle_count_release_open_{6};
  int obstacle_count_engage_dense_{40};
  int obstacle_count_release_moderate_{28};
  double wz_density_min_dwell_{0.4};

  // Subscriptions / publisher / timer
  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<Twist>::SharedPtr handle_sub_;
  rclcpp::Subscription<Joystick>::SharedPtr joystick_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<UInt8>::SharedPtr safety_sub_;
  rclcpp::Subscription<CarInfo>::SharedPtr car_info_sub_;
  rclcpp::Publisher<Twist>::SharedPtr out_pub_;
  rclcpp::Publisher<UInt8>::SharedPtr mode_pub_;
  rclcpp::Publisher<UInt8>::SharedPtr wz_tier_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex input_mutex_;
  Twist latest_twist_;
  rclcpp::Time latest_input_stamp_{0, 0, RCL_ROS_TIME};
  bool have_input_{false};

  std::mutex handle_mutex_;
  Twist latest_handle_;
  rclcpp::Time latest_handle_stamp_{0, 0, RCL_ROS_TIME};
  bool have_handle_{false};

  // Raw rocker state. Accessed from the onJoystick callback (~50 Hz) and
  // onTimer (50 Hz); atomics + rclcpp::Time are not copy-constructible, so we
  // store the stamp's nanoseconds as int64_t to keep the load cheap.
  std::atomic<bool> rocker_at_origin_{true};     // safe default: stop
  std::atomic<int64_t> latest_rocker_stamp_ns_{0};
  std::atomic<bool> have_rocker_{false};

  mutable std::mutex cloud_mutex_;
  std::vector<std::pair<float, float>> cloud_pts_;
  rclcpp::Time latest_cloud_stamp_{0, 0, RCL_ROS_TIME};
  bool have_cloud_{false};

  std::mutex safety_mutex_;
  uint8_t latest_safety_{0};
  bool have_safety_{false};

  mutable std::mutex gear_mutex_;
  int latest_gear_{0};
  bool have_gear_{false};

  // Arbitration state (touched only from the timer thread).
  Mode current_mode_{Mode::AVOID};
  rclcpp::Time last_mode_change_stamp_{0, 0, RCL_ROS_TIME};
  WzDensityTier current_wz_tier_{WzDensityTier::OPEN};
  rclcpp::Time last_wz_tier_change_stamp_{0, 0, RCL_ROS_TIME};

  Twist last_out_;
  rclcpp::Time last_pub_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
