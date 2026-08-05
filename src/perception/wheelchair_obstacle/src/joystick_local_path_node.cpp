// Joystick / handle intent -> short local path -> Nav2 FollowPath (MPPI).
//
// spatial_range: stick polar -> body-frame goal on concentric arcs (per gear).
//   stick_r = |rocker|/100, stick_theta = atan2(ry,rx); goal = r*R*(cos,sin)(theta).
//   Pure lateral (|rx| deadbanded) -> in-place turn via ry->wz. Moving path yaw
//   lerps toward stick bearing. Forward/back sector picks R from gear tables.
// velocity_integration: legacy /handle/control (v,w) integration.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
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

}  // namespace

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
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
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
      "gear_lateral_range_m", {-0.20, -0.40, -1.00, -1.50});
    gear_lateral_wz_max_ = declare_parameter<std::vector<double>>(
      "gear_lateral_wz_max", {0.55, 0.75, 0.95, 1.15});
    forward_sector_deg_ = declare_parameter<double>("forward_sector_deg", 300.0);
    backward_sector_deg_ = declare_parameter<double>("backward_sector_deg", 60.0);
    sector_hysteresis_deg_ = declare_parameter<double>("sector_hysteresis_deg", 10.0);
    gear_default_index_ = declare_parameter<int>("gear_default_index", 3);
    path_interpolation_steps_ = declare_parameter<int>("path_interpolation_steps", 20);
    min_goal_distance_m_ = declare_parameter<double>("min_goal_distance_m", 0.12);
    path_yaw_lateral_eps_m_ = declare_parameter<double>("path_yaw_lateral_eps_m", 0.03);
    goal_resend_xy_delta_m_ = declare_parameter<double>("goal_resend_xy_delta_m", 0.05);
    goal_resend_yaw_delta_ = declare_parameter<double>("goal_resend_yaw_delta", 0.08);

    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.30);

    obstacle_points_topic_ = declare_parameter<std::string>(
      "obstacle_points_topic", "/perception/obstacle_points");
    avoidance_assist_enabled_ = declare_parameter<bool>("avoidance_assist_enabled", false);
    assist_obstacle_timeout_ = declare_parameter<double>("assist_obstacle_timeout", 0.30);
    assist_x_min_ = declare_parameter<double>("assist_x_min", 0.35);
    assist_x_max_ = declare_parameter<double>("assist_x_max", 1.35);
    assist_y_abs_ = declare_parameter<double>("assist_y_abs", 0.75);
    assist_center_y_abs_ = declare_parameter<double>("assist_center_y_abs", 0.35);
    assist_wz_ = declare_parameter<double>("assist_wz", 0.16);
    assist_max_wz_ = declare_parameter<double>("assist_max_wz", 0.32);
    assist_smoothing_ = declare_parameter<double>("assist_smoothing", 0.75);
    assist_user_turn_override_ = declare_parameter<double>("assist_user_turn_override", 0.10);

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

    use_spatial_range_ = (path_generation_mode_ == "spatial_range");

    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, 10,
      std::bind(&JoystickLocalPathNode::onOdom, this, std::placeholders::_1));

    if (use_spatial_range_) {
      joystick_sub_ = create_subscription<Joystick>(
        joystick_topic_, 10,
        std::bind(&JoystickLocalPathNode::onJoystick, this, std::placeholders::_1));
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

    obstacle_sub_ = create_subscription<PointCloud2>(
      obstacle_points_topic_, 10,
      std::bind(&JoystickLocalPathNode::onObstaclePoints, this, std::placeholders::_1));

    path_pub_ = create_publisher<Path>(intent_path_topic_, 1);
    if (footprint_publish_enable_) {
      if (!nav2_costmap_2d::makeFootprintFromString(footprint_base_, footprint_base_pts_)) {
        RCLCPP_ERROR(get_logger(), "Invalid footprint_base: %s", footprint_base_.c_str());
        footprint_publish_enable_ = false;
      } else {
        footprint_pub_ = create_publisher<geometry_msgs::msg::Polygon>(footprint_topic_, 1);
        publishFootprintForGear(gear_default_index_);
      }
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
        "boundary=%.0fdeg steps=%d",
        joystick_topic_.c_str(), forward_sector_deg_, backward_sector_deg_,
        sector_hysteresis_deg_, 180.0 - half_back, path_interpolation_steps_);
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

  void onOdom(const Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = msg;
  }

  void onObstaclePoints(const PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    latest_obstacles_ = msg;
    latest_obstacles_stamp_ = now();
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
    if (use_spatial_range_) {
      if (!buildSpatialPath(odom, tnow, path)) {
        cancelActiveGoal();
        publishEmptyPath();
        return;
      }
    } else {
      if (!buildVelocityPath(odom, tnow, path)) {
        cancelActiveGoal();
        publishEmptyPath();
        return;
      }
    }

    path_pub_->publish(path);
    sendGoalIfChanged(path);
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

    // w_user = applyAvoidanceAssist(v_user, w_user, tnow);
    path = buildPathFromVelocity(*odom, v_user, w_user);
    return true;
  }

  bool buildSpatialPath(const Odometry::SharedPtr & odom, const rclcpp::Time & tnow, Path & path)
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

    const double rx_raw = static_cast<double>(joy->rocker_x);
    const double ry_raw = static_cast<double>(joy->rocker_y);
    const double rx = applyRockerDeadbandAxis(rx_raw);
    const double ry = applyRockerDeadbandAxis(ry_raw);
    if (std::abs(rx) < 1e-6 && std::abs(ry) < 1e-6) {
      return false;
    }

    const DriveSector sector = updateDriveSector(rx, ry);

    double max_forward = 0.0;
    double max_back = 0.0;
    double max_lateral_unused = 0.0;
    gearRanges(max_forward, max_back, max_lateral_unused);

    const double norm_x = rx / 100.0;
    const double norm_y = ry / 100.0;

    int gear = gear_default_index_;
    {
      std::lock_guard<std::mutex> lock(gear_mutex_);
      if (have_gear_ && latest_gear_ >= 1) {
        gear = latest_gear_;
      }
    }

    // Pure lateral: no forward/back on stick -> spin in place (same xy, yaw only).
    if (std::abs(rx) < 1e-6) {
      const double wz = clampNorm(norm_y, -1.0, 1.0) * gearLateralWzMax(gear);
      path = buildPathInPlaceTurn(*odom, wz);
      return true;
    }

    const double stick_r = std::min(1.0, std::hypot(norm_x, norm_y));
    const double stick_theta = std::atan2(norm_y, norm_x);
    if (stick_r < 1e-6) {
      return false;
    }

    const double R = (sector == DriveSector::Forward) ? max_forward : max_back;
    double dx_body = stick_r * std::cos(stick_theta) * R;
    double dy_body = stick_r * std::sin(stick_theta) * R;

    const double goal_dist = std::hypot(dx_body, dy_body);
    if (goal_dist < min_goal_distance_m_) {
      const double scale = min_goal_distance_m_ / goal_dist;
      dx_body *= scale;
      dy_body *= scale;
    }

    path = buildPathToBodyOffset(*odom, dx_body, dy_body, stick_theta);
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

  void publishFootprintForGear(int gear)
  {
    if (!footprint_pub_ || footprint_base_pts_.empty()) {
      return;
    }

    const double padding = gearFootprintPadding(gear);
    if (have_published_footprint_ && std::abs(padding - last_footprint_padding_) < 1e-6) {
      return;
    }

    auto pts = footprint_base_pts_;
    nav2_costmap_2d::padFootprint(pts, padding);

    geometry_msgs::msg::Polygon poly = nav2_costmap_2d::toPolygon(pts);
    footprint_pub_->publish(poly);
    last_footprint_padding_ = padding;
    have_published_footprint_ = true;

    RCLCPP_INFO(
      get_logger(), "local_costmap footprint gear=%d padding=%.0fcm",
      gear, padding * 100.0);
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

  // double applyAvoidanceAssist(double v_user, double w_user, const rclcpp::Time & tnow)
  // {
  //   if (!avoidance_assist_enabled_ || v_user <= lin_deadband_) {
  //     assist_wz_state_ *= assist_smoothing_;
  //     return w_user + assist_wz_state_;
  //   }

  //   if (std::abs(w_user) >= assist_user_turn_override_) {
  //     assist_wz_state_ *= assist_smoothing_;
  //     return w_user + assist_wz_state_;
  //   }

  //   PointCloud2::SharedPtr cloud;
  //   rclcpp::Time stamp;
  //   {
  //     std::lock_guard<std::mutex> lock(obstacle_mutex_);
  //     cloud = latest_obstacles_;
  //     stamp = latest_obstacles_stamp_;
  //   }

  //   if (!cloud || (tnow - stamp).seconds() > assist_obstacle_timeout_) {
  //     assist_wz_state_ *= assist_smoothing_;
  //     return w_user + assist_wz_state_;
  //   }

  //   double center_score = 0.0;
  //   double left_score = 0.0;
  //   double right_score = 0.0;

  //   sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  //   sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");

  //   for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
  //     const double x = *iter_x;
  //     const double y = *iter_y;

  //     if (x < assist_x_min_ || x > assist_x_max_ || std::abs(y) > assist_y_abs_) {
  //       continue;
  //     }

  //     const double weight = 1.0 / std::max(0.20, x);

  //     if (std::abs(y) <= assist_center_y_abs_) {
  //       center_score += weight;
  //     } else if (y > 0.0) {
  //       left_score += weight;
  //     } else {
  //       right_score += weight;
  //     }
  //   }

  //   double target_assist = 0.0;
  //   if (center_score > 0.0) {
  //     target_assist = (left_score <= right_score) ? assist_wz_ : -assist_wz_;
  //   }

  //   target_assist = std::clamp(target_assist, -assist_max_wz_, assist_max_wz_);
  //   assist_wz_state_ =
  //     assist_smoothing_ * assist_wz_state_ + (1.0 - assist_smoothing_) * target_assist;

  //   return w_user + assist_wz_state_;
  // }

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

    const auto & goal = path.poses.back().pose;
    last_sent_goal_x_ = goal.position.x;
    last_sent_goal_y_ = goal.position.y;
    last_sent_goal_yaw_ = yawFromQuaternion(goal.orientation);
    have_last_sent_goal_ = true;

    sendGoal(path);
  }

  void sendGoal(const Path & path)
  {
    auto goal = FollowPath::Goal();
    goal.path = path;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

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
      current_goal_handle_.reset();
    }

    action_client_->async_send_goal(goal, send_options);
  }

  void cancelActiveGoal()
  {
    have_last_sent_goal_ = false;
    drive_sector_ = DriveSector::Forward;
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!current_goal_handle_) {
      return;
    }
    action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();
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
  std::string obstacle_points_topic_;
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
  int gear_default_index_{3};
  int path_interpolation_steps_{20};
  double min_goal_distance_m_{0.12};
  double path_yaw_lateral_eps_m_{0.03};
  double goal_resend_xy_delta_m_{0.05};
  double goal_resend_yaw_delta_{0.08};
  double odom_timeout_{0.30};

  bool avoidance_assist_enabled_{false};
  double assist_obstacle_timeout_{0.30};
  double assist_x_min_{0.35};
  double assist_x_max_{1.35};
  double assist_y_abs_{0.75};
  double assist_center_y_abs_{0.35};
  double assist_wz_{0.16};
  double assist_max_wz_{0.32};
  double assist_smoothing_{0.75};
  double assist_user_turn_override_{0.10};
  double assist_wz_state_{0.0};
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

  DriveSector drive_sector_{DriveSector::Forward};

  bool have_last_sent_goal_{false};
  double last_sent_goal_x_{0.0};
  double last_sent_goal_y_{0.0};
  double last_sent_goal_yaw_{0.0};

  rclcpp::Subscription<Twist>::SharedPtr control_sub_;
  rclcpp::Subscription<Joystick>::SharedPtr joystick_sub_;
  rclcpp::Subscription<CarInfo>::SharedPtr car_info_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
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

  std::mutex obstacle_mutex_;
  PointCloud2::SharedPtr latest_obstacles_;
  rclcpp::Time latest_obstacles_stamp_{0, 0, RCL_ROS_TIME};

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
