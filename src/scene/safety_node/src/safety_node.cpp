/**
 * @file safety_node.cpp
 * @brief SafetyNode 主节点实现
 */
#include "safety_node/safety_node.h"

#include "safety_node/core/obstacle_field.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/perception_utils.h"

#include "../../../common/utils/topic.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <limits>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <tuple>
#include <vector>

namespace
{
constexpr const char * LOG_PREFIX = "[SafetyNode]";
constexpr const char * LOCAL_MAP_VIZ_TOPIC = "/safety/local_map";
constexpr const char * PERCEPTION_POINTS_VIZ_TOPIC = "/safety/debug/perception_points";
constexpr const char * PERCEPTION_MARKERS_VIZ_TOPIC = "/safety/debug/perception_markers";
constexpr const char * INTERVENTION_LOG_TOPIC = "/safety/intervention_log";

const char * emergencyReasonToString(EmergencyStopReason reason)
{
  switch (reason) {
    case EmergencyStopReason::NONE:
      return "none";
    case EmergencyStopReason::INPUT_FAULT:
      return "input_fault";
    case EmergencyStopReason::MODE_ESTOP:
      return "mode_estop";
    case EmergencyStopReason::MAP_VETO:
      return "map_veto";
    case EmergencyStopReason::EMERGENCY_TTC:
      return "emergency_ttc";
    case EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT:
      return "braking_distance_insufficient";
    case EmergencyStopReason::HARD_STOP_DISTANCE:
      return "hard_stop_distance";
    case EmergencyStopReason::RISK_EMERGENCY:
      return "risk_emergency";
    case EmergencyStopReason::BRAKE_HOLD:
      return "brake_hold";
    case EmergencyStopReason::TILT_NEAR_DISTANCE:
      return "tilt_near_distance";
    case EmergencyStopReason::TILT_TTC:
      return "tilt_ttc";
    case EmergencyStopReason::UNKNOWN:
      return "unknown";
  }
  return "unknown";
}

const char * recoveryStateToString(RecoveryState state)
{
  switch (state) {
    case RecoveryState::IDLE:
      return "idle";
    case RecoveryState::CONFIRMING:
      return "confirming";
    case RecoveryState::ACTIVE:
      return "active";
    case RecoveryState::BLOCKED:
      return "blocked";
    case RecoveryState::ABORTED_TIMEOUT:
      return "aborted_timeout";
    case RecoveryState::ABORTED_NO_PROGRESS:
      return "aborted_no_progress";
    case RecoveryState::ABORTED_FEEDBACK_STALL:
      return "aborted_feedback_stall";
  }
  return "idle";
}

void appendNullableNumber(std::ostringstream & oss, double value)
{
  if (std::isfinite(value)) {
    oss << value;
  } else {
    oss << "null";
  }
}

const char * debugSourceName(uint8_t source_id)
{
  switch (source_id) {
    case 1:
      return "right";
    case 2:
      return "back";
    case 3:
      return "left_tilt";
    case 4:
      return "right_tilt";
    case 0:
    default:
      return "left";
  }
}

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

interface::msg::TiltObstacleScan makeTiltObstacleScanMsg(
  const rclcpp::Time & stamp,
  const TiltSourceSnapshot & snapshot)
{
  interface::msg::TiltObstacleScan msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;
  msg.source_name = snapshot.source_name;
  msg.source_id = snapshot.source_id;
  msg.filtered_points.reserve(snapshot.filtered_points.size());
  for (const auto & point : snapshot.filtered_points) {
    interface::msg::TiltObstaclePoint out;
    out.x = static_cast<float>(point.x);
    out.y = static_cast<float>(point.y);
    out.z = static_cast<float>(point.z);
    msg.filtered_points.push_back(out);
  }
  msg.primitives.reserve(snapshot.primitive_field.obstacles.size());
  double nearest_forward_edge_m = std::numeric_limits<double>::infinity();
  for (const auto & prim : snapshot.primitive_field.obstacles) {
    if (!prim.valid) {
      continue;
    }
    interface::msg::TiltObstaclePoint out;
    out.x = static_cast<float>(prim.x);
    out.y = static_cast<float>(prim.y);
    out.z = static_cast<float>(prim.z);
    msg.primitives.push_back(out);
    if (prim.x > 0.0) {
      const double edge = std::max(0.0, prim.x - std::max(0.0, prim.radius));
      nearest_forward_edge_m = std::min(nearest_forward_edge_m, edge);
    }
  }
  msg.nearest_forward_edge_m = std::isfinite(nearest_forward_edge_m) ?
    static_cast<float>(nearest_forward_edge_m) :
    std::numeric_limits<float>::quiet_NaN();
  msg.min_ttc_s = std::numeric_limits<float>::quiet_NaN();
  return msg;
}

visualization_msgs::msg::Marker makeLineStrip(
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  float red,
  float green,
  float blue)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.025;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 0.9;
  marker.frame_locked = true;
  return marker;
}
} // namespace

SafetyNode::SafetyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_node", options)
  , mode_supervisor_(SafetyMode::SAFE_ASSIST)
{
  declareAndLoadParams();
}

void SafetyNode::attachComponents()
{
  setupComponents();
  setupTimers();

  RCLCPP_INFO(
    get_logger(), "%s Footprint + LocalMap 混合避障节点已启动 (mode=%s)",
    LOG_PREFIX, ModeSupervisor::modeToString(mode_supervisor_.mode()).c_str());
}

SafetyNode::~SafetyNode()
{
  RCLCPP_INFO(get_logger(), "%s 节点关闭", LOG_PREFIX);
}

void SafetyNode::declareAndLoadParams()
{
  declare_parameter<double>("tilt_z_filter_threshold", 0.0);
  declare_parameter<double>("local_map.update_rate_hz", 10.0);
  declare_parameter<bool>("local_map.publish_viz", true);
  declare_parameter<double>("local_map.viz_rate_hz", 5.0);
  declare_parameter<bool>("debug.perception.enabled", true);
  declare_parameter<double>("debug.perception.rate_hz", 5.0);
  declare_parameter<int64_t>("debug.perception.max_points", 30000);
  declare_parameter<int64_t>("perception.max_points_per_source", 30000);
  declare_parameter<int64_t>("perception.max_pending_points", 60000);
  declare_parameter<bool>("perception.self_filter.enabled", true);
  declare_parameter<double>("perception.self_filter.x_min_m", -0.36);
  declare_parameter<double>("perception.self_filter.x_max_m", 0.95);
  declare_parameter<double>("perception.self_filter.y_abs_m", 0.36);
  declare_parameter<double>("perception.self_filter.z_min_m", -0.15);
  declare_parameter<double>("perception.self_filter.z_max_m", 0.35);
  declare_parameter<double>("perception.mount_edge_reject_radius_m", 0.05);
  declare_parameter<double>("perception.ground_reject_below_z_m", -0.06);
  declare_parameter<std::string>("perception.tilt.input_type", "pointcloud2");
  declare_parameter<bool>("perception.tilt.publish_debug_scan", true);
  declare_parameter<bool>("perception.tilt.enabled", true);

  params_ = SafetyNodeParams::loadFromNode(*this);
  mode_supervisor_.setNarrowPassageConfig(params_.narrow_passage);
  mode_supervisor_.setMode(params_.default_mode);
  output_shaper_.setEmergencyReleaseDelay(params_.emergency_release_delay_s);
  output_shaper_.setMotionLimits(
    params_.avoidance.max_recover_accel_m_s2,
    params_.avoidance.max_decel_m_s2,
    params_.avoidance.braking_max_jerk_m_s3);
  perception_viz_enabled_ = get_parameter("debug.perception.enabled").as_bool();
  perception_viz_rate_hz_ = get_parameter("debug.perception.rate_hz").as_double();
}

void SafetyNode::setupComponents()
{
  const auto host = shared_from_this();

  perception_ = std::make_unique<LidarPerception>(host);
  LocalMapConfig active_local_map = params_.local_map;
  const ModeProfile profile = mode_supervisor_.profile();
  if (profile.avoidance_overrides_enabled) {
    active_local_map.inflation_radius_m =
      std::max(0.0, profile.inflation_radius_m);
  }
  local_map_ = std::make_unique<RollingLocalMap>(active_local_map);
  controller_ = std::make_unique<AvoidanceController>(params_.avoidance, params_.map_constraint);
  vehicle_io_ = std::make_unique<VehicleIo>(host);

  // 最终控制输出：与整车导航栈约定的安全控制话题
  control_pub_ = create_publisher<geometry_msgs::msg::Twist>(nav2ControlTopic, 10);
  map_viz_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(LOCAL_MAP_VIZ_TOPIC, 1);
  if (perception_viz_enabled_) {
    perception_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      PERCEPTION_POINTS_VIZ_TOPIC, rclcpp::SensorDataQoS());
    perception_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      PERCEPTION_MARKERS_VIZ_TOPIC, 1);
    if (params_.tilt_perception.publish_debug_scan && params_.tilt_perception.enabled) {
      tilt_left_debug_pub_ = create_publisher<interface::msg::TiltObstacleScan>(
        leftTiltDebugScanTopic, 1);
      tilt_right_debug_pub_ = create_publisher<interface::msg::TiltObstacleScan>(
        rightTiltDebugScanTopic, 1);
    }
  }
  log_pub_ = create_publisher<std_msgs::msg::String>(INTERVENTION_LOG_TOPIC, 10);

  last_control_time_ = now();
}

void SafetyNode::setupTimers()
{
  // ── 控制环：默认 20 Hz ──
  const double control_hz = std::max(1.0, params_.control_rate_hz);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_hz)),
    [this]() {onControlTick();});

  // ── LocalMap 融合：默认 10 Hz ──
  if (params_.local_map.enabled) {
    double map_hz = 10.0;
    if (has_parameter("local_map.update_rate_hz")) {
      map_hz = get_parameter("local_map.update_rate_hz").as_double();
    }
    map_update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, map_hz))),
      [this]() {onLocalMapUpdate();});
  }

  // ── 地图可视化：默认 5 Hz（低优先级） ──
  const bool publish_viz = get_parameter("local_map.publish_viz").as_bool();
  if (publish_viz && params_.local_map.enabled) {
    const double viz_hz = get_parameter("local_map.viz_rate_hz").as_double();
    map_viz_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, viz_hz))),
      [this]() {onMapVizTick();});
  }

  if (perception_viz_enabled_) {
    perception_viz_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / std::max(1.0, perception_viz_rate_hz_))),
      [this]() {onPerceptionVizTick();});
  }
}

void SafetyNode::onLocalMapUpdate()
{
  const ModeProfile profile = mode_supervisor_.profile();
  if (!profile.local_map_update_enabled || !params_.local_map.enabled) {
    return;
  }
  const AvoidanceParams active_avoidance =
    applyModeProfile(params_.avoidance, profile);

  const rclcpp::Time stamp = now();

  // 取出雷达回调累积的本周期点云，落格到 RollingLocalMap
  auto points = perception_->takePendingPoints();
  local_map_->integratePoints(points, stamp);

  // 可选：将 ObstacleField primitive 中心作为弱观测写入地图（V1.5）
  if (params_.local_map.integrate_primitive_centers) {
    const ObstacleField field = perception_->buildObstacleField(active_avoidance, stamp);
    local_map_->integratePrimitives(field, params_.local_map.primitive_observation_weight);
  }

  // 衰减、过期清除、膨胀，更新 grid 快照
  local_map_->finalizeFrame(stamp);
}

void SafetyNode::onMapVizTick()
{
  if (map_viz_pub_) {
    map_viz_pub_->publish(local_map_->toOccupancyGridMsg(now()));
  }
}

void SafetyNode::onPerceptionVizTick()
{
  if (!perception_points_pub_ || !perception_markers_pub_) {
    return;
  }

  const rclcpp::Time stamp = now();
  const AvoidanceParams active_avoidance =
    applyModeProfile(params_.avoidance, mode_supervisor_.profile());
  const auto debug_points = perception_->takeDebugPoints();

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = stamp;
  cloud_msg.height = 1;
  cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(debug_points.size());

  std::vector<ObstaclePoint> obstacle_input;
  obstacle_input.reserve(debug_points.size());
  if (!debug_points.empty()) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    for (const auto & point : debug_points) {
      const auto color = PerceptionUtils::debugColor(point.source_id);
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      *iter_r = color[0];
      *iter_g = color[1];
      *iter_b = color[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
      ObstaclePoint obstacle_point;
      obstacle_point.x = point.x;
      obstacle_point.y = point.y;
      obstacle_point.z = point.z;
      obstacle_point.source = debugSourceName(point.source_id);
      obstacle_input.push_back(obstacle_point);
    }
  }
  perception_points_pub_->publish(cloud_msg);

  const ObstacleField debug_field = ObstacleFieldBuilder::buildFromPoints(
    obstacle_input,
    stamp,
    active_avoidance,
    0.12,
    active_avoidance.max_obstacle_primitives);

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = "base_link";
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  auto roi = makeLineStrip(stamp, "perception_roi", 0, 1.0f, 0.85f, 0.1f);
  roi.points = {
    makePoint(-2.5, -1.2, 0.02),
    makePoint(5.0, -1.2, 0.02),
    makePoint(5.0, 1.2, 0.02),
    makePoint(-2.5, 1.2, 0.02),
    makePoint(-2.5, -1.2, 0.02)};
  markers.markers.push_back(roi);

  auto appendFootprintPolygon = [&](
    const FootprintPolygon & polygon,
    const std::string & ns,
    int id,
    float red,
    float green,
    float blue,
    float z,
    float line_width)
    {
      auto marker = makeLineStrip(stamp, ns, id, red, green, blue);
      marker.scale.x = line_width;
      for (const auto & vertex : polygon.vertices) {
        marker.points.push_back(makePoint(vertex.first, vertex.second, z));
      }
      if (!polygon.vertices.empty()) {
        marker.points.push_back(
          makePoint(polygon.vertices.front().first, polygon.vertices.front().second, z));
      }
      markers.markers.push_back(marker);
    };

  const VehicleFootprint footprint(active_avoidance);
  appendFootprintPolygon(
    footprint.inflatedPolygon(), "vehicle_footprint_margin", 0,
    0.25f, 0.85f, 1.0f, 0.03f, 0.025f);
  appendFootprintPolygon(
    footprint.physicalPolygon(), "vehicle_footprint_physical", 0,
    0.1f, 0.55f, 1.0f, 0.05f, 0.035f);

  visualization_msgs::msg::Marker horizontal_obstacles;
  horizontal_obstacles.header.frame_id = "base_link";
  horizontal_obstacles.header.stamp = stamp;
  horizontal_obstacles.ns = "horizontal_obstacle_primitives";
  horizontal_obstacles.id = 2;
  horizontal_obstacles.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  horizontal_obstacles.action = visualization_msgs::msg::Marker::ADD;
  horizontal_obstacles.pose.orientation.w = 1.0;
  const double diameter = 2.0 * std::max(
    active_avoidance.footprint_obstacle_radius_default, 0.01);
  horizontal_obstacles.scale.x = diameter;
  horizontal_obstacles.scale.y = diameter;
  horizontal_obstacles.scale.z = 0.10;
  horizontal_obstacles.color.r = 1.0;
  horizontal_obstacles.color.g = 0.15;
  horizontal_obstacles.color.b = 0.05;
  horizontal_obstacles.color.a = 0.75;
  horizontal_obstacles.frame_locked = true;
  horizontal_obstacles.points.reserve(debug_field.obstacles.size());

  visualization_msgs::msg::Marker tilt_obstacles = horizontal_obstacles;
  tilt_obstacles.ns = "tilt_obstacle_primitives";
  tilt_obstacles.id = 4;
  tilt_obstacles.color.r = 0.55;
  tilt_obstacles.color.g = 0.15;
  tilt_obstacles.color.b = 1.0;
  tilt_obstacles.points.clear();
  tilt_obstacles.points.reserve(debug_field.obstacles.size());

  for (const auto & obstacle : debug_field.obstacles) {
    const bool tilt =
      obstacle.source == "left_tilt" || obstacle.source == "right_tilt";
    auto & target = tilt ? tilt_obstacles : horizontal_obstacles;
    target.points.push_back(makePoint(obstacle.x, obstacle.y, 0.05));
  }
  markers.markers.push_back(horizontal_obstacles);
  markers.markers.push_back(tilt_obstacles);

  auto trajectory = makeLineStrip(
    stamp, "published_command_trajectory", 3, 0.2f, 1.0f, 0.25f);
  trajectory.scale.x = 0.05;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double elapsed = 0.0;
  const double dt = std::max(0.02, params_.avoidance.rollout_dt_s);
  const double horizon = std::max(dt, active_avoidance.rollout_horizon_s);
  trajectory.points.push_back(makePoint(x, y, 0.06));
  while (elapsed < horizon - 1e-9) {
    x += prev_output_.linear.x * std::cos(theta) * dt;
    y += prev_output_.linear.x * std::sin(theta) * dt;
    theta += prev_output_.angular.z * dt;
    elapsed += dt;
    trajectory.points.push_back(makePoint(x, y, 0.06));
  }
  markers.markers.push_back(trajectory);

  perception_markers_pub_->publish(markers);

  if (tilt_left_debug_pub_ && tilt_right_debug_pub_ &&
    params_.tilt_perception.publish_debug_scan && params_.tilt_perception.enabled)
  {
    const auto snapshots = perception_->snapshotTiltSources(active_avoidance, stamp);
    for (const auto & snapshot : snapshots) {
      const auto msg = makeTiltObstacleScanMsg(stamp, snapshot);
      if (snapshot.source_name == "left_tilt") {
        tilt_left_debug_pub_->publish(msg);
      } else if (snapshot.source_name == "right_tilt") {
        tilt_right_debug_pub_->publish(msg);
      }
    }
  }
}

void SafetyNode::onControlTick()
{
  const rclcpp::Time now_time = now();
  double dt_s = (now_time - last_control_time_).seconds();
  // 防止时钟跳变或首次 tick 导致 dt 异常
  if (dt_s <= 0.0 || dt_s > 0.5) {
    dt_s = 1.0 / std::max(1.0, params_.control_rate_hz);
  }
  last_control_time_ = now_time;

  // ── 1. 读输入 ──
  const ControlCommand control = vehicle_io_->desiredControl(
    now_time,
    params_.control_command_timeout_s,
    params_.joystick_timeout_s,
    params_.joystick_deadzone);
  const WheelFeedback wheels = vehicle_io_->wheelFeedback(
    now_time, params_.wheel_feedback_timeout_s);
  const geometry_msgs::msg::Twist desired = control.twist;
  const bool desired_finite =
    std::isfinite(desired.linear.x) && std::isfinite(desired.angular.z);
  const bool wheel_values_finite =
    std::isfinite(wheels.linear_velocity) &&
    std::isfinite(wheels.angular_velocity);
  const double fb_linear = wheel_values_finite ? wheels.linear_velocity : 0.0;
  const double fb_angular = wheel_values_finite ? wheels.angular_velocity : 0.0;
  const ModeProfile profile = mode_supervisor_.profile();
  const AvoidanceParams active_avoidance =
    applyModeProfile(params_.avoidance, profile);
  const PerceptionHealth perception_health =
    perception_->health(now_time, params_.perception_timeout_s);

  AvoidanceContext ctx;
  ctx.cruise_active = cruise_active_.load();
  ctx.cruise_linear_cap = cruise_linear_cap_;
  ctx.safety_mode = mode_supervisor_.mode();

  // ── 2. 构建 ObstacleField（Footprint 快路径） ──
  const ObstacleField field = profile.obstacle_field_enabled ?
    perception_->buildHorizontalObstacleField(active_avoidance, now_time) :
    ObstacleField{};
  const ObstacleField tilt_field =
    (profile.obstacle_field_enabled && params_.tilt_perception.enabled) ?
    perception_->buildTiltObstacleField(active_avoidance, now_time) :
    ObstacleField{};

  // ── 3. LocalMap 指针（Map veto 用，透传模式可为 nullptr） ──
  const RollingLocalMapGrid * map_ptr =
    (profile.local_map_update_enabled && params_.local_map.enabled) ?
    &local_map_->grid() :
    nullptr;

  // ── 4. 混合避障决策：Footprint 选速 → Map 裁判 → 急停兜底 ──
  const bool input_fault =
    !control.fresh ||
    !desired_finite ||
    (params_.require_wheel_feedback && !wheels.fresh) ||
    (wheels.received && !wheel_values_finite) ||
    (profile.obstacle_field_enabled && params_.require_perception &&
    perception_health.fresh_horizontal_source_count <
    static_cast<size_t>(std::max(1, params_.min_fresh_horizontal_sources))) ||
    (profile.obstacle_field_enabled && params_.require_tilt_perception &&
    params_.tilt_perception.enabled &&
    perception_health.fresh_tilt_source_count == 0U);
  AvoidanceResult result;
  if (input_fault) {
    result.safe_twist = geometry_msgs::msg::Twist{};
    result.emergency_brake = true;
    result.emergency_reason = EmergencyStopReason::INPUT_FAULT;
    result.risk.has_collision = true;
    result.risk.emergency = true;
    result.hybrid_log.mode = ctx.safety_mode;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%s input timeout: control_fresh=%s wheel_fresh=%s perception_fresh=%zu",
      LOG_PREFIX,
      control.fresh ? "true" : "false",
      wheels.fresh ? "true" : "false",
      perception_health.fresh_horizontal_source_count);
  } else {
    result = controller_->evaluate(
      desired,
      fb_linear,
      fb_angular,
      field,
      map_ptr,
      profile,
      prev_output_.linear.x,
      prev_output_.angular.z,
      dt_s,
      ctx,
      &tilt_field);
  }

  // ── 5. 输出斜率限制后发布 ──
  const bool hard_emergency =
    result.emergency_reason == EmergencyStopReason::MODE_ESTOP ||
    result.emergency_reason == EmergencyStopReason::INPUT_FAULT;
  geometry_msgs::msg::Twist shaped;
  if (control.driver_neutral && !result.emergency_brake) {
    // 原始摇杆回中优先于 /handle/control 的缓慢衰减，但使用一阶低通向零收敛，
    // 避免单周期硬清零带来的明显顿挫。
    shaped = output_shaper_.lowPassToZero(
      dt_s, params_.neutral_stop_filter_time_constant_s);
  } else {
    shaped = output_shaper_.shape(
      result.safe_twist, dt_s, result.emergency_brake, result.recovery_active, hard_emergency);
  }
  if (!std::isfinite(shaped.linear.x) || !std::isfinite(shaped.angular.z)) {
    shaped = geometry_msgs::msg::Twist{};
    output_shaper_.reset();
  }
  shaped.linear.y = 0.0;
  shaped.linear.z = 0.0;
  shaped.angular.x = 0.0;
  shaped.angular.y = 0.0;
  prev_output_ = shaped;
  publishControl(shaped);
  publishInterventionLog(result);
}

void SafetyNode::publishControl(const geometry_msgs::msg::Twist & cmd)
{
  if (control_pub_) {
    control_pub_->publish(cmd);
  }
}

void SafetyNode::publishInterventionLog(const AvoidanceResult & result)
{
  if (!log_pub_) {
    return;
  }
  // 无干预时不刷屏
  if (!result.hybrid_log.map_veto_applied && !result.emergency_brake &&
    !result.recovery_active &&
    !result.risk.braking_limited &&
    result.hybrid_log.tilt_intervention == "none" &&
    result.hybrid_log.footprint_selected_candidate == result.hybrid_log.final_candidate)
  {
    return;
  }

  std::ostringstream oss;
  oss << "{"
      << "\"mode\":\"" << ModeSupervisor::modeToString(result.hybrid_log.mode) << "\","
      << "\"footprint_selected\":" << result.hybrid_log.footprint_selected_candidate << ","
      << "\"final_candidate\":" << result.hybrid_log.final_candidate << ","
      << "\"map_veto\":" << (result.hybrid_log.map_veto_applied ? "true" : "false") << ","
      << "\"map_ttc\":";
  appendNullableNumber(oss, result.hybrid_log.map_ttc);
  oss << ","
      << "\"emergency_stop\":" << (result.emergency_brake ? "true" : "false") << ","
      << "\"recovery_active\":" << (result.recovery_active ? "true" : "false") << ","
      << "\"recovery_state\":\"" << recoveryStateToString(result.recovery_state) << "\","
      << "\"recovery_elapsed_s\":" << result.recovery_elapsed_s << ","
      << "\"recovery_clearance_gain_m\":" << result.recovery_clearance_gain_m << ","
      << "\"recovery_linear_x\":" << result.safe_twist.linear.x << ","
      << "\"recovery_angular_z\":" << result.safe_twist.angular.z << ","
      << "\"emergency_reason\":\"" << emergencyReasonToString(result.emergency_reason) << "\","
      << "\"has_collision\":" << (result.risk.has_collision ? "true" : "false") << ","
      << "\"braking_limited\":" << (result.risk.braking_limited ? "true" : "false") << ","
      << "\"braking_available_distance_m\":";
  appendNullableNumber(oss, result.risk.braking_available_distance_m);
  oss << ","
      << "\"braking_speed_limit_m_s\":";
  appendNullableNumber(oss, result.risk.braking_speed_limit_m_s);
  oss << ","
      << "\"braking_stopping_distance_m\":" << result.risk.braking_stopping_distance_m << ","
      << "\"min_ttc\":";
  appendNullableNumber(oss, result.risk.min_ttc);
  oss << ","
      << "\"min_clearance\":";
  appendNullableNumber(oss, result.risk.min_clearance);
  oss << ","
      << "\"tilt_intervention\":\"" << result.hybrid_log.tilt_intervention << "\","
      << "\"tilt_source\":\"" << result.hybrid_log.tilt_collision_source << "\","
      << "\"tilt_min_distance_m\":";
  appendNullableNumber(oss, result.hybrid_log.tilt_min_distance_m);
  oss << ","
      << "\"tilt_min_ttc\":";
  appendNullableNumber(oss, result.hybrid_log.tilt_min_ttc);
  oss << ","
      << "\"tilt_speed_limit_m_s\":";
  appendNullableNumber(oss, result.hybrid_log.tilt_speed_limit_m_s);
  oss << ","
      << "\"tick_ms\":" << result.hybrid_log.tick_ms
      << "}";

  std_msgs::msg::String msg;
  msg.data = oss.str();
  log_pub_->publish(msg);
}

void SafetyNode::resetPipelineState()
{
  local_map_->reset();
  output_shaper_.reset();
  perception_->clear();
  prev_output_ = geometry_msgs::msg::Twist{};
}
