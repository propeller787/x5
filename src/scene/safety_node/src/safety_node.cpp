/**
 * @file safety_node.cpp
 * @brief SafetyNode 主节点实现
 */
#include "safety_node/safety_node.h"

#include "safety_node/core/obstacle_field.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/perception_utils.h"
#include "safety_node/joystick_utils.h"

#include "../../../common/utils/topic.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <limits>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace
{
constexpr const char * LOG_PREFIX = "[SafetyNode]";
constexpr const char * LOCAL_MAP_VIZ_TOPIC = "/safety/local_map";
constexpr const char * PERCEPTION_POINTS_VIZ_TOPIC = "/safety/debug/perception_points";
constexpr const char * PERCEPTION_MARKERS_VIZ_TOPIC = "/safety/debug/perception_markers";
constexpr const char * TILT_LEFT_POINTS_VIZ_TOPIC = "/safety/debug/tilt_left/points";
constexpr const char * TILT_RIGHT_POINTS_VIZ_TOPIC = "/safety/debug/tilt_right/points";
constexpr const char * INTERVENTION_LOG_TOPIC = "/safety/intervention_log";

/**
 * /vlago/cruise/status 状态码。与 scene_node 及 tft_manager 的既有约定一致，
 * 消费端（physical_button_handler / service）按这四个值做弹窗与语音，不可改数值。
 */
enum CruiseStatusCode : int32_t
{
  CRUISE_END_NO_FORWARD_SPEED = 1,   ///< 启动窗口内未给出前进速度 → 巡航退出
  CRUISE_END_OBSTACLE_STOP = 2,      ///< 巡航中被障碍物长时间压停 → 巡航退出
  CRUISE_ACTIVATED = 3,              ///< 巡航激活成功（开关置 ON）
  CRUISE_STARTED = 4,                ///< 检测到前进速度，巡航开启成功
};

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
    case EmergencyStopReason::STANDSTILL_GUARD:
      return "standstill_guard";
    case EmergencyStopReason::SUDDEN_OBSTACLE:
      return "sudden_obstacle";
    case EmergencyStopReason::BRAKE_SATURATED:
      return "brake_saturated";
    case EmergencyStopReason::UNKNOWN:
      return "unknown";
  }
  return "unknown";
}

/**
 * 是否绕过 OutputShaper 斜率限制，直接硬闸。
 * 斜雷达近距 emergency 需快速置零，避免平滑延迟导致侵入加深。
 */
bool bypassOutputSmoothing(EmergencyStopReason reason)
{
  switch (reason) {
    case EmergencyStopReason::INPUT_FAULT:
    case EmergencyStopReason::MODE_ESTOP:
    case EmergencyStopReason::TILT_NEAR_DISTANCE:
    case EmergencyStopReason::TILT_TTC:
    // P0-B 停稳看护：障碍逼近，任何残余蠕行须立即置零，绕过整形。
    case EmergencyStopReason::STANDSTILL_GUARD:
    // L0-EB 突发障碍：绕过 max_decel 限幅，命令单周期归零，由底盘全力制动。
    case EmergencyStopReason::SUDDEN_OBSTACLE:
    case EmergencyStopReason::BRAKE_SATURATED:
      return true;
    default:
      return false;
  }
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
    case RecoveryState::DIRECTIONAL_ESCAPE:
      return "directional_escape";
    case RecoveryState::PINNED_ESCAPE:
      return "pinned_escape";
    case RecoveryState::ROTATION_ESCAPE:
      return "rotation_escape";
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

/** km/h → m/s */
constexpr double kmhToMs(double kmh) {return kmh / 3.6;}

/**
 * 按 CarInfo 档位与驾驶模式计算最大线速度 (m/s)，与 scene_node 巡航限速一致。
 * mode_type：1 舒适 2 运动。gear：1..4，其余按 1 档保守处理。
 */
double gearMaxLinearSpeedMs(int gear, int mode_type)
{
  const bool sport = (mode_type == 2);
  switch (gear) {
    case 1:
      return sport ? kmhToMs(2.0) : kmhToMs(1.5);
    case 2:
      return kmhToMs(3.0);
    case 3:
      return sport ? kmhToMs(6.0) : kmhToMs(4.5);
    case 4:
      return sport ? kmhToMs(8.0) : kmhToMs(6.0);
    default:
      return sport ? kmhToMs(2.0) : kmhToMs(1.5);
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

sensor_msgs::msg::PointCloud2 makeTiltDebugPointCloud2(
  const rclcpp::Time & stamp,
  const std::vector<ObstaclePoint> & points,
  uint8_t red,
  uint8_t green,
  uint8_t blue)
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "base_link";
  cloud_msg.header.stamp = stamp;
  cloud_msg.height = 1;
  cloud_msg.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(points.size());

  if (points.empty()) {
    return cloud_msg;
  }

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

  for (const auto & point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    *iter_r = red;
    *iter_g = green;
    *iter_b = blue;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
  return cloud_msg;
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
  setupModeServices();

  // 安全开关：生产默认关，再从统一用户配置 safety_warning_enabled 恢复（无文件/字段则保持关）。
  // 巡航启动值仍由 mode.default 映射：cruise → 开，其余 → 关。
  // 注意：setSafetyEnabled 会 setMode(SAFE_ASSIST/PASS_THROUGH) 覆写 supervisor 模式，
  // 故 mode.default 必须在其之前捕获，否则巡航默认（cruise）永远读不到。
  const SafetyMode dm = mode_supervisor_.mode();
  loadAuxiliaryModeState();
  setSafetyEnabled(safety_enabled_.load(std::memory_order_acquire));
  setCruiseEnabled(dm == SafetyMode::CRUISE);
  // 启动即发一帧，消费端无需等到首个心跳周期（参照 scene_node）。
  // setSafetyEnabled 只在跳变时发，而启动值与默认值相同时不算跳变，故此处必须补发。
  publishAuxiliaryStatus();

  RCLCPP_INFO(
    get_logger(), "%s 混合避障节点已启动 (安全=%s 标准=%s 巡航=%s 控制权=%s)", LOG_PREFIX,
    safety_enabled_.load() ? "on" : "off",
    standard_mode_enabled_.load() ? "on" : "off",
    cruise_enabled_.load() ? "on" : "off",
    hasControlAuthority() ? "on" : "off");
  logStartupConfigSummary();
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
  declare_parameter<bool>("debug.perception.joystick_trajectory.enabled", true);
  declare_parameter<double>("debug.perception.joystick_trajectory.max_linear_m_s", 2.22);
  declare_parameter<double>("debug.perception.joystick_trajectory.max_angular_rad_s", 0.8);
  declare_parameter<int64_t>("debug.perception.max_points", 30000);
  declare_parameter<bool>("debug.speed_intervention.enabled", true);
  declare_parameter<double>("debug.speed_intervention.min_delta_m_s", 0.03);
  declare_parameter<int64_t>("debug.speed_intervention.throttle_ms", 200);
  declare_parameter<int64_t>("perception.max_points_per_source", 30000);
  declare_parameter<int64_t>("perception.max_pending_points", 60000);
  declare_parameter<bool>("perception.self_filter.enabled", true);
  declare_parameter<double>("perception.self_filter.x_min_m", -0.36);
  declare_parameter<double>("perception.self_filter.x_max_m", 0.95);
  declare_parameter<double>("perception.self_filter.y_abs_m", 0.36);
  declare_parameter<double>("perception.self_filter.z_min_m", -0.15);
  declare_parameter<double>("perception.self_filter.z_max_m", 0.35);
  declare_parameter<double>("perception.self_filter.side_margin_m", 0.02);
  declare_parameter<double>("perception.mount_edge_reject_radius_m", 0.05);
  declare_parameter<double>("perception.ground_reject_below_z_m", -0.06);
  declare_parameter<double>("perception.roi.x_min_m", -2.5);
  declare_parameter<double>("perception.roi.x_max_m", 6.5);
  declare_parameter<double>("perception.roi.y_min_m", -1.8);
  declare_parameter<double>("perception.roi.y_max_m", 1.8);
  declare_parameter<bool>("perception.horizontal.enabled", true);
  declare_parameter<bool>("perception.horizontal.left_enabled", true);
  declare_parameter<bool>("perception.horizontal.right_enabled", true);
  declare_parameter<bool>("perception.horizontal.back_enabled", true);
  declare_parameter<bool>("perception.horizontal.feed_local_map", true);
  declare_parameter<std::string>("perception.tilt.coordinate_frame", "sensor");
  declare_parameter<bool>("perception.tilt.publish_debug_scan", true);
  declare_parameter<bool>("perception.tilt.enabled", true);
  // 与 scene_node 同源：TFT 维护的统一用户配置；空路径跳过启动恢复（单测）。
  declare_parameter<std::string>("user_config_file", "/opt/freego/user_config.yaml");

  params_ = SafetyNodeParams::loadFromNode(*this);
  user_config_file_path_ = get_parameter("user_config_file").as_string();
  // 方案 9：启动期校验（范围 + 跨参数一致性）。FATAL 抛异常拒启（fail-fast），
  // WARN 逐条打日志。把 yaml 注释里的约束变成代码强制。
  validateParamsOrThrow(params_, get_logger());
  mode_supervisor_.setNarrowPassageConfig(params_.narrow_passage);
  mode_supervisor_.setMode(params_.default_mode);
  output_shaper_.setEmergencyReleaseDelay(params_.emergency_release_delay_s);
  output_shaper_.setMotionLimits(
    params_.avoidance.max_recover_accel_m_s2,
    params_.avoidance.max_decel_m_s2,
    params_.avoidance.braking_max_jerk_m_s3);
  output_shaper_.setAngularSlewRate(
    params_.avoidance.max_angular_recover_rad_s2,
    params_.avoidance.max_angular_decel_rad_s2);
  front_lowpass_gate_.updateParams(params_.front_obstacle_lowpass_stop);
  front_lowpass_tilt_tracker_.updateParams(params_.front_obstacle_lowpass_stop);
  front_tilt_speed_cap_gate_.updateParams(params_.avoidance.front_tilt_speed_cap);
  perception_viz_enabled_ = get_parameter("debug.perception.enabled").as_bool();
  perception_viz_rate_hz_ = get_parameter("debug.perception.rate_hz").as_double();
  joystick_trajectory_viz_enabled_ =
    get_parameter("debug.perception.joystick_trajectory.enabled").as_bool();
  speed_intervention_debug_enabled_ =
    get_parameter("debug.speed_intervention.enabled").as_bool();
  speed_intervention_debug_min_delta_m_s_ = std::max(
    0.0, get_parameter("debug.speed_intervention.min_delta_m_s").as_double());
  speed_intervention_debug_throttle_ms_ = std::max<int64_t>(
    1, get_parameter("debug.speed_intervention.throttle_ms").as_int());
  joystick_trajectory_mapping_.deadzone = params_.joystick_deadzone;
  joystick_trajectory_mapping_.max_linear_m_s = std::max(
    0.0, get_parameter("debug.perception.joystick_trajectory.max_linear_m_s").as_double());
  joystick_trajectory_mapping_.max_angular_rad_s = std::max(
    0.0, get_parameter("debug.perception.joystick_trajectory.max_angular_rad_s").as_double());
}

void SafetyNode::setupComponents()
{
  const auto host = shared_from_this();

  // 方案 7：三个互斥回调组。单线程执行器下分组无害（行为不变），
  // 多线程执行器（runtime.multithreaded_executor=true）下隔离控制环。
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  perception_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  background_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  perception_ = std::make_unique<LidarPerception>(host, perception_group_);
  LocalMapConfig active_local_map = params_.local_map;
  const ModeProfile profile = mode_supervisor_.profile();
  if (profile.avoidance_overrides_enabled) {
    active_local_map.inflation_radius_m =
      std::max(0.0, profile.inflation_radius_m);
  }
  local_map_ = std::make_unique<RollingLocalMap>(active_local_map);
  controller_ = std::make_unique<AvoidanceController>(params_.avoidance, params_.map_constraint);
  vehicle_io_ = std::make_unique<VehicleIo>(host, perception_group_);

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
      const rclcpp::QoS tilt_points_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
      tilt_left_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        TILT_LEFT_POINTS_VIZ_TOPIC, tilt_points_qos);
      tilt_right_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        TILT_RIGHT_POINTS_VIZ_TOPIC, tilt_points_qos);
    }
  }
  log_pub_ = create_publisher<std_msgs::msg::String>(INTERVENTION_LOG_TOPIC, 10);
  // 方案 7：低频健康摘要（tick 分布 / 各源新鲜度 / 层开关位图 / 降级与影子统计）
  health_pub_ = create_publisher<std_msgs::msg::String>("/safety/health", 1);

  // ── 对外状态/预警上报：与 scene_node 同构的三个话题，供 TFT / 灯效 / 语音 / 底盘消费 ──
  if (params_.reporting.auxiliary_status_enabled) {
    auxiliary_status_pub_ = create_publisher<std_msgs::msg::Bool>(auxiliaryStatusTopic, 10);
  }
  if (params_.reporting.cruise_status_enabled) {
    cruise_status_pub_ = create_publisher<std_msgs::msg::Int32>(cursorAlarmTopic, 10);
  }
  if (params_.reporting.collision_warning.enabled) {
    // 深度 10：预警是状态机事件流，驶离事件一旦被丢弃，消费端提示会永久挂住。
    collision_warning_pub_ =
      create_publisher<interface::msg::CollisionWarning>(collisionWarningTopic, 10);
  }
  warning_reporter_.updateConfig(params_.reporting.collision_warning);

  last_control_time_ = now();
}

void SafetyNode::setupTimers()
{
  // ── 控制环：默认 20 Hz（control 组独占，方案 7）──
  const double control_hz = std::max(1.0, params_.control_rate_hz);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_hz)),
    [this]() {onControlTick();},
    control_group_);

  // ── LocalMap 融合：默认 10 Hz（background 组）──
  if (params_.local_map.enabled) {
    double map_hz = 10.0;
    if (has_parameter("local_map.update_rate_hz")) {
      map_hz = get_parameter("local_map.update_rate_hz").as_double();
    }
    map_update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, map_hz))),
      [this]() {onLocalMapUpdate();},
      background_group_);
  }

  // ── 地图可视化：默认 5 Hz（background 组，低优先级） ──
  const bool publish_viz = get_parameter("local_map.publish_viz").as_bool();
  if (publish_viz && params_.local_map.enabled) {
    const double viz_hz = get_parameter("local_map.viz_rate_hz").as_double();
    map_viz_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, viz_hz))),
      [this]() {onMapVizTick();},
      background_group_);
  }

  if (perception_viz_enabled_) {
    perception_viz_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / std::max(1.0, perception_viz_rate_hz_))),
      [this]() {onPerceptionVizTick();},
      background_group_);
  }

  // ── 方案 7 控制环看门狗 + 健康摘要：1 Hz（background 组）──
  watchdog_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() {onWatchdogTick();},
    background_group_);

  // ── 安全开关状态心跳：默认 1 Hz（background 组）──
  // 参照 scene_node：消费端（底盘 / TFT）可能晚于本节点启动，靠心跳而非仅靠跳变对齐状态。
  if (params_.reporting.auxiliary_status_enabled) {
    const double status_hz = std::max(0.1, params_.reporting.auxiliary_status_rate_hz);
    auxiliary_status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / status_hz)),
      [this]() {publishAuxiliaryStatus();},
      background_group_);
  }
}

void SafetyNode::onLocalMapUpdate()
{
  // 无控制权时不做地图融合（与控制环静默一致，避免空跑避障相关逻辑）。
  if (!hasControlAuthority()) {
    return;
  }
  const ModeProfile profile = mode_supervisor_.profile();
  if (!profile.local_map_update_enabled || !params_.local_map.enabled) {
    return;
  }
  const AvoidanceParams active_avoidance =
    applyModeProfile(params_.avoidance, profile);

  const rclcpp::Time stamp = now();

  // 方案 6A：消费控制环累加的轮速积分，先对历史栅格做自运动补偿
  //（仅 rebuild_each_frame=false 且补偿开启时有实际动作，否则空操作）。
  {
    double ego_dx = 0.0;
    double ego_dtheta = 0.0;
    {
      std::lock_guard<std::mutex> lock(ego_motion_mutex_);
      ego_dx = ego_dx_accum_m_;
      ego_dtheta = ego_dtheta_accum_rad_;
      ego_dx_accum_m_ = 0.0;
      ego_dtheta_accum_rad_ = 0.0;
    }
    // 差速车无侧向滑移，dy 恒为 0（unicycle 模型）
    local_map_->applyEgoMotion(ego_dx, 0.0, ego_dtheta);
  }

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
    // 方案 7：可视化线程读只读快照，不触碰地图工作栅格（消除跨组数据竞争）
    const auto grid = local_map_->snapshot();
    if (grid) {
      map_viz_pub_->publish(RollingLocalMap::gridToOccupancyMsg(*grid, now()));
    }
  }
}

void SafetyNode::onPerceptionVizTick()
{
  if (!hasControlAuthority()) {
    return;
  }
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
  const auto rb = perception_->roiBounds();
  roi.points = {
    makePoint(rb.x_min, rb.y_min, 0.02),
    makePoint(rb.x_max, rb.y_min, 0.02),
    makePoint(rb.x_max, rb.y_max, 0.02),
    makePoint(rb.x_min, rb.y_max, 0.02),
    makePoint(rb.x_min, rb.y_min, 0.02)};
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

  const double trajectory_dt = std::max(0.02, params_.avoidance.rollout_dt_s);
  const double trajectory_horizon =
    std::max(trajectory_dt, active_avoidance.rollout_horizon_s);

  // 方案 7：跨组读取用只读副本，prev_output_ 本体仅控制组访问
  geometry_msgs::msg::Twist viz_prev;
  {
    std::lock_guard<std::mutex> lock(viz_prev_output_mutex_);
    viz_prev = viz_prev_output_;
  }
  auto published_trajectory = makeLineStrip(
    stamp, "published_command_trajectory", 3, 0.2f, 1.0f, 0.25f);
  published_trajectory.scale.x = 0.05;
  for (const auto & pt : JoystickUtils::integrateUnicycleTrajectory(
       viz_prev.linear.x,
       viz_prev.angular.z,
       trajectory_horizon,
       trajectory_dt))
  {
    published_trajectory.points.push_back(pt);
  }
  markers.markers.push_back(published_trajectory);

  if (joystick_trajectory_viz_enabled_) {
    JoystickTwistMapping mapping = joystick_trajectory_mapping_;
    mapping.deadzone = params_.joystick_deadzone;
    const auto joystick = vehicle_io_->latestJoystick(
      stamp, params_.joystick_timeout_s);
    const geometry_msgs::msg::Twist intent = JoystickUtils::toTwist(
      joystick.msg, mapping, joystick.fresh);

    auto joystick_trajectory = makeLineStrip(
      stamp, "joystick_intent_trajectory", 5, 1.0f, 0.5f, 0.0f);
    joystick_trajectory.scale.x = 0.05;
    for (const auto & pt : JoystickUtils::integrateUnicycleTrajectory(
         intent.linear.x,
         intent.angular.z,
         trajectory_horizon,
         trajectory_dt))
    {
      joystick_trajectory.points.push_back(pt);
    }
    markers.markers.push_back(joystick_trajectory);
  }

  perception_markers_pub_->publish(markers);

  if (tilt_left_debug_pub_ && tilt_right_debug_pub_ &&
    params_.tilt_perception.publish_debug_scan && params_.tilt_perception.enabled)
  {
    const auto snapshots = perception_->snapshotTiltSources(active_avoidance, stamp);
    for (const auto & snapshot : snapshots) {
      const auto msg = makeTiltObstacleScanMsg(stamp, snapshot);
      const auto points_cloud = makeTiltDebugPointCloud2(
        stamp,
        snapshot.filtered_points,
        PerceptionUtils::debugColor(snapshot.source_id)[0],
        PerceptionUtils::debugColor(snapshot.source_id)[1],
        PerceptionUtils::debugColor(snapshot.source_id)[2]);
      if (snapshot.source_name == "left_tilt") {
        tilt_left_debug_pub_->publish(msg);
        if (tilt_left_points_pub_) {
          tilt_left_points_pub_->publish(points_cloud);
        }
      } else if (snapshot.source_name == "right_tilt") {
        tilt_right_debug_pub_->publish(msg);
        if (tilt_right_points_pub_) {
          tilt_right_points_pub_->publish(points_cloud);
        }
      }
    }
  }
}

void SafetyNode::onControlTick()
{
  const auto tick_start = std::chrono::steady_clock::now();
  const rclcpp::Time now_time = now();

  // 标准关且非巡航：对齐 scene_node，不发 /nav/control、不跑避障。
  // 仍刷新 tick 时间戳，避免看门狗把静默误判为控制环停摆并兜底发零速。
  if (!hasControlAuthority()) {
    last_control_time_ = now_time;
    last_tick_steady_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(),
      std::memory_order_relaxed);
    return;
  }

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
    params_.joystick_deadzone,
    params_.joystick_reverse_threshold);
  const WheelFeedback wheels = vehicle_io_->wheelFeedback(
    now_time, params_.wheel_feedback_timeout_s);
  // 巡航（与安全开关正交）：定速保持（松杆保持速度、转向降速、后退/超时/抱闸退出、档位限速）。
  // 将保持速度注入 desired；是否再经避障由安全开关（profile）独立决定。
  geometry_msgs::msg::Twist desired = control.twist;
  bool cruise_holding = false;
  if (cruise_enabled_.load()) {
    const CarInfoState car = vehicle_io_->carInfo(now_time, params_.car_info_timeout_s);
    // 档位限速：CarInfo 新鲜时用档位/驾驶模式最高速；否则传 0 表示未知（updateCruise 兜底为绝对上限）。
    const double gear_max_speed =
      car.fresh ? gearMaxLinearSpeedMs(car.gear, car.mode_type) : 0.0;
    const bool brake_engaged = car.fresh && car.brake_engaged;
    desired = updateCruise(control, gear_max_speed, brake_engaged, cruise_holding);
  }
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
  // 巡航已在上游注入 desired，不再使用 evaluate 内的限速通道。
  ctx.cruise_active = false;
  ctx.cruise_linear_cap = 0.0;
  ctx.safety_mode = mode_supervisor_.mode();

  // ── 方案 4 传感器降级矩阵：按 per-source 新鲜度合成分轴封锁 + 限速档 ──
  // evaluate 入口按 direction_mask 同款语义施加；全局 INPUT_FAULT 兜底不变。
  const DegradationVerdict degradation_verdict = degradation::evaluate(
    params_.degradation,
    perception_health.source_enabled,
    perception_health.source_fresh);
  ctx.degradation_active = degradation_verdict.active;
  ctx.degradation_block_forward = degradation_verdict.block_forward;
  ctx.degradation_block_backward = degradation_verdict.block_backward;
  ctx.degradation_block_left = degradation_verdict.block_left;
  ctx.degradation_block_right = degradation_verdict.block_right;
  ctx.degradation_speed_cap_m_s = degradation_verdict.speed_cap_m_s;

  // ── 2. 构建 ObstacleField（Footprint 快路径） ──
  const ObstacleField field =
    (profile.obstacle_field_enabled &&
    active_avoidance.horizontal_avoidance_enabled &&
    params_.horizontal_perception.enabled) ?
    perception_->buildHorizontalObstacleField(active_avoidance, now_time) :
    ObstacleField{};
  // 斜雷达独立障碍场：体素更细、不参与 LocalMap，供 TTC 近距通道专用
  const ObstacleField tilt_field =
    (profile.obstacle_field_enabled && params_.tilt_perception.enabled) ?
    perception_->buildTiltObstacleField(active_avoidance, now_time) :
    ObstacleField{};
  // 低通专用斜雷达快照始终按 left_tilt/right_tilt 原始源拆分。即使斜雷达配置为
  // treat_as_horizontal，也不借用合并后的 field，避免水平雷达参与低通判定。
  const bool front_lowpass_tilt_only =
    params_.front_obstacle_lowpass_stop.obstacle_source == "tilt_only";
  std::array<FrontLowpassSourceSample, 2> front_lowpass_tilt_samples {};
  // 斜雷达前向近障软限速：与低通共享同一次原始源快照，但用各自独立的走廊宽度算净空。
  const bool front_tilt_cap_enabled = params_.avoidance.front_tilt_speed_cap.enabled;
  double front_tilt_cap_clearance_m = std::numeric_limits<double>::infinity();
  const bool front_lowpass_needs_tilt =
    params_.front_obstacle_lowpass_stop.enabled && front_lowpass_tilt_only;
  if ((front_lowpass_needs_tilt || front_tilt_cap_enabled) &&
    profile.obstacle_field_enabled &&
    params_.tilt_perception.enabled)
  {
    const auto snapshots =
      perception_->snapshotTiltSources(active_avoidance, now_time);
    const size_t count = std::min(snapshots.size(), front_lowpass_tilt_samples.size());
    for (size_t i = 0; i < count; ++i) {
      if (front_lowpass_needs_tilt) {
        front_lowpass_tilt_samples[i].frame_valid = snapshots[i].frame_valid;
        front_lowpass_tilt_samples[i].frame_stamp_ns = snapshots[i].frame_stamp_ns;
        front_lowpass_tilt_samples[i].frame_age_s = snapshots[i].frame_age_s;
        front_lowpass_tilt_samples[i].clearance_m =
          controller_->frontCorridorClearance(
          snapshots[i].primitive_field,
          active_avoidance.sudden_obstacle.corridor_margin_m);
      }
      if (front_tilt_cap_enabled) {
        front_tilt_cap_clearance_m = std::min(
          front_tilt_cap_clearance_m,
          controller_->frontCorridorClearance(
            snapshots[i].primitive_field,
            params_.avoidance.front_tilt_speed_cap.corridor_margin_m));
      }
    }
  }
  // 每周期都更新闸（含未限速/未取数帧），维持确认与滞回计数；未取数时净空 +inf → 释放。
  FrontTiltSpeedCapInput front_tilt_cap_in;
  front_tilt_cap_in.forward_clearance_m = front_tilt_cap_clearance_m;
  front_tilt_cap_in.dt_s = dt_s;
  const FrontTiltSpeedCapResult front_tilt_cap =
    front_tilt_speed_cap_gate_.update(front_tilt_cap_in);
  ctx.front_tilt_speed_cap_m_s = front_tilt_cap.active ?
    front_tilt_cap.speed_cap_m_s : std::numeric_limits<double>::infinity();
  if (front_tilt_cap.just_latched) {
    RCLCPP_INFO(
      get_logger(),
      "%s 斜雷达前向软限速 触发 净空=%.3fm 限速=%.2fm/s(%.1fkm/h) (阈值 净空<=%.2fm 走廊±%.2fm)",
      LOG_PREFIX,
      front_tilt_cap_clearance_m, front_tilt_cap.speed_cap_m_s,
      front_tilt_cap.speed_cap_m_s * 3.6,
      params_.avoidance.front_tilt_speed_cap.trigger_distance_m,
      params_.avoidance.front_tilt_speed_cap.corridor_margin_m);
  } else if (front_tilt_cap.just_released) {
    RCLCPP_INFO(
      get_logger(),
      "%s 斜雷达前向软限速 解除 净空=%.3fm",
      LOG_PREFIX,
      std::isfinite(front_tilt_cap_clearance_m) ? front_tilt_cap_clearance_m : -1.0);
  } else if (front_tilt_cap.active) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "%s 斜雷达前向软限速 生效中 净空=%.3fm 限速=%.2fm/s",
      LOG_PREFIX,
      std::isfinite(front_tilt_cap_clearance_m) ? front_tilt_cap_clearance_m : -1.0,
      front_tilt_cap.speed_cap_m_s);
  }

  // ── 3. LocalMap 快照（Map veto 用，透传模式可为 nullptr） ──
  // 方案 7：控制环读只读快照（shared_ptr 保活至本 tick 结束），
  // 不再直接引用工作栅格，消除与地图融合线程的裸竞争。
  const auto map_snapshot = local_map_->snapshot();
  const RollingLocalMapGrid * map_ptr =
    (profile.local_map_update_enabled && params_.local_map.enabled &&
    map_snapshot) ?
    map_snapshot.get() :
    nullptr;

  // ── 4. 混合避障决策：Footprint 选速 → Map 裁判 → 急停兜底 ──
  const bool input_fault =
    !control.fresh ||
    !desired_finite ||
    (params_.require_wheel_feedback && !wheels.fresh) ||
    (wheels.received && !wheel_values_finite) ||
    (profile.obstacle_field_enabled && params_.require_perception &&
    params_.require_horizontal_perception &&
    params_.horizontal_perception.enabled &&
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
  const bool hard_emergency = bypassOutputSmoothing(result.emergency_reason);
  const double direction_reversal_release_speed =
    std::max(0.0, params_.direction_reversal_release_speed_m_s);
  const bool forward_motion_or_command_pending =
    fb_linear > direction_reversal_release_speed ||
    prev_output_.linear.x > direction_reversal_release_speed ||
    desired.linear.x > direction_reversal_release_speed ||
    result.safe_twist.linear.x > direction_reversal_release_speed;
  const bool direction_reversal_fast_stop =
    params_.direction_reversal_fast_stop_enabled &&
    control.driver_reverse_request &&
    !control.driver_neutral &&
    !result.emergency_brake &&
    !result.recovery_active &&
    forward_motion_or_command_pending;
  // ── 前向近障低通刹停：闸每周期必须更新（含未接管帧），否则锁存/确认计数会走陈 ──
  FrontLowpassStopInput front_lowpass_in;
  FrontLowpassClearanceEstimate front_lowpass_estimate;
  if (front_lowpass_tilt_only) {
    front_lowpass_estimate = front_lowpass_tilt_tracker_.update(
      front_lowpass_tilt_samples, fb_linear, dt_s);
    front_lowpass_in.forward_clearance_m = front_lowpass_estimate.clearance_m;
    front_lowpass_in.trigger_observation_valid =
      front_lowpass_estimate.valid_for_trigger;
  } else {
    front_lowpass_in.forward_clearance_m =
      result.hybrid_log.forward_corridor_clearance_m;
    front_lowpass_in.trigger_observation_valid = true;
  }
  result.hybrid_log.front_lowpass_clearance_m =
    front_lowpass_in.forward_clearance_m;
  result.hybrid_log.front_lowpass_predicted =
    front_lowpass_tilt_only && front_lowpass_estimate.predicted;
  result.hybrid_log.front_lowpass_observation_age_s =
    front_lowpass_estimate.observation_age_s;
  front_lowpass_in.fb_linear_m_s = fb_linear;
  front_lowpass_in.command_linear_m_s = prev_output_.linear.x;
  front_lowpass_in.reverse_intent = control.driver_reverse_request;
  // 前进意图取注入避障链的 desired（含巡航保持速度）：巡航中松杆仍算有前进意图，
  // 不能靠「松杆」把巡航下的前障刹停解掉。
  front_lowpass_in.forward_intent = desired.linear.x > 1e-3;
  // MAP_VETO 且仍在硬停线外：不阻断低通闸，让触发带内走柔停；其它急停仍硬阻断。
  // tilt_only 无法证明 MAP_VETO 的来源也是斜雷达，因此不覆盖地图/水平链的硬裁决。
  const bool map_veto_override_enabled =
    params_.front_obstacle_lowpass_stop.override_map_veto &&
    !front_lowpass_tilt_only;
  const bool map_veto_soft = mapVetoYieldsToFrontLowpass(
    map_veto_override_enabled,
    result.emergency_brake,
    result.emergency_reason == EmergencyStopReason::MAP_VETO,
    result.hybrid_log.forward_corridor_clearance_m,
    params_.avoidance.hard_stop_distance_m);
  front_lowpass_in.emergency_brake = result.emergency_brake && !map_veto_soft;
  front_lowpass_in.dt_s = dt_s;
  const FrontLowpassStopResult front_lowpass = front_lowpass_gate_.update(front_lowpass_in);
  result.hybrid_log.front_lowpass_stop_active = front_lowpass.active;
  result.hybrid_log.front_lowpass_stop_tau_s = front_lowpass.tau_s;
  if (front_lowpass.just_latched) {
    RCLCPP_INFO(
      get_logger(),
      "%s 前向近障低通刹停 触发 净空=%.3fm 反馈=%.3fm/s 命令=%.3fm/s τ=%.3fs "
      "(阈值 净空<=%.2fm 速度>%.2fm/s 目标停在%.2fm外)%s",
      LOG_PREFIX,
      front_lowpass_in.forward_clearance_m, fb_linear, prev_output_.linear.x,
      front_lowpass.tau_s,
      params_.front_obstacle_lowpass_stop.trigger_distance_m,
      params_.front_obstacle_lowpass_stop.trigger_speed_m_s,
      params_.front_obstacle_lowpass_stop.stop_buffer_m,
      map_veto_soft ? " [盖过map_veto]" : "");
  } else if (front_lowpass.active) {
    // 锁存期间周期性回报收敛过程，便于实车/回放核对「刹到哪、还剩多少净空」。
    // 净空为 +inf（障碍已进自车过滤盲区）时打印 -1 以示区分。
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 200,
      "%s 前向近障低通刹停 生效中 净空=%.3fm 反馈=%.3fm/s 输出=%.3fm/s τ=%.3fs 角速度=%.3frad/s",
      LOG_PREFIX,
      std::isfinite(front_lowpass_in.forward_clearance_m) ?
      front_lowpass_in.forward_clearance_m : -1.0,
      fb_linear, prev_output_.linear.x, front_lowpass.tau_s,
      params_.front_obstacle_lowpass_stop.preserve_steering ?
      result.safe_twist.angular.z : 0.0);
  }
  if (front_lowpass.just_released) {
    RCLCPP_INFO(
      get_logger(),
      "%s 前向近障低通刹停 解除(%s) 净空=%.3fm 反馈=%.3fm/s",
      LOG_PREFIX,
      front_lowpass.release_reason ? front_lowpass.release_reason : "未知",
      std::isfinite(front_lowpass_in.forward_clearance_m) ?
      front_lowpass_in.forward_clearance_m : -1.0,
      fb_linear);
  }

  const double previous_output_linear = prev_output_.linear.x;
  geometry_msgs::msg::Twist shaped;
  // 巡航保持期间摇杆回中不触发刹停（否则无法定速保持）。
  // map_veto 柔停放行时仍允许松杆回中优先生效。
  if (control.driver_neutral &&
    !(result.emergency_brake && !map_veto_soft) &&
    !cruise_holding)
  {
    // 原始摇杆回中优先于 /handle/control 的缓慢衰减，但使用一阶低通向零收敛，
    // 避免单周期硬清零带来的明显顿挫。
    shaped = output_shaper_.lowPassToZero(
      dt_s, params_.neutral_stop_filter_time_constant_s);
  } else if (direction_reversal_fast_stop) {
    shaped = output_shaper_.fastStopToZero(
      dt_s,
      params_.direction_reversal_stop_decel_m_s2,
      params_.direction_reversal_stop_jerk_m_s3);
  } else if (front_lowpass.active) {
    // 前方贴身近障且仍有速度：该区间在 hard_stop 之外，原本走常规 shape() 按 max_decel
    // 慢慢收敛、再蠕行到 hard-stop 线吃一次硬停顿挫。改为按剩余净空反算 τ 的低通柔和刹停。
    // 含「盖过 map_veto」：LocalMap 否决帧在触发带内改走柔停，输出优先级高于急停零速。
    // 角速度默认不参与衰减，避免把「转向脱离」一并冻死（见 preserve_steering）。
    shaped = params_.front_obstacle_lowpass_stop.preserve_steering ?
      output_shaper_.lowPassLinearToZero(
      dt_s, front_lowpass.tau_s, result.safe_twist.angular.z) :
      output_shaper_.lowPassToZero(dt_s, front_lowpass.tau_s);
  } else {
    const bool min_distance_braking_active =
      params_.avoidance.min_distance_braking_enabled &&
      result.risk.braking_limited &&
      !result.risk.emergency;
    const bool shape_as_emergency =
      result.emergency_brake && !min_distance_braking_active;
    shaped = output_shaper_.shape(
      result.safe_twist, dt_s, shape_as_emergency, result.recovery_active, hard_emergency);
  }
  bool output_nonfinite_guard = false;
  if (!std::isfinite(shaped.linear.x) || !std::isfinite(shaped.angular.z)) {
    output_nonfinite_guard = true;
    shaped = geometry_msgs::msg::Twist{};
    output_shaper_.reset();
  }
  shaped.linear.y = 0.0;
  shaped.linear.z = 0.0;
  shaped.angular.x = 0.0;
  shaped.angular.y = 0.0;

  // ── 速度干预可解释诊断 ──
  // 比较四层速度：摇杆控制 → 巡航/模式期望 → 避障目标 → 最终整形输出。
  // 显式刹停即使单周期变化小于 min_delta 也记录，避免低通/jerk 限制初始帧漏报。
  auto & speed_log = result.hybrid_log;
  speed_log.speed_debug_joystick_linear_x =
    std::isfinite(control.twist.linear.x) ? control.twist.linear.x : 0.0;
  speed_log.speed_debug_desired_linear_x =
    std::isfinite(desired.linear.x) ? desired.linear.x : 0.0;
  speed_log.speed_debug_controller_linear_x =
    std::isfinite(result.safe_twist.linear.x) ? result.safe_twist.linear.x : 0.0;
  speed_log.speed_debug_output_linear_x = shaped.linear.x;
  speed_log.speed_debug_previous_output_linear_x =
    std::isfinite(previous_output_linear) ? previous_output_linear : 0.0;
  speed_log.speed_debug_wheel_linear_x = fb_linear;

  const double joystick_abs = std::abs(speed_log.speed_debug_joystick_linear_x);
  const double desired_abs = std::abs(speed_log.speed_debug_desired_linear_x);
  const double controller_abs = std::abs(speed_log.speed_debug_controller_linear_x);
  const double output_abs = std::abs(speed_log.speed_debug_output_linear_x);
  const double previous_output_abs =
    std::abs(speed_log.speed_debug_previous_output_linear_x);
  const double reference_linear =
    desired_abs >= joystick_abs ?
    speed_log.speed_debug_desired_linear_x :
    speed_log.speed_debug_joystick_linear_x;
  const double reference_abs = std::max(joystick_abs, desired_abs);
  const double min_delta = speed_intervention_debug_min_delta_m_s_;
  const bool reference_direction_lost =
    reference_abs > 1e-6 && reference_linear * shaped.linear.x <= 0.0;
  const bool output_decreasing = output_abs + 1e-6 < previous_output_abs;
  const bool explicit_brake_state =
    control.driver_neutral ||
    direction_reversal_fast_stop ||
    front_lowpass.active ||
    result.emergency_brake;
  const bool controller_direction_lost =
    reference_abs > 1e-6 &&
    reference_linear * speed_log.speed_debug_controller_linear_x <= 0.0;
  const bool controller_reduced =
    reference_abs >= min_delta &&
    (controller_direction_lost || controller_abs + min_delta < reference_abs);
  // 正常起步时 output 受加速度限制尚未追上 reference，不属于衰减/制动，不记问题日志。
  const bool reference_reduced =
    reference_abs >= min_delta &&
    (reference_direction_lost || output_abs + min_delta < reference_abs) &&
    (controller_reduced || output_decreasing || explicit_brake_state);
  speed_log.speed_debug_braking =
    output_decreasing &&
    (explicit_brake_state || controller_abs + 1e-6 < previous_output_abs);
  speed_log.speed_debug_reduction_m_s =
    std::max(0.0, reference_abs - output_abs);
  speed_log.speed_intervention_active =
    speed_intervention_debug_enabled_ &&
    (reference_reduced || speed_log.speed_debug_braking);

  if (speed_log.speed_intervention_active) {
    auto add_reason = [&speed_log](const std::string & reason) {
        if (reason.empty()) {
          return;
        }
        if (speed_log.speed_intervention_reason == "none") {
          speed_log.speed_intervention_reason = reason;
        } else {
          speed_log.speed_intervention_reason += ";" + reason;
        }
      };

    if (!control.fresh) {
      add_reason("control_input_timeout");
    }
    if (!desired_finite) {
      add_reason("nonfinite_desired_command");
    }
    if ((params_.require_wheel_feedback && !wheels.fresh) ||
      (wheels.received && !wheel_values_finite))
    {
      add_reason("wheel_feedback_fault");
    }
    if (output_nonfinite_guard) {
      add_reason("nonfinite_output_guard");
    }
    if (result.emergency_brake) {
      add_reason(
        std::string("emergency_") + emergencyReasonToString(result.emergency_reason));
    }
    if (control.driver_neutral && !cruise_holding) {
      add_reason("joystick_neutral_brake");
    }
    if (direction_reversal_fast_stop) {
      add_reason("direction_reversal_fast_stop");
    }
    if (front_lowpass.active) {
      add_reason("front_obstacle_lowpass_stop");
      if (result.hybrid_log.front_lowpass_predicted) {
        add_reason("front_lowpass_wheel_prediction");
      }
    }
    if (result.recovery_active) {
      add_reason("recovery_override");
    }
    if (result.hybrid_log.degradation_active) {
      add_reason("sensor_degradation_policy");
    }
    if (result.hybrid_log.direction_mask_forward ||
      result.hybrid_log.direction_mask_backward ||
      result.hybrid_log.direction_mask_left ||
      result.hybrid_log.direction_mask_right)
    {
      add_reason("direction_mask");
    }
    if (result.hybrid_log.map_veto_applied) {
      add_reason("local_map_veto");
    }
    if (result.hybrid_log.sudden_obstacle_active) {
      add_reason("sudden_obstacle_emergency_brake");
    }
    if (result.hybrid_log.forward_stop_hold_active) {
      add_reason("forward_stop_hold");
    }
    if (result.hybrid_log.standstill_guard_active) {
      add_reason("standstill_guard");
    }
    if (result.hybrid_log.tilt_intervention != "none") {
      add_reason(
        std::string("tilt_") + result.hybrid_log.tilt_intervention +
        (result.hybrid_log.tilt_collision_source.empty() ?
        "" : "_" + result.hybrid_log.tilt_collision_source));
    }
    if (result.risk.braking_limited) {
      add_reason("min_distance_braking_limit");
    }
    if (std::isfinite(result.hybrid_log.turn_guard_speed_cap_m_s)) {
      add_reason("turn_guard_speed_cap");
    }
    if (result.hybrid_log.turn_envelope_scale < 1.0 - 1e-6) {
      add_reason("turn_envelope_scale");
    }
    if (result.hybrid_log.rapid_steer_cap_active) {
      add_reason("rapid_steer_speed_cap");
    }
    if (result.hybrid_log.narrow_planner_active &&
      std::isfinite(result.hybrid_log.narrow_speed_cap_m_s))
    {
      add_reason("narrow_corridor_speed_cap");
    }
    if (result.hybrid_log.authority_alpha > 1e-6) {
      add_reason("shared_control_authority");
    }
    if (std::abs(
        speed_log.speed_debug_desired_linear_x -
        speed_log.speed_debug_joystick_linear_x) > min_delta)
    {
      add_reason("cruise_or_mode_speed_adjustment");
    }
    if (controller_abs + min_delta < desired_abs) {
      add_reason("avoidance_controller_speed_limit");
    }
    if (output_abs + 1e-6 < controller_abs) {
      add_reason("output_acceleration_or_lowpass_limit");
    } else if (output_abs > controller_abs + 1e-6 && output_decreasing) {
      add_reason("output_decel_jerk_limit");
    }
    if (speed_log.speed_intervention_reason == "none") {
      add_reason("unclassified_speed_reduction");
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), speed_intervention_debug_throttle_ms_,
      "%s 速度干预 摇杆=%.3f 期望=%.3f 控制器=%.3f 上周期=%.3f 输出=%.3f "
      "轮速=%.3f 衰减=%.3f 制动=%s 原因=%s",
      LOG_PREFIX,
      speed_log.speed_debug_joystick_linear_x,
      speed_log.speed_debug_desired_linear_x,
      speed_log.speed_debug_controller_linear_x,
      speed_log.speed_debug_previous_output_linear_x,
      speed_log.speed_debug_output_linear_x,
      speed_log.speed_debug_wheel_linear_x,
      speed_log.speed_debug_reduction_m_s,
      speed_log.speed_debug_braking ? "true" : "false",
      speed_log.speed_intervention_reason.c_str());
  }

  prev_output_ = shaped;
  {
    // 方案 7：感知可视化跨组只读副本（prev_output_ 本体仅控制组访问）
    std::lock_guard<std::mutex> lock(viz_prev_output_mutex_);
    viz_prev_output_ = shaped;
  }
  // 方案 6A：轮速积分累加，供地图周期做自运动补偿（10Hz 消费清零）
  {
    std::lock_guard<std::mutex> lock(ego_motion_mutex_);
    ego_dx_accum_m_ += fb_linear * dt_s;
    ego_dtheta_accum_rad_ += fb_angular * dt_s;
  }
  publishControl(shaped);
  publishInterventionLog(result);

  // ── 对外预警/巡航状态上报 ──
  // 急停兜底方向按「驾驶员意图优先、其次实际运动」判定：map_veto / 斜雷达硬闸等
  // 急停路径 levels 为空，但停车本身须让 TFT/灯效/语音知道是哪一侧触发的。
  CollisionDirection fallback_direction = CollisionDirection::NONE;
  {
    constexpr double kMotionEps = 0.02;
    const double intent = desired_finite && std::abs(desired.linear.x) > kMotionEps ?
      desired.linear.x :
      fb_linear;
    if (intent > kMotionEps) {
      fallback_direction = CollisionDirection::FORWARD;
    } else if (intent < -kMotionEps) {
      fallback_direction = CollisionDirection::BACKWARD;
    }
  }
  publishCollisionWarning(result, fallback_direction, now_time);
  updateCruiseObstacleExit(shaped.linear.x, now_time);

  // ── 方案 7/8 健康统计：tick 耗时滑窗 + 影子/降级计数（看门狗 1Hz 汇总发布）──
  const double tick_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - tick_start).count();
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    tick_ms_ring_[tick_ring_index_] = tick_ms;
    tick_ring_index_ = (tick_ring_index_ + 1) % tick_ms_ring_.size();
    tick_ring_count_ = std::min(tick_ring_count_ + 1, tick_ms_ring_.size());
    if (result.hybrid_log.arbiter_shadow_mismatch) {
      ++arbiter_mismatch_count_;
    }
    if (result.hybrid_log.sudden_obstacle_shadow) {
      ++sudden_shadow_count_;
    }
    if (result.hybrid_log.pinned_escape_shadow) {
      ++pinned_shadow_count_;
    }
    last_degradation_ = degradation_verdict;
  }
  last_tick_steady_ns_.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count(),
    std::memory_order_relaxed);
}

void SafetyNode::publishControl(const geometry_msgs::msg::Twist & cmd)
{
  if (control_pub_) {
    control_pub_->publish(cmd);
  }
}

void SafetyNode::publishAuxiliaryStatus()
{
  if (!auxiliary_status_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = safety_enabled_.load(std::memory_order_acquire);
  auxiliary_status_pub_->publish(msg);
}

void SafetyNode::publishCruiseStatus(int32_t code)
{
  if (!cruise_status_pub_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = code;
  cruise_status_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "%s 发布巡航状态: %d (1=无前进速度退出 2=障碍物压停退出 3=激活成功 4=开启成功)",
    LOG_PREFIX, code);
}

void SafetyNode::publishCollisionWarning(
  const AvoidanceResult & result, CollisionDirection fallback_direction,
  const rclcpp::Time & now_time)
{
  if (!collision_warning_pub_) {
    return;
  }
  CollisionWarningInput in;
  in.levels = result.risk.levels;
  in.emergency_brake = result.emergency_brake;
  in.emergency_reason = result.emergency_reason;
  in.fallback_direction = fallback_direction;
  // 安全关闭（透传）时不产生新预警；已挂起的预警会由状态机收敛出驶离事件后归零。
  in.suppressed = !safety_enabled_.load(std::memory_order_acquire);
  in.now_s = now_time.seconds();

  for (const auto & event : warning_reporter_.update(in)) {
    interface::msg::CollisionWarning msg;
    msg.header.stamp = now_time;
    msg.header.frame_id = "base_link";
    msg.warning_type = event.warning_type;
    msg.warning_level = event.warning_level;
    collision_warning_pub_->publish(msg);
    RCLCPP_DEBUG(
      get_logger(), "%s 发布碰撞预警: type=%d level=%d",
      LOG_PREFIX, event.warning_type, event.warning_level);
  }
}

void SafetyNode::updateCruiseObstacleExit(double output_linear, const rclcpp::Time & now_time)
{
  // 仅在「巡航已建立」时判定：启动窗口内没速度属于 reason=1，由 updateCruise 负责。
  const bool cruise_running = cruise_enabled_.load() && !cruise_speed_timeout_active_ &&
    cruise_target_speed_ > 0.0;
  if (!cruise_running) {
    cruise_obstacle_zero_active_ = false;
    return;
  }

  const double eps = std::max(0.0, params_.cruise.obstacle_zero_speed_eps_m_s);
  const double v = std::isfinite(output_linear) ? std::abs(output_linear) : 0.0;
  if (v > eps) {
    cruise_obstacle_zero_active_ = false;
    return;
  }

  if (!cruise_obstacle_zero_active_) {
    cruise_obstacle_zero_active_ = true;
    cruise_obstacle_zero_since_ = now_time;
    return;
  }
  const double elapsed = (now_time - cruise_obstacle_zero_since_).seconds();
  // 时钟跳变（elapsed 非有限或为负）时重新对时，不误判超时。
  if (!std::isfinite(elapsed) || elapsed < 0.0) {
    cruise_obstacle_zero_since_ = now_time;
    return;
  }
  if (elapsed > std::max(0.0, params_.cruise.speed_timeout_s)) {
    publishCruiseStatus(CRUISE_END_OBSTACLE_STOP);
    setCruiseEnabled(false, true);
    RCLCPP_INFO(
      get_logger(), "%s 巡航输出被障碍物压停超过 %.1f s，巡航已自动关闭",
      LOG_PREFIX, params_.cruise.speed_timeout_s);
  }
}

void SafetyNode::publishInterventionLog(const AvoidanceResult & result)
{
  if (!log_pub_) {
    return;
  }
  // 无干预时不刷屏（影子触发 / 降级生效 / 仲裁不一致同样属于须记录事件）
  if (!result.hybrid_log.map_veto_applied && !result.emergency_brake &&
    !result.recovery_active &&
    !result.risk.braking_limited &&
    !result.hybrid_log.forward_stop_hold_active &&
    !result.hybrid_log.pulse_comp_active &&
    !result.hybrid_log.narrow_planner_active &&
    !result.hybrid_log.sudden_obstacle_shadow &&
    !result.hybrid_log.pinned_escape_shadow &&
    !result.hybrid_log.degradation_active &&
    !result.hybrid_log.arbiter_shadow_mismatch &&
    !result.hybrid_log.speed_intervention_active &&
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
      << "\"braking_effective_remaining_m\":";
  appendNullableNumber(oss, result.risk.braking_effective_remaining_m);
  oss << ","
      << "\"braking_at_min_stop\":" << (result.risk.braking_at_min_stop ? "true" : "false") << ","
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
      << "\"effective_remaining_d_m\":";
  appendNullableNumber(oss, result.hybrid_log.effective_remaining_d_m);
  oss << ","
      << "\"at_min_stop\":" << (result.hybrid_log.at_min_stop ? "true" : "false") << ","
      << "\"hard_stop_reached_latched\":"
      << (result.hybrid_log.hard_stop_reached_latched ? "true" : "false") << ","
      << "\"forward_stop_hold_active\":"
      << (result.hybrid_log.forward_stop_hold_active ? "true" : "false") << ","
      << "\"standstill_guard_active\":"
      << (result.hybrid_log.standstill_guard_active ? "true" : "false") << ","
      << "\"standstill_min_clearance_m\":";
  appendNullableNumber(oss, result.hybrid_log.standstill_min_clearance_m);
  oss << ","
      << "\"sudden_obstacle_active\":"
      << (result.hybrid_log.sudden_obstacle_active ? "true" : "false") << ","
      << "\"sudden_brake_saturated\":"
      << (result.hybrid_log.sudden_brake_saturated ? "true" : "false") << ","
      << "\"sudden_a_req_m_s2\":";
  appendNullableNumber(oss, result.hybrid_log.sudden_a_req_m_s2);
  oss << ","
      << "\"sudden_obstacle_shadow\":"
      << (result.hybrid_log.sudden_obstacle_shadow ? "true" : "false") << ","
      << "\"pinned_escape_shadow\":"
      << (result.hybrid_log.pinned_escape_shadow ? "true" : "false") << ","
      << "\"rotation_escape_active\":"
      << (result.hybrid_log.rotation_escape_active ? "true" : "false") << ","
      << "\"degradation_active\":"
      << (result.hybrid_log.degradation_active ? "true" : "false") << ","
      << "\"closing_speed_m_s\":";
  appendNullableNumber(oss, result.hybrid_log.closing_speed_m_s);
  oss << ","
      << "\"forward_corridor_clearance_m\":";
  appendNullableNumber(oss, result.hybrid_log.forward_corridor_clearance_m);
  oss << ","
      << "\"front_lowpass_stop_active\":"
      << (result.hybrid_log.front_lowpass_stop_active ? "true" : "false") << ","
      << "\"front_lowpass_stop_tau_s\":" << result.hybrid_log.front_lowpass_stop_tau_s << ","
      << "\"front_lowpass_clearance_m\":";
  appendNullableNumber(oss, result.hybrid_log.front_lowpass_clearance_m);
  oss << ","
      << "\"front_lowpass_predicted\":"
      << (result.hybrid_log.front_lowpass_predicted ? "true" : "false") << ","
      << "\"front_lowpass_observation_age_s\":";
  appendNullableNumber(oss, result.hybrid_log.front_lowpass_observation_age_s);
  oss << ","
      << "\"speed_intervention_active\":"
      << (result.hybrid_log.speed_intervention_active ? "true" : "false") << ","
      << "\"speed_intervention_reason\":\""
      << result.hybrid_log.speed_intervention_reason << "\","
      << "\"speed_debug_joystick_linear_x\":"
      << result.hybrid_log.speed_debug_joystick_linear_x << ","
      << "\"speed_debug_desired_linear_x\":"
      << result.hybrid_log.speed_debug_desired_linear_x << ","
      << "\"speed_debug_controller_linear_x\":"
      << result.hybrid_log.speed_debug_controller_linear_x << ","
      << "\"speed_debug_previous_output_linear_x\":"
      << result.hybrid_log.speed_debug_previous_output_linear_x << ","
      << "\"speed_debug_output_linear_x\":"
      << result.hybrid_log.speed_debug_output_linear_x << ","
      << "\"speed_debug_wheel_linear_x\":"
      << result.hybrid_log.speed_debug_wheel_linear_x << ","
      << "\"speed_debug_reduction_m_s\":"
      << result.hybrid_log.speed_debug_reduction_m_s << ","
      << "\"speed_debug_braking\":"
      << (result.hybrid_log.speed_debug_braking ? "true" : "false") << ","
      << "\"arbiter_shadow_mismatch\":"
      << (result.hybrid_log.arbiter_shadow_mismatch ? "true" : "false") << ","
      << "\"direction_mask\":\""
      << (result.hybrid_log.direction_mask_forward ? "F" : "-")
      << (result.hybrid_log.direction_mask_backward ? "B" : "-")
      << (result.hybrid_log.direction_mask_left ? "L" : "-")
      << (result.hybrid_log.direction_mask_right ? "R" : "-") << "\","
      << "\"turn_guard_speed_cap_m_s\":";
  appendNullableNumber(oss, result.hybrid_log.turn_guard_speed_cap_m_s);
  oss << ","
      << "\"turn_envelope_scale\":" << result.hybrid_log.turn_envelope_scale << ","
      << "\"rapid_steer_cap_active\":"
      << (result.hybrid_log.rapid_steer_cap_active ? "true" : "false") << ","
      << "\"braking_mode\":\""
      << (params_.avoidance.min_distance_braking_enabled ? "min_distance" : "legacy") << "\","
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
      << "\"authority_alpha\":" << result.hybrid_log.authority_alpha << ","
      << "\"pulse_comp_active\":"
      << (result.hybrid_log.pulse_comp_active ? "true" : "false") << ","
      << "\"narrow_planner_active\":"
      << (result.hybrid_log.narrow_planner_active ? "true" : "false") << ","
      << "\"narrow_corridor_width_m\":";
  appendNullableNumber(oss, result.hybrid_log.narrow_corridor_width_m);
  oss << ","
      << "\"narrow_speed_cap_m_s\":";
  appendNullableNumber(oss, result.hybrid_log.narrow_speed_cap_m_s);
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
  front_lowpass_gate_.reset();
  front_lowpass_tilt_tracker_.reset();
  front_tilt_speed_cap_gate_.reset();
  perception_->clear();
  prev_output_ = geometry_msgs::msg::Twist{};
  {
    std::lock_guard<std::mutex> lock(viz_prev_output_mutex_);
    viz_prev_output_ = geometry_msgs::msg::Twist{};
  }
  {
    std::lock_guard<std::mutex> lock(ego_motion_mutex_);
    ego_dx_accum_m_ = 0.0;
    ego_dtheta_accum_rad_ = 0.0;
  }
}

void SafetyNode::onWatchdogTick()
{
  // ── 方案 7 控制环时效监督 ──
  const int64_t last_ns = last_tick_steady_ns_.load(std::memory_order_relaxed);
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const double period_s = 1.0 / std::max(1.0, params_.control_rate_hz);
  const double age_s = last_ns > 0 ?
    static_cast<double>(now_ns - last_ns) * 1e-9 :
    std::numeric_limits<double>::infinity();

  bool degraded = false;
  if (last_ns > 0 && age_s > 3.0 * period_s) {
    degraded = true;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%s 控制环超时：距上次 tick 已 %.0f ms（周期 %.0f ms）",
      LOG_PREFIX, age_s * 1e3, period_s * 1e3);
    // 停摆超过 10× 周期：看门狗兜底发布一帧零速。
    // 根本兜底仍是底盘侧对控制话题的 dead-man 超时（清单第 3 项，接口对齐）。
    // 无控制权时禁止发零速，保持「标准关+非巡航」静默语义。
    if (age_s > 10.0 * period_s && hasControlAuthority()) {
      publishControl(geometry_msgs::msg::Twist{});
    }
  }

  // ── 健康摘要 JSON（1 Hz）：使"当前真正生效的安全边界"运行时可自证 ──
  if (!health_pub_) {
    return;
  }
  double p50 = 0.0;
  double p99 = 0.0;
  double max_ms = 0.0;
  uint64_t arbiter_mismatch = 0;
  uint64_t sudden_shadow = 0;
  uint64_t pinned_shadow = 0;
  DegradationVerdict degradation_state;
  std::vector<double> sorted;   // 锁外排序用，reserve 避免扩容
  sorted.reserve(512);          // tick_ms_ring_ 固定 512 槽
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    control_loop_degraded_ = degraded;
    // 锁内只做数据拷贝，排序等 CPU 密集操作移到锁外，避免阻塞控制环 tick 统计写入。
    if (tick_ring_count_ > 0) {
      sorted.assign(
        tick_ms_ring_.begin(),
        tick_ms_ring_.begin() + static_cast<std::ptrdiff_t>(tick_ring_count_));
    }
    arbiter_mismatch = arbiter_mismatch_count_;
    sudden_shadow = sudden_shadow_count_;
    pinned_shadow = pinned_shadow_count_;
    degradation_state = last_degradation_;
  }
  // 锁外排序 + 百分位计算
  if (!sorted.empty()) {
    std::sort(sorted.begin(), sorted.end());
    p50 = sorted[sorted.size() / 2];
    p99 = sorted[std::min(sorted.size() - 1, sorted.size() * 99 / 100)];
    max_ms = sorted.back();
  }
  // 方案 8 WCET 预算：p99 ≤ 10 ms（50Hz 周期的 50%），超限告警
  if (p99 > 10.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "%s 控制环 tick p99=%.1f ms 超出 10 ms 预算", LOG_PREFIX, p99);
  }

  const PerceptionHealth health = perception_->health(
    now(), params_.perception_timeout_s);
  std::ostringstream oss;
  oss << "{"
      << "\"control_loop_degraded\":" << (degraded ? "true" : "false") << ","
      << "\"tick_ms_p50\":" << p50 << ","
      << "\"tick_ms_p99\":" << p99 << ","
      << "\"tick_ms_max\":" << max_ms << ","
      << "\"safety_enabled\":" << (safety_enabled_.load() ? "true" : "false") << ","
      << "\"standard_mode_enabled\":" <<
    (standard_mode_enabled_.load() ? "true" : "false") << ","
      << "\"cruise_enabled\":" << (cruise_enabled_.load() ? "true" : "false") << ","
      << "\"control_authority\":" << (hasControlAuthority() ? "true" : "false") << ","
      << "\"sources\":{";
  for (size_t i = 0; i < health.source_fresh.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << "\"" << degradation::sourceName(i) << "\":\""
        << (!health.source_enabled[i] ? "disabled" :
      (health.source_fresh[i] ? "fresh" : "lost")) << "\"";
  }
  oss << "},"
      << "\"degradation_active\":" << (degradation_state.active ? "true" : "false") << ","
      << "\"degradation_lost_sources\":\"" << degradation_state.lost_sources << "\","
      << "\"arbiter_shadow_mismatch_count\":" << arbiter_mismatch << ","
      << "\"sudden_shadow_count\":" << sudden_shadow << ","
      << "\"pinned_shadow_count\":" << pinned_shadow << ","
      << "\"layers\":{"
      << "\"direction_mask\":" << (params_.avoidance.direction_mask_enabled ? "true" : "false") << ","
      << "\"turn_clearance_guard\":"
      << (params_.avoidance.turn_clearance_guard_enabled ? "true" : "false") << ","
      << "\"turn_speed_envelope\":"
      << (params_.avoidance.turn_speed_envelope_enabled ? "true" : "false") << ","
      << "\"rapid_steer_cap\":"
      << (params_.avoidance.rapid_steer_cap_enabled ? "true" : "false") << ","
      << "\"sudden_obstacle\":" << (params_.avoidance.sudden_obstacle.enabled ? "true" : "false") << ","
      << "\"sudden_shadow_mode\":"
      << (params_.avoidance.sudden_obstacle.shadow_mode ? "true" : "false") << ","
      << "\"pinned_escape\":" << (params_.avoidance.pinned_escape.enabled ? "true" : "false") << ","
      << "\"pinned_shadow_mode\":"
      << (params_.avoidance.pinned_escape.shadow_mode ? "true" : "false") << ","
      << "\"retreat_escape\":" << (params_.map_constraint.retreat_escape_enabled ? "true" : "false") << ","
      << "\"closing_speed\":" << (params_.avoidance.closing_speed.enabled ? "true" : "false") << ","
      << "\"precise_stop\":" << (params_.avoidance.precise_stop.enabled ? "true" : "false") << ","
      << "\"shared_control\":" << (params_.avoidance.shared_control.enabled ? "true" : "false") << ","
      << "\"narrow_planner\":" << (params_.avoidance.narrow_planner.enabled ? "true" : "false") << ","
      << "\"standstill_guard\":" << (params_.avoidance.standstill_guard_enabled ? "true" : "false") << ","
      << "\"degradation\":" << (params_.degradation.enabled ? "true" : "false") << ","
      << "\"ego_motion_compensation\":"
      << (params_.local_map.ego_motion_compensation ? "true" : "false")
      << "}}";
  std_msgs::msg::String msg;
  msg.data = oss.str();
  health_pub_->publish(msg);
}

void SafetyNode::logStartupConfigSummary()
{
  // 方案 9.3：一次性打印生效安全边界，避免"以为全链路在保护"的配置误解
  RCLCPP_INFO(
    get_logger(),
    "%s 生效配置摘要 schema=v%d 执行器=%s | 控制环 %.0fHz | "
    "hard_stop=%.3f/释放%.3f | 前向余量%.2f 侧余量%.2f | "
    "direction_mask=%d turn_guard=%d(曲率门控%.2f) turn_envelope=%d "
    "rapid_steer_cap=%d(>%.2frad/s→%.2fm/s) | "
    "sudden=%d(shadow=%d) pinned=%d(shadow=%d) retreat_escape=%d | "
    "closing_speed=%d degradation=%d | map: rebuild=%d ego_comp=%d 膨胀%.3f | "
    "swept_cull=%.2f arbiter_shadow=%d",
    LOG_PREFIX,
    params_.config_schema_version,
    params_.multithreaded_executor ? "multi" : "single",
    params_.control_rate_hz,
    params_.avoidance.hard_stop_distance_m,
    params_.avoidance.hard_stop_release_distance_m,
    params_.avoidance.footprint_front_margin,
    params_.avoidance.footprint_side_margin,
    params_.avoidance.direction_mask_enabled ? 1 : 0,
    params_.avoidance.turn_clearance_guard_enabled ? 1 : 0,
    params_.avoidance.turn_clearance_guard_min_curvature_1_m,
    params_.avoidance.turn_speed_envelope_enabled ? 1 : 0,
    params_.avoidance.rapid_steer_cap_enabled ? 1 : 0,
    params_.avoidance.rapid_steer_trigger_angular_rad_s,
    params_.avoidance.rapid_steer_max_linear_m_s,
    params_.avoidance.sudden_obstacle.enabled ? 1 : 0,
    params_.avoidance.sudden_obstacle.shadow_mode ? 1 : 0,
    params_.avoidance.pinned_escape.enabled ? 1 : 0,
    params_.avoidance.pinned_escape.shadow_mode ? 1 : 0,
    params_.map_constraint.retreat_escape_enabled ? 1 : 0,
    params_.avoidance.closing_speed.enabled ? 1 : 0,
    params_.degradation.enabled ? 1 : 0,
    params_.local_map.rebuild_each_frame ? 1 : 0,
    params_.local_map.ego_motion_compensation ? 1 : 0,
    params_.local_map.inflation_radius_m,
    params_.avoidance.swept_cull_band_m,
    params_.avoidance.arbiter_shadow_enabled ? 1 : 0);
}

void SafetyNode::setupModeServices()
{
  // 与 scene_node 同名同接口：interface::srv::SceneControls，start=1 开启 / 0 关闭。
  // 端点：/scene/auxiliary（安全避障）、/scene/standard（标准）、/scene/cruise（巡航）。
  // 方案 7：服务与控制环同组（MutuallyExclusive）→ 模式切换与 evaluate 天然互斥，
  // resetPipelineState / 巡航状态无需额外加锁。
  safe_mode_service_ = create_service<interface::srv::SceneControls>(
    sceneAuxiliaryService,
    [this](
      const std::shared_ptr<interface::srv::SceneControls::Request> request,
      std::shared_ptr<interface::srv::SceneControls::Response> response) {
      handleSafeService(request, response);
    },
    rmw_qos_profile_services_default,
    control_group_);
  standard_mode_service_ = create_service<interface::srv::SceneControls>(
    sceneStandardService,
    [this](
      const std::shared_ptr<interface::srv::SceneControls::Request> request,
      std::shared_ptr<interface::srv::SceneControls::Response> response) {
      handleStandardService(request, response);
    },
    rmw_qos_profile_services_default,
    control_group_);
  cruise_mode_service_ = create_service<interface::srv::SceneControls>(
    sceneCruiseService,
    [this](
      const std::shared_ptr<interface::srv::SceneControls::Request> request,
      std::shared_ptr<interface::srv::SceneControls::Response> response) {
      handleCruiseService(request, response);
    },
    rmw_qos_profile_services_default,
    control_group_);
  // 巡航自动退出时回调 scene_services 的 /manage/cruise（与 scene_node 一致）。
  cruise_close_client_ =
    create_client<interface::srv::SceneControls>(manageCruiseService);
  RCLCPP_INFO(
    get_logger(), "%s 模式开关服务就绪：安全=%s 标准=%s 巡航=%s；巡航关闭客户端=%s",
    LOG_PREFIX,
    sceneAuxiliaryService, sceneStandardService, sceneCruiseService,
    manageCruiseService);
}

bool SafetyNode::loadAuxiliaryModeState()
{
  // 从 tft_manager 维护的统一配置恢复安全开关；生产默认关（见下方各回退分支）。
  // 空路径 = 显式跳过持久化（单测 / 无持久化环境）：按 mode.default 推导启动值，
  //（safe_assist 等非透传模式→开，pass_through→关），复现恢复特性引入前的模式映射，
  // 使依赖 mode.default 的单测无需持久化文件即可跑在预期剖面上。
  if (user_config_file_path_.empty()) {
    const bool from_mode = mode_supervisor_.mode() != SafetyMode::PASS_THROUGH;
    safety_enabled_.store(from_mode, std::memory_order_release);
    RCLCPP_INFO(
      get_logger(), "%s 未配置 user_config_file，按 mode.default 推导安全状态: %s",
      LOG_PREFIX, from_mode ? "开启" : "关闭");
    return false;
  }

  try {
    std::ifstream in(user_config_file_path_);
    if (!in.is_open()) {
      RCLCPP_INFO(
        get_logger(),
        "%s 未找到统一配置文件，使用默认安全状态: %s (文件: %s)",
        LOG_PREFIX,
        safety_enabled_.load(std::memory_order_acquire) ? "开启" : "关闭",
        user_config_file_path_.c_str());
      return false;
    }

    std::string line;
    bool found = false;
    bool enabled = safety_enabled_.load(std::memory_order_acquire);
    while (std::getline(in, line)) {
      // 兼容 "safety_warning_enabled: 1" 与带前导空格的 YAML 行
      if (line.find("safety_warning_enabled:") == std::string::npos) {
        continue;
      }
      const auto pos = line.find(':');
      if (pos == std::string::npos) {
        continue;
      }
      std::string value_part = line.substr(pos + 1);
      std::stringstream ss(value_part);
      int parsed = 1;
      if (ss >> parsed) {
        enabled = (parsed != 0);
        found = true;
        break;
      }
    }

    if (!found) {
      RCLCPP_WARN(
        get_logger(),
        "%s 统一配置中未找到 safety_warning_enabled，使用默认安全状态: %s",
        LOG_PREFIX, enabled ? "开启" : "关闭");
      return false;
    }

    safety_enabled_.store(enabled, std::memory_order_release);
    RCLCPP_INFO(
      get_logger(),
      "%s 已从统一配置恢复安全功能状态: %s (文件: %s)",
      LOG_PREFIX, enabled ? "开启" : "关闭", user_config_file_path_.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      get_logger(),
      "%s 读取统一配置失败，使用默认安全状态: %s, error=%s",
      LOG_PREFIX,
      safety_enabled_.load(std::memory_order_acquire) ? "开启" : "关闭",
      e.what());
    return false;
  }
}

void SafetyNode::setSafetyEnabled(bool enabled)
{
  const bool was = safety_enabled_.exchange(enabled);
  // 安全开 → SAFE_ASSIST 剖面（完整避障）；安全关 → PASS_THROUGH 剖面（仅绝对限速透传）。
  const SafetyMode target = enabled ? SafetyMode::SAFE_ASSIST : SafetyMode::PASS_THROUGH;
  const bool changed = mode_supervisor_.setMode(target);
  if (changed) {
    // 切换后清空地图/滤波/急停锁存，避免残留「前/后向制动保持」把车速锁 0。
    resetPipelineState();
  }
  if (was != enabled) {
    RCLCPP_INFO(
      get_logger(), "%s 安全避障已%s", LOG_PREFIX, enabled ? "开启" : "关闭（透传）");
    // 跳变即时上报（心跳同时在跑）：底盘/TFT 靠此话题同步安全功能开关状态。
    publishAuxiliaryStatus();
  }
}

bool SafetyNode::hasControlAuthority() const
{
  return standard_mode_enabled_.load(std::memory_order_acquire) ||
         cruise_enabled_.load(std::memory_order_acquire);
}

void SafetyNode::clearControlOutputState()
{
  output_shaper_.reset();
  front_lowpass_gate_.reset();
  front_lowpass_tilt_tracker_.reset();
  front_tilt_speed_cap_gate_.reset();
  prev_output_ = geometry_msgs::msg::Twist{};
  {
    std::lock_guard<std::mutex> lock(viz_prev_output_mutex_);
    viz_prev_output_ = geometry_msgs::msg::Twist{};
  }
}

void SafetyNode::setCruiseEnabled(bool enabled, bool notify_manager)
{
  const bool was = cruise_enabled_.exchange(enabled);
  if (enabled) {
    // 进入巡航：清空保持速度，开启「等待首个前进速度」的启动超时窗口。
    cruise_target_speed_ = 0.0;
    cruise_output_speed_ = 0.0;
    cruise_speed_timeout_active_ = true;
    cruise_enable_time_ = now();
  } else {
    cruise_target_speed_ = 0.0;
    cruise_output_speed_ = 0.0;
    cruise_speed_timeout_active_ = false;
  }
  cruise_forward_speed_reported_ = false;
  cruise_obstacle_zero_active_ = false;
  if (was != enabled) {
    RCLCPP_INFO(get_logger(), "%s 巡航定速保持已%s", LOG_PREFIX, enabled ? "开启" : "关闭");
    // 激活成功即时上报；退出原因码由各退出点在调用本函数前发布（1/2），
    // 后退意图与手动抱闸退出不带原因码（与 scene_node 一致）。
    if (enabled) {
      publishCruiseStatus(CRUISE_ACTIVATED);
    } else if (!standard_mode_enabled_.load(std::memory_order_acquire)) {
      // 巡航关且标准已关 → 失去控制权，清空输出状态后静默。
      clearControlOutputState();
    }
  }
  // 仅自动退出路径通知管理端；手动 /scene/cruise 关断由 scene_services 自己编排。
  if (was && !enabled && notify_manager) {
    notifyManageCruiseClose();
  }
}

void SafetyNode::notifyManageCruiseClose()
{
  if (!cruise_close_client_) {
    return;
  }
  auto request = std::make_shared<interface::srv::SceneControls::Request>();
  request->start = 0;
  cruise_close_client_->async_send_request(
    request,
    [this](rclcpp::Client<interface::srv::SceneControls>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (response->code == 0) {
          RCLCPP_INFO(get_logger(), "%s 已通知 /manage/cruise 关闭巡航", LOG_PREFIX);
        } else {
          RCLCPP_ERROR(
            get_logger(), "%s /manage/cruise 关闭失败: code=%d",
            LOG_PREFIX, response->code);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(), "%s /manage/cruise 调用异常: %s", LOG_PREFIX, e.what());
      }
    });
}

void SafetyNode::handleSafeService(
  const std::shared_ptr<interface::srv::SceneControls::Request> request,
  std::shared_ptr<interface::srv::SceneControls::Response> response)
{
  setSafetyEnabled(request->start == 1);
  response->code = 0;
  response->message = request->start == 1 ? "安全避障已开启" : "安全避障已关闭";
}

void SafetyNode::handleStandardService(
  const std::shared_ptr<interface::srv::SceneControls::Request> request,
  std::shared_ptr<interface::srv::SceneControls::Response> response)
{
  // 对齐 scene_node：置位标准模式；开启时清空巡航保持速度缓存。
  // 关闭且非巡航时失去控制权 → 不发 /nav/control、跳过避障。
  if (request->start == 1) {
    standard_mode_enabled_.store(true, std::memory_order_release);
    cruise_target_speed_ = 0.0;
    cruise_output_speed_ = 0.0;
    RCLCPP_INFO(get_logger(), "%s 标准功能已开启", LOG_PREFIX);
    response->code = 0;
    response->message = "标准功能已开启";
  } else {
    standard_mode_enabled_.store(false, std::memory_order_release);
    if (!cruise_enabled_.load(std::memory_order_acquire)) {
      clearControlOutputState();
      RCLCPP_INFO(
        get_logger(), "%s 标准功能已关闭（非巡航：停止 /nav/control，跳过避障）",
        LOG_PREFIX);
    } else {
      RCLCPP_INFO(
        get_logger(), "%s 标准功能已关闭（巡航中：继续输出）", LOG_PREFIX);
    }
    response->code = 0;
    response->message = "标准功能已关闭";
  }
}

void SafetyNode::handleCruiseService(
  const std::shared_ptr<interface::srv::SceneControls::Request> request,
  std::shared_ptr<interface::srv::SceneControls::Response> response)
{
  setCruiseEnabled(request->start == 1, false);
  response->code = 0;
  response->message = request->start == 1 ? "巡航已开启" : "巡航已关闭";
}

geometry_msgs::msg::Twist SafetyNode::updateCruise(
  const ControlCommand & control, double gear_max_speed, bool brake_engaged, bool & holding)
{
  holding = false;
  const geometry_msgs::msg::Twist & handle = control.twist;
  const double handle_linear =
    std::isfinite(handle.linear.x) ? handle.linear.x : 0.0;
  const double handle_angular =
    std::isfinite(handle.angular.z) ? handle.angular.z : 0.0;

  // ── 退出条件 1：手动抱闸开启 → 关闭巡航（安全开关不变），并通知 /manage/cruise ──
  if (brake_engaged) {
    setCruiseEnabled(false, true);
    return handle;
  }

  // ── 退出条件 2：后退意图 → 关闭巡航，透传原始手柄命令（交由常规反向快停） ──
  const double reverse_release =
    std::max(0.0, params_.direction_reversal_release_speed_m_s);
  if (control.driver_reverse_request || handle_linear < -reverse_release) {
    setCruiseEnabled(false, true);
    return handle;
  }

  // 有效速度上限 = min(档位最高速, 绝对上限)；gear_max_speed<=0 表示 CarInfo 未知，仅用绝对上限。
  const double abs_cap = std::max(0.0, params_.avoidance.absolute_max_linear_speed);
  const double speed_cap = gear_max_speed > 0.0 ? std::min(gear_max_speed, abs_cap) : abs_cap;

  // 记录/提升保持目标速度：仅前进，且不超过有效上限。获得有效前进速度后关闭启动超时。
  if (handle_linear > 0.0 && handle_linear > cruise_target_speed_) {
    cruise_target_speed_ = std::min(handle_linear, speed_cap);
    cruise_speed_timeout_active_ = false;
    // 首次拿到前进速度 = 巡航真正开始生效，上报一次（本次巡航内只报一次）。
    if (!cruise_forward_speed_reported_) {
      cruise_forward_speed_reported_ = true;
      publishCruiseStatus(CRUISE_STARTED);
    }
  }
  // 档位下降导致上限降低时，把保持目标钳到当前上限（后续由 OutputShaper 平滑降速）。
  if (cruise_target_speed_ > speed_cap) {
    cruise_target_speed_ = speed_cap;
  }

  // ── 退出条件 3：启动超时 → 进入巡航后 speed_timeout_s 内未给出前进速度则关闭巡航 ──
  if (cruise_speed_timeout_active_) {
    const double elapsed = (now() - cruise_enable_time_).seconds();
    if (std::isfinite(elapsed) && elapsed > std::max(0.0, params_.cruise.speed_timeout_s)) {
      publishCruiseStatus(CRUISE_END_NO_FORWARD_SPEED);
      setCruiseEnabled(false, true);
      return handle;
    }
  }

  // 转向降速：|w| 超阈值时按 (阈值/|w|)² 压低线速度；否则每周期逐步恢复到目标速度。
  const double abs_w = std::abs(handle_angular);
  const double w_thresh = std::max(1e-3, params_.cruise.angular_speed_threshold_rad_s);
  double actual = cruise_output_speed_;
  if (cruise_target_speed_ <= 0.0) {
    actual = 0.0;
  } else if (abs_w > w_thresh) {
    double ratio = w_thresh / abs_w;
    ratio *= ratio;
    actual = std::max(0.0, cruise_target_speed_ * ratio);
  } else if (actual < cruise_target_speed_) {
    actual = std::min(
      cruise_target_speed_,
      actual + std::max(0.0, params_.cruise.speed_recovery_rate_m_s));
  } else {
    actual = cruise_target_speed_;
  }
  cruise_output_speed_ = actual;

  geometry_msgs::msg::Twist out;
  out.linear.x = actual > 0.0 ? actual : 0.0;
  out.angular.z = handle_angular;    // 角速度仍由摇杆控制
  holding = actual > 0.0;
  return out;
}
