/**
 * @file safety_params.h
 * @brief 从 ROS2 参数服务器加载 SafetyNode 全部配置
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/degradation_policy.h"
#include "safety_node/core/front_lowpass_stop.h"
#include "safety_node/core/warning_reporter.h"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

/** 水平雷达（left/right/back LaserScan）感知开关 */
struct HorizontalPerceptionConfig
{
  bool enabled = true;
  bool left_enabled = true;
  bool right_enabled = true;
  bool back_enabled = true;
  bool feed_local_map = true;
};

/** 节点级参数聚合 */
struct TiltPerceptionConfig
{
  /** @deprecated 已忽略；斜雷达始终经 TF 变换到 base_link */
  std::string coordinate_frame = "sensor";
  bool publish_debug_scan = true;
  bool enabled = true;
  bool feed_local_map = false;
  bool ground_reject_enabled = true;
  /** false：不对斜雷达点做 |z| 高度裁剪（仍要求 z 有限） */
  bool z_filter_enabled = false;
  /** true：lookupTransform 后 x/y 在 base_link，z 保留传感器 frame 原始值 */
  bool preserve_sensor_z = true;
  double z_filter_threshold = 0.0;
  double voxel_size_m = 0.08;
  int max_primitives = 48;
};

/** 感知 ROI 边界（替代 perception_utils.cpp 硬编码），默认 [-2.5,6.5]×[-1.8,1.8] */
struct PerceptionRoiConfig
{
  double x_min_m = -2.5;
  double x_max_m = 6.5;
  double y_min_m = -1.8;
  double y_max_m = 1.8;
};

/** 巡航（定速保持）模式参数，行为对齐 scene_node 巡航功能 */
struct CruiseConfig
{
  /** 角速度阈值 (rad/s)：|w| 超过时按 (阈值/|w|)² 压低线速度，防止转弯倾倒 */
  double angular_speed_threshold_rad_s = 0.5;
  /** 转向降速后线速度每控制周期的恢复增量 (m/s) */
  double speed_recovery_rate_m_s = 0.05;
  /** 启动超时 (s)：进入巡航后该时间内未给出前进速度则自动退出 */
  double speed_timeout_s = 8.0;
  /**
   * 巡航建立后，干预输出线速度 ≤ 此值视为「被障碍物压停」(m/s)。
   * 持续 speed_timeout_s 则退出巡航并上报 /vlago/cruise/status = 2，对齐 scene_node
   * 「预警速度为 0 超过 8 秒自动退出巡航」的行为。
   */
  double obstacle_zero_speed_eps_m_s = 0.02;
};

/**
 * 对外状态/预警上报参数（reporting:）。
 * 三个话题与 scene_node 完全同构，供 TFT / 灯效 / 语音 / 底盘消费：
 * - /scene_auxiliary/status  (std_msgs/Bool)  安全（避障）开关状态，周期心跳 + 跳变即时
 * - /vlago/cruise/status     (std_msgs/Int32) 巡航过程/结束原因码（1/2/3/4）
 * - /vlago/collision/warning (interface/CollisionWarning) 四向碰撞预警，见 core/warning_reporter.h
 */
struct ReportingConfig
{
  /** /scene_auxiliary/status 上报开关 */
  bool auxiliary_status_enabled = true;
  /** /scene_auxiliary/status 心跳频率 (Hz)；开关跳变时另外立即发一帧 */
  double auxiliary_status_rate_hz = 1.0;
  /** /vlago/cruise/status 上报开关 */
  bool cruise_status_enabled = true;
  /** /vlago/collision/warning 上报参数 */
  CollisionWarningReporterConfig collision_warning;
};

/** 参数校验问题（方案 9），FATAL 拒绝启动、WARN 记日志+健康摘要 */
struct ParamIssue
{
  enum Level { FATAL, WARN };
  Level level = FATAL;
  std::string message;
};

struct SafetyNodeParams
{
  /** 当前代码支持的配置 schema 主版本（方案 9.3） */
  static constexpr int kSupportedConfigSchemaVersion = 1;

  /** yaml config_schema_version；与 kSupported 不一致时校验 FATAL */
  int config_schema_version = kSupportedConfigSchemaVersion;
  /** 方案 7：true 时 main 使用 MultiThreadedExecutor + 回调分组（灰度默认关） */
  bool multithreaded_executor = false;
  double control_rate_hz = 20.0;
  double control_command_timeout_s = 0.25;
  double joystick_timeout_s = 0.25;
  int joystick_deadzone = 5;
  int joystick_reverse_threshold = 20;
  double wheel_feedback_timeout_s = 0.25;
  bool require_wheel_feedback = true;
  double car_info_timeout_s = 1.0;
  double perception_timeout_s = 0.35;
  double tf_lookup_timeout_s = 0.02;
  int min_fresh_perception_sources = 1;
  bool require_perception = true;
  int min_fresh_horizontal_sources = 1;
  bool require_horizontal_perception = true;
  bool require_tilt_perception = false;
  double emergency_release_delay_s = 0.15;
  double neutral_stop_filter_time_constant_s = 0.30;
  bool direction_reversal_fast_stop_enabled = true;
  double direction_reversal_stop_decel_m_s2 = 2.5;
  double direction_reversal_stop_jerk_m_s3 = 8.0;
  double direction_reversal_release_speed_m_s = 0.03;
  /** 前向近障低通刹停（启动级开关，默认关），见 core/front_lowpass_stop.h */
  FrontObstacleLowpassStopParams front_obstacle_lowpass_stop;

  AvoidanceParams avoidance;
  HorizontalPerceptionConfig horizontal_perception;
  TiltPerceptionConfig tilt_perception;
  PerceptionRoiConfig perception;
  NarrowPassageConfig narrow_passage;
  LocalMapConfig local_map;
  MapConstraintConfig map_constraint;
  CruiseConfig cruise;
  ReportingConfig reporting;
  DegradationConfig degradation;
  SafetyMode default_mode = SafetyMode::SAFE_ASSIST;

  /** 从已 declare 的 ROS 参数读取，键名见 config/safety_node_params.yaml */
  static SafetyNodeParams loadFromNode(rclcpp::Node & node);
};

/**
 * 方案 9 启动期参数校验：范围类 + 跨参数一致性类（把 yaml 注释里的约束代码化）。
 * 纯函数，无 ROS 依赖，可单测（test_param_validation.cpp）。
 */
std::vector<ParamIssue> validateParams(const SafetyNodeParams & params);

/**
 * WARN 逐条打日志；存在 FATAL 时打齐全部错误后抛 std::runtime_error 拒绝启动
 * （main.cpp 已捕获 → 退出码 1，fail-fast）。
 */
void validateParamsOrThrow(const SafetyNodeParams & params, const rclcpp::Logger & logger);
