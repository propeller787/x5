/**
 * @file avoidance_types.h
 * @brief safety_node 避障管线共享类型定义
 *
 * 本文件集中定义混合避障（Footprint + LocalMap）所需的全部数据结构，
 * 供 core 算法层与 ROS 节点层共同引用，避免循环依赖。
 *
 * 坐标系约定：除特别说明外，位姿与障碍物均在 base_link 下车体坐标系表达。
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>

/** 安全运行模式，对应 HYBRID_FOOTPRINT_LOCALMAP.md §6 */
enum class SafetyMode
{
  PASS_THROUGH,    ///< 透传：仅绝对限速，不做避障
  SAFE_ASSIST,     ///< 默认：Footprint 选速 + LocalMap 硬约束
  CRUISE,          ///< 巡航：叠加线速度上限
  NARROW,          ///< 窄道/过门：提高地图约束灵敏度
  ESTOP,           ///< 急停/故障
};

/** 从 ROS参数字符串解析安全模式 */
inline SafetyMode parseSafetyMode(const std::string & s)
{
  if (s == "pass_through") {
    return SafetyMode::PASS_THROUGH;
  }
  if (s == "cruise") {
    return SafetyMode::CRUISE;
  }
  if (s == "narrow") {
    return SafetyMode::NARROW;
  }
  if (s == "estop") {
    return SafetyMode::ESTOP;
  }
  return SafetyMode::SAFE_ASSIST;
}

/** 轻量圆形障碍物 primitive，供 Footprint 扫掠碰撞使用 */
struct ObstaclePrimitive
{
  double x = 0.0;                ///< 中心 x (m)
  double y = 0.0;                ///< 中心 y (m)
  double z = 0.0;                ///< 中心 z (m)，LocalMap 落格时可过滤
  double radius = 0.01;          ///< 等效碰撞半径 (m)
  std::string source;            ///< 来源雷达标识
  rclcpp::Time stamp {rclcpp::Time(0, 0, RCL_ROS_TIME)};
  bool valid = false;
};

struct ObstaclePoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::string source;
};

/** 当前帧障碍物场：Footprint 扫掠检测的输入 */
struct ObstacleField
{
  std::vector < ObstaclePrimitive > obstacles;
  rclcpp::Time stamp {rclcpp::Time(0, 0, RCL_ROS_TIME)};
};

/** 轨迹上某一时刻的预测位姿（相对当前 base_link） */
struct PredictedPose
{
  double x = 0.0;        ///< 预测 x (m)
  double y = 0.0;        ///< 预测 y (m)
  double theta = 0.0;    ///< 预测航向 (rad)
  double t = 0.0;        ///< 相对当前时刻的预测时间 (s)
};

/** 线速度 v + 角速度 w 候选 */
struct VelocityCandidate
{
  double v = 0.0;    ///< 线速度 (m/s)
  double w = 0.0;    ///< 角速度 (rad/s)
};

/** 单条候选速度对应的离散轨迹 */
struct TrajectoryRollout
{
  VelocityCandidate cmd;
  std::vector < PredictedPose > poses;
};

/** 单条轨迹的 Footprint 碰撞检测结果 */
struct CollisionCheckResult
{
  bool collision = false;
  double ttc = std::numeric_limits < double > ::infinity();         ///< 首次碰撞时间 (s)
  double min_distance = std::numeric_limits < double > ::infinity(); ///< 全程最小净空 (m)
  double path_length_m = 0.0;
  double hard_stop_path_distance_m = std::numeric_limits < double > ::infinity();
  double collision_path_distance_m = std::numeric_limits < double > ::infinity();
  double hard_stop_x = 0.0;
  double hard_stop_y = 0.0;
  double collision_x = 0.0;
  double collision_y = 0.0;
  std::string source;
  int pose_index = -1;    ///< 首次碰撞所在轨迹点索引
};

/** 碰撞相对车体的主方向 */
enum class CollisionDirection
{
  NONE,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
};

/** 四向制动/减速干预等级（供日志与扩展报警） */
struct SweptInterventionLevels
{
  CollisionDirection primary = CollisionDirection::NONE;
  bool forward_brake = false;
  bool forward_decel = false;
  bool backward_brake = false;
  bool backward_decel = false;
  bool left_brake = false;
  bool left_decel = false;
  bool right_brake = false;
  bool right_decel = false;
};

/** 多候选评估后的聚合碰撞风险 */
struct CollisionRisk
{
  bool has_collision = false;
  bool emergency = false;
  bool braking_limited = false;
  double min_ttc = std::numeric_limits < double > ::infinity();
  double min_clearance = std::numeric_limits < double > ::infinity();
  double braking_available_distance_m = std::numeric_limits < double > ::infinity();
  double braking_speed_limit_m_s = std::numeric_limits < double > ::infinity();
  double braking_stopping_distance_m = 0.0;
  CollisionCheckResult worst;
  CollisionDirection primary_direction = CollisionDirection::NONE;
  SweptInterventionLevels levels;
  double min_lateral_left_distance = 1e6;     ///< 左侧最近障碍边距 (m)
  double min_lateral_right_distance = 1e6;    ///< 右侧最近障碍边距 (m)
};

inline bool brakingEnvelopeCanStop(const CollisionRisk & risk)
{
  return risk.braking_limited &&
         std::isfinite(risk.braking_available_distance_m) &&
         risk.braking_available_distance_m > 1e-3 &&
         risk.braking_stopping_distance_m <= risk.braking_available_distance_m + 1e-6;
}

/** SafetyFilter 单步输出 */
struct SafetyFilterResult
{
  geometry_msgs::msg::Twist safe_twist;
  CollisionRisk risk;
  bool all_candidates_collide = false;
  int selected_candidate_index = -1;
};

/** 单次 evaluate 的上下文（模式、巡航上限等） */
struct AvoidanceContext
{
  bool cruise_active = false;
  double cruise_linear_cap = 0.0;
  SafetyMode safety_mode = SafetyMode::SAFE_ASSIST;
};

struct NarrowPassageConfig
{
  double footprint_side_margin_m = 0.01;
  double obstacle_radius_m = 0.01;
  double hard_stop_distance_m = 0.015;
  double hard_stop_release_distance_m = 0.02;
  double local_map_inflation_radius_m = 0.01;
  double lethal_threshold = 0.65;
};

/** Rolling LocalMap 配置，见 HYBRID_FOOTPRINT_LOCALMAP.md §3 */
struct LocalMapConfig
{
  bool enabled = true;
  double width_m = 6.0;
  double height_m = 6.0;
  double resolution_m = 0.05;
  double inflation_radius_m = 0.15;
  double lethal_threshold = 0.65;
  double decay_halflife_s = 0.35;
  double max_hit_age_s = 0.8;
  double observation_weight = 0.85;
  bool rebuild_each_frame = true;    ///< V1：每周期全量重建，无跨帧平移
  double tilt_z_threshold = 0.5;
  bool integrate_primitive_centers = false;
  double primitive_observation_weight = 0.6;
};

/** Map 硬约束（veto）配置 */
struct MapConstraintConfig
{
  bool enabled = true;
  bool veto_on_lethal = true;             ///< 地图 lethal 是否一票否决候选
  bool check_footprint_corners = true;    ///< 沿轨迹检查四角点而非仅中心
};

/** 滚动局部占据栅格快照（供 MapConstraintCheck 查询） */
struct RollingLocalMapGrid
{
  std::vector < float > probability; ///< 原始占据概率 [0,1]
  std::vector < float > inflated;      ///< 膨胀层概率 [0,1]
  rclcpp::Time stamp {rclcpp::Time(0, 0, RCL_ROS_TIME)};
  double resolution_m = 0.05;
  double origin_x_m = 0.0;    ///< 栅格 (0,0) 左下角世界坐标 x
  double origin_y_m = 0.0;
  uint32_t rows = 0;
  uint32_t cols = 0;
  double lethal_threshold = 0.65;
};

/** 轨迹沿 LocalMap 的 lethal 查询结果 */
struct MapConstraintResult
{
  bool lethal = false;
  double first_lethal_t = std::numeric_limits < double > ::infinity();
  int lethal_cell_col = -1;
  int lethal_cell_row = -1;
  double p_max_on_path = 0.0;
};

/** 混合避障可解释日志摘要（发布到 /safety/intervention_log） */
struct HybridAvoidanceLog
{
  SafetyMode mode = SafetyMode::SAFE_ASSIST;
  int footprint_selected_candidate = -1;    ///< Footprint 最优候选索引
  int final_candidate = -1;                 ///< Map veto 后最终候选索引
  bool map_veto_applied = false;
  double map_ttc = std::numeric_limits < double > ::infinity();
  double tilt_min_distance_m = std::numeric_limits < double > ::infinity();
  double tilt_min_ttc = std::numeric_limits < double > ::infinity();
  double tilt_speed_limit_m_s = std::numeric_limits < double > ::infinity();
  std::string tilt_intervention = "none";
  std::string tilt_collision_source;
  double tick_ms = 0.0;
};

enum class EmergencyStopReason
{
  NONE,
  INPUT_FAULT,
  MODE_ESTOP,
  MAP_VETO,
  EMERGENCY_TTC,
  BRAKING_DISTANCE_INSUFFICIENT,
  HARD_STOP_DISTANCE,
  RISK_EMERGENCY,
  BRAKE_HOLD,
  TILT_NEAR_DISTANCE,
  TILT_TTC,
  UNKNOWN,
};

enum class RecoveryState
{
  IDLE,
  CONFIRMING,
  ACTIVE,
  BLOCKED,
  ABORTED_TIMEOUT,
  ABORTED_NO_PROGRESS,
  ABORTED_FEEDBACK_STALL,
};

/** AvoidanceController 单周期完整输出 */
struct AvoidanceResult
{
  geometry_msgs::msg::Twist safe_twist;
  CollisionRisk risk;
  bool emergency_brake = false;
  bool recovery_active = false;
  RecoveryState recovery_state = RecoveryState::IDLE;
  double recovery_elapsed_s = 0.0;
  double recovery_clearance_gain_m = 0.0;
  EmergencyStopReason emergency_reason = EmergencyStopReason::NONE;
  double tick_duration_ms = 0.0;
  HybridAvoidanceLog hybrid_log;
};

/** 双梯形 footprint 单段参数（全宽 + 段长，base_link x 向前） */
struct FootprintSegmentParams
{
  double length_m = 0.0;
  double width_front_m = 0.0;   ///< 段前缘（x 较大侧）全宽
  double width_rear_m = 0.0;    ///< 段后缘（x 较小侧）全宽
};

/** Footprint 扫掠避障可调参数 */
struct TiltTtcParams
{
  bool enabled = false;
  double horizon_s = 0.5;
  double max_probe_distance_m = 1.0;
  double hard_stop_distance_m = 0.12;
  double limiting_distance_m = 0.35;
  double release_distance_m = 0.40;
  double limiting_ttc_s = 0.55;
  double emergency_ttc_s = 0.35;
  double emergency_ttc_release_s = 0.50;
  double emergency_clear_hold_s = 0.15;
  double min_speed_for_ttc_m_s = 0.25;
  double obstacle_radius_m = 0.01;
  bool use_command_velocity_for_ttc = true;
};

struct AvoidanceParams
{
  // ── 轨迹采样 ──
  double rollout_horizon_s = 1.2;
  double rollout_dt_s = 0.1;
  int rollout_speed_samples = 6;
  int rollout_angular_samples = 5;

  // ── 车体几何 ──
  double robot_length = 0.88;
  double robot_width_rear_m = 0.70;
  double robot_width_front_m = 0.70;
  double robot_rear_offset = 0.22;    ///< 后轴到几何中心的偏移
  /** true：6 顶点双梯形 physicalPolygon；false：4 顶点单梯形（旧参数） */
  bool footprint_use_dual_trapezoid = false;
  FootprintSegmentParams footprint_front_segment {};
  FootprintSegmentParams footprint_rear_segment {};

  double footprint_front_margin = 0.10;
  double footprint_back_margin = 0.12;
  double footprint_side_margin = 0.05;
  double footprint_obstacle_radius_default = 0.01;
  /** 是否启用物理车体框侵入判定（physicalPolygon）；关闭后不触发侵入型 recovery */
  bool footprint_intrusion_check_enabled = true;

  // ── TTC / 净空阈值 ──
  double comfort_clearance = 0.30;
  double warning_ttc_s = 1.5;
  double limiting_ttc_s = 1.0;
  bool use_ttc_emergency_stop = false;
  double emergency_ttc_s = 0.45;
  /** 是否启用硬停净空判据（关闭后仍保留 Footprint 碰撞、制动包络与 Map veto） */
  bool hard_stop_clearance_enabled = true;
  double hard_stop_distance_m = 0.12;
  double emergency_ttc_release_s = 0.70;
  double hard_stop_release_distance_m = 0.20;
  double emergency_clear_hold_s = 0.15;

  // ── 选速与输出整形 ──
  double safety_filter_angular_weight = 0.30;
  double max_decel_m_s2 = 1.2;
  double max_recover_accel_m_s2 = 0.5;
  bool braking_envelope_enabled = true;
  double braking_lookahead_m = 5.0;
  double braking_path_step_m = 0.025;
  double braking_reaction_delay_s = 0.10;
  double braking_comfort_decel_m_s2 = 0.60;
  double braking_max_jerk_m_s3 = 2.5;
  double braking_stop_buffer_m = 0.02;
  double candidate_speed_up_hold_s = 0.30;
  bool recovery_mode_enabled = true;
  double recovery_max_linear_speed = 0.12;
  double recovery_max_angular_speed = 0.35;
  double recovery_lookahead_s = 0.35;
  double recovery_min_command = 0.02;
  double recovery_min_clearance_improvement_m = 0.005;
  double recovery_max_transient_clearance_loss_m = 0.001;
  bool recovery_auto_select_enabled = true;
  double recovery_intrusion_confirm_s = 0.10;
  double recovery_candidate_hold_s = 0.30;
  double recovery_timeout_s = 1.50;
  double recovery_no_progress_timeout_s = 0.50;
  double recovery_motion_grace_s = 0.25;
  double recovery_feedback_stall_timeout_s = 0.35;
  double recovery_feedback_linear_min_m_s = 0.01;
  double recovery_feedback_angular_min_rad_s = 0.03;

  double absolute_max_linear_speed = 2.222;
  double absolute_max_angular_speed = 3.0;

  size_t max_obstacle_primitives = 128;

  bool enable_forward_collision = true;
  bool enable_lateral_collision = true;
  bool enable_backward_collision = true;

  double max_angular_decel_rad_s2 = 2.0;
  double max_angular_recover_rad_s2 = 1.0;
  TiltTtcParams tilt_ttc;
};

/** 候选轨迹净空是否满足硬停阈值（关闭硬停净空时恒为 true） */
inline bool footprintCandidateClearanceOk(
  double min_distance,
  const AvoidanceParams & params)
{
  return !params.hard_stop_clearance_enabled ||
         min_distance > params.hard_stop_distance_m;
}

/** 制动包络可用停车距离：启用硬停时用 hard_stop 边界，否则用碰撞边界 */
inline double brakingEnvelopeAvailableDistance(
  const CollisionCheckResult & check,
  const AvoidanceParams & params)
{
  if (params.hard_stop_clearance_enabled &&
    std::isfinite(check.hard_stop_path_distance_m))
  {
    return check.hard_stop_path_distance_m;
  }
  if (std::isfinite(check.collision_path_distance_m)) {
    return check.collision_path_distance_m;
  }
  return std::numeric_limits < double > ::infinity();
}

/** 由碰撞点坐标派生主碰撞方向 */
CollisionDirection deriveCollisionDirection(double x, double y);

/** 统计障碍物场左右侧最近边距 */
void estimateLateralDistances(const ObstacleField & field, double & left_dist, double & right_dist);

/** 根据 TTC/净空填充四向干预等级 */
void fillInterventionLevels(CollisionRisk & risk, const AvoidanceParams & params);
