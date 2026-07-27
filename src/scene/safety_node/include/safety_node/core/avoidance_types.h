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

#include "safety_node/core/front_tilt_speed_cap.h"

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
  /** min(路径 hard-stop 剩余, 净空剩余)；MDB 模式下用于限速 */
  double braking_effective_remaining_m = std::numeric_limits < double > ::infinity();
  bool braking_at_min_stop = false;
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
  // ── 传感器降级矩阵裁决（方案 4，由节点层按 per-source 新鲜度计算） ──
  // 生效方式与 direction_mask 同款：evaluate 入口对 eval_cmd 分轴裁剪 + 限速钳制。
  bool degradation_active = false;
  bool degradation_block_forward = false;
  bool degradation_block_backward = false;
  bool degradation_block_left = false;
  bool degradation_block_right = false;
  /** 降级合成限速 (m/s)；+inf = 不限速 */
  double degradation_speed_cap_m_s = std::numeric_limits < double > ::infinity();
  /** 斜雷达前向近障软限速 (m/s)（见 front_tilt_speed_cap.h）；+inf = 不限速。
   *  由节点层按斜雷达前向走廊净空计算，evaluate 入口仅对正向 eval_cmd 取 min。 */
  double front_tilt_speed_cap_m_s = std::numeric_limits < double > ::infinity();
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
  /**
   * 方案 6A 自运动补偿：地图固定在 base_link 车体系，跨帧保留（rebuild=false）
   * 必须先把历史栅格按自车位移反向平移，否则车动后旧障碍格全部错位。
   * true 且 rebuild_each_frame=false 时，节点每个地图周期用轮速积分调用
   * applyEgoMotion()：整格滚动搬移 + 亚格残差累积；旋转不做重采样，
   * 改为累计角行程超阈值时对全图概率乘额外衰减（转弯多则记忆快速失效）。
   * rebuild_each_frame=true 时本开关无效果（每帧重建天然无错位）。
   */
  bool ego_motion_compensation = false;
  /** 旋转惩罚触发阈值 (rad)：累计 |dθ| 超过后对全图施加一次额外衰减 */
  double rotation_decay_threshold_rad = 0.20;
  /** 旋转惩罚衰减系数 (0,1]：触发时全图概率乘以该值 */
  double rotation_decay_factor = 0.5;
};

/** Map 硬约束（veto）配置 */
struct MapConstraintConfig
{
  bool enabled = true;
  bool veto_on_lethal = true;             ///< 地图 lethal 是否一票否决候选
  bool check_footprint_corners = true;    ///< 沿轨迹检查四角点而非仅中心
  /**
   * L2 单调退离逃逸放宽（默认关）。trajectoryEscapesInitialLethal 原本要求
   * 「窗内完全清空初始 lethal」；贴障脱困（如前障贴身倒退）在受控速度下常清不及，
   * 被误判成不可逃逸而钉死。开启后额外接受「不进任何新 lethal 且对初始 lethal 集
   * 单调退离（覆盖格数逐位姿非增且末位姿严格更少）」的轨迹——方向天然朝安全。
   * 仅放行、可证明安全方向；关闭时行为逐位一致。见 map_constraint_check.cpp。
   */
  bool retreat_escape_enabled = false;
  /** 退离接受所需的最小进展：末位姿覆盖的初始-lethal 格数须比起始至少少这么多。 */
  double retreat_min_progress_cells = 1.0;
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

/** LocalMap 近障豁免圆（L1 方向解耦）：圆内 lethal 采样点在 veto 判定中忽略 */
struct MapExemptCircle
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
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
  /** 斜雷达通道：膨胀 Footprint 前缘到最近前向障碍的边距 (m) */
  double tilt_min_distance_m = std::numeric_limits < double > ::infinity();
  /** 斜雷达通道：沿短窗探测轨迹的最小 TTC (s) */
  double tilt_min_ttc = std::numeric_limits < double > ::infinity();
  /** 斜雷达 limiting 时计算出的线速度上限 (m/s) */
  double tilt_speed_limit_m_s = std::numeric_limits < double > ::infinity();
  /** 斜雷达干预级别：none | limiting | emergency */
  std::string tilt_intervention = "none";
  /** 触发干预的斜雷达图元来源：left_tilt | right_tilt 等 */
  std::string tilt_collision_source;
  double effective_remaining_d_m = std::numeric_limits < double > ::infinity();
  bool at_min_stop = false;
  bool hard_stop_reached_latched = false;
  bool forward_stop_hold_active = false;
  /** P1 连续接管系数 α∈[0,1]；0=完全跟手，1=完全过载驾驶员意图 */
  double authority_alpha = 0.0;
  /** P0 脉冲补偿本周期是否输出前向微脉冲 */
  bool pulse_comp_active = false;
  /** P2 本周期是否检出有效窄道并注入居中导引候选 */
  bool narrow_planner_active = false;
  /** P2 检出走廊的最小间隙宽度 (m) */
  double narrow_corridor_width_m = std::numeric_limits < double > ::infinity();
  /** P2 走廊侧向净空反算的线速度上限 (m/s) */
  double narrow_speed_cap_m_s = std::numeric_limits < double > ::infinity();
  /** P0-B 停稳看护本周期是否锁存（停稳后障碍逼近保护） */
  bool standstill_guard_active = false;
  /** P0-B 停稳看护观测到的最近物理净空 (m) */
  double standstill_min_clearance_m = std::numeric_limits < double > ::infinity();
  /** L0-EB 突发障碍紧急制动本周期是否锁存 */
  bool sudden_obstacle_active = false;
  /** L0-EB 计算的所需减速度 a_req (m/s²) */
  double sudden_a_req_m_s2 = 0.0;
  /** L0-EB 是否超底盘紧急减速能力（制动饱和，碰撞风险） */
  bool sudden_brake_saturated = false;
  /** L1 方向掩码本周期封锁的轴（true = 该方向分量被裁剪为零） */
  bool direction_mask_forward = false;
  bool direction_mask_backward = false;
  bool direction_mask_left = false;
  bool direction_mask_right = false;
  /** 转弯净空护栏本周期施加的线速度上限 (m/s)；未生效为 +inf */
  double turn_guard_speed_cap_m_s = std::numeric_limits<double>::infinity();
  /** 转弯合成速度包络本周期的 v/ω 等比缩放系数 (0,1]；未生效为 1 */
  double turn_envelope_scale = 1.0;
  /** 急速转向线速度硬帽本周期是否生效（|ω| 超阈值且 v 被钳低） */
  bool rapid_steer_cap_active = false;
  /** L0-EB 影子模式：本周期触发条件满足但未实际急停（方案 2.2 灰度） */
  bool sudden_obstacle_shadow = false;
  /** L2 影子模式：夹死死状态微动作复验通过但未放行（方案 2.3 灰度） */
  bool pinned_escape_shadow = false;
  /** 原地转向豁免：本周期急停全停帧下放行了受控低速原地转向脱困 */
  bool rotation_escape_active = false;
  /** 方案 4 传感器降级本周期是否生效（分轴封锁/限速已施加于 eval_cmd） */
  bool degradation_active = false;
  /** 方案 5 前向走廊障碍逼近速度估计 (m/s)，未启用/未确认为 0 */
  double closing_speed_m_s = 0.0;
  /**
   * 前向走廊内最近障碍到「物理车体前缘」的净空 (m)；+inf = 走廊内无障碍。
   * 由 currentForwardClearance 每周期计算（L0-EB / closing_speed 已在用，此处仅上报）。
   * 注意：走在本字段赋值之前的各条急停 early-return（侵入 recovery / 停稳看护 / L0-EB）
   * 不会写入本字段，保持 +inf——节点层的前向近障低通刹停据此天然不接管急停帧。
   */
  double forward_corridor_clearance_m = std::numeric_limits < double > ::infinity();
  /** 前向近障低通刹停本周期是否锁存（节点层输出整形层写入，见 core/front_lowpass_stop.h） */
  bool front_lowpass_stop_active = false;
  /** 前向近障低通刹停本周期实际使用的时间常数 (s)；未锁存为 0 */
  double front_lowpass_stop_tau_s = 0.0;
  /** 低通实际采用的独立净空；tilt_only 时与 forward_corridor_clearance_m 分离 */
  double front_lowpass_clearance_m = std::numeric_limits < double > ::infinity();
  /** 当前低通净空是否由旧斜雷达帧按轮速预测 */
  bool front_lowpass_predicted = false;
  /** 低通最小净空对应斜雷达帧年龄；无有限障碍为 +inf */
  double front_lowpass_observation_age_s = std::numeric_limits < double > ::infinity();
  /** 方案 1 影子仲裁：本周期仲裁器复算输出与真实输出不一致 */
  bool arbiter_shadow_mismatch = false;
  /** 摇杆/期望速度相对最终输出发生衰减或刹停，供 debug 定位干预来源 */
  bool speed_intervention_active = false;
  /** 多个原因用分号连接，使用稳定英文原因码便于 rosbag/脚本检索 */
  std::string speed_intervention_reason = "none";
  /** VehicleIo 输出的当前摇杆控制线速度 (m/s) */
  double speed_debug_joystick_linear_x = 0.0;
  /** 巡航/模式处理后送入避障器的期望线速度 (m/s) */
  double speed_debug_desired_linear_x = 0.0;
  /** 避障控制器给输出整形器的目标线速度 (m/s) */
  double speed_debug_controller_linear_x = 0.0;
  /** 最终发布线速度 (m/s) */
  double speed_debug_output_linear_x = 0.0;
  /** 本周期进入整形前的上一周期输出线速度 (m/s) */
  double speed_debug_previous_output_linear_x = 0.0;
  /** 当前轮速反馈 (m/s) */
  double speed_debug_wheel_linear_x = 0.0;
  /** 相对摇杆/巡航参考速度的绝对值衰减量 (m/s) */
  double speed_debug_reduction_m_s = 0.0;
  bool speed_debug_braking = false;
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
  /** 斜雷达近距硬闸：前向边距进入 nearest_stop 或 TTC 急停窗口 */
  TILT_NEAR_DISTANCE,
  /** 斜雷达 TTC 急停（预留原因码，当前主要使用 TILT_NEAR_DISTANCE） */
  TILT_TTC,
  /** P0-B 停稳看护：停稳后障碍逼近至看护带内，锁存急停保护车体 */
  STANDSTILL_GUARD,
  /** L0-EB 突发障碍：所需减速度超舒适档，动态加大减速度紧急制动 */
  SUDDEN_OBSTACLE,
  /** L0-EB 突发障碍且所需减速度超底盘能力：全力制动但物理停不住，碰撞风险 */
  BRAKE_SATURATED,
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
  /**
   * L1 定向脱困释放阀：物理侵入下 recovery 本应清零，但驾驶员正朝方向掩码
   * 未封锁（不将近障推向物理接触）的方向给令，放行该方向的受控低速逃逸，
   * 而非被 abort 自锁一并钉死（见 direction_mask.h / directionalEscapeTwist）。
   */
  DIRECTIONAL_ESCAPE,
  /**
   * L2 被困脱困（pinned escape）：四向候选全被 LocalMap veto、车已停稳且驾驶员仍在
   * 给令的「夹死」死状态下，用低速微动作在原 MAP_VETO 硬停前尝试脱困——每个微动作都经
   * 同一 Footprint 扫掠 + LocalMap veto 复验，均不过则回退到与原先完全一致的硬停。
   * 默认关闭（AvoidanceParams::pinned_escape），见 tryPinnedEscape。
   */
  PINNED_ESCAPE,
  /**
   * 原地转向豁免（rotation escape）：急停全停帧下，驾驶员在给纯原地转向，且该转向经
   * Footprint 扫掠 + LocalMap「不进新 lethal」复验证明「不恶化」（不把任一近障推向物理
   * 接触、不踏入起始姿势未占用的新 lethal 栅格）时，放行受控低速原地转向脱困（线速度恒零）。
   * 与 pinned_escape 的区别：判据是「不恶化」而非「净空严格改善」，故在已接触（净空≈0、
   * footprint 自身 collision）态也能只转向脱离。默认关闭（AvoidanceParams::rotation_escape）。
   */
  ROTATION_ESCAPE,
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

/**
 * 斜雷达近距 TTC 通道参数。
 *
 * 独立于水平雷达主避障链路，仅对前向命令生效。判据为「固定近距硬闸 + 短窗 TTC」混合：
 * - 低速时 TTC 很大，靠 hard_stop_distance_m / nearest_stop 距离触发急停；
 * - 高速时障碍晚出现、TTC 很短，靠 distance_emergency_m 与 emergency_ttc_s 兜底。
 *
 * 对应 YAML：avoidance.tilt_ttc.*
 */
struct TiltTtcParams
{
  /** 斜雷达 TTC 通道总开关；false 时 evaluateTiltIntervention 直接返回且不锁存 */
  bool enabled = false;
  /** 短窗探测时域 (s)；与 max_probe_distance_m 取 min 作为轨迹长度上限 */
  double horizon_s = 0.5;
  /** 前向探测轨迹最大长度 (m)，与斜雷达有效量程对齐（默认 0.7 m） */
  double max_probe_distance_m = 1.0;
  /** 低速最近停车边距 (m)；速度升高时经二次插值放大到 distance_emergency_m */
  double hard_stop_distance_m = 0.12;
  /** 进入舒适限速区的最大前向边距 (m)；障碍进入此距离即开始 jerk 限速 */
  double limiting_distance_m = 1.0;
  /** 急停锁存释放所需前向边距 (m)；须大于 nearest_stop 以避免阈值振荡 */
  double release_distance_m = 1.05;
  /** 最高速最近停车边距 (m)；如 8 km/h 时 0.45 m 障碍即触发硬闸 */
  double distance_emergency_m = 0.50;
  /** 急停锁存释放前的持续清空时间 (s)，抑制边距阈值附近的启停抖动 */
  double emergency_clear_hold_s = 0.15;
  /** 体素化斜雷达障碍图元时的默认半径 (m)，影响 Footprint 扫掠碰撞判定 */
  double obstacle_radius_m = 0.01;
  /** true：TTC 用 max(|命令线速度|, |反馈线速度|)；false：仅用反馈速度 */
  bool use_command_velocity_for_ttc = true;
  /** 进入 limiting 的 TTC 上限 (s)；短于该值则按距离/jerk 计算 speed_limit */
  double limiting_ttc_s = 0.55;
  /** 进入 emergency 的 TTC 上限 (s)；须同时满足距离在 emergency 窗口内 */
  double emergency_ttc_s = 0.35;
  /** 急停锁存释放时的 TTC 下限 (s)；通常略大于 emergency_ttc_s 形成滞回 */
  double emergency_ttc_release_s = 0.50;
  /** 低于该速度 (m/s) 时不启用 TTC 判据，仅依赖固定距离硬闸 */
  double min_speed_for_ttc_m_s = 0.25;
};

/**
 * P0 精准停车参数：近距指数速度律 + 停短脉冲补偿。
 *
 * 与 MDB（min_distance_braking）叠加：
 * - 中远距仍由 jerk 制动包络二分反解速度（二次函数区）；
 * - 剩余距离进入 near_zone_m 后，用 v = scale·(1−exp(−s/λ)) 渐近软着陆，
 *   s→0 时 v→0，天然无过冲，消除终端阈值附近的启停振荡；
 * - 底盘死区/静摩擦导致停短（残差 > stop_tolerance_m 却停住）时，
 *   由 PulseCompensator 发有界前向微脉冲逐步逼近 hard-stop 目标线。
 *
 * 对应 YAML：avoidance.precise_stop.*
 */
struct PreciseStopParams
{
  bool enabled = false;
  /** 进入近距指数段的剩余距离 (m)，以 hard-stop 线为零点 */
  double near_zone_m = 0.35;
  /** 指数律长度常数 λ (m)，越小终端收敛越急 */
  double exp_lambda_m = 0.08;
  /** 指数段速度尺度 (m/s)：v_cap = scale·(1−exp(−s/λ))，与包络取 min */
  double near_speed_scale_m_s = 0.45;
  /** 判定已精准到位的剩余距离容差 (m)，应 ≥ braking_at_stop_distance_eps_m */
  double stop_tolerance_m = 0.015;
  /** 脉冲补偿开关（enabled=true 时才生效） */
  bool pulse_enabled = true;
  /** 微脉冲速度 (m/s)，须略高于底盘速度死区才能推动车辆 */
  double pulse_creep_speed_m_s = 0.08;
  /** 单次脉冲最长持续时间 (s)；实际时长还受过冲护栏约束 */
  double pulse_max_duration_s = 0.25;
  /** 判定停短的反馈速度阈值 (m/s) */
  double pulse_stall_speed_eps_m_s = 0.02;
  /** 停短须持续该时间才发脉冲 (s)，过滤正常减速过程 */
  double pulse_stall_hold_s = 0.20;
  /** 两次脉冲之间的冷却时间 (s)，等待反馈稳定 */
  double pulse_cooldown_s = 0.20;
};

/**
 * P1 人机共享控制参数：连续接管系数 α + 转向保真。
 *
 * α(risk)∈[0,1] 由驾驶员自身轨迹的 TTC/净空归一化：
 *   output = driver + α·(safety_candidate − driver)
 * α=0 决策空间内完全尊重驾驶员；α 随风险逼近平滑上升（软接管）；
 * α=1 安全临界点完全过载驾驶员意图。α 上升瞬时、回落限速（滞回防抖）。
 * 硬保证不受影响：制动包络限速、Map veto、EmergencyBrake 均在融合之后仍然生效。
 *
 * 对应 YAML：avoidance.shared_control.*
 */
struct SharedControlParams
{
  bool enabled = false;
  /** true：限速干预时保留驾驶员角速度（转向保真），不随线速度同比例缩放 */
  bool preserve_steering = true;
  /** 驾驶员轨迹 TTC ≥ 此值 → α=0（完全跟手） */
  double alpha_ttc_zero_s = 1.5;
  /** 驾驶员轨迹 TTC ≤ 此值 → α=1（完全过载） */
  double alpha_ttc_full_s = 0.45;
  /** 驾驶员轨迹最小净空 ≥ 此值 → α=0 */
  double alpha_clearance_zero_m = 0.30;
  /** 驾驶员轨迹最小净空 ≤ 此值 → α=1；应 ≥ hard_stop_distance_m */
  double alpha_clearance_full_m = 0.05;
  /** α 回落速率 (1/s)；上升不限速以保证安全响应 */
  double alpha_release_rate_per_s = 2.0;
};

/**
 * P2 窄道通过规划参数：走廊提取 + 贝塞尔中线导引。
 *
 * 在前向 ROI 内按纵向 bin 提取左右壁内缘，检出「足够窄且足够长」的走廊后：
 * 用三次 Bézier 拟合居中通过曲线，取起点曲率生成 {v, w} 居中候选注入候选集，
 * 并按侧向净空平滑限速——将「触阈即停」变为「减速居中连贯通过」。
 *
 * 对应 YAML：avoidance.narrow_passage_planner.*
 */
struct NarrowPlannerParams
{
  bool enabled = false;
  /** true：SAFE_ASSIST/CRUISE 下自动检出窄道并启用导引（NARROW 模式恒启用） */
  bool auto_detect_enabled = true;
  /** 前向走廊提取距离 (m) */
  double look_ahead_m = 2.0;
  /** ROI 左右半宽 (m) */
  double roi_half_width_m = 1.2;
  /** 纵向 bin 尺寸 (m) */
  double bin_size_m = 0.15;
  /** 有效走廊最短连续长度 (m) */
  double min_corridor_len_m = 0.45;
  /** 每侧最小可行余量 (m)：gap ≥ 车宽 + 2·此值 才视为可通行 */
  double min_side_margin_m = 0.03;
  /** 窄道检出上限：gap − 车宽 ≤ 此值才注入导引候选（更宽视为开阔区不干预） */
  double narrow_width_max_m = 0.60;
  /** 净空→限速增益 (1/s)：v_cap = min_pass + gain·(侧向净空 − min_side_margin) */
  double speed_per_clearance = 3.0;
  /** 走廊有效时保持前进的下限速度 (m/s)，避免蠕行卡死 */
  double min_pass_speed_m_s = 0.15;
  /** 居中候选曲率上限 (1/m) */
  double max_curvature_1_m = 1.5;
  /** 居中候选 command_cost 折减，正值使其在同等安全下优先被选中 */
  double cost_bias = 0.15;
};

/**
 * L0-EB 突发障碍动态紧急制动参数。见 SUDDEN_BRAKE_PLAN.md。
 *
 * 每周期按"所需减速度" a_req = v² / (2·d_eff) 判定：a_req 超过舒适减速度即视为
 * 突发/切入，触发紧急制动并把减速度抬到底盘能力上限（emergency_decel），
 * 绕过正常 max_decel 限幅。d_eff = 前向净空 − v·t_react − d_safe，预留执行器
 * 延迟滑行与停车几何余量。与 MDB/制动包络正交：平缓接近 a_req 低不触发。
 *
 * 对应 YAML：avoidance.sudden_obstacle.*
 */
struct SuddenObstacleParams
{
  bool enabled = false;
  /**
   * 影子模式（方案 2.2 灰度上车）：触发条件满足时只写日志/计数器
   * （hybrid_log.sudden_obstacle_shadow），不实际急停。实车跑够灰度期、
   * 回看影子触发全部为真实突发后置 false 转正。
   */
  bool shadow_mode = true;
  /** 底盘紧急减速能力上限 (m/s²)，取实测值（勿超过实测峰值，实测约 2.0~2.2）。 */
  double emergency_decel_m_s2 = 2.0;
  /** 舒适减速度门限：a_req 低于此值交由 MDB/包络处理，不触发。 */
  double comfort_decel_m_s2 = 0.6;
  /** 执行器等效延迟 (s)，用于 d_eff 预留滑行距离。 */
  double t_react_s = 0.30;
  /** 仅当前向净空 < 此值 (m) 时参与判定，减少远距误算。 */
  double emergency_distance_m = 2.0;
  /** 低于此前向速度 (m/s) 不触发（静止/倒车不管）。 */
  double min_forward_speed_m_s = 0.15;
  /** 连续满足触发条件的帧数，防单帧噪声。 */
  int confirm_frames = 2;
  /** a_req 一阶低通系数 [0,1]，越大越跟手。 */
  double a_req_filter_alpha = 0.5;
  /** d_eff 下限 (m)，防分母趋零时 a_req 爆大。 */
  double d_eff_floor_m = 0.03;
  /** 前向走廊裁剪：只看 |y| ≤ 车体半宽 + 此余量 (m) 的前方障碍。 */
  double corridor_margin_m = 0.05;
  /** 滞回释放净空 (m)：净空恢复到此值以上持续 clear_hold_s 才解锁。 */
  double release_distance_m = 0.30;
  /** 释放前持续清空时间 (s)。 */
  double clear_hold_s = 0.20;
};

/**
 * L2 被困脱困（pinned escape）参数。
 *
 * 触发于「四向候选全被 LocalMap veto、车已停稳、驾驶员仍在给令」的夹死死状态——
 * 该状态原本无条件走 MAP_VETO 硬停清零。本层在硬停前枚举一组低速微动作，
 * 每个都经现有 Footprint 扫掠碰撞 + LocalMap veto 复验（同一把闸，不放松任何判据），
 * 择「净空严格改善且最贴驾驶员意图」者放行；均不过则回退到与原先完全一致的硬停。
 *
 * 结构体默认关闭；仅在该死状态触发，不影响任何既有可选到候选/侵入 recovery/正常制动的场景。
 * 对应 YAML：avoidance.pinned_escape.*
 */
struct PinnedEscapeParams
{
  bool enabled = false;
  /**
   * 影子模式（方案 2.3 灰度上车）：夹死死状态下微动作复验通过时只写日志
   * （hybrid_log.pinned_escape_shadow），仍回退到与原先一致的 MAP_VETO 硬停。
   * 用 tilt_on_6 bag 回放确认影子给出的微动作合理后置 false 转正。
   */
  bool shadow_mode = true;
  /** 连续处于夹死死状态的去抖确认时间 (s)，滤除瞬时误触。 */
  double confirm_s = 0.20;
  /** 脱困蠕行线速度 (m/s)，须略高于底盘速度死区。 */
  double creep_linear_m_s = 0.06;
  /** 脱困蠕行角速度 (rad/s)。 */
  double creep_angular_rad_s = 0.25;
  /** 微动作扫掠净空地板 (m)，取 max(本值, hard_stop_distance_m)，物理接触仍被硬停带兜住。 */
  double escape_margin_m = 0.03;
  /** 相对当前最近净空的最小改善量 (m)；微动作须使净空严格变大才放行，保证脱困有进展。 */
  double min_clearance_gain_m = 0.005;
  /** 判定「已停稳」的反馈线速度阈值 (m/s)。 */
  double standstill_speed_eps_m_s = 0.03;
};

/**
 * 原地转向豁免（rotation escape）参数。
 *
 * 触发于「急停全停帧 + 驾驶员在给纯原地转向」：该状态原本无条件把线速度与角速度整帧清零
 *（applyEmergencyBrake 返回全零 Twist），连「只转个向就能脱离」这种现实可行的动作也一并冻结。
 * 本层在急停输出后复验驾驶员的转向意图——沿实际转向方向做有界角探针扫掠，要求：
 *   ① Footprint 扫掠（复用 direction_mask 谓词）：转向不把任一近障推向物理接触（不恶化）；
 *   ② LocalMap：转向不踏入任何「起始姿势未占用」的新 lethal 栅格（不恶化）。
 * 两者均过则放行受控低速原地转向（线速度恒零），并以 recovery 输出契约下发（急停解除、
 * recovery_active=true），使 OutputShaper 正常整形转向而非收敛到零。
 *
 * 与 pinned_escape 的关键差异：判据是「不恶化」而非「净空严格改善」，故在已接触（净空≈0）
 * 态也能只转向脱离，且不依赖存在 footprint-safe 候选。仅作用于 MAP_VETO / BRAKE_HOLD /
 * 净空-TTC 类急停全停帧；不放宽 standstill_guard / sudden_obstacle / input_fault / mode_estop
 * 这些「有物主动逼近或系统失效」的急停（那些走早返回、不进本层）。
 *
 * 结构体默认关闭；enabled=false 时输出与旧行为逐字节一致。对应 YAML：avoidance.rotation_escape.*
 */
struct RotationEscapeParams
{
  bool enabled = false;
  /** 触发所需的最小转向意图 (rad/s)：|驾驶员角速度| ≥ 此值才尝试。 */
  double min_angular_rad_s = 0.10;
  /** 放行的原地转向速度上限 (rad/s)，受控低速；与 absolute_max_angular_speed 取 min。
   *  0.60 rad/s ≈ 34°/s，旋转 90° 约 2.6s；兼顾脱困效率与安全裕度。
   *  旧值 0.30 过保守（90° 需 5.3s），实车体感迟钝。 */
  double max_angular_rad_s = 0.60;
  /** 验证扫掠角行程 (rad)：沿转向方向的有界角探针总角度。 */
  double probe_angle_rad = 0.35;
  /** 角探针离散步长 (rad)：步长越小验证越精细但计算量越大。
   *  独立于 direction_mask_probe_step_rad，使 rotation_escape 的验证精度可单独调参。
   *  若 ≤0 则回退使用 direction_mask_probe_step_rad（兼容旧配置）。 */
  double probe_step_rad = 0.05;
};

/**
 * P0-A' 动态障碍逼近速度估计参数（方案 5，见 closing_speed_estimator.h）。
 * 对应 YAML：avoidance.closing_speed.*
 */
struct ClosingSpeedParams
{
  bool enabled = false;
  /** 一阶低通系数 [0,1]，越大越跟手 */
  double filter_alpha = 0.4;
  /** 连续观测到逼近的确认帧数，滤单帧体素抖动 */
  int confirm_frames = 3;
  /** 估计输出上限 (m/s)，防离散跳变爆大 */
  double max_rate_m_s = 2.0;
  /** 净空增大突变阈值 (m/帧)：超过视为漏检恢复/障碍离开，复位估计 */
  double jump_reset_m = 0.30;
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
  /**
   * 制动包络是否用 LocalMap 致命轨迹分支压制可用制动距离。
   * false 时制动距离仅由激光 footprint 链（checkTrajectory）决定，避免制动探针
   * 冲出 6m 地图前边界后被 gridCostAt「越界→满代价 1.0」误判为致命，从而把可用
   * 制动距离恒定钳到 ~2.025m、与真实前障无关地长期限速（见 speed_01/02 回放分析）。
   * 候选轨迹的 Map veto（map_constraint）不受此开关影响。
   */
  bool braking_use_local_map = true;
  double braking_lookahead_m = 5.0;
  double braking_path_step_m = 0.025;
  /**
   * 前向制动探针的累计航向变化上限 (rad)。
   * 大角速度+小线速度时转弯半径极小（v/w），若沿固定弧长积分会原地打转数圈，
   * 把车后/车侧障碍误当作前向即时硬停而冻结整车。探针一旦累计航向变化超过此值
   * 即截断——原地打转的弧长不计入前向可停距离；紧转弯净空交由 turn_clearance_guard
   * 与 direction_mask 角轴处理。≤0 表示不截断（旧行为）。
   */
  double braking_probe_max_heading_rad = 1.5708;
  double braking_reaction_delay_s = 0.10;
  double braking_comfort_decel_m_s2 = 0.60;
  double braking_max_jerk_m_s3 = 2.5;
  double braking_stop_buffer_m = 0.02;
  /**
   * P0-A 动态障碍接近速度储备 (m/s)。
   * 制动包络反算速度上限时，额外按此速度预留「障碍在整个停车过程中继续逼近」的距离，
   * 使车辆对向来物/行人靠近时提前减速、停稳后不被侵入。0 表示按静止障碍处理（旧行为）。
   */
  double braking_obstacle_closing_reserve_m_s = 0.0;
  /** 前向接近时仅由剩余距离反算速度，不用制动包络急停锁存 */
  bool min_distance_braking_enabled = true;
  /** 精确停到 hard-stop 线：忽略 braking_stop_buffer，仅留路径离散误差 */
  bool braking_exact_stop_enabled = true;
  /** 判定已到达最小停车距离的容差 (m) */
  double braking_at_stop_distance_eps_m = 0.0125;
  /** 到达 hard-stop 后禁止正向再加速 */
  bool braking_no_forward_reaccel_at_hard_stop = true;
  /** MDB 期间抑制 EmergencyBrake 的距离型锁存 */
  bool braking_suppress_distance_emergency = true;
  bool forward_stop_hold_enabled = true;
  double forward_stop_hold_confirm_s = 0.10;
  double forward_stop_hold_feedback_speed_m_s = 0.03;
  double forward_stop_hold_speed_limit_m_s = 0.20;
  double forward_stop_hold_remaining_m = 0.08;
  double forward_stop_hold_release_distance_m = 0.20;

  // ── P0-B 停稳看护（standstill guard）──
  // 车辆停稳后，制动包络/MDB/TTC 均以自车前进为门控而失效；此时若障碍物（人/对向车）
  // 主动逼近，系统原本无任何反应会被侵入车体。停稳看护在停稳状态下持续跟踪最近物理净空，
  // 检出「净空快速缩小（有物体接近）」或「净空低于绝对地板」时锁存急停并上报，
  // 迫使车辆保持零速；倒车意图始终放行，供驾驶员/脱困后退让位。
  bool standstill_guard_enabled = true;
  /** 仅当最近物理净空 ≤ 该值时才评估接近（避免贴墙静止误触发）。 */
  double standstill_guard_band_m = 0.12;
  /** 物理净空绝对地板：低于此值直接锁存急停（即将触碰车体）。 */
  double standstill_guard_floor_m = 0.03;
  /** 判定「障碍正在接近」的净空缩小速率阈值 (m/s)。 */
  double standstill_guard_approach_rate_m_s = 0.05;
  /** 触发前需持续满足的确认时间 (s)。 */
  double standstill_guard_confirm_s = 0.10;
  /** 净空恢复到该值以上（迟滞 > band）才解除锁存。 */
  double standstill_guard_release_m = 0.15;
  double candidate_speed_up_hold_s = 0.30;
  bool recovery_mode_enabled = true;
  double recovery_max_linear_speed = 0.12;
  double recovery_max_angular_speed = 0.35;
  double recovery_lookahead_s = 0.35;
  double recovery_min_command = 0.02;
  /** 停稳判定速度门控 (m/s)：|fb_linear| 和 |prev_output_linear_x| 均低于此值视为已停稳。
   *  统一替代代码中散落的硬编码 0.02，调参时一处生效全局一致。 */
  double standstill_speed_eps_m_s = 0.02;
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

  // ── L1 方向解耦掩码（direction mask）──
  // 障碍进入硬停带（含虚拟/物理车体内）后，用 ±v/±w 四组短探针判定各轴运动
  // 是否将其推向物理接触，仅封锁受威胁分量：前障不影响后向与左右，
  // 侧障不影响前后向与对侧转向。同一威胁谓词用于 Footprint 扫掠的近障方向豁免
  // 与 LocalMap veto 的近障豁免圆。false 时完全回退旧「全向封锁」行为。
  // 结构体默认关闭（与 sudden_obstacle 约定一致），生产由 YAML 开启。
  bool direction_mask_enabled = false;
  /** 线速度探针长度 (m)；应 ≥ standstill_guard_band_m 以覆盖看护带内近障 */
  double direction_mask_probe_distance_m = 0.15;
  /** 角速度探针角度 (rad)，覆盖单周期原地转向的角行程 */
  double direction_mask_probe_angle_rad = 0.30;
  /** 线探针步长 (m) */
  double direction_mask_probe_step_m = 0.03;
  /** 角探针步长 (rad) */
  double direction_mask_probe_step_rad = 0.06;
  /** 物理接触缓冲 (m)：探针使障碍符号净空跌破该值即判定威胁 */
  double direction_mask_contact_buffer_m = 0.03;
  /** 趋近判定死区 (m)，过滤数值噪声 */
  double direction_mask_approach_eps_m = 0.005;
  /** LocalMap 近障豁免圆半径 (m)：应 ≥ 地图膨胀半径 + 栅格分辨率 */
  double direction_mask_map_exempt_radius_m = 0.22;

  // ── 转弯净空护栏（turn clearance guard）──
  // 方向解耦令前向制动包络对侧向近障免疫（避免直线贴障蠕行误触发），但转弯时旋转
  // 车体会侧扫入前侧近障，前向净空与纯 ±v/±w 探针都漏判。此处仅在 |w| 显著（转弯）时，
  // 取「沿实际 (v,w) 曲线扫掠 Footprint 的最近净空」（SweptFootprintChecker 已含近障
  // 方向豁免：弧线掠离的障碍自动豁免、掠向的如实计入），按可停净空
  // v ≤ √(2·braking_comfort_decel·(净空−hard_stop)) 钳制线速度、角速度同比缩放
  //（保持转弯半径），把转弯速度收敛到「能在净空内停住」，让既有 map veto / 制动兜底
  // 来得及。|w| 低于门控（直线/微转）完全不介入，不改变既有直线行为。
  // 结构体默认关闭，生产由 YAML 开启。
  bool turn_clearance_guard_enabled = false;
  /** 转弯门控角速度 (rad/s)：|w| ≥ 此值才启用护栏 */
  double turn_clearance_guard_min_angular_rad_s = 0.5;
  /**
   * 曲率门控 (1/m)：>0 时替代角速度门控，|w|/max(|v|,0.1) ≥ 此值（转弯半径
   * 足够小）才介入——低速大转弯（tilt_on_2 场景）比固定角速度门控介入更及时，
   * 直行高速不受影响。=0 回退旧的 min_angular_rad_s 门控。
   */
  double turn_clearance_guard_min_curvature_1_m = 0.0;
  /** 生效净空带 (m)：曲线扫掠最近净空 ≤ 此值才参与限速，之外不介入 */
  double turn_clearance_guard_band_m = 0.6;

  double absolute_max_linear_speed = 2.222;
  double absolute_max_angular_speed = 3.0;

  // ── 转弯合成速度包络（turn speed envelope） ──
  // v/ω 各自钳位无法约束合成运动：两轴同时较大时，车体外缘线速度 |v|+|ω|·r 与
  // 横向加速度 |v·ω| 均无界（两轴打满时外缘≈4.2 m/s、横加≈6.7 m/s²，即「转弯外圈
  // 太快」）。超限时对 v、ω 等比缩放（曲率 ω/v 不变，保持转弯半径），只降快慢不改
  // 轨迹形状。直线（ω≈0）不介入；默认参数下原地旋转上限 2.222/0.655≈3.4 rad/s
  // 宽于绝对角速度上限 3.0，纯旋转（走廊原地调头）同样不介入。
  // 结构体默认关闭，生产由 YAML 开启。
  bool turn_speed_envelope_enabled = false;
  /** 车体外缘等效回转半径 (m)，默认取角点 sqrt((L/2)²+(W/2)²) */
  double turn_envelope_edge_radius_m = 0.655;
  /** 外缘合成线速度上限 |v|+|ω|·r (m/s)；≤0 表示不启用该判据 */
  double turn_envelope_max_edge_speed_m_s = 2.222;
  /** 横向加速度上限 |v·ω| (m/s²)；≤0 表示不启用该判据 */
  double turn_envelope_max_lat_accel_m_s2 = 1.2;

  // ── 急速转向线速度硬帽（rapid steer cap，见 avoidance_controller.cpp
  // applyRapidSteerLinearCap）──
  // turn_speed_envelope 等比缩放 v/ω 保转弯半径，缩放系数有下界：bag speed_test_tilt_on_2
  // 的临界帧（v=1.57、ω=1.04）只得 k≈0.86 → v≈1.35 m/s，仍高于已致碰的 ~1.3 m/s，
  // 「满线速 + 猛打转」仍会把净空在 ~0.3 s 内从 0.55 m 吃到 0（EB 到贴障才闩锁，来不及）。
  // 本层不保半径：|ω| 超阈值即判「急速转向」，把 v 硬钳到停得住的量级，压低前向逼近速度。
  // 与包络并存取更严者。直线/缓转不介入。结构体默认关闭，生产由 YAML 开启。
  bool rapid_steer_cap_enabled = false;
  /** 判定「急速转向」的角速度阈值 (rad/s)；≤0 视为未配置、本层不介入 */
  double rapid_steer_trigger_angular_rad_s = 0.6;
  /** 急速转向期间的线速度上限 (m/s)，前后向同帽 */
  double rapid_steer_max_linear_m_s = 0.6;

  size_t max_obstacle_primitives = 128;

  /**
   * 方案 8 扫掠预裁剪带宽 (m)：障碍到「轨迹 AABB 外扩 Footprint 外接圆半径」
   * 的距离超过此值时整体跳过（其净空必然 > 此值，超出一切决策阈值）。
   * 须 ≥ 所有净空判据阈值的最大值（turn_guard band 0.6 / comfort_clearance /
   * alpha_clearance_zero 0.3 等），由参数校验器把关。≤0 关闭预裁剪（旧行为）。
   * 被裁剪障碍不再计入 min_distance——远障净空只在日志中由有限值变为 +inf，
   * 不改变任何控制决策（所有阈值均 < 本带宽）。
   */
  double swept_cull_band_m = 1.0;

  /**
   * 方案 1 步骤 A：影子仲裁开关。开启时 evaluate() 各安全层旁路记录
   * LayerVerdict，末尾用 SafetyArbiter 复算输出并与真实输出比对，
   * 不一致置 hybrid_log.arbiter_shadow_mismatch（节点健康摘要累计）。
   * 纯观测，不改变任何输出。
   */
  bool arbiter_shadow_enabled = true;

  /** 水平雷达主避障链总开关；false 时仍可走斜雷达 tilt_ttc 近距通道 */
  bool horizontal_avoidance_enabled = true;

  bool enable_forward_collision = true;
  bool enable_lateral_collision = true;
  bool enable_backward_collision = true;

  double max_angular_decel_rad_s2 = 2.0;
  double max_angular_recover_rad_s2 = 1.0;
  /** 斜雷达近距 TTC 干预参数，见 TiltTtcParams */
  TiltTtcParams tilt_ttc;
  /** P0 精准停车：近距指数律 + 脉冲补偿 */
  PreciseStopParams precise_stop;
  /** P1 人机共享控制：连续接管 α + 转向保真 */
  SharedControlParams shared_control;
  /** P2 窄道通过：走廊提取 + Bézier 中线导引 */
  NarrowPlannerParams narrow_planner;
  /** L0-EB 突发障碍动态紧急制动 */
  SuddenObstacleParams sudden_obstacle;
  /** L2 被困脱困：夹死死状态下的低速微动作脱困（默认关） */
  PinnedEscapeParams pinned_escape;
  /** 原地转向豁免：急停全停帧下放行「可证明不恶化」的纯原地转向脱困（默认关） */
  RotationEscapeParams rotation_escape;
  /** P0-A' 动态障碍逼近速度估计（方案 5） */
  ClosingSpeedParams closing_speed;
  /** 斜雷达前向近障软限速：前向走廊探到障碍即钳前向速度上限（默认关，见 front_tilt_speed_cap.h） */
  FrontTiltSpeedCapParams front_tilt_speed_cap;
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
