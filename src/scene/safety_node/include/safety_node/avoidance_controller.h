/**
 * @file avoidance_controller.h
 * @brief 混合避障控制器：Footprint 选速 + Map 硬约束 + 急停
 *
 * V1 采用串行两步策略（见 HYBRID_FOOTPRINT_LOCALMAP.md §5.3）：
 *   ① 对所有候选轨迹做 Footprint 扫掠，按「安全 + 最接近手柄」排序
 *   ② 依次对安全候选做 LocalMap lethal 查询，首个通过者输出
 *   ③ 若全部被否决或碰撞 → 急停或 SafetyFilter 降级
 *
 * 核心原则：Footprint 负责跟手选速，LocalMap 仅作空间记忆裁判（veto）。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/closing_speed_estimator.h"
#include "safety_node/core/direction_mask.h"
#include "safety_node/core/emergency_brake.h"
#include "safety_node/core/safety_verdict.h"
#include "safety_node/core/sudden_obstacle_detector.h"
#include "safety_node/core/motion_rollout.h"
#include "safety_node/core/narrow_passage_planner.h"
#include "safety_node/core/obstacle_field.h"
#include "safety_node/core/pulse_compensator.h"
#include "safety_node/core/safety_filter.h"
#include "safety_node/core/swept_footprint_checker.h"
#include "safety_node/core/vehicle_footprint.h"
#include "safety_node/map_constraint_check.h"
#include "safety_node/mode_supervisor.h"

#include <geometry_msgs/msg/twist.hpp>
#include <limits>

class AvoidanceController {
public:
  AvoidanceController(
    const AvoidanceParams & avoidance,
    const MapConstraintConfig & map_config);

  void updateParams(const AvoidanceParams & avoidance, const MapConstraintConfig & map_config);

  /**
   * @brief 单周期混合避障评估
   * @param cmd           手柄期望 Twist
   * @param fb_linear     轮速反馈线速度 (m/s)
   * @param fb_angular    轮速反馈角速度 (rad/s)
   * @param field         当前 ObstacleField
   * @param local_map     LocalMap 栅格快照，可为 nullptr
   * @param mode_profile  当前模式下的子系统开关与阈值
   * @param prev_output_linear_x  上一周期输出线速度
   * @param prev_output_angular_z 上一周期输出角速度
   * @param dt_s          控制周期 (s)
   * @param context       巡航上限等上下文
   */
  AvoidanceResult evaluate(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    double fb_angular,
    const ObstacleField & field,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile,
    double prev_output_linear_x,
    double prev_output_angular_z,
    double dt_s,
    const AvoidanceContext & context = {},
    /** 斜雷达体素障碍场；非空时在 evaluate 末尾经 TTC 通道叠加近距限速/急停 */
    const ObstacleField * tilt_field = nullptr) const;

  /** 按物理车体半宽 + margin 构造前向走廊并求净空，供低通专用斜雷达场使用。 */
  double frontCorridorClearance(
    const ObstacleField & field, double corridor_margin_m) const;

private:
  struct FootprintIntrusion
  {
    bool active = false;
    bool proximity_blocked = false;
    double min_clearance = std::numeric_limits < double > ::infinity();
    CollisionCheckResult worst;
    ObstacleField external_field;

    bool recoveryEligible() const
    {
      return active || proximity_blocked;
    }
  };

  struct RecoveryCandidate
  {
    bool valid = false;
    VelocityCandidate command;
    double future_clearance = -std::numeric_limits < double > ::infinity();
    double clearance_gain = 0.0;
    double command_cost = std::numeric_limits < double > ::infinity();
  };

  struct BrakingEnvelope
  {
    bool active = false;
    bool emergency = false;
    bool controlled_escape_only = false;
    bool map_escape_only = false;
    double available_distance_m = std::numeric_limits < double > ::infinity();
    double speed_limit_m_s = std::numeric_limits < double > ::infinity();
    double stopping_distance_m = 0.0;
    double effective_remaining_m = std::numeric_limits < double > ::infinity();
    bool at_min_stop = false;
    CollisionCheckResult footprint_check;
  };

  /** 斜雷达 TTC 通道单周期评估结果，由 mergeTiltIntervention 写入 AvoidanceResult */
  struct TiltIntervention
  {
    bool active = false;       ///< 本周期是否产生干预（limiting 或 emergency）
    bool limiting = false;     ///< 限速干预（含锁存保持期间的零速）
    bool emergency = false;    ///< 近距硬闸，交由 EmergencyBrake 置零
    double speed_limit_m_s = std::numeric_limits < double > ::infinity();
    double min_distance_m = std::numeric_limits < double > ::infinity();  ///< 前向最近净空
    double min_ttc_s = std::numeric_limits < double > ::infinity();
    std::string source;        ///< 最近障碍图元来源
    EmergencyStopReason reason = EmergencyStopReason::NONE;
  };

  FootprintIntrusion analyzeFootprintIntrusion(const ObstacleField & field) const;
  void enrichProximityBlock(
    FootprintIntrusion & intrusion,
    const ObstacleField & field,
    double fb_linear,
    double prev_output_linear_x) const;
  double currentInflatedClearance(const ObstacleField & field) const;
  /** 最近障碍到物理车体轮廓的净空 (m)；无有效障碍返回 +inf。用于 P0-B 停稳看护。 */
  double currentPhysicalClearance(const ObstacleField & field) const;
  /** 前向走廊（车体前方、|y|≤corridor_half）内障碍到物理车体的最近净空。 */
  double currentForwardClearance(const ObstacleField & field, double corridor_half_m) const;
  double smoothBrakingSpeedLimit(double raw_limit_m_s, double dt_s) const;
  EmergencyBrakeContext buildEmergencyBrakeContext(
    const geometry_msgs::msg::Twist & cmd,
    const geometry_msgs::msg::Twist & output_cmd,
    double fb_linear,
    double fb_angular,
    double prev_output_linear_x,
    const ObstacleField & field) const;
  /** 停驻且沿手柄方向前视净空已达释放阈值（不含物理浸入场景） */
  bool isDirectionalEscapeViable(const EmergencyBrakeContext & context) const;
  bool isControlledMapEscape(
    const TrajectoryRollout & rollout,
    double fb_linear,
    double prev_output_linear_x,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile,
    double max_escape_time_s,
    MapConstraintResult & out) const;
  TrajectoryRollout buildDistanceProbe(const geometry_msgs::msg::Twist & cmd) const;
  /** 沿前向命令构建短窗探测轨迹，供斜雷达 Footprint 扫掠与 TTC 计算 */
  TrajectoryRollout buildTiltProbe(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear) const;
  /** 障碍图元相对膨胀 Footprint 前缘的前向净空；x<=0 返回 +inf（非前向障碍） */
  double tiltForwardClearance(const ObstaclePrimitive & obstacle) const;
  /**
   * 斜雷达近距 TTC 通道核心评估。
   * 混合判据：固定距离 hard_stop / nearest_stop + 短窗 TTC + 轨迹碰撞。
   * 倒车与通道关闭时不干预；急停支持锁存与滞回释放。
   */
  TiltIntervention evaluateTiltIntervention(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    const ObstacleField * tilt_field,
    double dt_s) const;
  /** 将斜雷达干预合并入主路径 safe_twist；emergency 仅置 risk，硬闸由 EmergencyBrake 完成 */
  void mergeTiltIntervention(
    AvoidanceResult & result,
    const TiltIntervention & tilt) const;
  BrakingEnvelope evaluateBrakingEnvelope(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    double prev_output_linear_x,
    const ObstacleField & field,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile,
    double control_dt_s,
    /** L1 方向掩码：enabled 时前向净空钳制只用 forward_clearance_m（定向），
        侧后近障不再压缩前向制动距离 */
    const DirectionBlockMask & direction_mask,
    /** 方案 5：本周期估计的障碍逼近速度 (m/s)；与静态参数下限取 max 作为动态预留 */
    double dynamic_closing_reserve_m_s = 0.0) const;
  /** L1：收集「该轨迹不将其推向物理接触」的近障豁免圆，供 Map veto 方向解耦 */
  std::vector<MapExemptCircle> collectMapExemptCircles(
    const ObstacleField & field,
    const std::vector<PredictedPose> & poses) const;
  /**
   * L1 同 collectMapExemptCircles，但写入调用方提供的 buffer 避免堆分配。
   * 用于 evaluate() 内候选循环热路径：复用同一 buffer，每帧零分配。
   */
  void fillMapExemptCircles(
    std::vector<MapExemptCircle> & out,
    const ObstacleField & field,
    const std::vector<PredictedPose> & poses) const;
  RecoveryCandidate evaluateRecoveryCandidate(
    const VelocityCandidate & candidate,
    const geometry_msgs::msg::Twist & requested_cmd,
    double fb_linear,
    double fb_angular,
    const ObstacleField & field,
    const FootprintIntrusion & intrusion,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile) const;
  bool trySelectRecoveryCommand(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    double fb_angular,
    const ObstacleField & field,
    const FootprintIntrusion & intrusion,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile,
    double dt_s,
    geometry_msgs::msg::Twist & recovery_twist,
    double & clearance_gain) const;
  /**
   * L1 定向脱困释放阀。物理侵入下 recovery 放弃(abort/blocked/confirming)时，
   * 若驾驶员命令朝方向掩码未封锁的方向(该轴运动不将任何近障推向物理接触)，
   * 产出仅含未受威胁分量、钳到 recovery 速度上限的受控逃逸 Twist，使车能退出
   * 体内侵入并自然解锁；无有效逃逸分量(命令方向被判威胁或未给令)返回 false。
   */
  bool directionalEscapeTwist(
    const geometry_msgs::msg::Twist & requested_cmd,
    const DirectionBlockMask & mask,
    geometry_msgs::msg::Twist & escape) const;
  /**
   * L2 被困脱困。四向候选全被 LocalMap veto、车已停稳且驾驶员仍在给令的「夹死」死状态下，
   * 在原 MAP_VETO 硬停前枚举一组低速微动作，每个都经 Footprint 扫掠碰撞 + LocalMap veto
   * 复验（同一把闸，不放松任何判据），择净空严格改善且最贴驾驶员意图者放行；均不过返回
   * false 回退到与原先完全一致的硬停。默认关闭（AvoidanceParams::pinned_escape.enabled）。
   */
  bool tryPinnedEscape(
    const geometry_msgs::msg::Twist & requested_cmd,
    double fb_linear,
    double fb_angular,
    const ObstacleField & field,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile,
    double prev_output_linear_x,
    double dt_s,
    geometry_msgs::msg::Twist & escape,
    double & clearance_gain) const;
  void resetPinnedEscape() const;
  /**
   * 原地转向豁免。急停全停帧下,若驾驶员在给纯原地转向(线速度分量近零、角速度显著),
   * 沿实际转向方向做有界角探针扫掠复验:① Footprint 扫掠(复用 direction_mask 谓词)不把
   * 任一近障推向物理接触;② LocalMap 不踏入起始姿势未占用的新 lethal 栅格。两者均过则产出
   * 仅含受控低速角速度的 escape Twist(线速度恒零)返回 true;否则返回 false 回退硬停。
   * 默认关闭(AvoidanceParams::rotation_escape.enabled)。判据为「不恶化」,故接触态亦可放行。
   */
  bool tryRotationEscape(
    const geometry_msgs::msg::Twist & requested_cmd,
    const ObstacleField & field,
    const ObstacleField * tilt_field,
    const RollingLocalMapGrid * local_map,
    const ModeProfile & mode_profile,
    geometry_msgs::msg::Twist & escape) const;
  void resetRecoveryState() const;
  /** 清除斜雷达急停锁存（模式切换、ESTOP、参数更新时调用） */
  void resetTiltEmergencyState() const;
  void resetForwardStopHold() const;
  /** 清除 P0-B 停稳看护锁存与状态（模式切换、ESTOP、参数更新时调用） */
  void resetStandstillGuard() const;
  void latchRecoveryAbort(RecoveryState reason) const;
  double applyCandidateSpeedUpHold(double selected_linear, double dt_s) const;
  void resetCandidateSpeedUpHold() const;
  bool updateForwardStopHold(
    const BrakingEnvelope & envelope,
    const geometry_msgs::msg::Twist & requested_cmd,
    double fb_linear,
    double prev_output_linear_x,
    bool recovery_active,
    bool emergency_brake,
    double dt_s) const;
  void updateMinDistanceBrakingLatch(
    const BrakingEnvelope & envelope,
    const geometry_msgs::msg::Twist & eval_cmd,
    double min_clearance_m,
    double effective_remaining_m) const;
  void configureForMode(const ModeProfile & mode_profile) const;
  /**
   * P1 连续接管系数 α∈[0,1]：由驾驶员自身轨迹的 TTC/净空归一化，
   * 上升瞬时、回落按 alpha_release_rate_per_s 限速（滞回防抖）。
   */
  double updateAuthorityAlpha(
    const CollisionCheckResult & driver_check,
    double dt_s) const;
  void resetSharedControlState() const;

  AvoidanceParams base_params_;
  mutable AvoidanceParams params_;
  MapConstraintConfig map_config_;
  mutable VehicleFootprint footprint_;
  mutable SweptFootprintChecker checker_;
  mutable MotionRollout rollout_;
  mutable SafetyFilter safety_filter_;
  mutable EmergencyBrake emergency_brake_;
  mutable SuddenObstacleDetector sudden_detector_;
  mutable MapConstraintCheck map_checker_;
  mutable bool has_held_candidate_linear_ = false;
  mutable double held_candidate_linear_ = 0.0;
  mutable double candidate_speed_up_clear_time_s_ = 0.0;
  mutable double braking_envelope_active_time_s_ = 0.0;
  mutable bool active_avoidance_overrides_enabled_ = false;
  mutable double recovery_intrusion_confirm_time_s_ = 0.0;
  mutable double recovery_elapsed_s_ = 0.0;
  mutable double recovery_no_progress_time_s_ = 0.0;
  mutable double recovery_feedback_stall_time_s_ = 0.0;
  mutable double recovery_best_clearance_ = -std::numeric_limits < double > ::infinity();
  mutable bool recovery_abort_latched_ = false;
  mutable RecoveryState recovery_abort_reason_ = RecoveryState::IDLE;
  mutable bool has_recovery_candidate_ = false;
  mutable VelocityCandidate held_recovery_candidate_;
  mutable double recovery_candidate_hold_time_s_ = 0.0;
  /** L2 被困脱困去抖计时：连续处于夹死死状态的累计时间 (s) */
  mutable double pinned_escape_confirm_time_s_ = 0.0;
  mutable bool has_smoothed_braking_speed_limit_ = false;
  mutable double smoothed_braking_speed_limit_m_s_ =
    std::numeric_limits < double > ::infinity();
  /** 斜雷达急停锁存：触发 emergency 后置位，释放条件持续满足 emergency_clear_hold_s 后清零 */
  mutable bool tilt_emergency_latched_ = false;
  /** 锁存期间障碍已清空条件的累计时长 (s) */
  mutable double tilt_emergency_clear_time_s_ = 0.0;
  /** 锁存时的急停原因码，用于 hold 期间日志与 OutputShaper bypass */
  mutable EmergencyStopReason tilt_emergency_reason_ = EmergencyStopReason::NONE;
  mutable bool hard_stop_reached_ = false;
  mutable bool forward_stop_hold_latched_ = false;
  mutable double forward_stop_hold_confirm_time_s_ = 0.0;
  // ── P0-B 停稳看护状态 ──
  mutable bool standstill_guard_latched_ = false;
  mutable double standstill_guard_confirm_time_s_ = 0.0;
  mutable double standstill_prev_physical_clearance_m_ =
    std::numeric_limits<double>::infinity();
  /** P0 终端脉冲补偿状态机 */
  mutable PulseCompensator pulse_comp_;
  /** P2 窄道走廊提取与 Bézier 中线导引 */
  mutable NarrowPassagePlanner narrow_planner_;
  /** 方案 5：前向走廊障碍逼近速度估计器 */
  mutable ClosingSpeedEstimator closing_estimator_;
  /** P1 平滑后的接管系数 α（上升瞬时、回落限速） */
  mutable double smoothed_authority_alpha_ = 0.0;
  mutable bool has_authority_alpha_ = false;
  /**
   * L1 近障子集缓存：collectMapExemptCircles 在单次 evaluate() 周期内被多处
   * 调用（7 处），每次 isNearObstacle 全量遍历 field.obstacles，但该步骤
   * 仅依赖 field+footprint，与 poses 无关。缓存 isNearObstacle 通过的子集，
   * 后续调用只对子集跑 obstacleThreatenedAlongPoses。
   * near_obstacle_cache_valid_ 在 evaluate() 入口置 false 以标记本周期未初始化。
   */
  mutable std::vector<const ObstaclePrimitive *> near_obstacle_cache_;
  mutable bool near_obstacle_cache_valid_ = false;
  /** L1 豁免圆复用缓冲：evaluate() 候选循环内避免每候选一次堆分配 */
  mutable std::vector<MapExemptCircle> exempt_circles_buf_;
};
