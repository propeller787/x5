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
#include "safety_node/core/emergency_brake.h"
#include "safety_node/core/motion_rollout.h"
#include "safety_node/core/obstacle_field.h"
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
    const ObstacleField * tilt_field = nullptr) const;

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
    CollisionCheckResult footprint_check;
  };

  struct TiltIntervention
  {
    bool active = false;
    bool limiting = false;
    bool emergency = false;
    double speed_limit_m_s = std::numeric_limits < double > ::infinity();
    double min_distance_m = std::numeric_limits < double > ::infinity();
    double min_ttc_s = std::numeric_limits < double > ::infinity();
    std::string source;
    EmergencyStopReason reason = EmergencyStopReason::NONE;
  };

  FootprintIntrusion analyzeFootprintIntrusion(const ObstacleField & field) const;
  void enrichProximityBlock(
    FootprintIntrusion & intrusion,
    const ObstacleField & field,
    double fb_linear,
    double prev_output_linear_x) const;
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
  TrajectoryRollout buildTiltProbe(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear) const;
  TiltIntervention evaluateTiltIntervention(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    const ObstacleField * tilt_field,
    double dt_s) const;
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
    double control_dt_s) const;
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
  void resetRecoveryState() const;
  void resetTiltEmergencyState() const;
  void latchRecoveryAbort(RecoveryState reason) const;
  double applyCandidateSpeedUpHold(double selected_linear, double dt_s) const;
  void resetCandidateSpeedUpHold() const;
  void configureForMode(const ModeProfile & mode_profile) const;

  AvoidanceParams base_params_;
  mutable AvoidanceParams params_;
  MapConstraintConfig map_config_;
  mutable VehicleFootprint footprint_;
  mutable SweptFootprintChecker checker_;
  mutable MotionRollout rollout_;
  mutable SafetyFilter safety_filter_;
  mutable EmergencyBrake emergency_brake_;
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
  mutable bool has_smoothed_braking_speed_limit_ = false;
  mutable double smoothed_braking_speed_limit_m_s_ =
    std::numeric_limits < double > ::infinity();
  mutable bool tilt_emergency_latched_ = false;
  mutable double tilt_emergency_clear_time_s_ = 0.0;
  mutable EmergencyStopReason tilt_emergency_reason_ = EmergencyStopReason::NONE;
};
