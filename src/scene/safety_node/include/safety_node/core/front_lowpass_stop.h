/**
 * @file front_lowpass_stop.h
 * @brief 前向近障低通刹停闸（输出整形层）
 *
 * 触发于「前向走廊内障碍距物理车体前缘 ≤ trigger_distance_m 且实际线速度 > trigger_speed_m_s」，
 * 用一阶低通把线速度柔和刹停到零。
 *
 * 与既有制动链的关系（按 config/safety_node_params.yaml 实配算，勿照搬结构体默认值）：
 * 生产配置 hard_stop_distance_m = 0.03、braking_comfort_decel_m_s2 = 0.6，MDB 由剩余距离
 * 反算的速度上限 v ≤ √(2·0.6·(d − 0.03)) 在 16 cm 处约 0.395 m/s。故 trigger_speed_m_s=0.35
 * 落在 MDB 允许区之内——本闸触发的是「MDB 视角下完全合规、但驾驶员逼近偏快」的正常状态
 *（净空 d ∈ [0.132, 0.16] 时 MDB 才允许 v > 0.35），而不是 MDB 失效的兜底。
 *
 * 因此本闸的效果是「快速逼近前障时，比 MDB/precise_stop 的设计落点更早、更果断地停住」：
 * MDB 会一路蠕行到 3 cm 硬停线，本闸在 stop_buffer_m（默认 8 cm）外就收敛到零。慢速逼近
 *（v ≤ 0.35）完全不介入，精准停靠仍由 precise_stop 的指数律负责。
 *
 * ⚠ 自车过滤盲区（实配）：physicalRect().x_max = 0.94（双梯形车头前缘），而
 * perception.self_filter.x_max_m = 1.00（刻意对齐膨胀 footprint 0.94+front_margin 0.05，再留 1cm）
 * ⇒ 正前方（|y| ≤ front_half 0.17 + side_margin 0.02）6 cm 以内的障碍被当作自车滤除，
 * ObstacleField 里看不到。故有效触发窗口是净空 ∈ (0.06, 0.16]，stop_buffer_m 必须 ≥ 0.06+余量。
 * 同一原因下 avoidance.hard_stop_distance_m=0.03 正面也永远测不到。
 *
 * 五条与朴素实现的关键差异：
 * ① 角速度不参与（preserve_steering，由节点层选择整形器方法）：低通只压线速度，否则驾驶员
 *    连「转个向脱离」都做不到，重演 rotation_escape 要解决的那类冻结。
 * ② 锁存 + 滞回：trigger_speed_m_s 是会自解除的条件（低通把速度压到阈值下 → 条件消失 →
 *    shape() 又把驾驶员推杆拉回去 → 阈值附近启停振荡），故触发即锁存，释放不看速度。
 * ③ 停稳解锁：仅靠净空滞回释放会把车锁在 stop_buffer 外直到退回 release_distance_m，
 *    贴近停靠（充电桩 / 桌边）永远做不到。故「已停稳 + 驾驶员撤销前进意图」也解锁——
 *    松杆是明确的确认手势，且从静止再推杆时 MDB 在该净空下的上限已远低于 trigger_speed_m_s，
 *    不会立刻重新触发，后续慢速逼近交回 MDB/precise_stop。
 * ④ 近距失去目标不算清空：净空 +inf 既可能是走廊真空了，也可能是障碍进了上述自车过滤盲区
 *    或本帧丢点。后者是常态——低通恰好把车刹到盲区边缘。若一律当清空解锁，而
 *    local_map.rebuild_each_frame=true 无跨帧记忆、MDB/hard_stop 用同一个 field 也看不见，
 *    车会解锁后重新加速撞上去。故只有「障碍在触发带之外就已消失」才算真清空。
 * ⑤ 动态时间常数：低通滑行距离有闭式解 d = τ·v₀，τ=0.30 时 0.35 m/s 滑 10.2 cm、
 *    0.54 m/s 即滑满 16 cm 触底。故按剩余净空反算 τ 并钳到 [min, max]：低速柔和、高速收紧。
 *    该动态律是自洽不动点：(d−buffer) 与 v 同比例衰减，τ 全程恒等于触发瞬间的 (d₀−buffer)/v₀。
 *
 * 急停与低通的优先级（由节点层折算后写入 emergency_brake 门控）：
 * - hard_stop / L0-EB / brake_hold / 输入故障等真硬急停：一律不接管并立即解锁；
 * - 仅 MAP_VETO、且前向走廊净空仍在硬停线之外时（override_map_veto=true）：允许本闸接管柔停，
 *   避免 LocalMap 在触发带之外就先急停把低通永久挡住；侧向 Footprint min_clearance 不参与；
 * - 后退意图：脱困出路，立即解锁。
 *
 * 结构体默认关闭；enabled=false 时闸恒不激活，输出与旧行为逐字节一致。
 * 纯状态机、无 ROS 依赖，便于单元测试（test_front_lowpass_stop.cpp）。
 */
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>

/** 对应 YAML：output.front_obstacle_lowpass_stop.* */
struct FrontObstacleLowpassStopParams
{
  bool enabled = false;
  /** 障碍来源：horizontal（旧行为）或 tilt_only（仅左/右斜雷达） */
  std::string obstacle_source = "horizontal";
  /** 触发净空 (m)：前向走廊内障碍到物理车体前缘的距离 ≤ 此值才参与判定 */
  double trigger_distance_m = 0.16;
  /** 触发速度 (m/s)：实际（轮速反馈）线速度 > 此值才触发 */
  double trigger_speed_m_s = 0.35;
  /** 低通时间常数上限 (s)，即动态 τ 的柔和端 */
  double filter_time_constant_s = 0.30;
  /** 低通时间常数下限 (s)，防 τ→0 退化成硬停顿挫 */
  double min_time_constant_s = 0.08;
  /**
   * 反算 τ 时预留的停车余量 (m)：目标停在离物理车体前缘该距离处。
   *
   * 必须 ≥ 自车过滤盲区，否则瞄准的落点根本测不到。实配下盲区 =
   * perception.self_filter.x_max_m(1.00) − physicalRect().x_max(0.94) = 6 cm，
   * 故默认 0.08（盲区 + 2 cm 余量）。也应 ≥ avoidance.hard_stop_distance_m。
   */
  double stop_buffer_m = 0.08;
  /** 连续满足触发条件的帧数，滤除单帧体素抖动 */
  int confirm_frames = 2;
  /** 滞回释放净空 (m)：净空恢复到此值以上持续 clear_hold_s 才解锁；须 > trigger_distance_m */
  double release_distance_m = 0.25;
  /** 释放前持续清空时间 (s) */
  double clear_hold_s = 0.15;
  /** 判定「已停稳」的反馈线速度阈值 (m/s)，用于停稳解锁（见文件头 ③） */
  double standstill_release_speed_eps_m_s = 0.03;
  /** true：低通只压线速度，保留角速度（转向脱离出路）；false：线角同时衰减 */
  bool preserve_steering = true;
  /**
   * true：仅 MAP_VETO 且前向走廊净空仍在硬停线外时，节点层不把该急停喂给本闸门控，
   * 允许低通在触发带内柔停；hard_stop / L0-EB 等仍绝对优先。
   */
  bool override_map_veto = true;

  /** tilt_only 时：相邻斜雷达帧之间是否按轮速反馈外推前向净空 */
  bool stale_prediction_enabled = true;
  /** 旧帧允许参与新触发的最长外推时间；超时后仅维持已经锁存的刹停 */
  double stale_prediction_max_s = 0.20;
  /** 外推帧附加的保守净空扣减，只在使用旧帧时施加一次 */
  double prediction_uncertainty_margin_m = 0.01;
};

/** 单个斜雷达源在当前控制周期看到的最新完整帧。 */
struct FrontLowpassSourceSample
{
  /** 是否至少有一帧成功完成 TF 与过滤；false 时不更新该源历史 */
  bool frame_valid = false;
  /** 成功帧时间戳；同一时间戳表示本控制周期没有新帧 */
  int64_t frame_stamp_ns = 0;
  /** 成功帧距当前控制时刻的年龄 (s) */
  double frame_age_s = std::numeric_limits < double > ::infinity();
  /** 该源前向走廊净空；+inf 表示该完整帧内没有前向障碍 */
  double clearance_m = std::numeric_limits < double > ::infinity();
};

/** 左/右斜雷达帧融合与轮速外推结果。 */
struct FrontLowpassClearanceEstimate
{
  double clearance_m = std::numeric_limits < double > ::infinity();
  bool predicted = false;
  double observation_age_s = std::numeric_limits < double > ::infinity();
  /** false：数据已超过外推窗口，不能据此从未锁存状态新触发 */
  bool valid_for_trigger = false;
  /** 0=left_tilt，1=right_tilt，-1=无有限障碍 */
  int source_index = -1;
};

/**
 * 低通专用斜雷达帧跟踪器：同一帧未刷新时按反馈轮速扣减净空；
 * 新时间戳到达后整体替换该源历史，不与上一帧叠加。
 */
class FrontLowpassTiltTracker
{
public:
  FrontLowpassTiltTracker() = default;
  explicit FrontLowpassTiltTracker(const FrontObstacleLowpassStopParams & params);

  void updateParams(const FrontObstacleLowpassStopParams & params);
  FrontLowpassClearanceEstimate update(
    const std::array < FrontLowpassSourceSample, 2 > & samples,
    double fb_linear_m_s,
    double dt_s);
  void reset();

private:
  struct SourceState
  {
    bool has_frame = false;
    int64_t frame_stamp_ns = 0;
    double measured_clearance_m = std::numeric_limits < double > ::infinity();
    double frame_age_s = 0.0;
    double forward_travel_m = 0.0;
  };

  FrontObstacleLowpassStopParams params_;
  std::array < SourceState, 2 > sources_;
};

/**
 * 节点层判定：当前急停是否可让位给前向近障低通。
 * is_map_veto_reason 由调用方用 emergency_reason == MAP_VETO 填入。
 * 仅看前向走廊净空（与低通闸同一判据）；不用全向 Footprint min_clearance——
 * 侧向/角点近障会把 min_clearance 提前打进硬停线，导致走廊仍在触发带内也无法柔停
 *（见 bag speed_test_tilt_on_18）。
 */
bool mapVetoYieldsToFrontLowpass(
  bool override_enabled,
  bool emergency_brake,
  bool is_map_veto_reason,
  double forward_corridor_clearance_m,
  double hard_stop_distance_m);

/** 单周期输入 */
struct FrontLowpassStopInput
{
  /** 前向走廊最近净空 (m)，取自 hybrid_log.forward_corridor_clearance_m；+inf = 无障碍 */
  double forward_clearance_m = 0.0;
  /** 轮速反馈线速度 (m/s)，用于触发判定 */
  double fb_linear_m_s = 0.0;
  /** 上一周期输出线速度 (m/s)；与反馈取大者用于反算 τ（取大更保守） */
  double command_linear_m_s = 0.0;
  /** 驾驶员是否在给后退意图（脱困出路，立即解锁） */
  bool reverse_intent = false;
  /**
   * 本周期是否仍有前进意图（注入避障链的 desired.linear.x > 0，含巡航注入的保持速度）。
   * 停稳 + 无前进意图 = 本次快速逼近事件结束 → 解锁，放行后续慢速贴近。
   */
  bool forward_intent = false;
  /** false 时本帧只允许维持已有锁存，不能由过期观测新触发 */
  bool trigger_observation_valid = true;
  /**
   * 本周期是否应由急停阻断本闸（节点层已折算：MAP_VETO 可放行时为 false）。
   * true → 立即解锁、不接管。
   */
  bool emergency_brake = false;
  double dt_s = 0.0;
};

/** 单周期裁决 */
struct FrontLowpassStopResult
{
  /** 本周期是否由低通刹停接管输出 */
  bool active = false;
  /** 本周期低通时间常数 (s)；未接管为 0 */
  double tau_s = 0.0;
  /** 本周期是否刚完成锁存（供节点层打一次日志） */
  bool just_latched = false;
  /** 本周期是否刚解锁（供节点层打一次日志） */
  bool just_released = false;
  /** 解锁原因（just_released 时有效），供日志；未解锁为 nullptr */
  const char * release_reason = nullptr;
};

class FrontLowpassStopGate
{
public:
  FrontLowpassStopGate() = default;
  explicit FrontLowpassStopGate(const FrontObstacleLowpassStopParams & params);

  void updateParams(const FrontObstacleLowpassStopParams & params);

  /** 每周期必须调用（含未接管帧），否则锁存/确认计数会走陈 */
  FrontLowpassStopResult update(const FrontLowpassStopInput & in);

  void reset();
  bool latched() const {return latched_;}

private:
  /** 按剩余净空反算时间常数：τ = (净空 − stop_buffer_m)/v，钳到 [min, filter] */
  double timeConstant(double forward_clearance_m, double speed_m_s) const;

  FrontObstacleLowpassStopParams params_;
  bool latched_ = false;
  int confirm_count_ = 0;
  double clear_time_s_ = 0.0;
  /**
   * 最近一次「有限」净空 (m)，用于判别 +inf 的两种含义（见 update 内注释）。
   * 0 = 尚无有限观测，保守视为障碍仍在。
   */
  double last_finite_clearance_m_ = 0.0;
};
