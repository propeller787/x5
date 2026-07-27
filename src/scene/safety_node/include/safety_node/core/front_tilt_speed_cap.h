/**
 * @file front_tilt_speed_cap.h
 * @brief 斜雷达前向近障软限速闸（避障入口限速层）
 *
 * 触发于「斜雷达前向走廊（|y| ≤ 车体半宽 + corridor_margin_m）内探到障碍且净空
 * ≤ trigger_distance_m」，把前向线速度上限钳到 speed_cap_m_s（默认 1.25 m/s = 4.5 km/h）。
 *
 * 与 front_lowpass_stop（贴身近障柔和刹到零，6~70cm）互补：本层是「较远处存在障碍
 * 就先降速巡行」的软限速，只压前向线速度上限，不刹停、不动角速度、不动后退。
 *
 * ── 只用斜雷达 ──
 * 障碍数据来自 LidarPerception::snapshotTiltSources() 的 left_tilt/right_tilt 原始源
 * （与 front_lowpass tilt_only 同一取数路径）。即使斜雷达 treat_as_horizontal=true，
 * 快照仍按原始源单独取，不借用合并后的场，也不读水平 ObstacleField/LocalMap/MDB/TTC。
 * 走廊净空由 AvoidanceController::frontCorridorClearance(field, corridor_margin_m) 逐源计算，
 * 左右取 min 后喂入本闸。
 *
 * ── 生效方式 ──
 * 节点层把本闸输出的 cap 写入 AvoidanceContext::front_tilt_speed_cap_m_s；evaluate 入口
 * 与 cruise_linear_cap / degradation_speed_cap_m_s 同款，仅对正向 eval_cmd 取 min。
 * 因限幅点在 eval_cmd 上，巡航保持速度（已在节点层注入 desired→cmd）同样被限。
 * 输出经 OutputShaper 按 max_decel 平滑降到帽值，无顿挫；障碍清除后 shape 平滑恢复。
 *
 * ── 不影响其他功能的契约 ──
 * ① enabled=false → cap 恒 +inf → evaluate 限幅分支永不进入 → 输出逐字节等价旧行为。
 * ② 只降不升：仅当命令 > 帽值才压，且仅正向；角速度 / 后退 / 低速命令零影响。
 * ③ 走 eval_cmd 入口 min，天然与其它限速取最小——只可能更慢，永不覆盖更强的停车/急停。
 * ④ requested_cmd（recovery/rotation_escape 意图判定）保持原始不受影响。
 *
 * ── 陈旧数据处理 ──
 * snapshotTiltSources 逐周期返回各源「最新完整帧」的点（新帧到达前保持不变，不清空），
 * 故控制频率高于雷达帧率时净空天然稳定，无需跨帧外推。传感器真失效由节点层
 * INPUT_FAULT（require_tilt_perception）兜底；本层在陈旧数据下保持限速是 fail-safe 方向。
 *
 * 纯状态机、无 ROS 依赖，便于单元测试（test_front_tilt_speed_cap.cpp）。
 */
#pragma once

#include <limits>

/** 对应 YAML：avoidance.front_tilt_speed_cap.* */
struct FrontTiltSpeedCapParams
{
  bool enabled = false;
  /** 走廊余量 (m)：走廊半宽 = 车体半宽（physicalRect().y_max）+ 此值。 */
  double corridor_margin_m = 0.30;
  /** 触发净空 (m)：前向走廊内障碍到物理车体前缘的距离 ≤ 此值才限速。 */
  double trigger_distance_m = 2.0;
  /** 滞回释放净空 (m)：净空恢复到此值以上持续 clear_hold_s 才解除；须 > trigger_distance_m。 */
  double release_distance_m = 2.3;
  /** 限速上限 (m/s)：4.5 km/h ≈ 1.25 m/s。仅作前向速度上限，不改转向、不刹停。 */
  double speed_cap_m_s = 1.25;
  /** 连续满足触发条件的帧数，滤除单帧体素抖动。 */
  int confirm_frames = 2;
  /** 释放前持续清空时间 (s)。 */
  double clear_hold_s = 0.15;
};

/** 单周期输入 */
struct FrontTiltSpeedCapInput
{
  /** 斜雷达前向走廊最近净空 (m)（左右源取 min）；+inf = 走廊内无障碍。 */
  double forward_clearance_m = std::numeric_limits<double>::infinity();
  double dt_s = 0.0;
};

/** 单周期裁决 */
struct FrontTiltSpeedCapResult
{
  /** 本周期是否限速。 */
  bool active = false;
  /** 本周期前向速度上限 (m/s)；未限速为 +inf。 */
  double speed_cap_m_s = std::numeric_limits<double>::infinity();
  /** 本周期是否刚进入限速（供节点层打一次日志）。 */
  bool just_latched = false;
  /** 本周期是否刚解除（供节点层打一次日志）。 */
  bool just_released = false;
};

class FrontTiltSpeedCapGate
{
public:
  FrontTiltSpeedCapGate() = default;
  explicit FrontTiltSpeedCapGate(const FrontTiltSpeedCapParams & params);

  void updateParams(const FrontTiltSpeedCapParams & params);

  /** 每周期必须调用（含未限速帧），否则确认/滞回计数会走陈。 */
  FrontTiltSpeedCapResult update(const FrontTiltSpeedCapInput & in);

  void reset();
  bool latched() const {return latched_;}

private:
  FrontTiltSpeedCapParams params_;
  bool latched_ = false;
  int confirm_count_ = 0;
  double clear_time_s_ = 0.0;
};
