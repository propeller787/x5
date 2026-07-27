/**
 * @file warning_reporter.h
 * @brief 四向碰撞预警状态机（/vlago/collision/warning 上报层）
 *
 * 把避障链每周期算出的 SweptInterventionLevels（前/后/左/右 × 刹停/减速）翻译成
 * 整车约定的 CollisionWarning 事件流，语义对齐 scene_node::DecisionMaker::updateWarningStatus：
 * - 等级取值见 common/utils/warning_type.h：1 减速 2 刹停 3 驶离减速区 4 驶离刹停区；
 * - 类型取值同上：1 地上前向 2 地上左向 3 地上右向 4 地上后向。
 *
 * ⚠ 只上报地上四向（1~4）。负平面类型（5~7）依赖 scene_node 的负平面（凹坑）检测，
 * safety_node 无对应感知通道，宁可不报也不伪造。
 *
 * 三条与朴素实现的关键差异：
 * ① 驶离事件由状态跳变生成，而非「无障碍就发」：等级 A→B 时先补一条 A 对应的驶离
 *    （减速→3 / 刹停→4）再发 B。消费端（TFT 弹窗 / 灯效 / 语音）是状态机，
 *    漏掉驶离会让提示永久挂住。DECEL→BRAKE 这类升级同样先发 3 再发 2，与 scene_node 逐位一致。
 * ② 稳态限频、跳变不限频：同方向同等级持续存在时按 repeat_interval_s 心跳重发
 *   （scene_node 是每个决策周期都发，50 Hz 下纯属刷屏），但任何等级跳变立即发，
 *    不受心跳节流影响——节流跳变会让驶离提示延迟最多 1 s。
 * ③ 关安全 = 喂全零输入而非 reset()：suppressed=true 时把四向目标等级压成 NONE，
 *    于是驶离事件由 ① 自然产生。若改用 reset() 静默清状态，消费端会卡在最后一次预警上。
 *    reset() 仅供测试与冷启动使用。
 *
 * 急停兜底（report_emergency_fallback）：map_veto / 斜雷达近距硬闸 / 突发障碍等
 * 急停路径不经过 fillInterventionLevels，levels 全空但车实际已经停住。此时用
 * levels.primary（无则用节点层给的运动意图方向）补一条刹停，否则真停车时消费端毫无提示。
 * 仅对障碍类急停生效：INPUT_FAULT / MODE_ESTOP / BRAKE_HOLD 不是碰撞，不报
 *（见 isObstacleEmergencyReason）。
 *
 * 纯状态机、无 ROS 依赖，便于单元测试（test_warning_reporter.cpp）。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <array>
#include <cstdint>
#include <vector>

/** 告警等级，值对齐 common/utils/warning_type.h 的 WarningLevel */
enum class CollisionWarningLevel : int16_t
{
  NONE = 0,
  DECELERATION = 1,           ///< 减速
  BRAKE = 2,                  ///< 刹停
  LEAVE_DECELERATION_ZONE = 3,  ///< 驶离减速区
  LEAVE_BRAKE_ZONE = 4,       ///< 驶离刹停区
};

/** 告警类型，值对齐 common/utils/warning_type.h 的 WarningType（仅地上四向） */
enum class CollisionWarningType : int16_t
{
  NONE = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  BACKWARD = 4,
};

/** 单条待发布的预警，节点层直接填进 interface::msg::CollisionWarning */
struct CollisionWarningEvent
{
  int16_t warning_type = 0;
  int16_t warning_level = 0;
};

/** 对应 YAML：reporting.collision_warning.* */
struct CollisionWarningReporterConfig
{
  /** 总开关；false 时 update() 恒返回空且不维护状态 */
  bool enabled = true;
  /** 同方向同等级的心跳重发间隔 (s)；<=0 表示每周期都发。等级跳变不受此节流约束 */
  double repeat_interval_s = 1.0;
  /** 障碍类急停但 levels 全空时，是否用 primary/运动意图方向补一条刹停 */
  bool report_emergency_fallback = true;
};

/** 单周期输入 */
struct CollisionWarningInput
{
  /** 避障链算出的四向干预等级，取自 result.risk.levels */
  SweptInterventionLevels levels;
  bool emergency_brake = false;
  EmergencyStopReason emergency_reason = EmergencyStopReason::NONE;
  /**
   * 急停兜底方向：levels.primary 为 NONE 时改用本方向（节点层按驾驶员意图/实际速度给出）。
   * NONE 表示无法判定方向 → 不补报。
   */
  CollisionDirection fallback_direction = CollisionDirection::NONE;
  /** true：本周期不产生新预警（如安全开关关闭/透传），已有预警会收敛出驶离事件 */
  bool suppressed = false;
  /** 当前时刻 (s)，仅用于心跳节流；时钟跳变/NaN 时退化为立即重发 */
  double now_s = 0.0;
};

/** 该急停原因是否由障碍物触发（决定是否值得上报为碰撞预警） */
bool isObstacleEmergencyReason(EmergencyStopReason reason);

/** CollisionDirection → CollisionWarning.warning_type；NONE/未知返回 0 */
int16_t warningTypeOf(CollisionDirection direction);

class CollisionWarningReporter
{
public:
  CollisionWarningReporter() = default;
  explicit CollisionWarningReporter(const CollisionWarningReporterConfig & config);

  void updateConfig(const CollisionWarningReporterConfig & config);

  /** 每周期必须调用（含无预警帧），否则驶离事件不会被生成 */
  std::vector<CollisionWarningEvent> update(const CollisionWarningInput & in);

  /** 静默清空四向等级；不产生驶离事件，仅供测试/冷启动（见文件头 ③） */
  void reset();

  /** 当前记录的方向等级（NONE/DECELERATION/BRAKE），供测试与日志 */
  int16_t levelOf(CollisionDirection direction) const;

private:
  CollisionWarningReporterConfig config_;
  /** 索引见 warning_reporter.cpp 的 dirIndex()：0 前 1 后 2 左 3 右 */
  std::array<int16_t, 4> level_ {};
  std::array<double, 4> last_publish_s_ {};
};
