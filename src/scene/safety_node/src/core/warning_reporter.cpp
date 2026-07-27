/**
 * @file warning_reporter.cpp
 * @brief 四向碰撞预警状态机实现，语义说明见 core/warning_reporter.h
 */
#include "safety_node/core/warning_reporter.h"

#include <cmath>

namespace
{
constexpr int kDirCount = 4;

/** CollisionDirection → level_/last_publish_s_ 数组下标；NONE/未知返回 -1 */
int dirIndex(CollisionDirection direction)
{
  switch (direction) {
    case CollisionDirection::FORWARD:
      return 0;
    case CollisionDirection::BACKWARD:
      return 1;
    case CollisionDirection::LEFT:
      return 2;
    case CollisionDirection::RIGHT:
      return 3;
    default:
      return -1;
  }
}

CollisionDirection directionOfIndex(int index)
{
  switch (index) {
    case 0:
      return CollisionDirection::FORWARD;
    case 1:
      return CollisionDirection::BACKWARD;
    case 2:
      return CollisionDirection::LEFT;
    case 3:
      return CollisionDirection::RIGHT;
    default:
      return CollisionDirection::NONE;
  }
}

int16_t levelValue(CollisionWarningLevel level)
{
  return static_cast<int16_t>(level);
}

/** 刹停优先于减速：同方向两个标志同时置位时按更严的等级上报 */
int16_t targetLevel(bool brake, bool decel)
{
  if (brake) {
    return levelValue(CollisionWarningLevel::BRAKE);
  }
  if (decel) {
    return levelValue(CollisionWarningLevel::DECELERATION);
  }
  return levelValue(CollisionWarningLevel::NONE);
}
}  // namespace

bool isObstacleEmergencyReason(EmergencyStopReason reason)
{
  switch (reason) {
    case EmergencyStopReason::MAP_VETO:
    case EmergencyStopReason::EMERGENCY_TTC:
    case EmergencyStopReason::BRAKING_DISTANCE_INSUFFICIENT:
    case EmergencyStopReason::HARD_STOP_DISTANCE:
    case EmergencyStopReason::RISK_EMERGENCY:
    case EmergencyStopReason::TILT_NEAR_DISTANCE:
    case EmergencyStopReason::TILT_TTC:
    case EmergencyStopReason::STANDSTILL_GUARD:
    case EmergencyStopReason::SUDDEN_OBSTACLE:
    case EmergencyStopReason::BRAKE_SATURATED:
      return true;
    // INPUT_FAULT/MODE_ESTOP/BRAKE_HOLD 是系统状态而非碰撞风险，报成碰撞预警会误导消费端。
    default:
      return false;
  }
}

int16_t warningTypeOf(CollisionDirection direction)
{
  switch (direction) {
    case CollisionDirection::FORWARD:
      return static_cast<int16_t>(CollisionWarningType::FORWARD);
    case CollisionDirection::BACKWARD:
      return static_cast<int16_t>(CollisionWarningType::BACKWARD);
    case CollisionDirection::LEFT:
      return static_cast<int16_t>(CollisionWarningType::LEFT);
    case CollisionDirection::RIGHT:
      return static_cast<int16_t>(CollisionWarningType::RIGHT);
    default:
      return static_cast<int16_t>(CollisionWarningType::NONE);
  }
}

CollisionWarningReporter::CollisionWarningReporter(
  const CollisionWarningReporterConfig & config)
: config_(config)
{
}

void CollisionWarningReporter::updateConfig(const CollisionWarningReporterConfig & config)
{
  config_ = config;
}

void CollisionWarningReporter::reset()
{
  level_.fill(levelValue(CollisionWarningLevel::NONE));
  last_publish_s_.fill(0.0);
}

int16_t CollisionWarningReporter::levelOf(CollisionDirection direction) const
{
  const int index = dirIndex(direction);
  return index >= 0 ? level_[index] : levelValue(CollisionWarningLevel::NONE);
}

std::vector<CollisionWarningEvent> CollisionWarningReporter::update(
  const CollisionWarningInput & in)
{
  std::vector<CollisionWarningEvent> events;
  if (!config_.enabled) {
    return events;
  }

  // suppressed 时目标全 NONE：已有预警会在下面的跳变分支收敛出驶离事件（见头文件 ③）
  std::array<int16_t, kDirCount> target {};
  target.fill(levelValue(CollisionWarningLevel::NONE));
  if (!in.suppressed) {
    target[0] = targetLevel(in.levels.forward_brake, in.levels.forward_decel);
    target[1] = targetLevel(in.levels.backward_brake, in.levels.backward_decel);
    target[2] = targetLevel(in.levels.left_brake, in.levels.left_decel);
    target[3] = targetLevel(in.levels.right_brake, in.levels.right_decel);

    // 急停兜底：车已停住但 levels 全空（map_veto / 斜雷达硬闸 / 突发障碍等旁路了
    // fillInterventionLevels）。用 primary、退而用节点层给的运动意图方向补一条刹停。
    if (config_.report_emergency_fallback && in.emergency_brake &&
      isObstacleEmergencyReason(in.emergency_reason))
    {
      const CollisionDirection direction =
        in.levels.primary != CollisionDirection::NONE ?
        in.levels.primary :
        in.fallback_direction;
      const int index = dirIndex(direction);
      if (index >= 0 && target[index] == levelValue(CollisionWarningLevel::NONE)) {
        target[index] = levelValue(CollisionWarningLevel::BRAKE);
      }
    }
  }

  for (int i = 0; i < kDirCount; ++i) {
    const int16_t prev = level_[i];
    const int16_t next = target[i];
    const int16_t type = warningTypeOf(directionOfIndex(i));

    if (prev != next) {
      // 先补上一等级对应的驶离事件（减速→3 / 刹停→4），再发新等级。
      if (prev == levelValue(CollisionWarningLevel::DECELERATION)) {
        events.push_back({type, levelValue(CollisionWarningLevel::LEAVE_DECELERATION_ZONE)});
      } else if (prev == levelValue(CollisionWarningLevel::BRAKE)) {
        events.push_back({type, levelValue(CollisionWarningLevel::LEAVE_BRAKE_ZONE)});
      }
      if (next != levelValue(CollisionWarningLevel::NONE)) {
        events.push_back({type, next});
        last_publish_s_[i] = in.now_s;
      }
      level_[i] = next;
      continue;
    }

    if (next == levelValue(CollisionWarningLevel::NONE)) {
      continue;
    }
    // 稳态心跳：elapsed 为负（时钟回跳）或 NaN 时条件为真 → 立即重发并重新对时。
    const double elapsed = in.now_s - last_publish_s_[i];
    if (!(elapsed < config_.repeat_interval_s) || !(elapsed >= 0.0)) {
      events.push_back({type, next});
      last_publish_s_[i] = in.now_s;
    }
  }
  return events;
}
