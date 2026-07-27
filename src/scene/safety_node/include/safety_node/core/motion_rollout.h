/**
 * @file motion_rollout.h
 * @brief 候选速度采样与 unicycle 轨迹积分
 *
 * 根据手柄 Twist 在速度空间采样 ≤20 条候选 (v,w)，
 * 对每条候选前向积分得到 TrajectoryRollout，供 Footprint 扫掠检测。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <geometry_msgs/msg/twist.hpp>
#include <vector>

class MotionRollout {
public:
  explicit MotionRollout(const AvoidanceParams & params);

  void updateParams(const AvoidanceParams & params);

  /** 生成候选速度列表（原地旋转 / 直线 / 弧线分场景采样） */
  std::vector < VelocityCandidate > sampleCandidates(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    double fb_angular) const;

  /** 对单条候选速度积分 unicycle 轨迹 */
  TrajectoryRollout rolloutCandidate(const VelocityCandidate & candidate) const;

  /** 从当前反馈速度起步，按配置的加减速度过渡到候选速度 */
  TrajectoryRollout rolloutCandidate(
    const VelocityCandidate & candidate,
    double initial_linear,
    double initial_angular) const;

  /** 生成全部候选轨迹 */
  std::vector < TrajectoryRollout > rolloutAll(
    const geometry_msgs::msg::Twist & cmd,
    double fb_linear,
    double fb_angular) const;

private:
  AvoidanceParams params_;
};
