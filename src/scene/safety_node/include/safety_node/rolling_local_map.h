/**
 * @file rolling_local_map.h
 * @brief 滚动局部占据栅格（ego-centric）
 *
 * 设计要点（HYBRID_FOOTPRINT_LOCALMAP.md §3）：
 * - 以 base_link 为中心，默认 6m×6m、分辨率 5cm
 * - V1 每周期全量重建（rebuild_each_frame=true），无跨帧平移累积
 * - 两层：probability（观测融合）+ inflated（圆形膨胀）
 * - 供 MapConstraintCheck 做轨迹 lethal 硬约束，不参与 Footprint 选速评分
 *
 * 融合公式（贝叶斯-lite）：
 *   p(cell,t) = clamp( p_prev*decay + w_obs*(1 - p_prev*decay), 0, 1 )
 *   cost = max(probability, inflated)
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>
#include <vector>

class RollingLocalMap {
public:
  explicit RollingLocalMap(const LocalMapConfig & config);

  void updateConfig(const LocalMapConfig & config);

  /**
   * @brief 写入一帧点云观测
   * @param points (x,y,z) base_link 下有效点
   * @param w_obs  单帧命中权重，<0 时使用 config_.observation_weight
   */
  void integratePoints(
    const std::vector < std::tuple < double, double, double >> & points,
    const rclcpp::Time & stamp,
    double w_obs = -1.0);

  /** 可选弱耦合：ObstacleField 中心点以较低权重命中地图 */
  void integratePrimitives(const ObstacleField & field, double w_obs);

  /** 衰减 / 过期清除 / 膨胀，刷新 grid_ 快照 */
  void finalizeFrame(const rclcpp::Time & stamp);

  const RollingLocalMapGrid & grid() const {return grid_;}

  /** 查询综合占据代价 [0,1] */
  float queryCost(double x, double y) const;

  /** cost >= lethal_threshold 视为 lethal */
  bool lethalQuery(double x, double y, double lethal_threshold) const;

  /** 转为 OccupancyGrid 供 RViz 调试 */
  nav_msgs::msg::OccupancyGrid toOccupancyGridMsg(const rclcpp::Time & stamp) const;

  /** 模式切换时概率全清零 */
  void reset();

private:
  void initGrid();
  bool worldToGrid(double x, double y, int & col, int & row) const;
  void gridToWorld(int col, int row, double & x, double & y) const;
  size_t cellIndex(int col, int row) const;

  /** 单格命中：饱和累加 + 时间衰减 */
  void hitCell(int col, int row, double w_obs, const rclcpp::Time & stamp);

  /** 对 hot cell 做圆形 falloff 膨胀 */
  void applyInflation();

  /** 超过 max_hit_age_s 未更新的格子清零（非 rebuild 模式） */
  void pruneStale(const rclcpp::Time & now);

  LocalMapConfig config_;
  RollingLocalMapGrid grid_;
  std::vector < rclcpp::Time > last_hit_; ///< 每格上次命中时间，用于衰减
  rclcpp::Time last_update_ {0, 0, RCL_ROS_TIME};
};
