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
#include <memory>
#include <mutex>
#include <tuple>
#include <vector>

class RollingLocalMap {
public:
  explicit RollingLocalMap(const LocalMapConfig & config);

  void updateConfig(const LocalMapConfig & config);

  /**
   * 方案 6A 自运动补偿：把历史栅格按自车位移反向平移（整格滚动 + 亚格残差累积），
   * 旋转按累计角行程施加惩罚衰减（见 LocalMapConfig::ego_motion_compensation）。
   * 仅 rebuild_each_frame=false 且补偿开启时有实际动作；地图更新周期调用，
   * 参数为自上次调用以来在旧 base_link 系下的位移/角行程（轮速积分）。
   */
  void applyEgoMotion(double dx_m, double dy_m, double dtheta_rad);

  /**
   * 方案 7 线程安全快照：finalizeFrame/reset 时原子发布只读副本。
   * 控制环 / 可视化跨线程读取一律走 snapshot()，不得直接引用 grid()。
   */
  std::shared_ptr<const RollingLocalMapGrid> snapshot() const;

  /** 从任意栅格快照构建 OccupancyGrid（可视化线程用，静态无锁） */
  static nav_msgs::msg::OccupancyGrid gridToOccupancyMsg(
    const RollingLocalMapGrid & grid, const rclcpp::Time & stamp);

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

  /** 返回工作栅格引用（仅测试用，非线程安全；生产代码请用 snapshot()） */
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

  /** 对 hot cell 做圆形 falloff 膨胀（方案 8.2：邻域偏移查找表） */
  void applyInflation();

  /** 超过 max_hit_age_s 未更新的格子清零（非 rebuild 模式） */
  void pruneStale(const rclcpp::Time & now);

  /** 栅格内容整格搬移（dc/dr 为内容位移的格数），暴露区域清零 */
  void shiftGrid(int dc, int dr);

  /** 把 grid_ 的只读副本发布到 snapshot_（finalizeFrame/reset 末尾调用） */
  void publishSnapshot();

  LocalMapConfig config_;
  RollingLocalMapGrid grid_;
  std::vector < rclcpp::Time > last_hit_; ///< 每格上次命中时间，用于衰减
  rclcpp::Time last_update_ {0, 0, RCL_ROS_TIME};

  /** 方案 8.2 膨胀核查找表：(dr, dc, falloff)，initGrid 时预计算 */
  std::vector < std::tuple < int, int, float >> inflation_kernel_;

  /** 方案 6A 自运动补偿状态：亚格残差与旋转角行程累计 */
  double ego_residual_x_m_ = 0.0;
  double ego_residual_y_m_ = 0.0;
  double rotation_accum_rad_ = 0.0;
  std::vector < float > shift_scratch_;              ///< 平移搬移临时缓冲
  std::vector < rclcpp::Time > shift_scratch_hits_;

  /**
   * 方案 7 跨回调组写保护：reset（control 组）与 applyEgoMotion /
   * integratePoints / finalizeFrame（background 组）可能并发执行，
   * 此 mutex 保证写操作互斥。queryCost / lethalQuery /
   * toOccupancyGridMsg 等直接读 grid_ 的方法也在此锁下执行。
   * snapshot_ 的发布与读取仍由 snapshot_mutex_ 独立保护。
   */
  mutable std::mutex ops_mutex_;

  /** 方案 7 快照双缓冲 */
  mutable std::mutex snapshot_mutex_;
  std::shared_ptr < const RollingLocalMapGrid > snapshot_;
};
