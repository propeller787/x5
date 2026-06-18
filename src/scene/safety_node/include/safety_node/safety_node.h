/**
 * @file safety_node.h
 * @brief Footprint + Rolling LocalMap 混合避障主节点
 *
 * 职责：
 * 1. 订阅多路雷达、手柄、轮速反馈
 * 2. 10 Hz 更新 RollingLocalMap，20 Hz 执行避障控制环
 * 3. 发布经安全干预后的 /nav/control
 *
 * 数据流：
 *   LidarPerception → ObstacleField + 点云
 *   RollingLocalMap ← 点云落格
 *   AvoidanceController ← 手柄 + ObstacleField + LocalMap
 *   OutputShaper → /nav/control
 *
 * 注意：须在 std::make_shared<SafetyNode>() 之后调用 attachComponents()，
 *       因内部需要使用 shared_from_this() 创建订阅与定时器。
 */
#pragma once

#include "safety_node/avoidance_controller.h"
#include "safety_node/lidar_perception.h"
#include "safety_node/mode_supervisor.h"
#include "safety_node/output_shaper.h"
#include "safety_node/rolling_local_map.h"
#include "safety_node/safety_params.h"
#include "safety_node/vehicle_io.h"

#include <geometry_msgs/msg/twist.hpp>
#include <interface/msg/tilt_obstacle_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <atomic>
#include <memory>

class SafetyNode: public rclcpp::Node {
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SafetyNode() override;

  /** 挂载感知、控制、定时器（make_shared 之后调用） */
  void attachComponents();

private:
  void declareAndLoadParams();
  void setupComponents();
  void setupTimers();

  /** 20 Hz 主控制环：读输入 → 避障 → 整形 → 发布 */
  void onControlTick();

  /** 10 Hz 局部地图融合与膨胀 */
  void onLocalMapUpdate();

  /** 5 Hz 占据栅格可视化（可选） */
  void onMapVizTick();

  void onPerceptionVizTick();

  void publishControl(const geometry_msgs::msg::Twist & cmd);

  /** 仅在 Map veto 或急停时发布 JSON 干预日志 */
  void publishInterventionLog(const AvoidanceResult & result);

  /** 模式切换时清空地图、滤波器与感知缓存 */
  void resetPipelineState();

  SafetyNodeParams params_;
  ModeSupervisor mode_supervisor_;

  std::unique_ptr < LidarPerception > perception_;
  std::unique_ptr < RollingLocalMap > local_map_;
  std::unique_ptr < AvoidanceController > controller_;
  std::unique_ptr < VehicleIo > vehicle_io_;
  OutputShaper output_shaper_;

  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr control_pub_;
  rclcpp::Publisher < nav_msgs::msg::OccupancyGrid > ::SharedPtr map_viz_pub_;
  rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr perception_points_pub_;
  rclcpp::Publisher < visualization_msgs::msg::MarkerArray > ::SharedPtr perception_markers_pub_;
  rclcpp::Publisher < interface::msg::TiltObstacleScan > ::SharedPtr tilt_left_debug_pub_;
  rclcpp::Publisher < interface::msg::TiltObstacleScan > ::SharedPtr tilt_right_debug_pub_;
  rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr log_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr map_update_timer_;
  rclcpp::TimerBase::SharedPtr map_viz_timer_;
  rclcpp::TimerBase::SharedPtr perception_viz_timer_;

  bool perception_viz_enabled_ = false;
  double perception_viz_rate_hz_ = 5.0;

  geometry_msgs::msg::Twist prev_output_;         ///< 上一周期输出，用于斜率限制
  rclcpp::Time last_control_time_ {0, 0, RCL_ROS_TIME};
  std::atomic < bool > cruise_active_ {false};    ///< 巡航模式（预留）
  double cruise_linear_cap_ = 0.0;
};
