/**
 * @file lidar_perception.h
 * @brief 多路雷达感知适配层
 *
 * 职责（对应架构 PerceptionAdapter）：
 * 1. 订阅左/右/后 LaserScan 与左/右倾斜 PointCloud2
 * 2. TF 变换到 base_link，ROI 裁剪
 * 3. 双通道输出：
 *    - pending_points_            → RollingLocalMap 落格（takePendingPoints 取走）
 *    - horizontal/tilt_obstacle_points_ → 体素化 ObstacleField（Footprint 快路径）
 *
 * 与 ObstacleField 关系：双通道不互相覆盖，LocalMap 负责空间记忆，primitive 负责几何扫掠。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/vehicle_footprint.h"

#include <array>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <tuple>
#include <vector>

struct PerceptionDebugPoint
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  uint8_t source_id = 0;
};

struct PerceptionHealth
{
  size_t fresh_source_count = 0;
  size_t received_source_count = 0;
  size_t fresh_horizontal_source_count = 0;
  size_t received_horizontal_source_count = 0;
  size_t fresh_tilt_source_count = 0;
  size_t received_tilt_source_count = 0;
  // ── 方案 4 降级矩阵输入：per-source 位图（索引与 PerceptionUtils::sourceId
  // 一致：0 left / 1 right / 2 back / 3 left_tilt / 4 right_tilt）──
  /** 各源是否被配置启用（未启用的源不参与降级判定） */
  std::array<bool, 5> source_enabled {};
  /** 各源当前是否新鲜（已收到消息 + TF 有效 + 均在时限内） */
  std::array<bool, 5> source_fresh {};
};

struct PerceptionCacheStats
{
  size_t pending_points = 0;
  size_t obstacle_points = 0;
  size_t debug_points = 0;
};

/** 单路斜雷达调试快照（base_link） */
struct TiltSourceSnapshot
{
  std::string source_name;
  uint8_t source_id = 0;
  /** 最近一帧成功完成 TF/过滤；即使该帧为空也为 true */
  bool frame_valid = false;
  /** 最近成功帧时间戳，供控制环判断是否刷新 */
  int64_t frame_stamp_ns = 0;
  /** 最近成功帧距 snapshot 时刻的年龄 (s) */
  double frame_age_s = std::numeric_limits < double > ::infinity();
  std::vector<ObstaclePoint> filtered_points;
  ObstacleField primitive_field;
};

class LidarPerception {
public:
  explicit LidarPerception(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);

  /** 取出并清空本周期待融合点云（供 LocalMap 10Hz 线程） */
  std::vector < std::tuple < double, double, double >> takePendingPoints();

  /** 从累积点云构建当前 ObstacleField（供 20Hz 控制环） */
  ObstacleField buildObstacleField(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;
  ObstacleField buildHorizontalObstacleField(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;
  /** 斜雷达点云体素化，输出供 TTC 近距通道 evaluateTiltIntervention 使用 */
  ObstacleField buildTiltObstacleField(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;

  std::vector < PerceptionDebugPoint > takeDebugPoints();

  PerceptionHealth health(const rclcpp::Time & now, double timeout_s) const;
  PerceptionCacheStats cacheStats() const;

  /** 返回感知 ROI 边界 [x_min, x_max, y_min, y_max] 供可视化对齐 */
  struct RoiBounds { double x_min, x_max, y_min, y_max; };
  RoiBounds roiBounds() const { return {roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_}; }

  /** 按 left_tilt / right_tilt 返回过滤点与体素 primitive，供 TiltObstacleScan 调试发布 */
  std::vector<TiltSourceSnapshot> snapshotTiltSources(
    const AvoidanceParams & params,
    const rclcpp::Time & stamp) const;

  /** 模式切换时清空缓存 */
  void clear();

private:
  bool acceptSourceMessage(
    const std::string & source_name,
    const builtin_interfaces::msg::Time & stamp);
  void markSourceTransformValid(
    const std::string & source_name,
    const rclcpp::Time & stamp);

  void processScan(
    const sensor_msgs::msg::LaserScan::SharedPtr & msg,
    const std::string & source_frame,
    const std::string & source_name);

  void processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    const std::string & default_frame,
    const std::string & source_name);

  /** 与水平雷达 processScan 相同：自车/安装区过滤 + 全局 ROI */
  bool acceptBaseLinkObstaclePoint(
    double x, double y, double z,
    bool cloud_source = false,
    bool for_tilt = false) const;

  /** 斜雷达在 base_link 下接受判据：可选 z 阈值 + acceptBaseLinkObstaclePoint */
  bool acceptTiltObstacleInBaseLink(double x, double y, double z) const;

  bool isHorizontalSourceEnabled(const std::string & source_name) const;
  bool isSourceEnabledForHealth(uint8_t source_id) const;

  void appendBaseLinkPoints(
    const std::vector < std::tuple < double, double, double >> & pts,
    const std::string & source_name,
    const rclcpp::Time & stamp);
  bool rejectBaseLinkPoint(
    double x, double y, double z, bool cloud_source, bool for_tilt = false) const;

  bool lookupTransform(
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr < tf2_ros::Buffer > tf_buffer_;
  std::shared_ptr < tf2_ros::TransformListener > tf_listener_;

  // 必须持有 shared_ptr，否则 create_subscription 临时对象析构后订阅会被销毁
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr left_scan_sub_;
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr right_scan_sub_;
  rclcpp::Subscription < sensor_msgs::msg::LaserScan > ::SharedPtr back_scan_sub_;
  rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr left_tilt_cloud_sub_;
  rclcpp::Subscription < sensor_msgs::msg::PointCloud2 > ::SharedPtr right_tilt_cloud_sub_;

  double tilt_z_threshold_ = 0.27;    ///< 倾斜雷达 z 过滤阈值

  bool debug_enabled_ = false;
  size_t debug_max_points_ = 30000;
  size_t max_points_per_source_ = 30000;
  size_t max_pending_points_ = 60000;
  bool local_map_enabled_ = false;
  bool horizontal_enabled_ = true;
  bool horizontal_left_enabled_ = true;
  bool horizontal_right_enabled_ = true;
  bool horizontal_back_enabled_ = true;
  bool horizontal_feed_local_map_ = true;
  bool tilt_enabled_ = true;
  bool tilt_ground_reject_enabled_ = true;
  bool tilt_z_filter_enabled_ = false;
  /** true：TF 后保留传感器 frame 原始 z；false：完整三维变换 z 到 base_link */
  bool tilt_preserve_sensor_z_ = true;
  bool tilt_feed_local_map_ = false;
  /** true：把斜雷达点并入水平雷达数据流（当作水平雷达用于 Footprint/制动包络与健康统计） */
  bool tilt_treat_as_horizontal_ = false;
  double tilt_voxel_size_m_ = 0.08;
  size_t tilt_max_primitives_ = 48;
  double tf_lookup_timeout_s_ = 0.02;
  double perception_timeout_s_ = 0.35;
  bool self_filter_enabled_ = true;
  double self_filter_x_min_ = -0.36;
  double self_filter_x_max_ = 0.95;
  double self_filter_y_abs_ = 0.36;
  double self_filter_z_min_ = -0.15;
  double self_filter_z_max_ = 0.35;
  /** 自车过滤半宽在真实车体轮廓之外附加的横向余量（m），车头斜边段随轮廓收窄 */
  double self_filter_side_margin_ = 0.02;
  /** 真实车体轮廓（含前段斜边），用于按 x 处半宽收窄自车过滤，避免斜边外障碍被误滤 */
  std::optional<VehicleFootprint> self_filter_footprint_;
  double mount_edge_reject_radius_m_ = 0.05;
  double ground_reject_below_z_m_ = -0.06;

  /** 感知 ROI 边界（从 ROS 参数加载），默认 [-2.5,6.5]×[-1.8,1.8] */
  double roi_x_min_ = -2.5;
  double roi_x_max_ = 6.5;
  double roi_y_min_ = -1.8;
  double roi_y_max_ = 1.8;

  mutable std::mutex mutex_;
  std::vector < std::tuple < double, double, double >> pending_points_; ///< LocalMap 用
  std::vector < PerceptionDebugPoint > debug_points_;
  std::vector < ObstaclePoint > horizontal_obstacle_points_;
  std::vector < ObstaclePoint > tilt_obstacle_points_;
  std::array < std::vector < std::tuple < double, double, double >>, 5 >
  latest_source_points_;
  std::array < rclcpp::Time, 5 > latest_source_points_stamp_;
  std::array < bool, 5 > latest_source_points_valid_ {{false, false, false, false, false}};
  std::array < rclcpp::Time, 5 > last_source_message_;
  std::array < rclcpp::Time, 5 > last_source_transform_;
  std::array < bool, 5 > source_received_ {{false, false, false, false, false}};
  std::array < bool, 5 > source_transform_valid_ {{false, false, false, false, false}};
};
