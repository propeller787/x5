// =============================================================================
// lidar_imu_fusion_node.cpp —— 三激光雷达 + IMU 融合节点实现
//
// 本文件实现三部分：
//   1) ScanSynchronizer      三路 10 Hz LaserScan 的近似时间同步
//   2) ImuBuffer             100 Hz IMU 角速度缓存与旋转积分
//   3) ThreeLidarImuFusionNode  节点主体：订阅/同步/去畸变/栅格化/发布
//
// 设计要点：
//   - 仅做旋转去畸变（不消费轮速，无平移补偿）
//   - 单传感器 TF 失败只跳过该雷达，不丢整帧
//   - require_imu_for_publish=true 时，IMU 覆盖不足的同步组暂存于 pending 队列等待
//   - IMU→target 旋转带缓存，TF 查询从 100 Hz 降至 ~1 Hz
// =============================================================================

#include "lidar_imu_fusion/lidar_imu_fusion_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lidar_imu_fusion
{
namespace
{

// 匿名命名空间内的文件级常量与工具函数

constexpr double kPi = 3.14159265358979323846;   // 圆周率
constexpr double kNsToSec = 1.0e-9;               // 纳秒 → 秒
constexpr double kSecToNs = 1.0e9;                // 秒 → 纳秒

// 构造单位四元数 (x=0, y=0, z=0, w=1)，表示无旋转
tf2::Quaternion identityQuaternion()
{
  tf2::Quaternion q;
  q.setValue(0.0, 0.0, 0.0, 1.0);
  return q;
}

// 构造单位变换（无平移、无旋转），用于静态外参的默认初值
tf2::Transform identityTransform()
{
  tf2::Transform transform;
  transform.setIdentity();
  return transform;
}

double stampSec(int64_t stamp_ns)
{
  return static_cast<double>(stamp_ns) * kNsToSec;
}

}  // namespace

// -----------------------------------------------------------------------------
// ScanSynchronizer 实现：三路 LaserScan 近似同步
// -----------------------------------------------------------------------------

// 设置三路 scan 时间戳允许的最大跨度（秒），内部转成纳秒存储
void ScanSynchronizer::setMaxSlop(double max_slop_sec)
{
  max_slop_ns_ = static_cast<int64_t>(std::max(0.0, max_slop_sec) * kSecToNs);
}

// 设置每条队列的最大长度，至少为 1
void ScanSynchronizer::setQueueSize(std::size_t queue_size)
{
  queue_size_ = std::max<std::size_t>(1, queue_size);
}

// 启用节流诊断日志（节点构造时注入 logger/clock；单测不调用则保持静默）
void ScanSynchronizer::setDiagnostics(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
{
  logger_ = logger;
  clock_ = std::move(clock);
  diagnostics_enabled_ = static_cast<bool>(clock_);
}

// 将一帧 scan 入队对应传感器，并按 queue_size_ 截断
void ScanSynchronizer::push(SensorId sensor_id, sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  auto & queue = queueFor(sensor_id);
  queue.push_back(std::move(scan));
  trimQueue(queue);
}

// 尝试从三条队列中弹出一组时间戳尽量接近的三元组。
// 返回 std::nullopt 表示当前无法组成有效三元组。
std::optional<TimedScanSet> ScanSynchronizer::tryPop()
{
  // 任一队列为空则无法配对
  if (left_queue_.empty() || right_queue_.empty() || back_queue_.empty()) {
    return std::nullopt;
  }

  // 每次配对前总是清理陈旧帧：把"比最新帧早 max_slop 以上"的旧帧移除，
  // 避免它们阻碍与 newer 帧的匹配，也避免上一轮匹配成功后遗留的旧帧堆积。
  const std::size_t pruned = pruneStaleScans();
  if (left_queue_.empty() || right_queue_.empty() || back_queue_.empty()) {
    if (diagnostics_enabled_ && pruned > 0) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3000,
        "LaserScan queues became empty after stale-frame prune "
        "(max_sync_slop=%.1f ms, pruned=%zu); waiting for fresh scans",
        static_cast<double>(max_slop_ns_) * kNsToSec * 1000.0,
        pruned);
    }
    return std::nullopt;
  }

  bool found = false;
  std::size_t best_left = 0;
  std::size_t best_right = 0;
  std::size_t best_back = 0;
  int64_t best_span = std::numeric_limits<int64_t>::max();
  int64_t best_any_span = std::numeric_limits<int64_t>::max();

  // 三重循环暴力搜索：在三条队列中找时间跨度最小且 ≤ max_slop 的三元组
  // 队列长度默认 10，O(L·R·B) 最多 1000 次，对 10 Hz 场景开销可忽略
  for (std::size_t li = 0; li < left_queue_.size(); ++li) {
    const int64_t left_time = stampNs(left_queue_[li]);
    for (std::size_t ri = 0; ri < right_queue_.size(); ++ri) {
      const int64_t right_time = stampNs(right_queue_[ri]);
      for (std::size_t bi = 0; bi < back_queue_.size(); ++bi) {
        const int64_t back_time = stampNs(back_queue_[bi]);
        // 跨度 = 三帧中最新与最早的差值
        const int64_t earliest = std::min({left_time, right_time, back_time});
        const int64_t latest = std::max({left_time, right_time, back_time});
        const int64_t span = latest - earliest;
        if (span < best_any_span) {
          best_any_span = span;
        }
        if (span <= max_slop_ns_ && span < best_span) {
          found = true;
          best_span = span;
          best_left = li;
          best_right = ri;
          best_back = bi;
        }
      }
    }
  }

  if (!found) {
    if (diagnostics_enabled_) {
      const int64_t left_front_ns = stampNs(left_queue_.front());
      const int64_t left_back_ns = stampNs(left_queue_.back());
      const int64_t right_front_ns = stampNs(right_queue_.front());
      const int64_t right_back_ns = stampNs(right_queue_.back());
      const int64_t back_front_ns = stampNs(back_queue_.front());
      const int64_t back_back_ns = stampNs(back_queue_.back());
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3000,
        "No synchronized lidar triplet within max_sync_slop (%.1f ms); "
        "best span=%.1f ms, queue sizes L/R/B=%zu/%zu/%zu, "
        "front stamps L=%.6f R=%.6f B=%.6f, back stamps L=%.6f R=%.6f B=%.6f",
        static_cast<double>(max_slop_ns_) * kNsToSec * 1000.0,
        static_cast<double>(best_any_span) * kNsToSec * 1000.0,
        left_queue_.size(), right_queue_.size(), back_queue_.size(),
        stampSec(left_front_ns), stampSec(right_front_ns), stampSec(back_front_ns),
        stampSec(left_back_ns), stampSec(right_back_ns), stampSec(back_back_ns));
    }
    return std::nullopt;
  }

  // 取出匹配的三帧
  TimedScanSet scans{
    left_queue_[best_left],
    right_queue_[best_right],
    back_queue_[best_back],
  };

  // 只删除匹配帧本身，保留更旧的帧供后续匹配（可能仍能与未来帧配对）；
  // 真正过期的旧帧由开头的 pruneStaleScans() 统一清理。
  left_queue_.erase(left_queue_.begin() + static_cast<long>(best_left));
  right_queue_.erase(right_queue_.begin() + static_cast<long>(best_right));
  back_queue_.erase(back_queue_.begin() + static_cast<long>(best_back));
  return scans;
}

// 取 scan 的 header.stamp 转纳秒，用于同步判据
int64_t ScanSynchronizer::stampNs(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  return rclcpp::Time(scan->header.stamp).nanoseconds();
}

// 按 SensorId 取对应队列的引用
ScanSynchronizer::ScanQueue & ScanSynchronizer::queueFor(SensorId sensor_id)
{
  switch (sensor_id) {
    case SensorId::Left:
      return left_queue_;
    case SensorId::Right:
      return right_queue_;
    case SensorId::Back:
      return back_queue_;
  }
  return left_queue_;
}

// 把队列长度截断到 queue_size_，丢掉最旧的帧
void ScanSynchronizer::trimQueue(ScanQueue & queue) const
{
  while (queue.size() > queue_size_) {
    queue.pop_front();
  }
}

// 清理陈旧帧：以三条队列中最新的帧为基准，
// 删除各队列中比"基准 - max_slop"更早的帧（每队列至少保留 1 帧）。
std::size_t ScanSynchronizer::pruneStaleScans()
{
  // 三队列最新帧的时间戳取最大
  const int64_t newest = std::max(
    {
      stampNs(left_queue_.back()),
      stampNs(right_queue_.back()),
      stampNs(back_queue_.back()),
    });

  std::size_t pruned = 0;

  // 对每条队列：从队首弹出"时间 + max_slop < newest"的旧帧
  auto prune = [this, newest, &pruned](ScanQueue & queue) {
      while (queue.size() > 1 && stampNs(queue.front()) + max_slop_ns_ < newest) {
        queue.pop_front();
        ++pruned;
      }
    };

  prune(left_queue_);
  prune(right_queue_);
  prune(back_queue_);

  if (diagnostics_enabled_ && pruned > 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000,
      "Pruned %zu stale LaserScan frame(s) exceeding max_sync_slop (%.1f ms); "
      "queue sizes L/R/B=%zu/%zu/%zu, front stamps L=%.6f R=%.6f B=%.6f, "
      "back stamps L=%.6f R=%.6f B=%.6f, newest=%.6f",
      pruned,
      static_cast<double>(max_slop_ns_) * kNsToSec * 1000.0,
      left_queue_.size(), right_queue_.size(), back_queue_.size(),
      stampSec(stampNs(left_queue_.front())),
      stampSec(stampNs(right_queue_.front())),
      stampSec(stampNs(back_queue_.front())),
      stampSec(stampNs(left_queue_.back())),
      stampSec(stampNs(right_queue_.back())),
      stampSec(stampNs(back_queue_.back())),
      stampSec(newest));
  }

  return pruned;
}

// -----------------------------------------------------------------------------
// ImuBuffer 实现：IMU 角速度缓存与旋转积分
// -----------------------------------------------------------------------------

// 设置样本保留时长（秒），最小 0.1 s
void ImuBuffer::setRetention(double retention_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  retention_sec_ = std::max(0.1, retention_sec);
}

// 设置角速度外推上限（秒），用于 angularVelocityAt 的端点外推
void ImuBuffer::setMaxExtrapolation(double max_extrapolation_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_extrapolation_sec_ = std::max(0.0, max_extrapolation_sec);
}

// 插入一条 IMU 样本：按时间戳有序插入，随后裁剪超出 retention 的旧样本
void ImuBuffer::push(const rclcpp::Time & stamp, const tf2::Vector3 & angular_velocity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  ImuSample sample{stamp, angular_velocity};
  // 线性查找插入位置以保持时间有序（100 Hz × 2 s ≈ 200 样本，开销可忽略）
  auto insert_it = samples_.begin();
  while (insert_it != samples_.end() && insert_it->stamp.nanoseconds() < stamp.nanoseconds()) {
    ++insert_it;
  }
  samples_.insert(insert_it, sample);

  if (samples_.empty()) {
    return;
  }

  // 以最新样本为基准，弹出超出保留时长的旧样本
  const int64_t newest_ns = samples_.back().stamp.nanoseconds();
  const int64_t retention_ns = static_cast<int64_t>(retention_sec_ * kSecToNs);
  while (!samples_.empty() && newest_ns - samples_.front().stamp.nanoseconds() > retention_ns) {
    samples_.pop_front();
  }
}

// 检查 [from, to] 区间是否被 IMU 样本覆盖（含端点外推容差）。
// 由于 angularVelocityAt 对区间内任意点必能插值成功，端点检查等价于整区间检查。
bool ImuBuffer::hasCoverage(const rclcpp::Time & from, const rclcpp::Time & to) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {
    return false;
  }

  tf2::Vector3 unused;
  return angularVelocityAt(from, &unused) && angularVelocityAt(to, &unused);
}

// 积分 [from, to] 区间的角速度，得到累积旋转四元数 Δq。
// 采用中点法（每个子区间取中点角速度），对小子区间二阶精度。
// reverse=true 表示 to < from，最终取逆四元数。
bool ImuBuffer::integrateRotation(
  const rclcpp::Time & from,
  const rclcpp::Time & to,
  tf2::Quaternion * delta_rotation) const
{
  if (delta_rotation == nullptr) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {
    return false;
  }

  // 归一化为 from <= to，记录是否需要反向
  int64_t from_ns = from.nanoseconds();
  int64_t to_ns = to.nanoseconds();
  bool reverse = false;
  if (to_ns < from_ns) {
    std::swap(from_ns, to_ns);
    reverse = true;
  }

  // 端点覆盖检查：任一端点无法取角速度则整体失败
  tf2::Vector3 unused;
  if (!angularVelocityAt(rclcpp::Time(from_ns, RCL_ROS_TIME), &unused) ||
    !angularVelocityAt(rclcpp::Time(to_ns, RCL_ROS_TIME), &unused))
  {
    return false;
  }

  // 构造积分时间节点（knots）：from、(from,to) 内的所有样本、to
  // 复用 knots_buffer_ 成员避免每 beam 一次堆分配（3 雷达 × ~1000 beam/帧）
  std::vector<int64_t> & knots = knots_buffer_;
  knots.clear();
  knots.reserve(samples_.size() + 2);
  knots.push_back(from_ns);
  for (const auto & sample : samples_) {
    const int64_t sample_ns = sample.stamp.nanoseconds();
    if (sample_ns > from_ns && sample_ns < to_ns) {
      knots.push_back(sample_ns);
    }
  }
  knots.push_back(to_ns);

  // 逐子区间中点法积分
  tf2::Quaternion delta = identityQuaternion();
  for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
    const int64_t start_ns = knots[i];
    const int64_t end_ns = knots[i + 1];
    if (end_ns <= start_ns) {
      continue;  // 跳过零长度区间（如 from==to 或重复样本）
    }

    // 子区间中点的角速度
    const int64_t mid_ns = start_ns + (end_ns - start_ns) / 2;
    tf2::Vector3 angular_velocity;
    if (!angularVelocityAt(rclcpp::Time(mid_ns, RCL_ROS_TIME), &angular_velocity)) {
      return false;
    }

    // 角轴 = ω · dt，转成四元数累乘
    const double dt = static_cast<double>(end_ns - start_ns) * kNsToSec;
    const tf2::Vector3 angle_axis = angular_velocity * dt;
    const double angle = angle_axis.length();
    if (angle <= 1.0e-12) {
      continue;  // 旋转量极小，跳过避免数值噪声
    }

    tf2::Vector3 axis = angle_axis / angle;
    tf2::Quaternion step;
    step.setRotation(axis, angle);
    delta *= step;       // body-frame 乘法：delta = delta ⊗ step
    delta.normalize();
  }

  if (reverse) {
    delta = delta.inverse();  // 反向积分取逆
  }

  *delta_rotation = delta;
  return true;
}

bool ImuBuffer::empty() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.empty();
}

// 取指定时刻的角速度：
//   - 早于首样本：允许 max_extrapolation_sec 外推，否则失败
//   - 晚于末样本：同上
//   - 区间内：线性插值（必成功，无空洞检测）
bool ImuBuffer::angularVelocityAt(
  const rclcpp::Time & stamp,
  tf2::Vector3 * angular_velocity) const
{
  if (angular_velocity == nullptr || samples_.empty()) {
    return false;
  }

  const int64_t target_ns = stamp.nanoseconds();
  const int64_t extrapolation_ns = static_cast<int64_t>(max_extrapolation_sec_ * kSecToNs);
  const int64_t first_ns = samples_.front().stamp.nanoseconds();
  const int64_t last_ns = samples_.back().stamp.nanoseconds();

  // 早于首样本：边界外推
  if (target_ns <= first_ns) {
    if (first_ns - target_ns > extrapolation_ns) {
      return false;
    }
    *angular_velocity = samples_.front().angular_velocity;
    return true;
  }

  // 晚于末样本：边界外推
  if (target_ns >= last_ns) {
    if (target_ns - last_ns > extrapolation_ns) {
      return false;
    }
    *angular_velocity = samples_.back().angular_velocity;
    return true;
  }

  // 区间内：找到包含 target 的相邻样本对，做线性插值
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    const int64_t current_ns = samples_[i].stamp.nanoseconds();
    if (current_ns < target_ns) {
      continue;
    }

    const auto & previous = samples_[i - 1];
    const auto & current = samples_[i];
    const int64_t previous_ns = previous.stamp.nanoseconds();
    const int64_t span_ns = current_ns - previous_ns;
    if (span_ns <= 0) {
      // 重复时间戳，直接取当前值
      *angular_velocity = current.angular_velocity;
      return true;
    }

    // 线性插值 (lerp)
    const double ratio = static_cast<double>(target_ns - previous_ns) /
      static_cast<double>(span_ns);
    *angular_velocity = previous.angular_velocity.lerp(current.angular_velocity, ratio);
    return true;
  }

  return false;
}

// -----------------------------------------------------------------------------
// ThreeLidarImuFusionNode 实现：节点主体
// -----------------------------------------------------------------------------

// 构造函数：初始化默认外参 → 加载参数 → 配置同步器/IMU 缓冲 → 创建订阅/发布/定时器
ThreeLidarImuFusionNode::ThreeLidarImuFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("three_lidar_imu_fusion", options)
{
  // 静态外参初值设为单位变换（无偏移、无旋转），由 loadParameters 用 YAML 覆盖
  static_lidar_transforms_.fill(identityTransform());
  static_imu_transform_ = identityTransform();

  loadParameters();

  // 把加载的参数下发到同步器与 IMU 缓冲
  scan_synchronizer_.setMaxSlop(max_sync_slop_sec_);
  scan_synchronizer_.setQueueSize(scan_queue_size_);
  scan_synchronizer_.setDiagnostics(get_logger(), get_clock());
  imu_buffer_.setRetention(imu_queue_duration_sec_);
  imu_buffer_.setMaxExtrapolation(max_imu_extrapolation_sec_);

  // TF 缓冲与监听器
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 订阅三路雷达与 IMU，统一使用 SensorDataQoS（KEEP_LAST, depth=5, BEST_EFFORT）
  auto scan_qos = rclcpp::SensorDataQoS();
  left_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    left_scan_topic_, scan_qos,
    [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
      handleScan(SensorId::Left, std::move(scan));
    });
  right_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    right_scan_topic_, scan_qos,
    [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
      handleScan(SensorId::Right, std::move(scan));
    });
  back_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    back_scan_topic_, scan_qos,
    [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
      handleScan(SensorId::Back, std::move(scan));
    });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, scan_qos,
    [this](sensor_msgs::msg::Imu::SharedPtr imu) {
      handleImu(std::move(imu));
    });

  // 发布融合后的 LaserScan
  fused_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    output_scan_topic_, rclcpp::SensorDataQoS());
  // 10 ms wall timer 兜底驱动 drainPendingScans（IMU 回调也会驱动）
  pending_scan_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ThreeLidarImuFusionNode::drainPendingScans, this));

  RCLCPP_INFO(
    get_logger(),
    "three_lidar_imu_fusion started: left=%s right=%s back=%s imu=%s output=%s",
    left_scan_topic_.c_str(),
    right_scan_topic_.c_str(),
    back_scan_topic_.c_str(),
    imu_topic_.c_str(),
    output_scan_topic_.c_str());
}

// 声明并加载所有 ROS 参数，并对取值做合法性校验
void ThreeLidarImuFusionNode::loadParameters()
{
  // —— 话题与坐标系 ——
  left_scan_topic_ = declare_parameter<std::string>("left_scan_topic", left_scan_topic_);
  right_scan_topic_ = declare_parameter<std::string>("right_scan_topic", right_scan_topic_);
  back_scan_topic_ = declare_parameter<std::string>("back_scan_topic", back_scan_topic_);
  imu_topic_ = declare_parameter<std::string>("imu_topic", imu_topic_);
  output_scan_topic_ = declare_parameter<std::string>("output_scan_topic", output_scan_topic_);
  target_frame_ = declare_parameter<std::string>("target_frame", target_frame_);

  // —— 行为开关 ——
  use_tf_transforms_ = declare_parameter<bool>("use_tf_transforms", use_tf_transforms_);
  fallback_to_static_transforms_ =
    declare_parameter<bool>("fallback_to_static_transforms", fallback_to_static_transforms_);
  enable_imu_deskew_ = declare_parameter<bool>("enable_imu_deskew", enable_imu_deskew_);
  require_imu_for_publish_ =
    declare_parameter<bool>("require_imu_for_publish", require_imu_for_publish_);
  publish_empty_scan_ = declare_parameter<bool>("publish_empty_scan", publish_empty_scan_);

  // —— 输出栅格与时间参数 ——
  output_angle_min_ = declare_parameter<double>("output_angle_min", output_angle_min_);
  output_angle_max_ = declare_parameter<double>("output_angle_max", output_angle_max_);
  output_angle_increment_ =
    declare_parameter<double>("output_angle_increment", output_angle_increment_);
  output_range_min_ = declare_parameter<double>("output_range_min", output_range_min_);
  output_range_max_ = declare_parameter<double>("output_range_max", output_range_max_);
  default_scan_time_ = declare_parameter<double>("default_scan_time", default_scan_time_);
  max_sync_slop_sec_ = declare_parameter<double>("max_sync_slop_sec", max_sync_slop_sec_);
  imu_queue_duration_sec_ =
    declare_parameter<double>("imu_queue_duration_sec", imu_queue_duration_sec_);
  max_imu_extrapolation_sec_ =
    declare_parameter<double>("max_imu_extrapolation_sec", max_imu_extrapolation_sec_);

  // —— 队列长度（int 转 size_t，至少 1）——
  const int scan_queue_size =
    declare_parameter<int>("scan_queue_size", static_cast<int>(scan_queue_size_));
  scan_queue_size_ = static_cast<std::size_t>(std::max(1, scan_queue_size));

  const int max_pending_scan_sets =
    declare_parameter<int>("max_pending_scan_sets", static_cast<int>(max_pending_scan_sets_));
  max_pending_scan_sets_ = static_cast<std::size_t>(std::max(1, max_pending_scan_sets));

  // —— 合法性校验：非法值重置为安全默认并告警 ——
  if (!std::isfinite(output_angle_increment_) || output_angle_increment_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid output_angle_increment, reset to ~0.25 deg (pi/720)");
    output_angle_increment_ = kPi / 720.0;
  }
  if (output_angle_max_ <= output_angle_min_) {
    RCLCPP_WARN(get_logger(), "Invalid output angle range, reset to [-pi, pi]");
    output_angle_min_ = -kPi;
    output_angle_max_ = kPi;
  }
  if (output_range_max_ <= output_range_min_) {
    RCLCPP_WARN(get_logger(), "Invalid output range, reset to [0.05, 10.0]");
    output_range_min_ = 0.05;
    output_range_max_ = 10.0;
  }
  if (default_scan_time_ <= 0.0) {
    default_scan_time_ = 0.1;
  }

  // —— 加载四个静态外参（TF 不可用时的兜底）——
  static_lidar_transforms_[sensorIndex(SensorId::Left)] = loadStaticTransform("left_lidar");
  static_lidar_transforms_[sensorIndex(SensorId::Right)] = loadStaticTransform("right_lidar");
  static_lidar_transforms_[sensorIndex(SensorId::Back)] = loadStaticTransform("back_lidar");
  static_imu_transform_ = loadStaticTransform("imu");
}

// 从参数前缀（如 "left_lidar"）读取 x/y/z/roll/pitch/yaw，构造 tf2::Transform
tf2::Transform ThreeLidarImuFusionNode::loadStaticTransform(const std::string & prefix)
{
  const double x = declare_parameter<double>(prefix + ".x", 0.0);
  const double y = declare_parameter<double>(prefix + ".y", 0.0);
  const double z = declare_parameter<double>(prefix + ".z", 0.0);
  const double roll = declare_parameter<double>(prefix + ".roll", 0.0);
  const double pitch = declare_parameter<double>(prefix + ".pitch", 0.0);
  const double yaw = declare_parameter<double>(prefix + ".yaw", 0.0);

  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);  // 按 RPY 顺序构造旋转
  rotation.normalize();

  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(x, y, z));
  transform.setRotation(rotation);
  return transform;
}

// 雷达回调：入队 → 尝试同步 → 若得到三元组则压入 pending 队列
void ThreeLidarImuFusionNode::handleScan(
  SensorId sensor_id,
  sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::optional<TimedScanSet> scan_set;
  {
    // scan_mutex_ 保护 ScanSynchronizer（仅在此处访问）
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_synchronizer_.push(sensor_id, std::move(scan));
    scan_set = scan_synchronizer_.tryPop();
  }

  if (scan_set.has_value()) {
    enqueueSynchronizedScans(*scan_set);
  }
}

// IMU 回调：取时间戳 → 旋到 target_frame → 入 IMU 缓冲 → 触发 drain
void ThreeLidarImuFusionNode::handleImu(sensor_msgs::msg::Imu::SharedPtr imu)
{
  if (!imu) {
    return;
  }

  rclcpp::Time stamp(imu->header.stamp);
  // 驱动未填时间戳时回退 now()，并 WARN_ONCE 提示检查驱动时钟
  if (stamp.nanoseconds() == 0) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "IMU header.stamp is 0; falling back to node now(). Check the IMU driver clock.");
    stamp = now();
  }

  imu_buffer_.push(stamp, imuAngularVelocityInTarget(*imu));
  drainPendingScans();  // 新 IMU 数据可能让 pending 组获得覆盖
}

// 把已同步的三元组压入 pending 队列；超出上限则丢最旧并告警。
// 注意：此处不触发 drain——新入组不改变 IMU 覆盖，由 IMU 回调 + 10ms 定时器驱动。
void ThreeLidarImuFusionNode::enqueueSynchronizedScans(const TimedScanSet & scans)
{
  {
    std::lock_guard<std::mutex> lock(pending_scan_mutex_);
    pending_scan_sets_.push_back(scans);
    while (pending_scan_sets_.size() > max_pending_scan_sets_) {
      pending_scan_sets_.pop_front();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Dropping oldest synchronized lidar group because pending queue is full");
    }
  }
  // Drain is intentionally NOT triggered here: a newly enqueued group does not
  // change IMU coverage, and the IMU callback + the 10 ms timer already drive
  // draining. This avoids redundant work on the scan callback path.
}

// 从 pending 队列取出队首一组处理。
// 每次最多处理 1 组，避免单线程执行器被长时间占用；后续组由下次 IMU/定时器触发。
void ThreeLidarImuFusionNode::drainPendingScans()
{
  // Process at most one pending group per call so we never monopolize the
  // single-threaded executor; subsequent groups are handled on the next IMU
  // sample or timer tick.
  std::optional<TimedScanSet> scans;
  {
    std::lock_guard<std::mutex> lock(pending_scan_mutex_);
    if (pending_scan_sets_.empty()) {
      return;
    }
    // IMU 覆盖门控：require_imu_for_publish=true 时，覆盖不足则保留该组等待
    if (!imuCoversScans(pending_scan_sets_.front())) {
      return;
    }
    scans = pending_scan_sets_.front();
    pending_scan_sets_.pop_front();
  }

  processSynchronizedScans(*scans);
}

// 判断 IMU 是否覆盖该同步组所需时间区间。
// 仅当 enable_imu_deskew && require_imu_for_publish 时生效；否则直接放行。
bool ThreeLidarImuFusionNode::imuCoversScans(const TimedScanSet & scans) const
{
  if (!enable_imu_deskew_ || !require_imu_for_publish_) {
    return true;
  }

  // 覆盖区间 = [最早 scan 起点, 最晚 scan 终点]，即所有 beam 积分区间的并集
  return imu_buffer_.hasCoverage(earliestScanStartTime(scans), fusionReferenceTime(scans));
}

// 处理一组同步 scan：逐雷达去畸变累加点 → 栅格化 → 发布
void ThreeLidarImuFusionNode::processSynchronizedScans(const TimedScanSet & scans)
{
  const rclcpp::Time fusion_time = fusionReferenceTime(scans);
  std::vector<FusedPoint> points;
  points.reserve(
    scans.left->ranges.size() + scans.right->ranges.size() +
    scans.back->ranges.size());

  // 单传感器容错：TF 失败只跳过该雷达，IMU 覆盖丢失才丢整组
  std::size_t skipped_sensors = 0;
  const SensorId sensor_ids[3] = {SensorId::Left, SensorId::Right, SensorId::Back};
  const sensor_msgs::msg::LaserScan::SharedPtr scans_arr[3] = {
    scans.left, scans.right, scans.back,
  };

  for (std::size_t i = 0; i < 3; ++i) {
    const AppendResult result =
      appendDeskewedScanPoints(*scans_arr[i], sensor_ids[i], fusion_time, &points);
    if (result == AppendResult::kImuCoverageLost) {
      // require_imu_for_publish_ is on and IMU coverage disappeared mid-scan:
      // drop the whole group as requested by the operator.
      return;  // IMU 覆盖中段丢失且要求严格模式：丢弃整组
    }
    if (result == AppendResult::kSensorUnavailable) {
      ++skipped_sensors;  // 该雷达 TF 缺失，跳过
    }
  }

  // 三雷达全部 TF 缺失：无可用数据，丢整帧
  if (skipped_sensors == 3) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "All three lidar transforms unavailable, dropping synchronized group");
    return;
  }

  // 无有效点且不允许发空帧：丢整帧
  if (points.empty() && !publish_empty_scan_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Synchronized lidar scans produced no valid fused points");
    return;
  }

  // 输出 scan_time 取三帧 scanDuration 的最大值
  const double output_scan_time = std::max(
    {
      scanDuration(*scans.left),
      scanDuration(*scans.right),
      scanDuration(*scans.back),
    });
  fused_scan_pub_->publish(
    buildOutputScan(points, fusion_time, output_scan_time, outputAngleIncrement(scans)));
}

// 对单帧 scan 逐 beam 去畸变并累加点到 points。
// 返回 AppendResult：
//   kSensorUnavailable - 该雷达 TF 缺失，跳过该传感器
//   kImuCoverageLost   - require_imu_for_publish 且 beam 中段 IMU 缺失，丢整组
//   kOk                - 正常完成（部分 beam 可能未去畸变）
AppendResult ThreeLidarImuFusionNode::appendDeskewedScanPoints(
  const sensor_msgs::msg::LaserScan & scan,
  SensorId sensor_id,
  const rclcpp::Time & fusion_time,
  std::vector<FusedPoint> * points)
{
  if (points == nullptr) {
    return AppendResult::kSensorUnavailable;
  }

  // 1) 取该雷达到 target_frame 的变换（TF 优先，静态兜底）
  tf2::Transform lidar_to_target;
  if (!lidarTransform(sensor_id, scan.header.frame_id, &lidar_to_target)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "No transform available for %s scan, skipping this sensor only",
      sensorName(sensor_id));
    return AppendResult::kSensorUnavailable;  // 只跳过该传感器
  }

  const rclcpp::Time start_time = scanStartTime(scan);
  const double time_increment = pointTimeIncrement(scan);
  std::size_t imu_miss_count = 0;  // IMU 覆盖缺失的 beam 数
  std::size_t valid_count = 0;     // 有效 beam 数（含未去畸变）

  // 2) 逐 beam 处理
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!rangeIsValid(range, scan)) {
      continue;  // 无效距离（NaN/Inf/<=0/超出范围）
    }

    // beam 角度 = angle_min + i * angle_increment
    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    // 雷达系下的点（2D，z=0）
    const tf2::Vector3 lidar_point{
      static_cast<double>(range) * std::cos(angle),
      static_cast<double>(range) * std::sin(angle),
      0.0};

    // 变换到 target_frame（刚性安装下为常量变换）
    tf2::Vector3 target_point = lidar_to_target * lidar_point;

    // 3) IMU 旋转去畸变
    if (enable_imu_deskew_) {
      // 该 beam 的采样时刻 = scan 起点 + i * time_increment
      const rclcpp::Time point_time =
        start_time + rclcpp::Duration::from_seconds(static_cast<double>(i) * time_increment);
      tf2::Quaternion delta_rotation;
      if (imu_buffer_.integrateRotation(point_time, fusion_time, &delta_rotation)) {
        // Δq = R(t_i→t_f)；施加 Δq⁻¹ 把点回旋到融合时刻的 target_frame
        tf2::Transform deskew_transform(delta_rotation.inverse(), tf2::Vector3(0.0, 0.0, 0.0));
        target_point = deskew_transform * target_point;
      } else {
        // IMU 覆盖不足
        ++imu_miss_count;
        if (require_imu_for_publish_) {
          // 严格模式：丢整组
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "IMU coverage is not enough for deskew, dropping synchronized group");
          return AppendResult::kImuCoverageLost;
        }
        // 宽松模式：该 beam 保留但不去畸变，继续
      }
    }

    // 4) 累加点（带强度）
    FusedPoint point;
    point.x = target_point.x();
    point.y = target_point.y();
    point.z = target_point.z();
    if (i < scan.intensities.size() && std::isfinite(scan.intensities[i])) {
      point.intensity = scan.intensities[i];
    }
    points->push_back(point);
    ++valid_count;
  }

  // 5) 统计告警：有 beam 未去畸变（仍保留在输出中）
  if (imu_miss_count > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s scan: %zu/%zu valid points kept un-deskewed due to missing IMU coverage",
      sensorName(sensor_id), imu_miss_count, valid_count);
  }

  return AppendResult::kOk;
}

// 把融合后的点云栅格化为单帧 LaserScan：
//   - 按 [angle_min, angle_max] 以 angle_increment 等距分 bin
//   - 每个 bin 保留最近的点及其强度
sensor_msgs::msg::LaserScan ThreeLidarImuFusionNode::buildOutputScan(
  const std::vector<FusedPoint> & points,
  const rclcpp::Time & stamp,
  double scan_time,
  double angle_increment) const
{
  sensor_msgs::msg::LaserScan output;
  output.header.stamp = stamp;
  output.header.frame_id = target_frame_;
  output.angle_min = static_cast<float>(output_angle_min_);
  output.angle_max = static_cast<float>(output_angle_max_);
  output.angle_increment = static_cast<float>(angle_increment);
  output.time_increment = 0.0F;  // 输出已栅格化，无逐 beam 时间增量
  output.scan_time = static_cast<float>(std::max(0.0, scan_time));
  output.range_min = static_cast<float>(output_range_min_);
  output.range_max = static_cast<float>(output_range_max_);

  // 计算 bin 数：floor((max-min)/inc) + 1
  const auto bin_count = static_cast<std::size_t>(
    std::floor((output_angle_max_ - output_angle_min_) / angle_increment)) + 1;
  output.ranges.assign(bin_count, std::numeric_limits<float>::infinity());  // 默认 inf（无回波）
  output.intensities.assign(bin_count, 0.0F);

  // 逐点投射到对应 bin，保留最近点
  for (const auto & point : points) {
    const double range = std::hypot(point.x, point.y);
    if (!std::isfinite(range) || range < output_range_min_ || range > output_range_max_) {
      continue;  // 距离无效或超出范围
    }

    const double angle = std::atan2(point.y, point.x);
    if (angle < output_angle_min_ || angle > output_angle_max_) {
      continue;  // 角度超出输出区间
    }

    // 四舍五入到最近 bin
    const auto index = static_cast<std::size_t>(
      std::floor((angle - output_angle_min_) / angle_increment + 0.5));
    if (index >= output.ranges.size()) {
      continue;  // 越界保护
    }

    // 仅当比现有值更近时覆盖（最近点优先），强度随之更新
    if (range < output.ranges[index]) {
      output.ranges[index] = static_cast<float>(range);
      output.intensities[index] = point.intensity;
    }
  }

  return output;
}

// 取 scan 起始时间；header.stamp=0 时回退 now() 并 WARN_ONCE
rclcpp::Time ThreeLidarImuFusionNode::scanStartTime(const sensor_msgs::msg::LaserScan & scan) const
{
  rclcpp::Time stamp(scan.header.stamp);
  if (stamp.nanoseconds() == 0) {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "LaserScan header.stamp is 0; falling back to node now(). Check the lidar driver clock.");
    return now();
  }
  return stamp;
}

// scan 结束时间 = 起始时间 + scanDuration
rclcpp::Time ThreeLidarImuFusionNode::scanEndTime(const sensor_msgs::msg::LaserScan & scan) const
{
  return scanStartTime(scan) + rclcpp::Duration::from_seconds(scanDuration(scan));
}

// 三帧 scan 中最早的起始时间（IMU 覆盖区间左端点）
rclcpp::Time ThreeLidarImuFusionNode::earliestScanStartTime(const TimedScanSet & scans) const
{
  const auto left_start = scanStartTime(*scans.left);
  const auto right_start = scanStartTime(*scans.right);
  const auto back_start = scanStartTime(*scans.back);

  rclcpp::Time earliest = left_start;
  if (right_start.nanoseconds() < earliest.nanoseconds()) {
    earliest = right_start;
  }
  if (back_start.nanoseconds() < earliest.nanoseconds()) {
    earliest = back_start;
  }
  return earliest;
}

// 融合参考时间 = 三帧 scan 结束时间的最大值。
// 所有 beam 都回旋到该时刻，保证融合帧表达在统一时刻的 target_frame 下。
rclcpp::Time ThreeLidarImuFusionNode::fusionReferenceTime(const TimedScanSet & scans) const
{
  const auto left_end = scanEndTime(*scans.left);
  const auto right_end = scanEndTime(*scans.right);
  const auto back_end = scanEndTime(*scans.back);

  rclcpp::Time fusion_time = left_end;
  if (right_end.nanoseconds() > fusion_time.nanoseconds()) {
    fusion_time = right_end;
  }
  if (back_end.nanoseconds() > fusion_time.nanoseconds()) {
    fusion_time = back_end;
  }
  return fusion_time;
}

// 计算 scan 的持续时长（秒）。
// 优先用 scan.scan_time，但做上限裁剪防止异常值把 fusionReferenceTime 拉到未来；
// 其次用 time_increment * (size-1)；最后回退 default_scan_time_。
double ThreeLidarImuFusionNode::scanDuration(const sensor_msgs::msg::LaserScan & scan) const
{
  // Cap scan_time to protect fusionReferenceTime() from bogus driver values that
  // would otherwise push the fusion reference instant far into the future and
  // starve the IMU coverage check.
  if (std::isfinite(scan.scan_time) && scan.scan_time > 0.0F) {
    const double cap = std::max(default_scan_time_ * 2.0, 0.2);
    return std::min(static_cast<double>(scan.scan_time), cap);
  }

  // 用 time_increment 估算
  if (std::isfinite(scan.time_increment) && scan.time_increment > 0.0F &&
    scan.ranges.size() > 1)
  {
    return static_cast<double>(scan.time_increment) *
           static_cast<double>(scan.ranges.size() - 1);
  }

  return default_scan_time_;
}

// 计算逐 beam 的时间增量（秒）。
// 优先用 scan.time_increment；否则用 scanDuration/(size-1) 估算；单点返回 0。
double ThreeLidarImuFusionNode::pointTimeIncrement(const sensor_msgs::msg::LaserScan & scan) const
{
  if (std::isfinite(scan.time_increment) && scan.time_increment > 0.0F) {
    return scan.time_increment;
  }

  if (scan.ranges.size() <= 1) {
    return 0.0;
  }

  return scanDuration(scan) / static_cast<double>(scan.ranges.size() - 1);
}

// 决定输出 LaserScan 的角度增量。
// output_angle_increment_ 有效（>0 且有限）时直接用；否则固定回退 π/720（~0.25°）。
// 不再从输入 scan 推导，避免三雷达分辨率不同时 bin 数被异常放大。
double ThreeLidarImuFusionNode::outputAngleIncrement(const TimedScanSet & /*scans*/) const
{
  if (output_angle_increment_ > 0.0 && std::isfinite(output_angle_increment_)) {
    return output_angle_increment_;
  }

  // Predictable fixed fallback (~0.25 deg). We intentionally do NOT derive the
  // output resolution from the input scans, since mixing three lidars with
  // different angular resolutions would silently inflate the output bin count.
  return kPi / 720.0;
}

// 判定 beam 距离是否有效。
// 拒绝 NaN/Inf/<=0（range=0 常表示"无回波"，防止画成原点伪障碍）；
// 有效区间取 output_range_* 与 scan 自身 range_* 的交集。
bool ThreeLidarImuFusionNode::rangeIsValid(
  float range,
  const sensor_msgs::msg::LaserScan & scan) const
{
  // range == 0.0 is commonly used by drivers to encode "no return"; reject it
  // explicitly so a misconfigured output_range_min of 0 cannot paint phantom
  // obstacles at the origin.
  if (!std::isfinite(range) || range <= 0.0F) {
    return false;
  }

  const double min_range = std::max(output_range_min_, static_cast<double>(scan.range_min));
  double max_range = output_range_max_;
  if (std::isfinite(scan.range_max) && scan.range_max > 0.0F) {
    max_range = std::min(max_range, static_cast<double>(scan.range_max));
  }

  return range >= min_range && range <= max_range;
}

// 取某雷达到 target_frame 的变换。
// 优先 TF（TimePointZero，依赖刚性安装）；失败且允许兜底时用静态外参。
bool ThreeLidarImuFusionNode::lidarTransform(
  SensorId sensor_id,
  const std::string & source_frame,
  tf2::Transform * transform)
{
  if (transform == nullptr) {
    return false;
  }

  if (use_tf_transforms_ && !source_frame.empty()) {
    // NOTE: we look up the latest available transform (TimePointZero) rather
    // than the transform at scan time. This relies on the lidars being rigidly
    // mounted on target_frame (the assumption stated in the README). For a
    // lidar on a moving joint the transform should instead be queried at the
    // scan stamp, which requires a non-zero timeout and a time-aware TF source.
    // 注意：这里取最新可用变换而非 scan 时刻变换，依赖雷达刚性安装在 target_frame。
    // 若雷达装在活动关节上，应改为按 scan 时间戳查询 TF。
    try {
      const auto transform_msg =
        tf_buffer_->lookupTransform(target_frame_, source_frame, tf2::TimePointZero);
      tf2::fromMsg(transform_msg.transform, *transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF lookup failed from %s to %s: %s. Static fallback is %s",
        source_frame.c_str(),
        target_frame_.c_str(),
        ex.what(),
        fallback_to_static_transforms_ ? "enabled" : "disabled");
    }
  }

  if (!fallback_to_static_transforms_) {
    return false;
  }

  // 静态外参兜底
  *transform = static_lidar_transforms_[sensorIndex(sensor_id)];
  return true;
}

// 把 IMU 角速度旋到 target_frame。
// 缓存 IMU→target 的旋转矩阵，约每 100 条 IMU（~1 s）刷新一次 TF，
// 避免 100 Hz 高频查询；TF 瞬时失败时沿用缓存。
tf2::Vector3 ThreeLidarImuFusionNode::imuAngularVelocityInTarget(
  const sensor_msgs::msg::Imu & imu)
{
  const tf2::Vector3 imu_angular_velocity{
    imu.angular_velocity.x,
    imu.angular_velocity.y,
    imu.angular_velocity.z};

  // Refresh the cached IMU->target basis periodically instead of doing a TF
  // lookup on every 100 Hz IMU sample. The extrinsics are constant for a
  // rigidly mounted IMU, so a ~1 s refresh cadence is more than enough.
  // 缓存刷新周期：100 条 IMU ≈ 1 s（100 Hz）
  constexpr std::size_t kBasisRefreshInterval = 100;
  const bool cache_valid = imu_basis_cached_;
  const bool cache_expired = (++imu_basis_refresh_counter_ >= kBasisRefreshInterval);

  if (use_tf_transforms_ && !imu.header.frame_id.empty() && (!cache_valid || cache_expired)) {
    try {
      const auto transform_msg =
        tf_buffer_->lookupTransform(target_frame_, imu.header.frame_id, tf2::TimePointZero);
      tf2::Transform imu_to_target;
      tf2::fromMsg(transform_msg.transform, imu_to_target);
      cached_imu_basis_ = imu_to_target.getBasis();  // 只取旋转（角速度是矢量，与平移无关）
      imu_basis_cached_ = true;
      imu_basis_refresh_counter_ = 0;
    } catch (const tf2::TransformException & ex) {
      if (!cache_valid) {
        // 缓存无效且 TF 失败：告警并用静态兜底
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF lookup failed from IMU frame %s to %s: %s. Using static IMU fallback",
          imu.header.frame_id.c_str(),
          target_frame_.c_str(),
          ex.what());
      }
      // If the cache is valid but this refresh failed (transient TF hiccup),
      // keep using the stale cached basis silently.
      // 缓存有效但本次刷新失败（TF 瞬时抖动）：静默沿用旧缓存
    }
  }

  if (imu_basis_cached_) {
    return cached_imu_basis_ * imu_angular_velocity;
  }

  // 静态外参兜底（只用旋转矩阵）
  return static_imu_transform_.getBasis() * imu_angular_velocity;
}

// SensorId → 可读名称（用于日志）
const char * ThreeLidarImuFusionNode::sensorName(SensorId sensor_id)
{
  switch (sensor_id) {
    case SensorId::Left:
      return "left";
    case SensorId::Right:
      return "right";
    case SensorId::Back:
      return "back";
  }
  return "unknown";
}

// SensorId → 数组下标（Left=0, Right=1, Back=2），用于索引 static_lidar_transforms_
std::size_t ThreeLidarImuFusionNode::sensorIndex(SensorId sensor_id)
{
  return static_cast<std::size_t>(sensor_id);
}

}  // namespace lidar_imu_fusion
