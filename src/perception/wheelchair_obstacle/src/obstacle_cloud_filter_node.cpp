#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wheelchair_obstacle/obstacle_cloud_filter_core.hpp"

using wheelchair_obstacle::FilterPoint;
using wheelchair_obstacle::FilterStats;
using wheelchair_obstacle::IsolationConfig;
using wheelchair_obstacle::ObstacleCloudFilterCore;
using wheelchair_obstacle::PolygonMask;
using wheelchair_obstacle::RangeConfig;
using wheelchair_obstacle::RoiConfig;
using wheelchair_obstacle::VoxelConfig;

namespace
{
using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(stamp.nanosec);
}

}  // namespace

class ObstacleCloudFilterNode : public rclcpp::Node
{
public:
  ObstacleCloudFilterNode()
  : Node("obstacle_cloud_filter"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();
    configureCore();

    output_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());

    if (publish_markers_) {
      self_mask_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        self_mask_marker_topic_, rclcpp::QoS(1).transient_local());
      roi_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        roi_marker_topic_, rclcpp::QoS(1).transient_local());
      marker_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(marker_period_sec_ * 1000.0)),
        std::bind(&ObstacleCloudFilterNode::publishDebugMarkers, this));
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleCloudFilterNode::onCloud, this, std::placeholders::_1));

    narrow_gate_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_pass_mode_topic", "/mppi/narrow_gate_pass_mode");
    narrow_gate_side_pass_mode_topic_ = declare_parameter<std::string>(
      "narrow_gate_side_pass_mode_topic", "/mppi/narrow_gate_side_pass_mode");
    narrow_gate_pass_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      narrow_gate_pass_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&ObstacleCloudFilterNode::onNarrowGatePassMode, this, std::placeholders::_1));
    narrow_gate_side_pass_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
      narrow_gate_side_pass_mode_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(
        &ObstacleCloudFilterNode::onNarrowGateSidePassMode, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "obstacle_cloud_filter started: in=%s out=%s frame=%s pass_mode=%s side_pass=%s",
      input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), target_frame_.c_str(),
      narrow_gate_pass_mode_topic_.c_str(), narrow_gate_side_pass_mode_topic_.c_str());
  }

private:
  void onNarrowGatePassMode(const std_msgs::msg::Bool::SharedPtr msg)
  {
    narrow_gate_pass_active_ = msg->data;
    syncPassModeFilterProfile();
  }

  void onNarrowGateSidePassMode(const std_msgs::msg::Bool::SharedPtr msg)
  {
    narrow_gate_side_pass_active_ = msg->data;
    syncPassModeFilterProfile();
  }

  void syncPassModeFilterProfile()
  {
    const bool pass_mode = narrow_gate_pass_active_ || narrow_gate_side_pass_active_;
    if (pass_mode == pass_mode_filter_active_) {
      return;
    }
    pass_mode_filter_active_ = pass_mode;
    if (pass_mode) {
      range_ = normal_range_;
      range_.range_max = pass_range_max_m_;
      roi_.x_min = pass_body_rear_x_m_ - pass_obstacle_margin_m_;
      roi_.x_max = pass_body_front_x_m_ + pass_obstacle_margin_m_;
      roi_.y_min = -(pass_body_half_width_m_ + pass_obstacle_margin_m_);
      roi_.y_max = pass_body_half_width_m_ + pass_obstacle_margin_m_;
      isolation_ = normal_isolation_;
      isolation_.grid_resolution = pass_resolution_m_;
      voxel_ = normal_voxel_;
      voxel_.voxel_size_xy = pass_resolution_m_;
      self_mask_enabled_ = false;
      RCLCPP_INFO(
        get_logger(),
        "pass_mode filter ON (forward=%s side_pass=%s): res=%.2fm roi x=[%.2f,%.2f] "
        "y=[%.2f,%.2f] self_mask=off",
        narrow_gate_pass_active_ ? "yes" : "no",
        narrow_gate_side_pass_active_ ? "yes" : "no",
        pass_resolution_m_, roi_.x_min, roi_.x_max, roi_.y_min, roi_.y_max);
    } else {
      range_ = normal_range_;
      roi_ = normal_roi_;
      isolation_ = normal_isolation_;
      voxel_ = normal_voxel_;
      self_mask_enabled_ = normal_self_mask_enabled_;
      RCLCPP_INFO(
        get_logger(), "pass_mode filter restored (self_mask=%s)",
        self_mask_enabled_ ? "on" : "off");
    }
    configureCore();
  }
  void loadParameters()
  {
    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/perception/lidar_fusion/points");
    output_cloud_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/perception/obstacle_points");
    self_mask_marker_topic_ = declare_parameter<std::string>(
      "self_mask_marker_topic", "/perception/obstacle_points/debug/self_mask");
    roi_marker_topic_ = declare_parameter<std::string>(
      "roi_marker_topic", "/perception/obstacle_points/debug/roi");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    max_input_age_sec_ = declare_parameter<double>("max_input_age_sec", 0.30);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);
    publish_empty_cloud_ = declare_parameter<bool>("publish_empty_cloud", true);

    {
      RangeConfig cfg;
      cfg.range_min = declare_parameter<double>("range_min", 0.05);
      cfg.range_max = declare_parameter<double>("range_max", 3.00);
      cfg.z_min = declare_parameter<double>("z_min", -0.30);
      cfg.z_max = declare_parameter<double>("z_max", 0.80);
      range_ = cfg;
    }

    {
      RoiConfig cfg;
      cfg.x_min = declare_parameter<double>("roi.x_min", -1.20);
      cfg.x_max = declare_parameter<double>("roi.x_max", 2.50);
      cfg.y_min = declare_parameter<double>("roi.y_min", -1.50);
      cfg.y_max = declare_parameter<double>("roi.y_max", 1.50);
      roi_ = cfg;
    }

    self_mask_enabled_ = declare_parameter<bool>("self_mask_enabled", true);

    const auto mask_names = declareOptionalArray<std::vector<std::string>>(
      "self_mask_names", std::vector<std::string>{});
    const auto mask_flat = declareOptionalArray<std::vector<double>>(
      "self_mask_polygons_flat", std::vector<double>{});
    const auto mask_sizes = declareOptionalArray<std::vector<int64_t>>(
      "self_mask_sizes", std::vector<int64_t>{});

    custom_masks_ = buildMasksFromFlat(mask_names, mask_flat, mask_sizes);
    if (!custom_masks_.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "self_mask override parameters invalid or incomplete, falling back to built-in defaults");
    }

    isolation_enabled_ = declare_parameter<bool>("isolation_filter_enabled", true);
    {
      IsolationConfig cfg;
      cfg.neighbor_radius = declare_parameter<double>("neighbor_radius", 0.08);
      cfg.min_neighbors = static_cast<int>(declare_parameter<int64_t>("min_neighbors", 2));
      cfg.grid_resolution = declare_parameter<double>("isolation_grid_resolution", 0.05);
      cfg.skip_isolation_range = declare_parameter<double>("skip_isolation_range", 0.50);
      isolation_ = cfg;
    }

    voxel_enabled_ = declare_parameter<bool>("voxel_downsample_enabled", true);
    {
      VoxelConfig cfg;
      cfg.voxel_size_xy = declare_parameter<double>("voxel_size_xy", 0.04);
      cfg.keep_nearest = declare_parameter<bool>("keep_nearest_in_voxel", true);
      voxel_ = cfg;
    }

    publish_markers_ = declare_parameter<bool>("debug.publish_markers", true);
    marker_period_sec_ = declare_parameter<double>("debug.marker_period_sec", 1.0);

    normal_range_ = range_;
    normal_roi_ = roi_;
    normal_isolation_ = isolation_;
    normal_voxel_ = voxel_;
    normal_self_mask_enabled_ = self_mask_enabled_;

    pass_resolution_m_ = declare_parameter<double>("gate_pass_resolution_m", 0.02);
    pass_obstacle_margin_m_ = declare_parameter<double>("gate_pass_obstacle_margin_m", 1.00);
    pass_body_front_x_m_ = declare_parameter<double>("gate_pass_body_front_x_m", 0.935);
    pass_body_rear_x_m_ = declare_parameter<double>("gate_pass_body_rear_x_m", -0.20);
    pass_body_half_width_m_ = declare_parameter<double>("gate_pass_body_half_width_m", 0.34);
    const double fwd = pass_body_front_x_m_ + pass_obstacle_margin_m_;
    const double back = -pass_body_rear_x_m_ + pass_obstacle_margin_m_;
    const double side = pass_body_half_width_m_ + pass_obstacle_margin_m_;
    pass_range_max_m_ = std::hypot(std::max(fwd, back), side) + 0.10;
  }

  // Declare an array parameter that may legitimately be empty. An empty "[]" in
  // a YAML params file has no element type, so rclcpp loads it as an untyped
  // (NOT_SET) value and declare_parameter<T> throws
  // InvalidParameterValueException. Catch that and fall back to the typed
  // default instead of crashing the node.
  template <typename T>
  T declareOptionalArray(const std::string & name, const T & default_value)
  {
    try {
      return declare_parameter<T>(name, default_value);
    } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter '%s' is empty/untyped in YAML; using built-in default (%s)",
        name.c_str(), e.what());
      return default_value;
    }
  }

  static std::optional<std::vector<PolygonMask>> buildMasksFromFlat(
    const std::vector<std::string> & names,
    const std::vector<double> & flat,
    const std::vector<int64_t> & sizes)
  {
    if (names.empty() && flat.empty() && sizes.empty()) {
      return std::nullopt;
    }
    if (names.size() != sizes.size()) {
      return std::nullopt;
    }
    std::size_t consumed = 0;
    std::vector<PolygonMask> masks;
    masks.reserve(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      const std::size_t pts = static_cast<std::size_t>(sizes[i]);
      if (pts < 3) {
        return std::nullopt;
      }
      if (consumed + pts * 2 > flat.size()) {
        return std::nullopt;
      }
      PolygonMask mask;
      mask.name = names[i];
      mask.polygon.reserve(pts);
      for (std::size_t k = 0; k < pts; ++k) {
        const double x = flat[consumed + k * 2];
        const double y = flat[consumed + k * 2 + 1];
        mask.polygon.emplace_back(x, y);
      }
      consumed += pts * 2;
      masks.push_back(std::move(mask));
    }
    return masks;
  }

  void configureCore()
  {
    core_.set_range_config(range_);
    core_.set_roi_config(roi_);
    core_.set_isolation_config(isolation_);
    core_.set_voxel_config(voxel_);
    core_.set_self_mask_enabled(self_mask_enabled_);
    core_.set_isolation_enabled(isolation_enabled_);
    core_.set_voxel_enabled(voxel_enabled_);
    core_.set_self_masks(
      custom_masks_.has_value() ? *custom_masks_ : ObstacleCloudFilterCore::default_self_masks());
  }

  bool isStale(const builtin_interfaces::msg::Time & stamp) const
  {
    // A zero stamp carries no usable time; treat it as fresh so a producer that
    // leaves header.stamp unset (or uses a different clock source) does not make
    // us drop every cloud.
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return false;
    }
    const int64_t stamp_ns = stampToNs(stamp);
    const int64_t now_ns = static_cast<int64_t>(this->now().nanoseconds());
    const double age_sec =
      std::llabs(now_ns - stamp_ns) / static_cast<double>(kNanosecondsPerSecond);
    return age_sec > max_input_age_sec_;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (isStale(msg->header.stamp)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Drop stale input cloud (age > %.3fs)", max_input_age_sec_);
      if (publish_empty_cloud_) {
        publishEmpty(msg->header.stamp);
      }
      return;
    }

    auto points = parseAndTransformToBaseLink(*msg);
    if (!points.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Drop input cloud: TF_MISSING for %s -> %s",
        msg->header.frame_id.c_str(), target_frame_.c_str());
      if (publish_empty_cloud_) {
        publishEmpty(msg->header.stamp);
      }
      return;
    }

    FilterStats stats;
    auto filtered = core_.filter(*points, stats);

    publishObstacleCloud(filtered, msg->header.stamp);

    RCLCPP_DEBUG(
      get_logger(),
      "filter stats: in=%zu finite_range=%zu roi=%zu mask=%zu iso=%zu voxel=%zu",
      stats.input, stats.after_finite_range, stats.after_roi,
      stats.after_self_mask, stats.after_isolation, stats.after_voxel);
  }

  std::optional<std::vector<FilterPoint>> parseAndTransformToBaseLink(
    const sensor_msgs::msg::PointCloud2 & msg)
  {
    PointCloud pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    const std::string & source_frame = msg.header.frame_id;
    std::optional<tf2::Transform> transform;
    if (source_frame.empty() || source_frame == target_frame_) {
      tf2::Transform identity;
      identity.setIdentity();
      transform = identity;
    } else {
      transform = lookupTransformToTarget(source_frame, msg.header.stamp);
      if (!transform.has_value()) {
        return std::nullopt;
      }
    }

    std::vector<FilterPoint> out;
    out.reserve(pcl_cloud.points.size());
    for (const auto & p : pcl_cloud.points) {
      const tf2::Vector3 v(static_cast<double>(p.x), static_cast<double>(p.y),
        static_cast<double>(p.z));
      const tf2::Vector3 t = transform.value() * v;
      FilterPoint fp;
      fp.x = static_cast<float>(t.x());
      fp.y = static_cast<float>(t.y());
      fp.z = static_cast<float>(t.z());
      fp.intensity = p.intensity;
      out.push_back(fp);
    }
    return out;
  }

  std::optional<tf2::Transform> lookupTransformToTarget(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tf_buffer_->lookupTransform(
        target_frame_, source_frame, rclcpp::Time(stamp),
        tf2::durationFromSec(transform_timeout_sec_));
    } catch (const tf2::TransformException &) {
      try {
        transform_msg = tf_buffer_->lookupTransform(
          target_frame_, source_frame, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_sec_));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "TF lookup failed: %s", ex.what());
        return std::nullopt;
      }
    }
    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);
    return transform;
  }

  void publishObstacleCloud(
    const std::vector<FilterPoint> & points,
    const builtin_interfaces::msg::Time & stamp)
  {
    PointCloud out_cloud;
    out_cloud.header.frame_id = target_frame_;
    out_cloud.header.stamp = pcl_conversions::toPCL(stamp);
    out_cloud.is_dense = true;
    out_cloud.points.reserve(points.size());
    for (const auto & p : points) {
      PointType pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = p.intensity;
      out_cloud.points.push_back(pt);
    }
    out_cloud.width = static_cast<uint32_t>(out_cloud.points.size());
    out_cloud.height = 1;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(out_cloud, out_msg);
    out_msg.header.stamp = stamp;
    out_msg.header.frame_id = target_frame_;
    output_pub_->publish(out_msg);
  }

  void publishEmpty(const builtin_interfaces::msg::Time & stamp)
  {
    PointCloud out_cloud;
    out_cloud.header.frame_id = target_frame_;
    out_cloud.header.stamp = pcl_conversions::toPCL(stamp);
    out_cloud.is_dense = true;
    out_cloud.width = 0;
    out_cloud.height = 1;
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(out_cloud, out_msg);
    out_msg.header.stamp = stamp;
    out_msg.header.frame_id = target_frame_;
    output_pub_->publish(out_msg);
  }

  void publishDebugMarkers()
  {
    if (self_mask_marker_pub_) {
      visualization_msgs::msg::MarkerArray markers;
      const auto & masks = core_.self_masks();
      for (std::size_t i = 0; i < masks.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = target_frame_;
        m.header.stamp = this->now();
        m.ns = "self_mask";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.02;
        m.color.r = 1.0F;
        m.color.g = 0.2F;
        m.color.b = 0.2F;
        m.color.a = 0.8F;
        m.lifetime = rclcpp::Duration::from_seconds(marker_period_sec_ * 2.0);
        for (const auto & pt : masks[i].polygon) {
          geometry_msgs::msg::Point gp;
          gp.x = pt.first;
          gp.y = pt.second;
          gp.z = 0.0;
          m.points.push_back(gp);
        }
        if (!m.points.empty()) {
          m.points.push_back(m.points.front());
        }
        markers.markers.push_back(m);
      }
      self_mask_marker_pub_->publish(markers);
    }

    if (roi_marker_pub_) {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = target_frame_;
      m.header.stamp = this->now();
      m.ns = "roi";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.02;
      m.color.r = 0.2F;
      m.color.g = 1.0F;
      m.color.b = 0.2F;
      m.color.a = 0.6F;
      m.lifetime = rclcpp::Duration::from_seconds(marker_period_sec_ * 2.0);
      const auto & r = roi_;
      std::vector<std::pair<double, double>> rect = {
        {r.x_min, r.y_min}, {r.x_max, r.y_min}, {r.x_max, r.y_max}, {r.x_min, r.y_max}};
      for (const auto & pt : rect) {
        geometry_msgs::msg::Point gp;
        gp.x = pt.first;
        gp.y = pt.second;
        gp.z = 0.0;
        m.points.push_back(gp);
      }
      m.points.push_back(m.points.front());
      markers.markers.push_back(m);
      roi_marker_pub_->publish(markers);
    }
  }

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string self_mask_marker_topic_;
  std::string roi_marker_topic_;
  std::string target_frame_;

  double max_input_age_sec_ = 0.30;
  double transform_timeout_sec_ = 0.05;
  bool publish_empty_cloud_ = true;

  RangeConfig range_;
  RoiConfig roi_;
  IsolationConfig isolation_;
  VoxelConfig voxel_;
  RangeConfig normal_range_;
  RoiConfig normal_roi_;
  IsolationConfig normal_isolation_;
  VoxelConfig normal_voxel_;
  bool self_mask_enabled_ = true;
  bool normal_self_mask_enabled_ = true;
  bool isolation_enabled_ = true;
  bool voxel_enabled_ = true;
  std::optional<std::vector<PolygonMask>> custom_masks_;

  bool publish_markers_ = true;
  double marker_period_sec_ = 1.0;

  std::string narrow_gate_pass_mode_topic_;
  std::string narrow_gate_side_pass_mode_topic_;
  bool narrow_gate_pass_active_{false};
  bool narrow_gate_side_pass_active_{false};
  bool pass_mode_filter_active_{false};
  double pass_resolution_m_{0.02};
  double pass_obstacle_margin_m_{1.00};
  double pass_body_front_x_m_{0.935};
  double pass_body_rear_x_m_{-0.20};
  double pass_body_half_width_m_{0.34};
  double pass_range_max_m_{2.50};

  ObstacleCloudFilterCore core_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr narrow_gate_pass_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr narrow_gate_side_pass_mode_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr self_mask_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr roi_marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
