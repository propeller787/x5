#include "wheelchair_obstacle/external_occupymap_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace wheelchair_obstacle
{

void ExternalOccupymapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ExternalOccupymapLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("costmap_topic", rclcpp::ParameterValue("/localmap/occupymap_raw"));
  declareParameter(
    "costmap_update_topic", rclcpp::ParameterValue("/localmap/occupymap_updates"));

  node->get_parameter(name_ + ".costmap_topic", costmap_topic_);
  node->get_parameter(name_ + ".costmap_update_topic", costmap_update_topic_);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic_, qos,
    std::bind(&ExternalOccupymapLayer::incomingCostmap, this, std::placeholders::_1));
  update_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
    costmap_update_topic_, qos,
    std::bind(&ExternalOccupymapLayer::incomingUpdate, this, std::placeholders::_1));

  current_ = true;
  default_value_ = nav2_costmap_2d::FREE_SPACE;

  RCLCPP_INFO(
    rclcpp::get_logger(name_),
    "ExternalOccupymapLayer: subscribing to %s and %s",
    costmap_topic_.c_str(), costmap_update_topic_.c_str());
}

void ExternalOccupymapLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_map_ = false;
  data_.clear();
}

void ExternalOccupymapLayer::incomingCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  if (!msg || msg->data.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  map_frame_ = msg->header.frame_id;
  origin_x_ = msg->metadata.origin.position.x;
  origin_y_ = msg->metadata.origin.position.y;
  resolution_ = msg->metadata.resolution;
  size_x_ = msg->metadata.size_x;
  size_y_ = msg->metadata.size_y;
  data_ = msg->data;
  has_map_ = true;
}

unsigned char ExternalOccupymapLayer::occupancyToCost(int8_t value)
{
  if (value == 0) {
    return nav2_costmap_2d::FREE_SPACE;
  }
  if (value < 0) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  if (value >= 100) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  if (value >= 99) {
    return nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
  return static_cast<unsigned char>(1 + (251.0 * (value - 1)) / 97.0);
}

void ExternalOccupymapLayer::incomingUpdate(
  const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  if (!update || update->data.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_map_ || update->width == 0 || update->height == 0) {
    return;
  }

  const unsigned int x_end = update->x + update->width;
  const unsigned int y_end = update->y + update->height;
  if (x_end > size_x_ || y_end > size_y_) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(name_), *node_.lock()->get_clock(), 5000,
      "ExternalOccupymapLayer: ignoring out-of-bounds update");
    return;
  }

  size_t idx = 0;
  for (unsigned int y = update->y; y < y_end; ++y) {
    for (unsigned int x = update->x; x < x_end; ++x) {
      data_[y * size_x_ + x] = occupancyToCost(update->data[idx++]);
    }
  }
}

bool ExternalOccupymapLayer::lookupExternalCost(
  double wx, double wy, unsigned char & cost) const
{
  if (!has_map_ || data_.empty() || resolution_ <= 0.0) {
    return false;
  }

  const int mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  const int my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  if (mx < 0 || my < 0 ||
    static_cast<unsigned int>(mx) >= size_x_ ||
    static_cast<unsigned int>(my) >= size_y_)
  {
    return false;
  }

  cost = data_[static_cast<size_t>(my * size_x_ + mx)];
  return true;
}

void ExternalOccupymapLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;

  if (!has_map_) {
    return;
  }

  // Refresh the full rolling window each cycle (localmap already fuses + inflates).
  *min_x = std::min(*min_x, origin_x_);
  *min_y = std::min(*min_y, origin_y_);
  *max_x = std::max(*max_x, origin_x_ + size_x_ * resolution_);
  *max_y = std::max(*max_y, origin_y_ + size_y_ * resolution_);
}

void ExternalOccupymapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_map_) {
    return;
  }

  const std::string & global_frame = layered_costmap_->getGlobalFrameID();
  if (!map_frame_.empty() && map_frame_ != global_frame) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(name_), *node_.lock()->get_clock(), 5000,
      "ExternalOccupymapLayer: frame mismatch (map=%s global=%s)",
      map_frame_.c_str(), global_frame.c_str());
  }

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);

      unsigned char cost = nav2_costmap_2d::FREE_SPACE;
      if (!lookupExternalCost(wx, wy, cost)) {
        continue;
      }
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      master_grid.setCost(i, j, cost);
    }
  }
}

}  // namespace wheelchair_obstacle

PLUGINLIB_EXPORT_CLASS(
  wheelchair_obstacle::ExternalOccupymapLayer, nav2_costmap_2d::Layer)
