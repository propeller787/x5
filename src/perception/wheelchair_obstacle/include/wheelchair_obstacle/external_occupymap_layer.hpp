#ifndef WHEELCHAIR_OBSTACLE__EXTERNAL_OCCUPYMAP_LAYER_HPP_
#define WHEELCHAIR_OBSTACLE__EXTERNAL_OCCUPYMAP_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wheelchair_obstacle
{

// Bridges localmap_occupymap (/localmap/occupymap_raw) into Nav2 local_costmap so
// MPPI can keep using in-process Costmap2DROS without forking nav2_costmap_2d.
class ExternalOccupymapLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void incomingCostmap(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void incomingUpdate(const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  bool lookupExternalCost(double wx, double wy, unsigned char & cost) const;
  static unsigned char occupancyToCost(int8_t value);

  std::string costmap_topic_;
  std::string costmap_update_topic_;

  mutable std::mutex mutex_;
  bool has_map_{false};
  std::string map_frame_;
  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.05};
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  std::vector<unsigned char> data_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_sub_;
};

}  // namespace wheelchair_obstacle

#endif  // WHEELCHAIR_OBSTACLE__EXTERNAL_OCCUPYMAP_LAYER_HPP_
