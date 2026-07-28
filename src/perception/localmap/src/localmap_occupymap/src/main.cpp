// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include "localmap_occupymap/occupymap_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localmap_occupymap::OccupymapNode>());
  rclcpp::shutdown();
  return 0;
}
