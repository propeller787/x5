#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Relay nav2_msgs/Costmap to nav_msgs/OccupancyGrid for RViz Map display."""

from __future__ import annotations

import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from costmap_colormap import cost_to_occupancy


class CostmapRawViz(Node):
    def __init__(self) -> None:
        super().__init__("costmap_raw_viz")
        self.declare_parameter("input_topic", "/local_costmap/costmap_raw")
        self.declare_parameter("output_topic", "/local_costmap/costmap_raw/viz")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_ = self.create_publisher(OccupancyGrid, output_topic, qos)
        self.sub_ = self.create_subscription(Costmap, input_topic, self.on_costmap, qos)

        self.get_logger().info(f"Relay {input_topic} -> {output_topic}")

    def on_costmap(self, msg: Costmap) -> None:
        meta = msg.metadata
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.info.resolution = meta.resolution
        grid.info.width = meta.size_x
        grid.info.height = meta.size_y
        grid.info.origin = meta.origin

        import numpy as np

        cost = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(meta.size_y, meta.size_x)
        occ = cost_to_occupancy(cost)
        grid.data = occ.flatten().astype(np.int8).tolist()
        self.pub_.publish(grid)


def main() -> None:
    rclpy.init()
    node = CostmapRawViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
