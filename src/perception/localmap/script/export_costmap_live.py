#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Subscribe to nav2_msgs/Costmap and save each frame as PNG (for live replay export)."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from costmap_render import nearest_yaw, render_costmap


class CostmapLiveExporter(Node):
    def __init__(
        self,
        topic: str,
        output_dir: Path,
        flip_y: bool,
        idle_timeout_sec: float,
        prefix: str,
        rotate_base_link: bool,
        odom_yaw_series: list[tuple[float, float]],
    ) -> None:
        super().__init__("costmap_live_exporter")
        self.output_dir = output_dir.resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.flip_y = flip_y
        self.idle_timeout_sec = idle_timeout_sec
        self.prefix = prefix
        self.rotate_base_link = rotate_base_link
        self.odom_yaw_series = odom_yaw_series
        self.live_yaws: list[tuple[float, float]] = []
        self.count = 0
        self.last_msg_time = time.monotonic()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Costmap, topic, self.on_costmap, qos)
        if rotate_base_link and not odom_yaw_series:
            odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Odometry, "/odom/fused", self.on_odom, odom_qos)
        self.create_timer(0.5, self.check_idle)
        self.get_logger().info(f"Saving {topic} -> {self.output_dir}")

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.live_yaws.append((stamp, yaw))

    def lookup_yaw(self, stamp_sec: float) -> float | None:
        if self.odom_yaw_series:
            return nearest_yaw(stamp_sec, self.odom_yaw_series)
        if self.live_yaws:
            return nearest_yaw(stamp_sec, self.live_yaws)
        return None

    def on_costmap(self, msg: Costmap) -> None:
        meta = msg.metadata
        cost = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(meta.size_y, meta.size_x)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = self.lookup_yaw(stamp) if self.rotate_base_link else None
        rgb = render_costmap(cost, flip_y=self.flip_y, yaw_rad=yaw)
        filename = f"{self.prefix}_{msg.header.stamp.sec:010d}_{msg.header.stamp.nanosec:09d}.png"
        Image.fromarray(rgb).save(self.output_dir / filename)
        self.count += 1
        self.last_msg_time = time.monotonic()

        if self.count % 20 == 0:
            self.get_logger().info(f"Saved {self.count} frames")

    def check_idle(self) -> None:
        if self.count > 0 and (time.monotonic() - self.last_msg_time) >= self.idle_timeout_sec:
            self.get_logger().info(
                f"Idle for {self.idle_timeout_sec:.0f}s, stopping with {self.count} frames"
            )
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("-t", "--topic", default="/localmap/occupymap_raw")
    parser.add_argument("--idle-timeout", type=float, default=3.0)
    parser.add_argument("--no-flip-y", action="store_true")
    parser.add_argument("--prefix", default="")
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--bag-dir", type=Path, default=None, help="Preload odom yaw from bag")
    parser.add_argument(
        "--rotate-base-link",
        action="store_true",
        default=False,
    )
    parser.add_argument("--no-rotate-base-link", action="store_false", dest="rotate_base_link")
    args, ros_args = parser.parse_known_args()

    prefix = args.prefix or args.topic.strip("/").replace("/", "_")
    if args.use_sim_time:
        ros_args.extend(["--ros-args", "-p", "use_sim_time:=true"])

    odom_series: list[tuple[float, float]] = []
    if args.rotate_base_link and args.bag_dir is not None:
        from costmap_render import load_odom_yaw_series

        odom_series = load_odom_yaw_series(str(args.bag_dir.resolve()))

    rclpy.init(args=ros_args)
    node = CostmapLiveExporter(
        args.topic,
        args.output,
        flip_y=not args.no_flip_y,
        idle_timeout_sec=args.idle_timeout,
        prefix=prefix,
        rotate_base_link=args.rotate_base_link,
        odom_yaw_series=odom_series,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        count = node.count
        output_dir = node.output_dir
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"Exported {count} frames to {output_dir}")
    return 0 if count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
