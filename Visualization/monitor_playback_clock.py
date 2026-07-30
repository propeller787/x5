#!/usr/bin/env python3

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class PlaybackClockMonitor(Node):
    def __init__(self, start_offset_sec: float) -> None:
        super().__init__("playback_clock_monitor")
        self.start_offset_sec = float(start_offset_sec)
        self.first_clock_sec: float | None = None
        self.last_printed_whole_sec: int | None = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, "/clock", self.on_clock, qos)

    def on_clock(self, msg: Clock) -> None:
        current_clock_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.first_clock_sec is None:
            self.first_clock_sec = current_clock_sec

        elapsed_sec = max(0.0, current_clock_sec - self.first_clock_sec)
        bag_time_sec = self.start_offset_sec + elapsed_sec
        whole_sec = int(bag_time_sec)

        if whole_sec != self.last_printed_whole_sec:
            print(f"[CLOCK] 当前回放到 bag 第 {bag_time_sec:6.2f} s", flush=True)
            self.last_printed_whole_sec = whole_sec


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor rosbag playback /clock and print current bag second.")
    parser.add_argument("--start-offset", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = PlaybackClockMonitor(args.start_offset)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
