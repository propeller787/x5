#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Export /local_costmap/costmap_raw frames from a rosbag2 directory to PNG images."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from costmap_render import load_odom_yaw_series, nearest_yaw, render_costmap


def export_bag(
    bag_dir: Path,
    output_dir: Path,
    topic: str,
    flip_y: bool,
    rotate_base_link: bool,
) -> int:
    bag_dir = bag_dir.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    yaw_series = load_odom_yaw_series(str(bag_dir)) if rotate_base_link else []

    storage_options = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics = {info.name: info.type for info in reader.get_all_topics_and_types()}
    if topic not in topics:
        available = ", ".join(sorted(topics))
        raise RuntimeError(f"Topic {topic!r} not found in bag. Available: {available}")

    msg_type = get_message(topics[topic])
    count = 0

    while reader.has_next():
        name, data, _timestamp = reader.read_next()
        if name != topic:
            continue

        msg = deserialize_message(data, msg_type)
        meta = msg.metadata
        cost = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(meta.size_y, meta.size_x)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = nearest_yaw(stamp, yaw_series) if rotate_base_link else None
        rgb = render_costmap(cost, flip_y=flip_y, yaw_rad=yaw)
        filename = f"costmap_raw_{msg.header.stamp.sec:010d}_{msg.header.stamp.nanosec:09d}.png"
        Image.fromarray(rgb).save(output_dir / filename)
        count += 1

    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag_dir",
        type=Path,
        help="rosbag2 directory (e.g. ../bags/case1)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("output/costmap_raw"),
        help="Output directory for PNG files",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default="/local_costmap/costmap_raw",
        help="Costmap topic to export",
    )
    parser.add_argument(
        "--no-flip-y",
        action="store_true",
        help="Do not flip rows (RViz Map display uses image-style Y-up after flip)",
    )
    parser.add_argument(
        "--rotate-base-link",
        action="store_true",
        default=False,
        help="Rotate PNG so base_link forward points up (odom map shows as tilted polygon)",
    )
    parser.add_argument(
        "--no-rotate-base-link",
        action="store_false",
        dest="rotate_base_link",
        help="Keep odom-aligned orientation",
    )
    args = parser.parse_args()

    bag_name = args.bag_dir.resolve().name
    output_dir = args.output / bag_name

    try:
        count = export_bag(
            args.bag_dir,
            output_dir,
            args.topic,
            flip_y=not args.no_flip_y,
            rotate_base_link=args.rotate_base_link,
        )
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Exported {count} frames to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
