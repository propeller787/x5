#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Shared costmap PNG rendering helpers."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from PIL import Image

from costmap_colormap import cost_to_rgb


def render_costmap(
    cost: np.ndarray,
    *,
    flip_y: bool = True,
    yaw_rad: Optional[float] = None,
) -> np.ndarray:
    """Render raw cost grid to RGB; optionally rotate so base_link forward points up."""
    grid = np.flipud(cost) if flip_y else cost
    rgb = cost_to_rgb(grid)
    if yaw_rad is not None:
        rgb = rotate_base_link_up(rgb, yaw_rad)
    return rgb


def rotate_base_link_up(rgb: np.ndarray, yaw_rad: float) -> np.ndarray:
    """Rotate odom-aligned image so vehicle forward (+x in base_link) points up."""
    # After flip_y: image x -> odom x, image up -> odom y.
    # Forward in image plane = (cos(yaw), -sin(yaw)); rotate to (0, -1) in PIL coords.
    angle_deg = math.degrees(-yaw_rad)
    return np.array(
        Image.fromarray(rgb).rotate(angle_deg, resample=Image.NEAREST, expand=False)
    )


def load_odom_yaw_series(bag_dir: str) -> list[tuple[float, float]]:
    """Return sorted (stamp_sec, yaw_rad) from /odom/fused in a rosbag2 directory."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id="sqlite3"), ConverterOptions("", ""))
    topics = {info.name: info.type for info in reader.get_all_topics_and_types()}
    odom_type = get_message(topics["/odom/fused"])

    series: list[tuple[float, float]] = []
    while reader.has_next():
        name, data, _timestamp = reader.read_next()
        if name != "/odom/fused":
            continue
        msg = deserialize_message(data, odom_type)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        series.append((stamp, yaw))
    series.sort()
    return series


def nearest_yaw(stamp: float, series: list[tuple[float, float]]) -> Optional[float]:
    if not series:
        return None
    _stamp, yaw = min(series, key=lambda item: abs(item[0] - stamp))
    return yaw
