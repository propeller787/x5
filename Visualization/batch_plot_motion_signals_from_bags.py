#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Batch plot motion-signal overview figures from ROS2 bag directories.

For each bag under the given root folders, this script extracts:
    1. /handle/control     -> handle_x
    2. /cmd_vel            -> cmd_x, cmd_wz
    3. /nav/control        -> nav_x, nav_wz
    4. /odom/fused         -> odom_vx, odom_wz

It then creates one overview figure similar to plot_motion_signals.py and saves:
    <root_dir>/<bag_name>_motion_signals_plot.png
    <root_dir>/<bag_name>_motion_signals_plot.pdf
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from matplotlib.ticker import FuncFormatter, MultipleLocator
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPIC_SPECS = {
    "/handle/control": ("geometry_msgs/msg/Twist", "handle"),
    "/cmd_vel": ("geometry_msgs/msg/Twist", "cmd"),
    "/nav/control": ("geometry_msgs/msg/Twist", "nav"),
    "/odom/fused": ("nav_msgs/msg/Odometry", "odom"),
}


@dataclass
class SignalSeries:
    t: List[float] = field(default_factory=list)
    a: List[float] = field(default_factory=list)
    b: List[float] = field(default_factory=list)


RESAMPLE_DT_SEC = 0.02


def setup_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.linewidth": 0.9,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "legend.frameon": True,
            "figure.dpi": 120,
            "savefig.dpi": 300,
        }
    )


def get_bag_dirs(root_dir: Path) -> List[Path]:
    return sorted(
        path
        for path in root_dir.rglob("metadata.yaml")
        if path.parent.joinpath(f"{path.parent.name}_0.db3").exists()
    )


def get_symmetric_ylim(values: Iterable[float], pad_ratio: float = 0.12) -> Tuple[float, float]:
    arr = np.asarray(list(values), dtype=float)
    arr = arr[np.isfinite(arr)]

    if arr.size == 0:
        return -1.0, 1.0

    max_abs = max(abs(arr.min()), abs(arr.max()))
    if max_abs < 1e-6:
        max_abs = 1.0

    max_abs *= 1.0 + pad_ratio
    return -max_abs, max_abs


def save_figure(fig: plt.Figure, out_base: Path) -> None:
    png_path = out_base.with_suffix(".png")
    pdf_path = out_base.with_suffix(".pdf")
    fig.savefig(png_path, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {png_path}")
    print(f"Saved: {pdf_path}")


def format_time_tick(value: float, _pos: float) -> str:
    if abs(value - round(value)) < 1e-9:
        return f"{int(round(value))}"
    return f"{value:.1f}"


def resample_last_per_bin(series: SignalSeries, dt_sec: float = RESAMPLE_DT_SEC) -> SignalSeries:
    if not series.t:
        return SignalSeries()

    bins: Dict[int, Tuple[float, float, float]] = {}
    for t_val, a_val, b_val in zip(series.t, series.a, series.b):
        bin_idx = int(round(t_val / dt_sec))
        bins[bin_idx] = (t_val, a_val, b_val)

    resampled = SignalSeries()
    for bin_idx in sorted(bins):
        t_val, a_val, b_val = bins[bin_idx]
        resampled.t.append(bin_idx * dt_sec)
        resampled.a.append(a_val)
        resampled.b.append(b_val)

    return resampled


def read_bag_signals(bag_dir: Path) -> Dict[str, SignalSeries]:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(topic_type)
        for topic, topic_type in topic_types.items()
        if topic in TOPIC_SPECS
    }

    if len(message_types) < len(TOPIC_SPECS):
        missing = sorted(set(TOPIC_SPECS) - set(message_types))
        raise RuntimeError(f"Missing required topics in {bag_dir.name}: {missing}")

    first_timestamp_ns = None
    signals = {
        "handle": SignalSeries(),
        "cmd": SignalSeries(),
        "nav": SignalSeries(),
        "odom": SignalSeries(),
    }

    while reader.has_next():
        topic, raw_data, timestamp_ns = reader.read_next()
        if topic not in TOPIC_SPECS:
            continue

        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns

        t_sec = (timestamp_ns - first_timestamp_ns) / 1e9
        msg = deserialize_message(raw_data, message_types[topic])
        signal_key = TOPIC_SPECS[topic][1]
        series = signals[signal_key]

        if topic == "/odom/fused":
            assert isinstance(msg, Odometry)
            series.t.append(t_sec)
            series.a.append(float(msg.twist.twist.linear.x))
            series.b.append(float(msg.twist.twist.angular.z))
            continue

        assert isinstance(msg, Twist)
        series.t.append(t_sec)
        series.a.append(float(msg.linear.x))
        series.b.append(float(msg.angular.z))

    return {name: resample_last_per_bin(series) for name, series in signals.items()}


def plot_overview(
    signals: Dict[str, SignalSeries],
    title_name: str,
    out_base: Path,
    x_tick_sec: float,
) -> None:
    fig, axes = plt.subplots(
        4,
        1,
        figsize=(13, 9),
        sharex=True,
        constrained_layout=True,
    )

    colors = {
        "handle_x": "#2E7D32",
        "cmd_x": "#1565C0",
        "cmd_wz": "#64B5F6",
        "nav_x": "#E65100",
        "nav_wz": "#FFB74D",
        "odom_vx": "#6A1B9A",
        "odom_wz": "#BA68C8",
    }

    handle_ylim = get_symmetric_ylim(signals["handle"].a)
    motion_ylim = get_symmetric_ylim(
        list(signals["cmd"].a)
        + list(signals["cmd"].b)
        + list(signals["nav"].a)
        + list(signals["nav"].b)
        + list(signals["odom"].a)
        + list(signals["odom"].b)
    )

    axes[0].plot(
        signals["handle"].t,
        signals["handle"].a,
        label="handle_x (/handle/control linear.x)",
        color=colors["handle_x"],
        linewidth=1.8,
    )
    axes[0].set_ylabel("Handle\n[m/s]")
    axes[0].set_title("Joystick / Handle Input")
    axes[0].set_ylim(handle_ylim)
    axes[0].legend(loc="upper right")

    axes[1].plot(
        signals["cmd"].t,
        signals["cmd"].a,
        label="cmd_x (/cmd_vel linear.x)",
        color=colors["cmd_x"],
        linewidth=1.6,
    )
    axes[1].plot(
        signals["cmd"].t,
        signals["cmd"].b,
        label="cmd_wz (/cmd_vel angular.z)",
        color=colors["cmd_wz"],
        linewidth=1.4,
    )
    axes[1].set_ylabel("Cmd\n[m/s, rad/s]")
    axes[1].set_title("MPPI Output /cmd_vel")
    axes[1].set_ylim(motion_ylim)
    axes[1].legend(loc="upper right")

    axes[2].plot(
        signals["nav"].t,
        signals["nav"].a,
        label="nav_x (/nav/control linear.x)",
        color=colors["nav_x"],
        linewidth=1.6,
    )
    axes[2].plot(
        signals["nav"].t,
        signals["nav"].b,
        label="nav_wz (/nav/control angular.z)",
        color=colors["nav_wz"],
        linewidth=1.4,
    )
    axes[2].set_ylabel("Nav\n[m/s, rad/s]")
    axes[2].set_title("Safety Gate Output /nav/control")
    axes[2].set_ylim(motion_ylim)
    axes[2].legend(loc="upper right")

    axes[3].plot(
        signals["odom"].t,
        signals["odom"].a,
        label="odom_vx (/odom/fused twist.linear.x)",
        color=colors["odom_vx"],
        linewidth=1.6,
    )
    axes[3].plot(
        signals["odom"].t,
        signals["odom"].b,
        label="odom_wz (/odom/fused twist.angular.z)",
        color=colors["odom_wz"],
        linewidth=1.4,
    )
    axes[3].set_ylabel("Odom\n[m/s, rad/s]")
    axes[3].set_title("Vehicle Feedback /odom/fused")
    axes[3].set_xlabel("Time t [s]")
    axes[3].set_ylim(motion_ylim)
    axes[3].legend(loc="upper right")

    max_t = max(
        max(series.t) if series.t else 0.0
        for series in signals.values()
    )

    for ax in axes:
        ax.axhline(0.0, linewidth=0.8, color="black", alpha=0.35)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.set_xlim(0.0, max_t)
        ax.xaxis.set_major_locator(MultipleLocator(x_tick_sec))
        ax.xaxis.set_major_formatter(FuncFormatter(format_time_tick))

    fig.suptitle(
        f"Motion Signal Comparison: {title_name}",
        fontsize=14,
        fontweight="bold",
    )
    save_figure(fig, out_base)


def process_root(root_dir: Path, x_tick_sec: float) -> None:
    bag_metadata_files = get_bag_dirs(root_dir)
    if not bag_metadata_files:
        raise RuntimeError(f"No bag directories found under {root_dir}")

    print(f"Found {len(bag_metadata_files)} bag(s) under {root_dir}")

    for metadata_file in bag_metadata_files:
        bag_dir = metadata_file.parent
        bag_name = bag_dir.name
        out_base = root_dir / f"{bag_name}_motion_signals_plot"
        print(f"Processing {bag_dir}")
        signals = read_bag_signals(bag_dir)
        plot_overview(signals, bag_name, out_base, x_tick_sec)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Batch plot motion-signal overview figures from ROS2 bag directories."
    )
    parser.add_argument("root_dirs", nargs="+", help="Root directories that contain ROS2 bag folders.")
    parser.add_argument(
        "--x-tick-sec",
        type=float,
        default=1.0,
        help="Major tick spacing on the time axis in seconds. Default: 1.0",
    )
    return parser.parse_args(argv[1:])


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    if args.x_tick_sec <= 0.0:
        raise ValueError("--x-tick-sec must be > 0")

    setup_style()
    for root_arg in args.root_dirs:
        process_root(Path(root_arg).resolve(), args.x_tick_sec)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))


#   python3.10 /home/wzq/wzq/rosbag/batch_plot_motion_signals_from_bags.py \
#   /home/wzq/wzq/rosbag/0729mppi_stack_cyclic1/mppi_stack_cyclic/bag_20260729_101824 \
#   --x-tick-sec 1
