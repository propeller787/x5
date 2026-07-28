#!/usr/bin/env python3
"""Compare MPPI stack bag metrics: baseline vs replay after code/config changes.

Offline mode (baseline only):
  python3 scripts/compare_mppi_bag_replay.py \\
    --baseline ../../../rosbag/bag_20260728_114751

After live replay (see replay_mppi_bag.sh):
  python3 scripts/compare_mppi_bag_replay.py \\
    --baseline ../../../rosbag/bag_20260728_114751 \\
    --replay   bags/replay_mppi_114751 \\
    --output   reports/mppi_114751_compare.md
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


HANDLE_LINEAR_THRESHOLD = 0.02
HANDLE_ANGULAR_THRESHOLD = 0.02
NAV_OUTPUT_THRESHOLD = 0.02
WHEEL_STOPPED_THRESHOLD = 0.05
MATCH_TOLERANCE_S = 0.15


@dataclass
class TwistSample:
    t: float
    linear_x: float
    angular_z: float


@dataclass
class ModeSample:
    t: float
    mode: int  # 0=PASSTHROUGH, 1=AVOID


@dataclass
class BagSeries:
    bag_name: str
    handle: list[TwistSample] = field(default_factory=list)
    cmd_vel: list[TwistSample] = field(default_factory=list)
    nav: list[TwistSample] = field(default_factory=list)
    wheel: list[TwistSample] = field(default_factory=list)
    modes: list[ModeSample] = field(default_factory=list)


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def load_bag_series(bag_path: Path) -> BagSeries:
    reader = open_reader(bag_path)
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_types: dict[str, Any] = {}

    wanted_topics = {
        "/handle/control",
        "/cmd_vel",
        "/nav/control",
        "/wheel/twist",
        "/cmd_vel_safety_gate_node/mode",
    }

    def msg_type_for(topic: str):
        if topic not in type_map:
            return None
        if topic not in msg_types:
            msg_types[topic] = get_message(type_map[topic])
        return msg_types[topic]

    series = BagSeries(bag_name=bag_path.name)
    t0: float | None = None

    def stamp_to_t(stamp_ns: int) -> float:
        nonlocal t0
        t = stamp_ns / 1e9
        if t0 is None:
            t0 = t
        return t - t0

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted_topics:
            continue
        msg_type = msg_type_for(topic)
        if msg_type is None:
            continue
        t = stamp_to_t(stamp)
        if topic == "/handle/control":
            msg = deserialize_message(data, msg_type)
            series.handle.append(TwistSample(t, msg.linear.x, msg.angular.z))
        elif topic == "/cmd_vel":
            msg = deserialize_message(data, msg_type)
            series.cmd_vel.append(TwistSample(t, msg.linear.x, msg.angular.z))
        elif topic == "/nav/control":
            msg = deserialize_message(data, msg_type)
            series.nav.append(TwistSample(t, msg.linear.x, msg.angular.z))
        elif topic == "/wheel/twist":
            msg = deserialize_message(data, msg_type)
            series.wheel.append(TwistSample(t, msg.twist.linear.x, msg.twist.angular.z))
        elif topic == "/cmd_vel_safety_gate_node/mode":
            msg = deserialize_message(data, msg_type)
            series.modes.append(ModeSample(t, int(msg.data)))

    return series


def nearest_sample(samples: list[TwistSample], t: float) -> TwistSample | None:
    if not samples:
        return None
    lo, hi = 0, len(samples) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if samples[mid].t < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(samples[lo - 1].t - t) < abs(samples[lo].t - t):
        return samples[lo - 1]
    return samples[lo]


def compute_derivatives(samples: list[TwistSample]) -> dict[str, float]:
    if len(samples) < 2:
        return {
            "max_linear_step": 0.0,
            "max_angular_step": 0.0,
            "max_abs_decel": 0.0,
            "max_abs_accel": 0.0,
            "max_abs_jerk": 0.0,
            "worst_linear_step_t": 0.0,
        }

    max_linear_step = 0.0
    max_angular_step = 0.0
    max_abs_decel = 0.0
    max_abs_accel = 0.0
    max_abs_jerk = 0.0
    worst_linear_step_t = 0.0
    prev_accel = None

    for before, after in zip(samples, samples[1:]):
        dt = after.t - before.t
        if dt <= 1e-6:
            continue
        d_linear = after.linear_x - before.linear_x
        d_angular = after.angular_z - before.angular_z
        accel = d_linear / dt
        jerk = None if prev_accel is None else (accel - prev_accel) / dt
        prev_accel = accel

        if abs(d_linear) > max_linear_step:
            max_linear_step = abs(d_linear)
            worst_linear_step_t = after.t
        max_angular_step = max(max_angular_step, abs(d_angular))
        if accel < 0.0:
            max_abs_decel = max(max_abs_decel, abs(accel))
        else:
            max_abs_accel = max(max_abs_accel, accel)
        if jerk is not None:
            max_abs_jerk = max(max_abs_jerk, abs(jerk))

    return {
        "max_linear_step": max_linear_step,
        "max_angular_step": max_angular_step,
        "max_abs_decel": max_abs_decel,
        "max_abs_accel": max_abs_accel,
        "max_abs_jerk": max_abs_jerk,
        "worst_linear_step_t": worst_linear_step_t,
    }


def first_stop_time(
    wheel: list[TwistSample],
    stop_threshold: float = WHEEL_STOPPED_THRESHOLD,
    motion_threshold: float = 0.10,
) -> float | None:
    saw_motion = False
    for sample in wheel:
        moving = (
            abs(sample.linear_x) > motion_threshold
            or abs(sample.angular_z) > motion_threshold
        )
        stopped = (
            abs(sample.linear_x) <= stop_threshold
            and abs(sample.angular_z) <= stop_threshold
        )
        if moving:
            saw_motion = True
        if saw_motion and stopped:
            return sample.t
    return None


def classify_handle(sample: TwistSample) -> str | None:
    lx = sample.linear_x
    az = sample.angular_z
    if lx > HANDLE_LINEAR_THRESHOLD:
        return "forward"
    if lx < -HANDLE_LINEAR_THRESHOLD:
        return "backward"
    if az > HANDLE_ANGULAR_THRESHOLD:
        return "turn_left"
    if az < -HANDLE_ANGULAR_THRESHOLD:
        return "turn_right"
    return None


def nav_responds(sample: TwistSample, direction: str) -> bool:
    if direction == "forward":
        return sample.linear_x > NAV_OUTPUT_THRESHOLD
    if direction == "backward":
        return sample.linear_x < -NAV_OUTPUT_THRESHOLD
    if direction == "turn_left":
        return sample.angular_z > NAV_OUTPUT_THRESHOLD
    if direction == "turn_right":
        return sample.angular_z < -NAV_OUTPUT_THRESHOLD
    return False


def post_stop_block_stats(
    handle: list[TwistSample],
    nav: list[TwistSample],
    stop_t: float | None,
) -> dict[str, Any]:
    if stop_t is None or not handle or not nav:
        return {
            "post_stop_window_s": 0.0,
            "direction_counts": {},
            "blocked_rates": {},
            "overall_blocked_rate": None,
        }

    direction_counts: dict[str, int] = {}
    blocked_counts: dict[str, int] = {}

    for hs in handle:
        if hs.t < stop_t:
            continue
        direction = classify_handle(hs)
        if direction is None:
            continue
        ns = nearest_sample(nav, hs.t)
        if ns is None:
            continue
        direction_counts[direction] = direction_counts.get(direction, 0) + 1
        if not nav_responds(ns, direction):
            blocked_counts[direction] = blocked_counts.get(direction, 0) + 1

    blocked_rates = {
        direction: blocked_counts.get(direction, 0) / count
        for direction, count in direction_counts.items()
    }
    total = sum(direction_counts.values())
    blocked_total = sum(blocked_counts.values())
    return {
        "post_stop_window_s": max((hs.t for hs in handle), default=stop_t) - stop_t,
        "direction_counts": direction_counts,
        "blocked_rates": blocked_rates,
        "overall_blocked_rate": blocked_total / total if total else None,
    }


def handle_cmd_tracking_stats(
    handle: list[TwistSample],
    cmd_vel: list[TwistSample],
) -> dict[str, Any]:
    ratios: list[float] = []
    angular_ratios: list[float] = []
    for hs in handle:
        cs = nearest_sample(cmd_vel, hs.t)
        if cs is None or abs(hs.t - cs.t) > MATCH_TOLERANCE_S:
            continue
        if abs(hs.linear_x) > HANDLE_LINEAR_THRESHOLD:
            ratios.append(cs.linear_x / hs.linear_x)
        if abs(hs.angular_z) > HANDLE_ANGULAR_THRESHOLD:
            angular_ratios.append(cs.angular_z / hs.angular_z)

    def summarize(values: list[float]) -> dict[str, float | None]:
        if not values:
            return {"mean": None, "min": None, "max": None, "samples": 0}
        return {
            "mean": sum(values) / len(values),
            "min": min(values),
            "max": max(values),
            "samples": len(values),
        }

    return {
        "linear_ratio": summarize(ratios),
        "angular_ratio": summarize(angular_ratios),
    }


def avoid_mode_stats(modes: list[ModeSample]) -> dict[str, Any]:
    if not modes:
        return {"samples": 0, "avoid_ratio": None, "avoid_segments": 0, "avoid_duration_s": 0.0}

    avoid_samples = sum(1 for m in modes if m.mode == 1)
    segments = 0
    in_avoid = False
    segment_start = 0.0
    avoid_duration = 0.0

    for sample in modes:
        active = sample.mode == 1
        if active and not in_avoid:
            in_avoid = True
            segment_start = sample.t
        elif not active and in_avoid:
            segments += 1
            avoid_duration += sample.t - segment_start
            in_avoid = False
    if in_avoid:
        segments += 1
        avoid_duration += modes[-1].t - segment_start

    return {
        "samples": len(modes),
        "avoid_ratio": avoid_samples / len(modes),
        "avoid_segments": segments,
        "avoid_duration_s": avoid_duration,
    }


def rmse_against_reference(
    reference: list[TwistSample],
    candidate: list[TwistSample],
    tolerance_s: float = MATCH_TOLERANCE_S,
) -> dict[str, float | None]:
    if not reference or not candidate:
        return {"linear_rmse": None, "angular_rmse": None, "matched_samples": 0}

    linear_sq = 0.0
    angular_sq = 0.0
    matched = 0

    for ref in reference:
        cand = nearest_sample(candidate, ref.t)
        if cand is None or abs(ref.t - cand.t) > tolerance_s:
            continue
        linear_sq += (ref.linear_x - cand.linear_x) ** 2
        angular_sq += (ref.angular_z - cand.angular_z) ** 2
        matched += 1

    if matched == 0:
        return {"linear_rmse": None, "angular_rmse": None, "matched_samples": 0}

    return {
        "linear_rmse": math.sqrt(linear_sq / matched),
        "angular_rmse": math.sqrt(angular_sq / matched),
        "matched_samples": matched,
    }


def compute_metrics(series: BagSeries) -> dict[str, Any]:
    duration = 0.0
    if series.nav:
        duration = series.nav[-1].t - series.nav[0].t
    elif series.cmd_vel:
        duration = series.cmd_vel[-1].t - series.cmd_vel[0].t

    stop_t = first_stop_time(series.wheel)

    return {
        "bag": series.bag_name,
        "duration_s": duration,
        "handle_count": len(series.handle),
        "cmd_vel_count": len(series.cmd_vel),
        "nav_count": len(series.nav),
        "wheel_count": len(series.wheel),
        "mode_count": len(series.modes),
        "first_stop_t_s": stop_t,
        "cmd_vel_smoothness": compute_derivatives(series.cmd_vel),
        "nav_smoothness": compute_derivatives(series.nav),
        "handle_cmd_tracking": handle_cmd_tracking_stats(series.handle, series.cmd_vel),
        "avoid_mode": avoid_mode_stats(series.modes),
        "post_stop": post_stop_block_stats(series.handle, series.nav, stop_t),
    }


def fmt_pct(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{100.0 * value:.1f}%"


def fmt_num(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def delta_arrow(baseline: float, replay: float, lower_is_better: bool = True) -> str:
    if abs(baseline) < 1e-12 and abs(replay) < 1e-12:
        return "→"
    if lower_is_better:
        if replay < baseline * 0.95:
            return "↓ better"
        if replay > baseline * 1.05:
            return "↑ worse"
        return "→ similar"
    if replay > baseline * 1.05:
        return "↑ better"
    if replay < baseline * 0.95:
        return "↓ worse"
    return "→ similar"


def render_report(
    baseline: dict[str, Any],
    replay: dict[str, Any] | None,
    replay_diff: dict[str, Any] | None,
) -> str:
    lines: list[str] = []
    lines.append("# MPPI stack bag comparison")
    lines.append("")
    lines.append(f"- Baseline: `{baseline['bag']}`")
    if replay:
        lines.append(f"- Replay: `{replay['bag']}`")
    lines.append("")

    lines.append("## cmd_vel smoothness")
    b_cmd = baseline["cmd_vel_smoothness"]
    if replay:
        r_cmd = replay["cmd_vel_smoothness"]
        lines.append("")
        lines.append("| Metric | Baseline | Replay | Delta |")
        lines.append("|--------|----------|--------|-------|")
        for label, key, lower_better in [
            ("max linear step (m/s)", "max_linear_step", True),
            ("max decel (m/s²)", "max_abs_decel", True),
            ("max jerk (m/s³)", "max_abs_jerk", True),
        ]:
            b_val = b_cmd[key]
            r_val = r_cmd[key]
            lines.append(
                f"| {label} | {fmt_num(b_val)} | {fmt_num(r_val)} | "
                f"{delta_arrow(b_val, r_val, lower_better)} |"
            )
    else:
        lines.append(
            f"- max linear step: **{fmt_num(b_cmd['max_linear_step'])}** m/s, "
            f"max decel: **{fmt_num(b_cmd['max_abs_decel'])}** m/s²"
        )

    lines.append("")
    lines.append("## Safety gate AVOID mode")
    b_avoid = baseline["avoid_mode"]
    if replay:
        r_avoid = replay["avoid_mode"]
        lines.append("")
        lines.append("| Metric | Baseline | Replay |")
        lines.append("|--------|----------|--------|")
        lines.append(
            f"| AVOID ratio | {fmt_pct(b_avoid['avoid_ratio'])} | {fmt_pct(r_avoid['avoid_ratio'])} |"
        )
        lines.append(
            f"| AVOID segments | {b_avoid['avoid_segments']} | {r_avoid['avoid_segments']} |"
        )
        lines.append(
            f"| AVOID duration (s) | {fmt_num(b_avoid['avoid_duration_s'])} | "
            f"{fmt_num(r_avoid['avoid_duration_s'])} |"
        )
    else:
        lines.append(f"- AVOID ratio: **{fmt_pct(b_avoid['avoid_ratio'])}**")
        lines.append(f"- AVOID duration: **{fmt_num(b_avoid['avoid_duration_s'])}** s")

    lines.append("")
    lines.append("## Handle → cmd_vel tracking")
    b_track = baseline["handle_cmd_tracking"]["linear_ratio"]
    if replay:
        r_track = replay["handle_cmd_tracking"]["linear_ratio"]
        lines.append(
            f"- linear ratio mean: baseline **{fmt_num(b_track['mean'])}**, "
            f"replay **{fmt_num(r_track['mean'])}** "
            f"({b_track['samples']} / {r_track['samples']} samples)"
        )
    else:
        lines.append(
            f"- linear ratio mean: **{fmt_num(b_track['mean'])}** ({b_track['samples']} samples)"
        )

    lines.append("")
    lines.append("## Post-stop handle vs nav")
    b_post = baseline["post_stop"]
    lines.append(f"- first wheel stop: **{fmt_num(baseline['first_stop_t_s'])}** s")
    if replay:
        r_post = replay["post_stop"]
        lines.append(f"- replay first stop: **{fmt_num(replay['first_stop_t_s'])}** s")
        lines.append(
            f"- overall blocked: baseline **{fmt_pct(b_post['overall_blocked_rate'])}**, "
            f"replay **{fmt_pct(r_post['overall_blocked_rate'])}**"
        )
    else:
        lines.append(f"- overall blocked: **{fmt_pct(b_post['overall_blocked_rate'])}**")

    if replay_diff:
        lines.append("")
        lines.append("## Replay vs baseline RMSE (time-aligned)")
        lines.append("")
        lines.append("| Topic | Linear RMSE (m/s) | Angular RMSE (rad/s) | Matched |")
        lines.append("|-------|-------------------|----------------------|---------|")
        for label, key in [("cmd_vel", "cmd_vel_rmse"), ("nav/control", "nav_rmse")]:
            item = replay_diff[key]
            lines.append(
                f"| {label} | {fmt_num(item['linear_rmse'])} | "
                f"{fmt_num(item['angular_rmse'])} | {item['matched_samples']} |"
            )

    lines.append("")
    return "\n".join(lines)


def print_summary(metrics: dict[str, Any], label: str) -> None:
    cmd = metrics["cmd_vel_smoothness"]
    avoid = metrics["avoid_mode"]
    track = metrics["handle_cmd_tracking"]["linear_ratio"]
    post = metrics["post_stop"]
    print(f"[{label}] {metrics['bag']}")
    print(
        f"  cmd_vel: step={cmd['max_linear_step']:.3f} decel={cmd['max_abs_decel']:.1f} "
        f"jerk={cmd['max_abs_jerk']:.1f}"
    )
    print(
        f"  avoid: ratio={fmt_pct(avoid['avoid_ratio'])} "
        f"duration={avoid['avoid_duration_s']:.1f}s"
    )
    print(
        f"  handle/cmd linear ratio mean={fmt_num(track['mean'])} "
        f"post_stop blocked={fmt_pct(post['overall_blocked_rate'])}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare MPPI bag replay metrics.")
    parser.add_argument(
        "--baseline",
        type=Path,
        required=True,
        help="Original recorded bag directory.",
    )
    parser.add_argument(
        "--replay",
        type=Path,
        default=None,
        help="Replay bag recorded after running the MPPI stack on baseline inputs.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Write markdown report to this path.",
    )
    args = parser.parse_args()

    if not args.baseline.is_dir():
        print(f"Baseline bag not found: {args.baseline}")
        return 2
    if args.replay is not None and not args.replay.is_dir():
        print(f"Replay bag not found: {args.replay}")
        return 2

    baseline_series = load_bag_series(args.baseline)
    baseline_metrics = compute_metrics(baseline_series)
    print_summary(baseline_metrics, "baseline")

    replay_metrics = None
    replay_diff = None
    if args.replay:
        replay_series = load_bag_series(args.replay)
        replay_metrics = compute_metrics(replay_series)
        print_summary(replay_metrics, "replay")
        replay_diff = {
            "cmd_vel_rmse": rmse_against_reference(
                baseline_series.cmd_vel, replay_series.cmd_vel
            ),
            "nav_rmse": rmse_against_reference(
                baseline_series.nav, replay_series.nav
            ),
        }
        print(
            f"  rmse cmd_vel linear={fmt_num(replay_diff['cmd_vel_rmse']['linear_rmse'])} "
            f"nav linear={fmt_num(replay_diff['nav_rmse']['linear_rmse'])}"
        )

    report = render_report(baseline_metrics, replay_metrics, replay_diff)
    print("")
    print(report)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
        print(f"\nReport written to {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
