#!/usr/bin/env python3
"""Compare safety_node bag metrics: baseline vs replay after code changes.

Offline mode (baseline only):
  python3 tools/compare_bag_replay.py --baseline bags/20260610_101927_T3_SIM-031_car01

After live replay (see replay_and_compare_bag.sh):
  python3 tools/compare_bag_replay.py \\
    --baseline bags/20260610_101927_T3_SIM-031_car01 \\
    --replay   bags/replay_SIM-031_YYYYMMDD \\
    --output   reports/SIM-031_compare.md
"""

from __future__ import annotations

import argparse
import json
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
BRAKING_SPEED_LIMIT_KEY = "braking_speed_limit_m_s"


@dataclass
class TwistSample:
    t: float
    linear_x: float
    angular_z: float


@dataclass
class LogSample:
    t: float
    data: dict[str, Any]


@dataclass
class BagSeries:
    bag_name: str
    handle: list[TwistSample] = field(default_factory=list)
    nav: list[TwistSample] = field(default_factory=list)
    wheel: list[TwistSample] = field(default_factory=list)
    logs: list[LogSample] = field(default_factory=list)


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
    msg_types = {name: get_message(type_name) for name, type_name in type_map.items()}

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
        t = stamp_to_t(stamp)
        if topic == "/handle/control":
            msg = deserialize_message(data, msg_types[topic])
            series.handle.append(TwistSample(t, msg.linear.x, msg.angular.z))
        elif topic == "/nav/control":
            msg = deserialize_message(data, msg_types[topic])
            series.nav.append(TwistSample(t, msg.linear.x, msg.angular.z))
        elif topic == "/wheel/twist":
            msg = deserialize_message(data, msg_types[topic])
            series.wheel.append(TwistSample(t, msg.twist.linear.x, msg.twist.angular.z))
        elif topic == "/safety/intervention_log":
            msg = deserialize_message(data, msg_types[topic])
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                payload = {"_parse_error": msg.data}
            series.logs.append(LogSample(t, payload))

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
    """First stop after the vehicle has moved (ignores initial standstill at bag start)."""
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


def braking_speed_limit_stats(logs: list[LogSample]) -> dict[str, Any]:
    values: list[tuple[float, float]] = []
    for entry in logs:
        if "_parse_error" in entry.data:
            continue
        raw = entry.data.get(BRAKING_SPEED_LIMIT_KEY)
        if isinstance(raw, (int, float)) and math.isfinite(float(raw)):
            values.append((entry.t, float(raw)))

    if len(values) < 2:
        return {
            "samples": len(values),
            "zero_to_max_edges": 0,
            "max_step": 0.0,
            "oscillation_ratio": 0.0,
        }

    max_step = 0.0
    zero_to_max_edges = 0
    oscillations = 0
    prev = values[0][1]
    prev_significant = prev > 0.05

    for _, value in values[1:]:
        max_step = max(max_step, abs(value - prev))
        significant = value > 0.05
        if significant != prev_significant:
            zero_to_max_edges += 1
            if significant:
                oscillations += 1
        prev = value
        prev_significant = significant

    return {
        "samples": len(values),
        "zero_to_max_edges": zero_to_max_edges,
        "max_step": max_step,
        "oscillation_ratio": oscillations / max(len(values) - 1, 1),
    }


def intervention_stats(logs: list[LogSample]) -> dict[str, Any]:
    reasons: dict[str, int] = {}
    recovery_states: dict[str, int] = {}
    emergency_samples = 0
    recovery_active_samples = 0
    parse_errors = 0
    emergency_segments: list[tuple[float, float]] = []
    in_emergency = False
    segment_start = 0.0

    for entry in logs:
        if "_parse_error" in entry.data:
            parse_errors += 1
            continue
        data = entry.data
        if data.get("emergency_stop"):
            emergency_samples += 1
        reason = str(data.get("emergency_reason", "missing"))
        reasons[reason] = reasons.get(reason, 0) + 1
        state = str(data.get("recovery_state", "missing"))
        recovery_states[state] = recovery_states.get(state, 0) + 1
        if data.get("recovery_active"):
            recovery_active_samples += 1

        active = bool(data.get("emergency_stop"))
        if active and not in_emergency:
            in_emergency = True
            segment_start = entry.t
        elif not active and in_emergency:
            emergency_segments.append((segment_start, entry.t))
            in_emergency = False

    if in_emergency and logs:
        emergency_segments.append((segment_start, logs[-1].t))

    total = max(len(logs) - parse_errors, 1)
    return {
        "count": len(logs),
        "parse_errors": parse_errors,
        "emergency_samples": emergency_samples,
        "emergency_ratio": emergency_samples / total,
        "recovery_active_samples": recovery_active_samples,
        "recovery_active_ratio": recovery_active_samples / total,
        "reasons": reasons,
        "recovery_states": recovery_states,
        "emergency_segment_count": len(emergency_segments),
        "emergency_total_duration_s": sum(end - start for start, end in emergency_segments),
    }


def compute_metrics(series: BagSeries) -> dict[str, Any]:
    duration = 0.0
    if series.nav:
        duration = series.nav[-1].t - series.nav[0].t
    stop_t = first_stop_time(series.wheel)
    nav_deriv = compute_derivatives(series.nav)

    return {
        "bag": series.bag_name,
        "duration_s": duration,
        "handle_count": len(series.handle),
        "nav_count": len(series.nav),
        "wheel_count": len(series.wheel),
        "log_count": len(series.logs),
        "first_stop_t_s": stop_t,
        "nav_smoothness": nav_deriv,
        "braking_speed_limit": braking_speed_limit_stats(series.logs),
        "intervention": intervention_stats(series.logs),
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


def render_report(baseline: dict[str, Any], replay: dict[str, Any] | None) -> str:
    lines: list[str] = []
    lines.append("# safety_node bag comparison")
    lines.append("")
    lines.append(f"- Baseline: `{baseline['bag']}`")
    if replay:
        lines.append(f"- Replay: `{replay['bag']}`")
    lines.append("")

    lines.append("## Smoothness (nav/control)")
    b_nav = baseline["nav_smoothness"]
    if replay:
        r_nav = replay["nav_smoothness"]
        lines.append("")
        lines.append("| Metric | Baseline | Replay | Delta |")
        lines.append("|--------|----------|--------|-------|")
        rows = [
            ("max linear step (m/s)", "max_linear_step", True),
            ("max decel (m/s²)", "max_abs_decel", True),
            ("max jerk (m/s³)", "max_abs_jerk", True),
            ("worst step time (s)", "worst_linear_step_t", False),
        ]
        for label, key, lower_better in rows:
            b_val = b_nav[key]
            r_val = r_nav[key]
            lines.append(
                f"| {label} | {fmt_num(b_val)} | {fmt_num(r_val)} | "
                f"{delta_arrow(b_val, r_val, lower_better)} |"
            )
    else:
        lines.append("")
        lines.append(f"- max linear step: **{fmt_num(b_nav['max_linear_step'])}** m/s @ t={fmt_num(b_nav['worst_linear_step_t'])} s")
        lines.append(f"- max decel: **{fmt_num(b_nav['max_abs_decel'])}** m/s²")
        lines.append(f"- max jerk: **{fmt_num(b_nav['max_abs_jerk'])}** m/s³")

    lines.append("")
    lines.append("## Braking speed limit oscillation")
    b_brake = baseline["braking_speed_limit"]
    if replay:
        r_brake = replay["braking_speed_limit"]
        lines.append("")
        lines.append("| Metric | Baseline | Replay | Delta |")
        lines.append("|--------|----------|--------|-------|")
        for label, key, lower_better in [
            ("zero↔max edges", "zero_to_max_edges", True),
            ("max step (m/s)", "max_step", True),
            ("oscillation ratio", "oscillation_ratio", True),
        ]:
            b_val = b_brake[key]
            r_val = r_brake[key]
            lines.append(
                f"| {label} | {fmt_num(b_val)} | {fmt_num(r_val)} | "
                f"{delta_arrow(b_val, r_val, lower_better)} |"
            )
    else:
        lines.append(
            f"- zero↔max edges: **{b_brake['zero_to_max_edges']}**, "
            f"max step: **{fmt_num(b_brake['max_step'])}** m/s"
        )

    lines.append("")
    lines.append("## Post-stop handle vs nav")
    b_post = baseline["post_stop"]
    lines.append(f"- first wheel stop: **{fmt_num(baseline['first_stop_t_s'])}** s")
    if replay:
        r_post = replay["post_stop"]
        lines.append(f"- replay first stop: **{fmt_num(replay['first_stop_t_s'])}** s")
        lines.append("")
        lines.append("| Direction | Baseline blocked | Replay blocked |")
        lines.append("|-----------|------------------|----------------|")
        directions = sorted(set(b_post["direction_counts"]) | set(r_post["direction_counts"]))
        for direction in directions:
            b_rate = b_post["blocked_rates"].get(direction)
            r_rate = r_post["blocked_rates"].get(direction)
            lines.append(
                f"| {direction} | {fmt_pct(b_rate)} ({b_post['direction_counts'].get(direction, 0)}) | "
                f"{fmt_pct(r_rate)} ({r_post['direction_counts'].get(direction, 0)}) |"
            )
        lines.append("")
        lines.append(
            f"- overall blocked: baseline **{fmt_pct(b_post['overall_blocked_rate'])}**, "
            f"replay **{fmt_pct(r_post['overall_blocked_rate'])}**"
        )
    else:
        for direction, count in sorted(b_post["direction_counts"].items()):
            rate = b_post["blocked_rates"].get(direction)
            lines.append(f"- {direction}: blocked **{fmt_pct(rate)}** ({count} samples)")
        lines.append(f"- overall blocked: **{fmt_pct(b_post['overall_blocked_rate'])}**")

    lines.append("")
    lines.append("## Intervention log")
    b_int = baseline["intervention"]
    if replay:
        r_int = replay["intervention"]
        lines.append("")
        lines.append("| Metric | Baseline | Replay |")
        lines.append("|--------|----------|--------|")
        lines.append(
            f"| emergency ratio | {fmt_pct(b_int['emergency_ratio'])} | {fmt_pct(r_int['emergency_ratio'])} |"
        )
        lines.append(
            f"| recovery_active ratio | {fmt_pct(b_int['recovery_active_ratio'])} | "
            f"{fmt_pct(r_int['recovery_active_ratio'])} |"
        )
        lines.append(
            f"| emergency duration (s) | {fmt_num(b_int['emergency_total_duration_s'])} | "
            f"{fmt_num(r_int['emergency_total_duration_s'])} |"
        )
        lines.append("")
        lines.append("Recovery states (baseline → replay):")
        states = sorted(set(b_int["recovery_states"]) | set(r_int["recovery_states"]))
        for state in states:
            lines.append(
                f"- `{state}`: {b_int['recovery_states'].get(state, 0)} → "
                f"{r_int['recovery_states'].get(state, 0)}"
            )
    else:
        lines.append(f"- emergency ratio: **{fmt_pct(b_int['emergency_ratio'])}**")
        lines.append(f"- recovery_active ratio: **{fmt_pct(b_int['recovery_active_ratio'])}**")
        lines.append(f"- emergency duration: **{fmt_num(b_int['emergency_total_duration_s'])}** s")
        lines.append(f"- reasons: `{b_int['reasons']}`")
        lines.append(f"- recovery states: `{b_int['recovery_states']}`")

    lines.append("")
    return "\n".join(lines)


def print_summary(metrics: dict[str, Any], label: str) -> None:
    nav = metrics["nav_smoothness"]
    post = metrics["post_stop"]
    brake = metrics["braking_speed_limit"]
    inter = metrics["intervention"]
    print(f"[{label}] {metrics['bag']}")
    print(
        f"  nav: step={nav['max_linear_step']:.3f} decel={nav['max_abs_decel']:.1f} "
        f"jerk={nav['max_abs_jerk']:.1f} @ t={nav['worst_linear_step_t']:.2f}s"
    )
    print(
        f"  brake_limit: edges={brake['zero_to_max_edges']} max_step={brake['max_step']:.3f}"
    )
    print(
        f"  post_stop blocked={fmt_pct(post['overall_blocked_rate'])} "
        f"recovery_active={fmt_pct(inter['recovery_active_ratio'])}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare safety_node bag replay metrics.")
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
        help="Replay bag recorded after running safety_node on baseline inputs.",
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

    baseline_metrics = compute_metrics(load_bag_series(args.baseline))
    print_summary(baseline_metrics, "baseline")

    replay_metrics = None
    if args.replay:
        replay_metrics = compute_metrics(load_bag_series(args.replay))
        print_summary(replay_metrics, "replay")

    report = render_report(baseline_metrics, replay_metrics)
    print("")
    print(report)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
        print(f"\nReport written to {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
