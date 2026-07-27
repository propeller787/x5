#!/usr/bin/env python3
"""Offline safety_node rosbag checks for T3/T4 validation."""

import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REQUIRED_TOPICS = (
    "/handle/control",
    "/wheel/twist",
    "/scan/left",
    "/scan/right",
    "/scan/back",
    "/perception/left_tilt/pointcloud",
    "/perception/right_tilt/pointcloud",
    "/nav/control",
    "/safety/debug/perception_points",
    "/safety/debug/perception_markers",
    "/safety/local_map",
    "/safety/intervention_log",
    "/tf",
    "/tf_static",
    "/rosout",
)


def open_reader(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def analyze_bag(bag_path: Path) -> dict:
    reader = open_reader(bag_path)
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(type_name) for name, type_name in type_map.items()}

    counts = {name: 0 for name in type_map}
    nav = []
    logs = []

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == "/nav/control":
            msg = deserialize_message(data, msg_types[topic])
            nav.append(
                (
                    stamp / 1e9,
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z,
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z,
                )
            )
        elif topic == "/safety/intervention_log":
            msg = deserialize_message(data, msg_types[topic])
            try:
                logs.append(json.loads(msg.data))
            except json.JSONDecodeError:
                logs.append({"_parse_error": msg.data})

    duration = nav[-1][0] - nav[0][0] if len(nav) > 1 else 0.0
    frequency = (len(nav) - 1) / duration if duration > 0.0 else 0.0

    nonfinite = 0
    nonplanar = 0
    max_abs_linear = 0.0
    max_abs_angular = 0.0
    max_linear_step = 0.0
    zero_nonzero_edges = 0
    prev_linear = None
    prev_nonzero = None

    for _, lx, ly, lz, ax, ay, az in nav:
        values = (lx, ly, lz, ax, ay, az)
        if not all(math.isfinite(value) for value in values):
            nonfinite += 1
        if any(abs(value) > 1e-9 for value in (ly, lz, ax, ay)):
            nonplanar += 1
        max_abs_linear = max(max_abs_linear, abs(lx))
        max_abs_angular = max(max_abs_angular, abs(az))
        if prev_linear is not None:
            max_linear_step = max(max_linear_step, abs(lx - prev_linear))
        prev_linear = lx

        nonzero = abs(lx) > 0.02 or abs(az) > 0.02
        if prev_nonzero is not None and nonzero != prev_nonzero:
            zero_nonzero_edges += 1
        prev_nonzero = nonzero

    reasons = {}
    emergency_count = 0
    candidates = []
    parse_errors = 0
    min_ttc = []
    min_clearance = []

    for item in logs:
        if "_parse_error" in item:
            parse_errors += 1
            continue
        if item.get("emergency_stop"):
            emergency_count += 1
        reason = item.get("emergency_reason", "missing")
        reasons[reason] = reasons.get(reason, 0) + 1
        if "final_candidate" in item:
            candidates.append(item.get("final_candidate"))
        if isinstance(item.get("min_ttc"), (int, float)):
            min_ttc.append(float(item["min_ttc"]))
        if isinstance(item.get("min_clearance"), (int, float)):
            min_clearance.append(float(item["min_clearance"]))

    candidate_changes = sum(
        1 for before, after in zip(candidates, candidates[1:]) if before != after
    )

    missing = [topic for topic in REQUIRED_TOPICS if topic not in counts]
    zero_required = [
        topic
        for topic in REQUIRED_TOPICS
        if topic in counts
        and counts[topic] == 0
        and topic not in ("/tf", "/safety/intervention_log")
    ]

    return {
        "bag": bag_path.name,
        "topics": len(type_map),
        "missing": missing,
        "zero_required": zero_required,
        "nav_count": len(nav),
        "nav_frequency": frequency,
        "max_abs_linear": max_abs_linear,
        "max_abs_angular": max_abs_angular,
        "max_linear_step": max_linear_step,
        "nonfinite": nonfinite,
        "nonplanar": nonplanar,
        "zero_nonzero_edges": zero_nonzero_edges,
        "logs": len(logs),
        "emergency_count": emergency_count,
        "reasons": reasons,
        "candidate_changes": candidate_changes,
        "min_ttc_min": min(min_ttc) if min_ttc else None,
        "min_ttc_max": max(min_ttc) if min_ttc else None,
        "min_clearance_min": min(min_clearance) if min_clearance else None,
        "min_clearance_max": max(min_clearance) if min_clearance else None,
        "parse_errors": parse_errors,
    }


def fmt_optional(value: float | None) -> str:
    return "-" if value is None else f"{value:.3f}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bags",
        nargs="*",
        type=Path,
        help="Bag directories. Defaults to all bags/* directories with metadata.yaml.",
    )
    parser.add_argument("--root", type=Path, default=Path("bags"))
    args = parser.parse_args()

    bags = args.bags
    if not bags:
        bags = sorted(path.parent for path in args.root.glob("*/metadata.yaml"))

    if not bags:
        print("No bags found.")
        return 2

    for bag in bags:
        result = analyze_bag(bag)
        print(f"BAG {result['bag']}")
        print(
            "  topics={topics} missing={missing} zero_required={zero_required}".format(
                topics=result["topics"],
                missing=result["missing"] or "-",
                zero_required=result["zero_required"] or "-",
            )
        )
        print(
            "  nav_count={nav_count} nav_freq={freq:.1f}Hz "
            "max_abs_linear={lin:.3f} max_abs_angular={ang:.3f} "
            "max_linear_step={step:.3f}".format(
                nav_count=result["nav_count"],
                freq=result["nav_frequency"],
                lin=result["max_abs_linear"],
                ang=result["max_abs_angular"],
                step=result["max_linear_step"],
            )
        )
        print(
            "  nonfinite={nonfinite} nonplanar={nonplanar} "
            "zero_nonzero_edges={edges}".format(
                nonfinite=result["nonfinite"],
                nonplanar=result["nonplanar"],
                edges=result["zero_nonzero_edges"],
            )
        )
        print(
            "  logs={logs} emergencies={emergencies} reasons={reasons} "
            "candidate_changes={candidate_changes} min_ttc_minmax={ttc_min}/{ttc_max} "
            "min_clearance_minmax={clear_min}/{clear_max} parse_errors={parse_errors}".format(
                logs=result["logs"],
                emergencies=result["emergency_count"],
                reasons=result["reasons"] or "-",
                candidate_changes=result["candidate_changes"],
                ttc_min=fmt_optional(result["min_ttc_min"]),
                ttc_max=fmt_optional(result["min_ttc_max"]),
                clear_min=fmt_optional(result["min_clearance_min"]),
                clear_max=fmt_optional(result["min_clearance_max"]),
                parse_errors=result["parse_errors"],
            )
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
