#!/usr/bin/env python3
"""Summarize gcov JSON coverage for safety-critical implementation files."""

import argparse
import gzip
import json
import pathlib
import subprocess
import tempfile


CORE_SOURCES = [
    "core/avoidance_utils.cpp",
    "core/vehicle_footprint.cpp",
    "core/motion_rollout.cpp",
    "core/swept_footprint_checker.cpp",
    "core/safety_filter.cpp",
    "core/emergency_brake.cpp",
    "core/obstacle_field.cpp",
    "rolling_local_map.cpp",
    "map_constraint_check.cpp",
    "mode_supervisor.cpp",
    "avoidance_controller.cpp",
    "output_shaper.cpp",
    "perception_utils.cpp",
]


def coverage_for_source(
    build_dir: pathlib.Path, source: str
) -> tuple[int, int, int, int, list[int]]:
    gcda = (
        build_dir
        / "CMakeFiles"
        / "safety_node_lib.dir"
        / "src"
        / f"{source}.gcda"
    )
    if not gcda.exists():
        raise FileNotFoundError(f"missing coverage data: {gcda}")

    with tempfile.TemporaryDirectory(prefix="safety_node_gcov_") as temp_dir:
        subprocess.run(
            [
                "gcov",
                "--json-format",
                "--branch-probabilities",
                "--branch-counts",
                str(gcda),
            ],
            cwd=temp_dir,
            check=True,
            stdout=subprocess.DEVNULL,
        )
        reports = list(pathlib.Path(temp_dir).glob("*.gcov.json.gz"))
        if len(reports) != 1:
            raise RuntimeError(f"expected one gcov report for {source}, got {len(reports)}")
        with gzip.open(reports[0], "rt", encoding="utf-8") as report:
            data = json.load(report)

    suffix = f"/src/{source}"
    source_data = next(
        (entry for entry in data["files"] if entry["file"].endswith(suffix)),
        None,
    )
    if source_data is None:
        raise RuntimeError(f"source {source} not present in gcov output")

    lines = source_data["lines"]
    line_total = len(lines)
    line_covered = sum(1 for line in lines if line["count"] > 0)
    missed_lines = [line["line_number"] for line in lines if line["count"] == 0]
    branches = [
        branch
        for line in lines
        for branch in line.get("branches", [])
        if not branch.get("throw", False)
    ]
    branch_total = len(branches)
    branch_covered = sum(1 for branch in branches if branch["count"] > 0)
    return line_covered, line_total, branch_covered, branch_total, missed_lines


def percent(covered: int, total: int) -> float:
    return 100.0 if total == 0 else 100.0 * covered / total


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--build-dir",
        type=pathlib.Path,
        default=pathlib.Path("build-coverage/safety_node"),
    )
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--show-missed", action="store_true")
    parser.add_argument("--line-threshold", type=float, default=90.0)
    parser.add_argument("--branch-threshold", type=float, default=80.0)
    args = parser.parse_args()

    total_lines_covered = 0
    total_lines = 0
    total_branches_covered = 0
    total_branches = 0

    print(f"{'source':42} {'lines':>14} {'branches':>14}")
    for source in CORE_SOURCES:
        (
            line_hit,
            line_total,
            branch_hit,
            branch_total,
            missed_lines,
        ) = coverage_for_source(args.build_dir.resolve(), source)
        total_lines_covered += line_hit
        total_lines += line_total
        total_branches_covered += branch_hit
        total_branches += branch_total
        print(
            f"{source:42} "
            f"{percent(line_hit, line_total):6.2f}% "
            f"{branch_hit:4}/{branch_total:<4} "
            f"{percent(branch_hit, branch_total):6.2f}%"
        )
        if args.show_missed and missed_lines:
            print(f"  missed lines: {', '.join(str(line) for line in missed_lines)}")

    line_rate = percent(total_lines_covered, total_lines)
    branch_rate = percent(total_branches_covered, total_branches)
    print(
        f"{'TOTAL':42} "
        f"{line_rate:6.2f}% "
        f"{total_branches_covered:4}/{total_branches:<4} "
        f"{branch_rate:6.2f}%"
    )

    if args.check and (
        line_rate < args.line_threshold or branch_rate < args.branch_threshold
    ):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
