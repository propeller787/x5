#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Pair costmap_raw and occupymap_raw PNG exports by nearest header stamp."""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

STAMP_RE = re.compile(r"^(?P<prefix>.+)_(?P<sec>\d+)_(?P<nsec>\d+)\.png$")


def stamp_from_path(path: Path) -> float:
    match = STAMP_RE.match(path.name)
    if not match:
        raise ValueError(f"Unexpected filename: {path.name}")
    return int(match.group("sec")) + int(match.group("nsec")) * 1e-9


def load_index(directory: Path, pattern: str) -> list[tuple[float, Path]]:
    files = sorted(directory.glob(pattern), key=stamp_from_path)
    return [(stamp_from_path(p), p) for p in files]


def nearest(target: float, items: list[tuple[float, Path]]) -> tuple[Path, float]:
    best_stamp, best_path = min(items, key=lambda item: abs(item[0] - target))
    return best_path, target - best_stamp


def align_occupymap_to_cost(occ: np.ndarray, target_shape: tuple[int, int]) -> np.ndarray:
    """Center-crop occupymap PNG to match costmap dimensions for side-by-side."""
    th, tw = target_shape
    oh, ow = occ.shape[:2]
    if oh == th and ow == tw:
        return occ
    y0 = max(0, (oh - th) // 2)
    x0 = max(0, (ow - tw) // 2)
    cropped = occ[y0 : y0 + th, x0 : x0 + tw]
    if cropped.shape[:2] == target_shape:
        return cropped
    out = np.zeros((th, tw, 3), dtype=occ.dtype)
    ch, cw = cropped.shape[:2]
    y_off = max(0, (th - ch) // 2)
    x_off = max(0, (tw - cw) // 2)
    out[y_off : y_off + ch, x_off : x_off + cw] = cropped
    return out


def save_side_by_side(cost_path: Path, occ_path: Path, out_path: Path, gap: int) -> None:
    cost = np.array(Image.open(cost_path))
    occ = np.array(Image.open(occ_path))
    if occ.shape[:2] != cost.shape[:2]:
        # Nav2 costmap 120x160 vs occupymap 160x160: crop/pad occupymap center to cost size
        occ = align_occupymap_to_cost(occ, cost.shape[:2])
    if occ.shape[:2] != cost.shape[:2]:
        raise ValueError(f"Size mismatch: cost={cost.shape[:2]} occ={occ.shape[:2]}")
    gap_col = np.full((cost.shape[0], gap, 3), 200, dtype=np.uint8)
    merged = np.concatenate([cost, gap_col, occ], axis=1)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(merged).save(out_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--costmap-dir",
        type=Path,
        default=Path("output/costmap_raw/case1"),
        help="Directory with costmap_raw_*.png",
    )
    parser.add_argument(
        "--occupymap-dir",
        type=Path,
        default=Path("output/occupymap_raw/case1"),
        help="Directory with occupymap_raw_*.png",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("output/pairs/case1"),
        help="Output directory for pairs.csv and optional side-by-side images",
    )
    parser.add_argument(
        "--max-dt-ms",
        type=float,
        default=100.0,
        help="Skip pairs when |dt| exceeds this many milliseconds",
    )
    parser.add_argument(
        "--side-by-side",
        action="store_true",
        help="Write side-by-side PNGs (costmap | occupymap transposed)",
    )
    parser.add_argument(
        "--gap",
        type=int,
        default=4,
        help="Pixel gap between costmap and occupymap in side-by-side images",
    )
    args = parser.parse_args()

    cost_dir = args.costmap_dir.resolve()
    occ_dir = args.occupymap_dir.resolve()
    if not cost_dir.is_dir():
        print(f"Missing costmap dir: {cost_dir}", file=sys.stderr)
        return 1
    if not occ_dir.is_dir():
        print(f"Missing occupymap dir: {occ_dir}", file=sys.stderr)
        return 1

    cost_index = load_index(cost_dir, "costmap_raw_*.png")
    occ_index = load_index(occ_dir, "occupymap_raw_*.png")
    if not cost_index:
        print(f"No costmap PNGs in {cost_dir}", file=sys.stderr)
        return 1
    if not occ_index:
        print(f"No occupymap PNGs in {occ_dir}", file=sys.stderr)
        return 1

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "pairs.csv"
    side_dir = args.output / "side_by_side"

    rows: list[dict[str, str | float]] = []
    skipped = 0
    for cost_stamp, cost_path in cost_index:
        occ_path, dt = nearest(cost_stamp, occ_index)
        dt_ms = dt * 1000.0
        if abs(dt_ms) > args.max_dt_ms:
            skipped += 1
            continue
        row = {
            "costmap_stamp": f"{cost_stamp:.9f}",
            "costmap_file": cost_path.name,
            "occupymap_file": occ_path.name,
            "dt_ms": f"{dt_ms:.3f}",
        }
        rows.append(row)
        if args.side_by_side:
            out_name = f"pair_{cost_path.stem}.png"
            save_side_by_side(cost_path, occ_path, side_dir / out_name, args.gap)

    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["costmap_stamp", "costmap_file", "occupymap_file", "dt_ms"],
        )
        writer.writeheader()
        writer.writerows(rows)

    dts = [abs(float(r["dt_ms"])) for r in rows]
    print(f"Paired {len(rows)} frames -> {csv_path}")
    if dts:
        print(f"  |dt|: min={min(dts):.1f}ms max={max(dts):.1f}ms avg={sum(dts)/len(dts):.1f}ms")
    if skipped:
        print(f"  skipped {skipped} costmap frames (|dt| > {args.max_dt_ms}ms)")
    if args.side_by_side:
        print(f"  side-by-side -> {side_dir}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
