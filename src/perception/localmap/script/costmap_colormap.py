#!/usr/bin/env python3
# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

"""Nav2 costmap raw value (0-255) to RGB for visualization."""

from __future__ import annotations

import numpy as np

COST_FREE = 0
COST_INSCRIBED = 253
COST_LETHAL = 254
COST_UNKNOWN = 255


def cost_to_occupancy(cost: np.ndarray) -> np.ndarray:
    """Match localmap_occupymap/cost_value.hpp costToOccupancyValue."""
    occ = np.full(cost.shape, -1, dtype=np.int16)
    occ[cost == COST_FREE] = 0
    occ[cost == COST_INSCRIBED] = 99
    occ[cost == COST_LETHAL] = 100
    mask = (cost > 0) & (cost < COST_INSCRIBED)
    occ[mask] = 1 + (97 * (cost[mask].astype(np.int16) - 1)) // 251
    return occ


def cost_to_rgb(cost: np.ndarray) -> np.ndarray:
    """Render raw Nav2 costs as an RGB image (H, W, 3), uint8."""
    rgb = np.zeros((*cost.shape, 3), dtype=np.uint8)

    free = cost == COST_FREE
    rgb[free] = (255, 255, 255)

    unknown = cost == COST_UNKNOWN
    rgb[unknown] = (128, 128, 128)

    lethal = cost == COST_LETHAL
    rgb[lethal] = (0, 0, 0)

    inscribed = cost == COST_INSCRIBED
    rgb[inscribed] = (0, 192, 255)

    inflated = (cost > 0) & (cost < COST_INSCRIBED)
    if np.any(inflated):
        t = cost[inflated].astype(np.float32) / 252.0
        rgb[inflated, 0] = (255 * t).astype(np.uint8)
        rgb[inflated, 1] = 0
        rgb[inflated, 2] = (255 * (1.0 - t)).astype(np.uint8)

    return rgb
