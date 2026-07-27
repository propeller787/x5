# safety_node bag comparison

- Baseline: `20260610_101927_T3_SIM-031_car01`

## Smoothness (nav/control)

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
- max linear step: **0.597** m/s @ t=4.369 s
- max decel: **22.853** m/s²
- max jerk: **984.654** m/s³

## Braking speed limit oscillation
- zero↔max edges: **23**, max step: **1.610** m/s

## Post-stop handle vs nav
- first wheel stop: **1.057** s
- backward: blocked **99.6%** (228 samples)
- forward: blocked **42.4%** (316 samples)
- turn_left: blocked **100.0%** (30 samples)
- overall blocked: **68.1%**

## Intervention log
- emergency ratio: **58.7%**
- recovery_active ratio: **0.0%**
- emergency duration: **6.557** s
- reasons: `{'none': 171, 'braking_distance_insufficient': 10, 'brake_hold': 62, 'hard_stop_distance': 171}`
- recovery states: `{'idle': 414}`
