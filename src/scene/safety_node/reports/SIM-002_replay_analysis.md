# safety_node bag comparison

- Baseline: `20260615_201948_T3_SIM-002_car01`
- Replay: `replay_SIM-002_analysis`

## Smoothness (nav/control)

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| max linear step (m/s) | 0.001 | 0.015 | ↑ worse |
| max decel (m/s²) | 0.059 | 0.500 | ↑ worse |
| max jerk (m/s³) | 3.943 | 26.798 | ↑ worse |
| worst step time (s) | 10.221 | 11.139 | ↑ better |

## Braking speed limit oscillation

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| zero↔max edges | 32.000 | 4.000 | ↓ better |
| max step (m/s) | 0.666 | 0.417 | ↓ better |
| oscillation ratio | 0.027 | 0.002 | ↓ better |

## Post-stop handle vs nav
- first wheel stop: **-** s
- replay first stop: **-** s

| Direction | Baseline blocked | Replay blocked |
|-----------|------------------|----------------|

- overall blocked: baseline **-**, replay **-**

## Intervention log

| Metric | Baseline | Replay |
|--------|----------|--------|
| emergency ratio | 98.5% | 86.6% |
| recovery_active ratio | 0.0% | 0.0% |
| emergency duration (s) | 16.418 | 14.291 |

Recovery states (baseline → replay):
- `aborted_timeout`: 0 → 1
- `blocked`: 0 → 468
- `confirming`: 0 → 71
- `idle`: 793 → 366
