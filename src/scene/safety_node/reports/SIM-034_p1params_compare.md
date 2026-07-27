# safety_node bag comparison

- Baseline: `20260610_195705_T3_SIM-034_car01`
- Replay: `replay_SIM-034_p1params`

## Smoothness (nav/control)

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| max linear step (m/s) | 0.000 | 0.000 | → |
| max decel (m/s²) | 0.000 | 0.000 | → |
| max jerk (m/s³) | 0.000 | 0.000 | → |
| worst step time (s) | 0.000 | 0.000 | → |

## Braking speed limit oscillation

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| zero↔max edges | 0.000 | 0.000 | → |
| max step (m/s) | 0.000 | 0.000 | → |
| oscillation ratio | 0.000 | 0.000 | → |

## Post-stop handle vs nav
- first wheel stop: **-** s
- replay first stop: **-** s

| Direction | Baseline blocked | Replay blocked |
|-----------|------------------|----------------|

- overall blocked: baseline **-**, replay **-**

## Intervention log

| Metric | Baseline | Replay |
|--------|----------|--------|
| emergency ratio | 100.0% | 100.0% |
| recovery_active ratio | 0.0% | 0.0% |
| emergency duration (s) | 8.962 | 8.963 |

Recovery states (baseline → replay):
- `aborted_timeout`: 360 → 278
- `blocked`: 0 → 123
- `confirming`: 0 → 4
- `idle`: 0 → 831
