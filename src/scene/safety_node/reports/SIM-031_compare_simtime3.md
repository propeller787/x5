# safety_node bag comparison

- Baseline: `20260610_101927_T3_SIM-031_car01`
- Replay: `replay_SIM-031_simtime3`

## Smoothness (nav/control)

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| max linear step (m/s) | 0.597 | 0.023 | ↓ better |
| max decel (m/s²) | 22.853 | 0.800 | ↓ better |
| max jerk (m/s³) | 984.654 | 38.654 | ↓ better |
| worst step time (s) | 4.369 | 4.890 | ↑ better |

## Braking speed limit oscillation

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| zero↔max edges | 23.000 | 34.000 | ↑ worse |
| max step (m/s) | 1.610 | 1.620 | → similar |
| oscillation ratio | 0.027 | 0.040 | ↑ worse |

## Post-stop handle vs nav
- first wheel stop: **5.119** s
- replay first stop: **5.104** s

| Direction | Baseline blocked | Replay blocked |
|-----------|------------------|----------------|
| backward | 99.6% (228) | 93.0% (228) |
| forward | 62.4% (149) | 59.7% (149) |
| turn_left | 100.0% (30) | 100.0% (30) |

- overall blocked: baseline **86.0%**, replay **81.3%**

## Intervention log

| Metric | Baseline | Replay |
|--------|----------|--------|
| emergency ratio | 58.7% | 58.9% |
| recovery_active ratio | 0.0% | 4.0% |
| emergency duration (s) | 6.557 | 8.866 |

Recovery states (baseline → replay):
- `active`: 0 → 19
- `blocked`: 0 → 10
- `confirming`: 0 → 86
- `idle`: 414 → 360
