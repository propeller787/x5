# safety_node bag comparison

- Baseline: `20260610_195836_T3_SIM-035_car01`
- Replay: `replay_SIM-035_p1params`

## Smoothness (nav/control)

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| max linear step (m/s) | 0.021 | 0.035 | ↑ worse |
| max decel (m/s²) | 0.995 | 0.898 | ↓ better |
| max jerk (m/s³) | 30.238 | 201.877 | ↑ worse |
| worst step time (s) | 5.852 | 11.233 | ↑ better |

## Braking speed limit oscillation

| Metric | Baseline | Replay | Delta |
|--------|----------|--------|-------|
| zero↔max edges | 3.000 | 12.000 | ↑ worse |
| max step (m/s) | 1.608 | 1.620 | → similar |
| oscillation ratio | 0.002 | 0.010 | ↑ worse |

## Post-stop handle vs nav
- first wheel stop: **6.151** s
- replay first stop: **6.129** s

| Direction | Baseline blocked | Replay blocked |
|-----------|------------------|----------------|
| backward | 100.0% (211) | 18.0% (211) |
| forward | 100.0% (124) | 100.0% (124) |
| turn_left | 83.3% (6) | 100.0% (6) |

- overall blocked: baseline **99.7%**, replay **49.3%**

## Intervention log

| Metric | Baseline | Replay |
|--------|----------|--------|
| emergency ratio | 71.3% | 63.4% |
| recovery_active ratio | 7.8% | 0.0% |
| emergency duration (s) | 8.995 | 9.361 |

Recovery states (baseline → replay):
- `active`: 39 → 0
- `blocked`: 49 → 0
- `confirming`: 148 → 0
- `idle`: 266 → 604
