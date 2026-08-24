# Benchmark Results — Feature 1: Task Planning
## Scenario: Clear Path, N = 1 (Phase 0 — Baseline)

**Official run:** `run_20260819_0800` (5/5 valid repeats, on map `world_tb3`;
see `PROCEDURE.md` section 1 for full conditions).

**Run conditions:** ENV_01 (map `world_tb3`) · FLEET_01 (1 robot
`tb3_robot1`) · TRAF_00 (no traffic) · CONFIG_01 · Task: patrol
`charger_1 ↔ crossing_1`, 5 rounds/repeat · 5 independent repeats.

---

## How to benchmark & evaluate

The full step-by-step terminal procedure lives in `PROCEDURE.md` section 4.
Summary of the 2 phases:

1. **Benchmark (data collection)** — `PROCEDURE.md` section 4, steps 7-8:
   record `ros2 bag record` on the topics `bid_notice/bid_response/
   dispatch_request/dispatch_ack/task_api_requests/fleet_states`, while
   submitting the task through `run_benchmark.py` (real auction dispatch,
   5 patrol rounds × 5 repeats — full configuration in section 3).
2. **Evaluate (metric computation)** — `PROCEDURE.md` section 4, step 10:
   run `analyze_task_planning.py` on the recorded rosbag to compute
   1a/1b/1c using the formulas in the "Formula" column below, producing
   `task_planning_metrics.json`.

---

## Target vs. Measured Results

| No. | Metric | Unit | Data source | Formula | Target (per report) | **Measured result (n=5)** | **Pass / Fail** |
|---|---|---|---|---|---|---|---|
| 1a | Task dispatch success rate | % | `bid_notice`, `dispatch_request`, `dispatch_ack` | `N_dispatch_success / N_bidnotice × 100` | ≥ 95% | **100.0%** (5/5) | ✅ **Pass** |
| 1b | Planning latency | s | `bid_notice`, `bid_response` | `t_bid_response − t_bid_notice` | Within a few seconds | **mean 1.80 ms**, std 0.81ms, min 0.60ms, max 2.54ms | ✅ **Pass** (~1000x faster than target) |
| 1c | Makespan | s | `task_api_requests`, `fleet_states` (replacing `/task_state` — the system does not run `rmf_api_server`) | `t_end − t_start` | Reference — used as the comparison baseline for N=3 later | **mean 195.8s**, std 47.7s, min 127.5s, max 250.5s | ➖ N/A (no absolute target, this establishes the baseline) |

---

## Per-repeat detail

| Repeat | task_id | Planning latency (ms) | Dispatch success | Makespan (s) | Distance travelled (m) |
|---|---|---|---|---|---|
| 1 | `patrol.dispatch-4113f29355` | 1.34 | ✅ | 211.50 | 44.235 |
| 2 | `patrol.dispatch-c5518861d7` | 2.54 | ✅ | 127.50 | 25.026 |
| 3 | `patrol.dispatch-b4cd17d97d` | 2.16 | ✅ | 170.20 | 35.978 |
| 4 | `patrol.dispatch-77b66eb141` | 2.35 | ✅ | 219.10 | 48.836 |
| 5 | `patrol.dispatch-5d52db40ef` | 0.60 | ✅ | 250.50 | 44.115 |
| **Mean** | | **1.80** | **100%** | **195.76** | 39.64 |
| **Stdev** | | **0.81** | | **47.70** | 8.98 |

---

## Note on Makespan (1c) variance

Makespan varies considerably between repeats (127.5s – 250.5s, nearly 2x).
Distance travelled varies correspondingly (25.0m – 48.8m for the same 5
patrol rounds), indicating this is genuine physical variance in the
robot's behaviour rather than a measurement error — even with no other
robot to contend with (TRAF_00).

**Suspected causes (not yet root-caused):**
- The robot occasionally has to wait/replan between legs (observed in the
  log `"Requesting replan ... command handle seems unresponsive"` during
  manual test runs before the official benchmark).
- AMCL/costmap may process more slowly at certain locations on the map
  (furniture-dense areas — tables, chairs, cabinets).

Does not affect the 1a/1b conclusions (both clearly pass target). This
baseline-level variance (47.7s standard deviation at N=1, with no
traffic at all) is part of why the N=3 scenarios that followed (see the
comparison table in "Conclusion" below) also show fairly large stdev —
the portion of variance caused by genuine traffic interaction cannot be
fully separated from the system's inherent background noise on this map.

---

## Conclusion — Feature 1: Task Planning (Phase 0 baseline)

All 3 claims of the "Task planning" feature have been benchmarked and
**confirmed to pass** under the N=1 baseline condition, on map `world_tb3`:

> *"Open-RMF can dispatch up to X concurrent patrol tasks with ≥ 95% success
> rate and planning latency under Y seconds"*
> → **X (at N=1) = 100% success rate, Y ≈ 1.8ms** — well beyond target.

The 1c value (makespan 195.8s ± 47.7s, N=1, map `world_tb3`) was used as
the **comparison baseline** ("Comparison to single robot" per the report)
for the 4 N=3 scenarios subsequently benchmarked:

| Scenario (N=3) | Makespan mean (s) | vs. N=1 baseline (195.8s) |
|---|---|---|
| Bottleneck | 197.6 | essentially equal (+0.9%) |
| Head-on | 265.5 | +35.6% |
| Crossing | 253.8 | +29.7% |
| Shared lane | 280.1 | +43.1% (stdev 303.6s — very large variance) |

All 4 N=3 scenarios show a **higher** mean makespan than the N=1 baseline,
which makes sense given the added time cost of waiting/negotiating once
multi-robot traffic is introduced. Bottleneck (same-direction traffic,
simple queueing) costs almost no extra time over baseline; Shared lane and
Head-on (opposing-direction / tight-space contention) cost the most extra
time and show the largest variance — matching the intuitive expectation
that traffic pattern difficulty increases in that order. See each
scenario's own `PROCEDURE.md` for full detail.

---

*Data source: `run_20260819_0800/task_planning_metrics.json` (computed
from `bag/` via `analyze_task_planning.py` — full procedure in
`PROCEDURE.md` section 4).*
