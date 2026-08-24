# Benchmark Results — Feature 1: Task Planning
## Scenario: Bottleneck, N = 3 (TRAF_03)

**Official run:** `run_20260821_0847_N3` (15/15 tasks dispatched, on map `world_tb3`;
see `PROCEDURE.md` section 1 for full conditions).

**Run conditions:** ENV_01 (map `world_tb3`) · FLEET_02 (3 robots
`tb3_robot1`/`tb3_robot2`/`tb3_robot3`) · TRAF_03 (single-lane mutex corridor) · CONFIG_01 · Routes:
`bottleneck_1↔bottleneck_3`, `sharedlane_3↔loop_1`, `bottleneck_3↔sharedlane_3`, 1 round/repeat ·
5 independent repeats (3 concurrent tasks/repeat, 15 tasks total).

---

## How to benchmark & evaluate

The full step-by-step terminal procedure lives in `PROCEDURE.md` section 4.
Summary of the 2 phases:

1. **Benchmark (data collection)** — `PROCEDURE.md` section 4, steps 7-8:
   record `ros2 bag record` on the topics `bid_notice/bid_response/dispatch_request/dispatch_ack/
   task_api_requests/fleet_states` plus the negotiation/blockade topics, while submitting the 3
   concurrent routes through `run_benchmark_concurrent.py` (real auction dispatch, 1 round × 5
   repeats — full configuration in section 3).
2. **Evaluate (metric computation)** — `PROCEDURE.md` section 4, step 10:
   run `analyze_task_planning_concurrent.py` on the recorded rosbag to compute 1a/1b/1c using the
   formulas in the "Formula" column below, producing `task_planning_metrics.json`.

---

## Target vs. Measured Results

| No. | Metric | Unit | Data source | Formula | Target (per report) | **Measured result (n=15)** | **Pass / Fail** |
|---|---|---|---|---|---|---|---|
| 1a | Task dispatch success rate | % | `bid_notice`, `dispatch_request`, `dispatch_ack` | `N_dispatch_success / N_bidnotice × 100` | ≥ 95% | **100.0%** (15/15) | ✅ **Pass** |
| 1b | Planning latency | s | `bid_notice`, `bid_response` | `t_bid_response − t_bid_notice` | Within a few seconds | **mean 1.58 ms**, std 1.75ms, min 0.57ms, max 6.48ms | ✅ **Pass** (~1900x faster than target) |
| 1c | Makespan | s | `task_api_requests`, `fleet_states` | `t_end − t_start` | Compare to N=1 baseline (195.8s, Clear Path) | **mean 197.6s**, std 69.0s, min 91.3s, max 336.0s (n=15, 0 unresolved) | ➖ N/A (comparative only, +0.9% vs. baseline) |

---

## Per-task detail

| Repeat | Route | Robot | Planning latency (ms) | Dispatch success | Makespan (s) |
|---|---|---|---|---|---|
| 1 | `bottleneck_1→bottleneck_3` | tb3_robot2 | 4.65 | ✅ | 147.95 |
| 1 | `sharedlane_3→loop_1` | tb3_robot3 | 6.48 | ✅ | 335.95 |
| 1 | `bottleneck_3→sharedlane_3` | tb3_robot2 | 2.74 | ✅ | 221.75 |
| 2 | `bottleneck_1→bottleneck_3` | tb3_robot3 | 1.41 | ✅ | 129.60 |
| 2 | `sharedlane_3→loop_1` | tb3_robot2 | 0.73 | ✅ | 233.60 |
| 2 | `bottleneck_3→sharedlane_3` | tb3_robot3 | 0.69 | ✅ | 219.50 |
| 3 | `bottleneck_1→bottleneck_3` | tb3_robot3 | 0.62 | ✅ | 98.00 |
| 3 | `sharedlane_3→loop_1` | tb3_robot3 | 1.09 | ✅ | 226.50 |
| 3 | `bottleneck_3→sharedlane_3` | tb3_robot2 | 0.71 | ✅ | 249.50 |
| 4 | `bottleneck_1→bottleneck_3` | tb3_robot2 | 0.66 | ✅ | 91.30 |
| 4 | `sharedlane_3→loop_1` | tb3_robot2 | 0.62 | ✅ | 225.30 |
| 4 | `bottleneck_3→sharedlane_3` | tb3_robot3 | 0.68 | ✅ | 245.90 |
| 5 | `bottleneck_1→bottleneck_3` | tb3_robot3 | 0.58 | ✅ | 105.60 |
| 5 | `sharedlane_3→loop_1` | tb3_robot3 | 1.52 | ✅ | 193.20 |
| 5 | `bottleneck_3→sharedlane_3` | tb3_robot2 | 0.57 | ✅ | 239.70 |
| **Mean** | | | **1.58** | **100%** | **197.56** |
| **Stdev** | | | **1.75** | | **69.02** |

All 15/15 tasks resolved a completion time — no unresolved makespan for this scenario.

---

## Per-repeat outcome (distance travelled)

| Repeat | tb3_robot1 | tb3_robot2 | tb3_robot3 |
|---|---|---|---|
| 1 | 0.0 m (short_distance) | 31.64 m (completed) | 25.98 m (completed) |
| 2 | 0.0 m (short_distance) | 15.61 m (completed) | 32.68 m (completed) |
| 3 | 0.0 m (short_distance) | 17.07 m (completed) | 33.91 m (completed) |
| 4 | 0.0 m (short_distance) | 32.40 m (completed) | 16.53 m (completed) |
| 5 | 0.0 m (short_distance) | 17.87 m (completed) | 31.86 m (completed) |

`tb3_robot1` shows 0.0m in all 5 repeats — it never wins a bid (see note below), not a dispatch
failure (1a unaffected). `tb3_robot2`/`tb3_robot3` split the 15 tasks 7/8 and both complete cleanly
every time.

---

## Note on `tb3_robot1` non-participation

All 3 routes are concentrated on the east side of the map (`bottleneck`/`sharedlane_3`/`loop_1`,
x≈25–29m), while `tb3_robot1` spawns at `charger_1` (x≈5.4m, farthest west). Its bid cost is higher
than `tb3_robot2`/`tb3_robot3` for every route in this set, so it loses 100% of the 15 bids — a
consequence of route design, not a bug. 1a still reaches 100% since every task is dispatched
successfully to robot2 or robot3. Adding a route with an endpoint near `charger_1` would let robot1
participate, but is not required to hit the report's 1a target.

---

## Conclusion

All 3 metrics pass cleanly on the first official run, no infrastructure fixes required: 1a=100%,
1b≈1.58ms, 1c=197.6s (+0.9% vs. the N=1 baseline — essentially no extra cost over solo operation).
The single-lane mutex (`bottleneck_corridor`, pre-existing in the nav graph) does its job correctly —
robots queue and take turns with no negotiation failures or unresolved tasks.

**Caveat on cross-scenario comparison**: because `tb3_robot1` never wins a bid in this route set (see
note above), the corridor's real contention is between only **2 robots** (`tb3_robot2` /
`tb3_robot3`), not the full N=3 fleet. Head-on, Crossing, and Shared Lane all have genuine
participation from all 3 robots. So while 1c=197.6s is the lowest overhead of the 4 N=3 scenarios,
this should be read as "2-robot same-direction queueing costs almost nothing over baseline" rather
than a direct, fully-matched comparison against the other 3 scenarios' true 3-robot contention — the
gap may be partly due to lighter effective load, not only traffic-pattern difficulty. See Clear
Path's `RESULTS.md` for the full N=1-vs-N=3 comparison across all 4 scenarios.

---

*Data source: `run_20260821_0847_N3/task_planning_metrics.json` and `repeats_summary.json` (computed
from `bag/` via `analyze_task_planning_concurrent.py` — full procedure in `PROCEDURE.md` section 4).*
