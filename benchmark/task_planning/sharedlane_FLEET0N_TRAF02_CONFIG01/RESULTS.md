# Benchmark Results — Feature 1: Task Planning
## Scenario: Shared Lane, N = 3 (TRAF_02)

**Official run:** `run_20260821_0449_N3` (15/15 tasks dispatched, on map `world_tb3`;
see `PROCEDURE.md` section 1 for full conditions).

**Run conditions:** ENV_01 (map `world_tb3`) · FLEET_02 (3 robots
`tb3_robot1`/`tb3_robot2`/`tb3_robot3`) · TRAF_02 (shared corridor) · CONFIG_01 · Routes:
`sharedlane_1↔sharedlane_3`, `sharedlane_3↔sharedlane_1`, `charger_1↔sharedlane_2`, 1 round/repeat ·
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
| 1b | Planning latency | s | `bid_notice`, `bid_response` | `t_bid_response − t_bid_notice` | Within a few seconds | **mean 1.35 ms**, std 0.72ms, min 0.58ms, max 3.28ms | ✅ **Pass** (~1500x faster than target) |
| 1c | Makespan | s | `task_api_requests`, `fleet_states` | `t_end − t_start` | Compare to N=1 baseline (195.8s, Clear Path) | **mean 280.1s**, std 303.6s, min 108.2s, max 1231.9s (n=13, 2 unresolved) | ➖ N/A (comparative only, +43.1% vs. baseline) |

---

## Per-task detail

| Repeat | Route | Robot | Planning latency (ms) | Dispatch success | Makespan (s) |
|---|---|---|---|---|---|
| 1 | `sharedlane_1→sharedlane_3` | tb3_robot2 | 3.28 | ✅ | 113.52 |
| 1 | `sharedlane_3→sharedlane_1` | tb3_robot2 | 2.29 | ✅ | 158.02 |
| 1 | `charger_1→sharedlane_2` | tb3_robot1 | 1.48 | ✅ | 488.62 |
| 2 | `sharedlane_1→sharedlane_3` | tb3_robot2 | 0.83 | ✅ | 158.00 |
| 2 | `sharedlane_3→sharedlane_1` | tb3_robot2 | 1.89 | ✅ | 217.30 |
| 2 | `charger_1→sharedlane_2` | tb3_robot3 | 1.70 | ✅ | 1231.90 |
| 3 | `sharedlane_1→sharedlane_3` | tb3_robot2 | 0.61 | ✅ | 108.20 |
| 3 | `sharedlane_3→sharedlane_1` | tb3_robot2 | 1.00 | ✅ | 173.20 |
| 3 | `charger_1→sharedlane_2` | tb3_robot2 | 1.29 | ✅ | 304.90 |
| 4 | `sharedlane_1→sharedlane_3` | tb3_robot2 | 0.70 | ✅ | 114.00 |
| 4 | `sharedlane_3→sharedlane_1` | tb3_robot2 | 0.58 | ✅ | 174.30 |
| 4 | `charger_1→sharedlane_2` | *(unresolved)* | 1.00 | ✅ | — |
| 5 | `sharedlane_1→sharedlane_3` | tb3_robot2 | 1.01 | ✅ | 167.60 |
| 5 | `sharedlane_3→sharedlane_1` | tb3_robot2 | 1.17 | ✅ | 232.10 |
| 5 | `charger_1→sharedlane_2` | *(unresolved)* | 1.38 | ✅ | — |
| **Mean** | | | **1.35** | **100%** | **280.13** (n=13) |
| **Stdev** | | | **0.72** | | **303.62** |

Dispatch success (1a) is computed from `dispatch_ack`, independent of whether a completion time was
later found for 1c — all 15/15 tasks dispatched successfully even for the 2 rows with unresolved
makespan.

---

## Per-repeat outcome (distance travelled)

| Repeat | tb3_robot1 | tb3_robot2 | tb3_robot3 |
|---|---|---|---|
| 1 | 101.30 m (completed) | 24.30 m (completed) | 0.0 m (short_distance) |
| 2 | 11.48 m (completed) | 18.97 m (completed) | 0.0 m (short_distance) |
| 3 | 0.0 m (short_distance) | 59.11 m (completed) | 0.0 m (short_distance) |
| 4 | 0.0 m (short_distance) | 29.79 m (completed) | 24.77 m (completed) |
| 5 | 0.0 m (short_distance) | 28.58 m (completed) | 320.63 m (completed) |

`short_distance` (< 5m, see `PROCEDURE.md` section 3) does **not** mean dispatch failure — the task
was still dispatched and accepted (1a counts it as success); it means the robot moved less than the
threshold during that repeat's window, consistent with `tb3_robot2` winning most bids (see note below).

---

## Note on Makespan (1c) variance

Makespan varies far more than the N=1 baseline (108.2s–1231.9s, more than 11x) — substantially larger
than Clear Path's 127.5s–250.5s range (N=1, no traffic). Two effects combine here:

1. **Bidding imbalance**: `tb3_robot2` (spawned closest to the corridor) wins 11/15 tasks because its
   cost is consistently lower than robot1/robot3 on routes repeated inside the corridor — its
   makespans stay tight (108–305s, close to the N=1 baseline). Whenever robot1 or robot3 wins instead
   (repeat 1's `charger_1→sharedlane_2`, repeat 2's `charger_1→sharedlane_2`), makespan spikes sharply
   (488.6s, 1231.9s) — these 2 outliers alone explain most of the 303.6s stdev.
2. **Real negotiation cost**: `negotiation_forfeit=48` across the run (vs. 0 for Bottleneck at the same
   N=3) confirms genuine yielding/renegotiation happens on the shared corridor — the outliers are a
   robot losing multiple negotiation rounds before finally proceeding, not a stuck/broken task.

**2 tasks (repeat 4 and 5's `charger_1→sharedlane_2`) have no completion time resolved via
`/fleet_states`** within the recording window — the corresponding robot was still winning/executing a
route when the run ended; excluded from the 1c aggregate (n=13) per the analysis script's method, not
counted as a dispatch failure (1a unaffected).

Does not affect the 1a/1b conclusions (both clearly pass target). The wide 1c spread here — much wider
than Bottleneck at the same N=3 (197.6s ± 69.0s) — is itself the expected signature of Shared Lane's
opposite-direction contention, consistent with the report's traffic-difficulty ranking (see Clear
Path's `RESULTS.md` conclusion for the full cross-scenario comparison table).

---

## Conclusion

1a and 1b both pass target comfortably (100% dispatch success, ~1.35ms planning latency — negotiation
and route contention on the shared corridor have no measurable effect on the dispatch/auction layer
itself). 1c makespan (280.1s mean, +43.1% vs. the N=1 baseline) is the **highest of the 4 N=3
scenarios** and carries the largest variance, matching the expectation that opposite-direction
shared-corridor traffic is harder than same-direction (Bottleneck) or isolated-intersection (Crossing)
traffic. See Clear Path's `RESULTS.md` for the full N=1-vs-N=3 comparison across all 4 scenarios.

---

*Data source: `run_20260821_0449_N3/task_planning_metrics.json` and `repeats_summary.json` (computed
from `bag/` via `analyze_task_planning_concurrent.py` — full procedure in `PROCEDURE.md` section 4).*
