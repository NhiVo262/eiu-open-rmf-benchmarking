# Benchmark procedure — Feature 7: Safety and collision avoidance

Computing the F7 metrics from a collected N=3 run. Scenario routes, spawn
coordinates and the collection procedure itself are in
[collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §F7. This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 7. Safety and collision avoidance (7a minimum inter-robot separation, 7b near-collision episodes) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| ENV_ID | ENV_01 — Gazebo fleet, for 7a, 7b and 7c; ENV_02 — physical fleet, for 7c (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| CONFIG_ID | CONFIG_01 |
| Repeats | 5 per scenario |

---

## 2. Input

**Dataset**: `../../data/<platform>/scenarios/<scenario_dir>/run_*`, for `platform`
in `hardware` and `simulation`, 4 scenarios × 5 repeats per platform, one bag per
repeat. Each scenario directory holds one run. No separate collection pass is run
for this feature; the signals it reads are recorded by
`collect/<platform>/run_<platform>_scenarios.sh` into the same bags every other
feature reads.

**Signals read**:

| Signal | Carries | Reported as |
|---|---|---|
| `ground_truth_poses.jsonl` | Gazebo's own state per robot, the position reference, simulation only | the ground-truth check of 7c; the analyser's ground-truth 7a and 7b are written but not reported |
| `/fleet_states` | `RobotState.location`, the position RMF holds, the requirement's data source | 7a and 7b as specified, on both platforms |
| `/scan` | the lidar returns, for the laser-centre separation | 7c, on both platforms |

7a and 7b are reported from `/fleet_states`, the requirement's data source, although
its rationale is physical safety (`METRIC_DESIGN.md` §F7, R1). `METRIC_DESIGN.md` §2 has the inequality that bounds
their difference. The as-specified figures do not describe physical proximity by
themselves: the reported pose can move smoothly while being wrong.

**Threshold.** `d_safe = 2 × footprint radius = 0.356 m`, with the footprint
radius 0.178 m from
`tb3_fleet/config/simulation/fleet/multi_robot_config_sim.yaml`. This is the
distance at which the two bodies would touch, consistent with the robot SDF's
collision geometry (`base_collision`, a 0.14 × 0.14 × 0.14 m box). RMF's own
conflict separation, footprint + vicinity = 0.628 m, is also reported; crossing
it is a coordination event rather than a safety one.

---

## 3. Steps

Run inside the `open-rmf` container, with `RUN` under `data/<platform>/scenarios/`. 7a and
7b, once per repeat on either platform; the ground-truth figures are added where the
repeat holds `ground_truth_poses.jsonl`:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/<platform>/scenarios/<scenario_dir>/<run_dir>

python3 $BENCHMARK/analyse/safety/analyze_safety_tracker.py \
  --repeat-dir "$RUN/repeat_N" \
  --scenario "<Scenario>, N=3, CONFIG_01, <PLATFORM>, repeat N" \
  --output "$RUN/repeat_N/safety_tracker_metrics.json"
```

7c, once per repeat on either platform:

```bash
python3 $BENCHMARK/analyse/safety/analyze_safety_lidar.py \
  --bag "$RUN/repeat_N/bag" \
  --scenario "<Scenario>, N=3, CONFIG_01, <PLATFORM>, repeat N" \
  --output "$RUN/repeat_N/safety_lidar_metrics.json"
```

`analyze_safety_lidar.py --self-test` checks 7c's arithmetic on synthetic scans and
exits non-zero on a failure.

The checks on 7c, once per collection, each taking one `--run-dir` per scenario run:

```bash
# simulation only: every 7c match against ground truth
python3 $BENCHMARK/analyse/validity/analyze_lidar_attribution.py \
  --run-dir <run_dir> [--run-dir ...] \
  --output $BENCHMARK/data/simulation/lidar_attribution_metrics.json

# either platform: head presence and corroboration by /fleet_states
python3 $BENCHMARK/analyse/validity/analyze_lidar_corroboration.py \
  --run-dir <run_dir> [--run-dir ...] \
  --output $BENCHMARK/diagnostics/<platform>/lidar_corroboration_metrics.json
```

Per repeat: 7a is the pairwise Euclidean minimum over every frame and every
robot pair; 7b counts distinct events below each threshold, with consecutive
under-threshold frames counted as one event, and reports the frame count beside it. Across a scenario, 7a is the
minimum over the five repeats and 7b is the sum.

---

## 4. Output

`repeat_N/safety_tracker_metrics.json` per repeat on either platform,
`repeat_N/safety_lidar_metrics.json` per repeat on either platform, and the two
collection-level check files above.

7a is written from both position sources. 7b is written as episode counts, not
sample counts: counting samples would make the result proportional to the
recording rate.

---

## 5. Insufficient data

A repeat without `ground_truth_poses.jsonl`, every hardware repeat, yields the
as-specified 7a and 7b only; the output records `ground_truth: null` and the reason. 7c
needs `/scan` from at least two robots and exits otherwise.

F7 is not gated by the position-error validity criterion, because it reads the
position reference rather than an RMF decision
(`SYSTEM_UNDER_TEST.md` §4).

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md),
generated from the per-repeat metrics by
`../summary/show_collection_summary.py`. They are not repeated here, so that one
copy cannot drift from another.

Per-repeat metrics files are the only place a figure is stored. Pooled
per-scenario figures are generated on demand and are not kept on disk.
