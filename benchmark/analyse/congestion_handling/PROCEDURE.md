# Benchmark procedure — Feature 6: Congestion handling

Computing the F6 metrics from a collected N=3 run. Scenario routes, spawn
coordinates and the collection procedure itself are in
[collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §F6. This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 6. Congestion handling (6a time blocked per robot, 6b waiting time per episode) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane (only Bottleneck and Head-on produce episodes — see §2) |
| ENV_ID | ENV_01 — Gazebo fleet; ENV_02 — physical fleet (map `world_tb3`) |
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

| Source | Carries |
|---|---|
| `/<robot>/cmd_vel` | the command driving each robot, about 20 Hz: the stationary classification |
| `task_planning_metrics.json` | each task's robot, route, `t_award` and `t_completed`: the assignment window and the route the N=1 comparison is matched on |

`RobotState.mode`, which the metric table names, is set only to IDLE, DOCKING and
CHARGING by this adapter, and `/mode_requests` is not published by it
(`METRIC_DESIGN.md` §1.5, §F6). The table's exclusion, waiting at doors and lifts, has
nothing to exclude: the graph has neither.

**Scope of the measurand.** F6 measures a robot that is stationary while it holds an
assignment. It does not decide, per interval, why the robot stopped; the waiting caused
by the other robots is the excess over the same route run alone.

---

## 3. Steps

Run inside the `open-rmf` container, once per repeat, on every scenario repeat and every
baseline repeat:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/<platform>/<scenarios|baselines>/<dir>/<run_dir>

python3 $BENCHMARK/analyse/congestion_handling/analyze_congestion_handling.py \
  --bag "$RUN/repeat_N/bag" \
  --task-planning-metrics "$RUN/repeat_N/task_planning_metrics.json" \
  --scenario "<label>" \
  --output "$RUN/repeat_N/congestion_handling_metrics.json"
```

The robots are the ones that carried a task, read from `--task-planning-metrics`.

---

## 4. Output

`repeat_N/congestion_handling_metrics.json` per repeat, `analysis_version` 2: per robot,
the route, the window, the time blocked, the number of episodes and the longest; per
repeat, the mean episode at the 0.3 s and the 1.0 s minimum and the episode counts.
6a sums a robot's episodes; 6b means over episodes.

---

## 5. Insufficient data

A robot whose `cmd_vel` holds no message in the bag has no figure: its time blocked is
null, with the reason, and `n_robots_scored` says how many robots the repeat covers. A
task without an award or a completion has no window and no figure.

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md),
generated from the per-repeat metrics by
`../summary/show_collection_summary.py`. They are not repeated here, so that one
copy cannot drift from another.

Per-repeat metrics files are the only place a figure is stored. Pooled
per-scenario figures are generated on demand and are not kept on disk.
