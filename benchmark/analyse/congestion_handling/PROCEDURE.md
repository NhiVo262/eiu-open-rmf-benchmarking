# Benchmark procedure - Feature 6: Congestion handling

Computing the F6 metrics from collected N=3 runs and their matched N=1 route
baselines. Scenario routes, spawn coordinates and the collection procedure
itself are in [collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md)
and [collect/hardware/PROCEDURE.md](../../collect/hardware/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §7 (F6). This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 6. Congestion handling (6a stationary duration, 6b episode duration) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| ENV_ID | ENV_01 — Gazebo; ENV_02 — physical TurtleBot3 fleet (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`), and matched N = 1 route baselines |
| CONFIG_ID | CONFIG_01 |
| Repeats | 5 per scenario and 5 per baseline route, on each platform |

All four scenarios are analysed. Whether a scenario produced qualifying
stationary episodes is an output, not an assumption.

---

## 2. Input

**Dataset**: `../../data/<platform>/scenarios/<scenario_dir>/run_*/repeat_N` and
`../../data/<platform>/baselines/<scenario>_r<N>/run_*/repeat_N`, for `platform`
in `hardware` and `simulation`. No separate collection pass is run for this
feature; the signals it reads are recorded by
`collect/<platform>/run_<platform>_scenarios.sh` and
`collect/<platform>/run_<platform>_baselines.sh` into the same bags every other
feature reads.

**Signals read**:

| Signal | Carries |
|---|---|
| `/<robot>/cmd_vel` | the command driving each robot; a stationary interval is one holding no motion command |
| `task_planning_metrics.json` | each task's robot, route, `t_award` and `t_completed`, which define the window and the route identity |

`cmd_vel` is published by the Nav2 controller server while it is executing a
goal, at `controller_frequency`, 10 Hz in `tb3_fleet/config/simulation/nav2_params.yaml`.
The analyser assumes no rate: a command is held for at most `STALE_S` = 0.25 s,
or until the next command, whichever is sooner.

Robot identities come from `task_planning_metrics.json`; only robots carrying an
assignment are analysed.

**Measurement scope**: F6 measures commanded stationary behaviour during an
assigned task and does not identify the cause of a stop. `RobotState.mode` is
`MODE_IDLE` in every sample this adapter publishes, so it separates no stop from
another, and the adapter publishes no `/mode_requests`. The test graph has no
doors or lifts to exclude. See [METRIC_DESIGN.md](../../METRIC_DESIGN.md) §10.

---

## 3. Steps

### Step 1 — Analyse each repeat

Run inside the `open-rmf` container, once per repeat, for the N=3 scenarios and
for the N=1 route baselines:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
REPEAT=$BENCHMARK/data/simulation/scenarios/bottleneck/run_20261020_0838/repeat_1

python3 $BENCHMARK/analyse/congestion_handling/analyze_congestion_handling.py \
  --bag "$REPEAT/bag" \
  --task-planning-metrics "$REPEAT/task_planning_metrics.json" \
  --scenario "Bottleneck, N=3, CONFIG_01" \
  --output "$REPEAT/congestion_handling_metrics.json"
```

`--scenario` is a free-text label copied into the output and used by nothing
else. The label stored in the current dataset is the repeat's path.

**Method, per task**:

1. Take the window `[t_award, t_completed]` from `task_planning_metrics.json`.
2. Build the robot's moving intervals from `/<robot>/cmd_vel`, using the same
   motion thresholds as F4b: `|v| >= 0.005` m/s or `|w| >= 0.05` rad/s, each
   command held for at most `STALE_S`.
3. Every remaining stretch of the window at least `MIN_EPISODE_S` = 0.3 s long
   is a stationary episode. The 1.0 s figures repeat the same step at
   `SENSITIVITY_EPISODE_S`.
4. 6a is the sum of a robot's episodes; 6b is the mean episode duration.

### Step 2 — Compare against the N=1 baselines

Additional stationary time is `F6a(N=3) − F6a(N=1)` on the same route and
platform, as [METRIC_DESIGN.md](../../METRIC_DESIGN.md) §7 defines it. The
baseline for route `r<N>` of a scenario is `baselines/<scenario>_r<N>`.

This difference is an observed change between two matched conditions. It is not
time proven to have been caused by another robot.

### Step 3 — Produce the summaries

```bash
python3 $BENCHMARK/analyse/summary/show_collection_summary.py \
  --scenario "Bottleneck=data/simulation/scenarios/bottleneck/run_20261020_0838" \
  --scenario "Crossing=data/simulation/scenarios/crossing/run_<...>" \
  --only F6
```

Check the repeat count and `n_robots_scored` before reading an aggregate.

---

## 4. Output

`repeat_N/congestion_handling_metrics.json` per repeat, carrying
`analysis_version: 2`:

| Key | Holds |
|---|---|
| `6a_per_robot` | per robot: route, `task_id`, window, stationary time, episode count, longest episode |
| `6a_time_blocked_per_robot_s` | the stationary time alone, per robot |
| `6b_mean_per_episode_s`, `6b_mean_per_episode_s_min_1s` | mean episode duration at each threshold |
| `n_episodes_total`, `n_episodes_total_min_1s` | episode counts at each threshold |
| `n_robots_scored` | how many assigned robots produced a figure |

An episode is not an independent repeat. Uncertainty across repeats is reported
over repeats, not over episodes.

---

## 5. Insufficient data

| Condition | Treatment |
|---|---|
| Valid window, no qualifying episode | A measured zero |
| No `cmd_vel` message for an assigned robot | `time_blocked_s` is null with `reason: no cmd_vel`; the robot is not counted in `n_robots_scored` |
| Missing `t_award` or `t_completed` | No window, `reason: no completed window`, no figure for that task |
| A gap in the recorded `cmd_vel` stream | Counted as stationary. The analyser reads the absence of a motion command, and does not separate a robot that was commanded to hold from a stream that stopped arriving |
| No N=1 baseline for a route | No additional-stationary-time figure for that route |

Zero and null are distinct and must stay distinct through aggregation.

**Interpretation limit**: `cmd_vel` is commanded motion, not measured movement.
A stationary interval can come from navigation, localisation, recovery or
traffic coordination, and F6 cannot separate them from these inputs alone.
`analyse/validity/analyze_motion_reference.py` compares this classifier against
Gazebo ground truth where ground truth exists.

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md) and
[APPENDIX.md](../../APPENDIX.md) table A9, generated from the per-repeat metrics
by `../summary/show_collection_summary.py`. 

Before reporting, verify that the platform, route, task windows, repeat count,
`n_robots_scored` and episode thresholds match the intended comparison.
