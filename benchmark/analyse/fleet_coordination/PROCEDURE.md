# Benchmark procedure - Feature 4: Fleet coordination

Computing the F4 metrics from the collected N=3 runs and their route-matched N=1
baselines. Scenario routes, spawn coordinates and the collection procedure itself
are in [collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md)
and [collect/hardware/PROCEDURE.md](../../collect/hardware/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §5 (F4). This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 4. Fleet coordination (4a task completion, 4b robot utilisation, throughput and scaling) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane (N=3); the twelve route-matched baselines (N=1) |
| ENV_ID | ENV_01 — Gazebo; ENV_02 — physical TurtleBot3 fleet (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| CONFIG_ID | CONFIG_01 |
| Repeats | 5 per scenario and 5 per baseline route, on each platform |

---

## 2. Input

**Dataset**: `../../data/<platform>/scenarios/<scenario_dir>/run_*` and
`../../data/<platform>/baselines/<scenario>_r<N>/run_*`, for `platform` in
`hardware` and `simulation`: 4 scenarios × 5 repeats and 12 baseline routes × 5
repeats per platform, one bag per repeat, one run per directory. No separate
collection pass is run for this feature; the signals it reads are recorded by
`collect/<platform>/run_<platform>_scenarios.sh` and
`collect/<platform>/run_<platform>_baselines.sh` into the same bags every other
feature reads.

**Signals read**:

| Signal | Carries | Used by |
|---|---|---|
| `api_state_attempt<N>.jsonl` | the task state stream over `rmf_websocket`, `queued → standby → underway → completed` | 4a |
| `task_planning_metrics.json` | each task's robot, `t_award` and `t_completed` | 4b window, throughput |
| `/<robot>/cmd_vel` | the command driving each robot, published by the Nav2 controller server at `controller_frequency`, 10 Hz | 4b |

`/dispatch_state`, which the metric table names for 4a, does not exist in this
deployment. `RobotState.mode` is `MODE_IDLE` in every sample, so it separates no
robot state from another; `METRIC_DESIGN.md` §10 has the measurement scope.

`record_api_state.py` must have captured the run: 4a reads the terminal task
transition from `api_state_attempt<N>.jsonl` and computes no completion figure
without it.

---

## 3. Steps

Run inside the `open-rmf` container. Per repeat, on every scenario repeat and every
baseline repeat:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/<platform>/<scenarios|baselines>/<dir>/<run_dir>

python3 $BENCHMARK/analyse/fleet_coordination/analyze_fleet_coordination_4a.py \
  --repeat-dir "$RUN/repeat_N" \
  --run-dir "$RUN" \
  --scenario "<label>" \
  --output "$RUN/repeat_N/fleet_coordination_4a_metrics.json"

python3 $BENCHMARK/analyse/fleet_coordination/analyze_fleet_coordination_4b.py \
  --task-planning-metrics "$RUN/repeat_N/task_planning_metrics.json" \
  --bag "$RUN/repeat_N/bag" \
  --scenario "<label>" \
  --output "$RUN/repeat_N/fleet_coordination_metrics.json"
```

Then once per platform, after both collections are analysed:

```bash
python3 $BENCHMARK/analyse/fleet_coordination/analyze_fleet_scaling.py \
  --platform-dir $BENCHMARK/data/<platform> \
  --output $BENCHMARK/data/<platform>/fleet_scaling_metrics.json
```

`analyze_fleet_scaling.py` pairs each scenario with the baselines named after it,
`<scenario>_r1` to `_r3`, and pools tasks and time as totals.

---

## 4. Output

`repeat_N/fleet_coordination_4a_metrics.json` and `repeat_N/fleet_coordination_metrics.json`
per repeat; `fleet_scaling_metrics.json` per platform. The per-repeat 4b file carries the
window, each robot's moving time and utilisation, the number of robots scored and the
repeat's throughput. The platform file carries, per scenario, Th(N), Th(1), the scaling,
its workload ceiling, the fleet's utilisation and its routes' utilisation alone.

4a reports `status == "completed"` against tasks submitted, and reports a
non-completed task by its terminal state rather than pooling it.

---

## 5. Insufficient data

A repeat with no `api_state_attempt<N>.jsonl` yields no 4a figure; the analyser
reports the absence rather than inferring completion from motion.

A repeat with a task that never completed has no execution window, and its 4b and
throughput are null.

A robot whose `cmd_vel` holds no message in the bag has no utilisation: it is reported as
null and left out of the mean, and `4b_n_robots_scored` says how many robots the mean
covers.

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md) §5 and
[APPENDIX.md](../../APPENDIX.md), generated from the per-repeat metrics by
`../summary/show_collection_summary.py`. They are not repeated here, so that one
copy cannot drift from another.

Per-repeat metrics files are the only place a figure is stored. Pooled
per-scenario figures are generated on demand and are not kept on disk.
