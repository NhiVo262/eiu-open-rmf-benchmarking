# Benchmark procedure — Feature 5: Execution fidelity

Computing the F5 metrics from a collected N=3 run. Scenario routes, spawn
coordinates and the collection procedure itself are in
[collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §F5. This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 5. Execution fidelity (5a duration error, 5b finish-time offset) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
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

| Signal | Carries |
|---|---|
| `/rmf_task/bid_response` | `proposal.finish_time`, a sim-clock `rmf_traffic::Time`, and `proposal.expected_robot_name` |
| `/clock` | the (wall, sim) pair at 100 Hz, which converts `finish_time` to wall time |
| `api_state_attempt<N>.jsonl` | the task's terminal transition, the actual finish instant |
| `task_planning_metrics.json` | the per-task award and arrival instants for the same repeat |

`/task_summaries`, which the metric table names, has no publisher in this
deployment. `finish_time` is an instant on the sim clock, not a duration;
confirmed from `rmf_task_ros2`'s `bidding/Response.hpp` and `.cpp` and
`rmf_fleet_adapter`'s `FleetUpdateHandle.cpp`.

The repeat's bag must carry `/clock`, and the run must carry an API state
capture. Both are in the current collection's topic and recorder list.

---

## 3. Steps

Run inside the `open-rmf` container, once per repeat:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/simulation/scenarios/<scenario_dir>/<run_dir>

python3 $BENCHMARK/analyse/execution_fidelity/analyze_execution_fidelity.py \
  --task-planning-metrics "$RUN/repeat_N/task_planning_metrics.json" \
  --bag "$RUN/repeat_N/bag" \
  --api-state "$RUN/api_state_attempt<N>.jsonl" \
  --scenario "<Scenario>, N=3, CONFIG_01" \
  --output "$RUN/repeat_N/execution_fidelity_metrics.json"
```

After all five repeats:

```bash
python3 $BENCHMARK/analyse/execution_fidelity/merge_repeat_metrics.py \
  --run-dir "$RUN" \
  --scenario "<Scenario>, N=3, CONFIG_01" \
  --output "$RUN/execution_fidelity_metrics.json"
```

**Method, per task**:

1. Take the winning robot's `bid_response`, matched by `task_id` **and**
   `proposal.expected_robot_name == task['robot']`. Losing bids appear on the
   same topic and must be excluded.
2. `finish_sim = proposal.finish_time`, in sim-clock seconds.
3. Find the `/clock` samples bracketing `finish_sim` **by their sim value** and
   read off the corresponding wall time. This is a direct lookup, not an offset:
   `finish_time` is a future instant, and the sim clock falls behind the wall
   clock by 1–1.5% under three-robot contention, so an offset sampled at bid time
   is stale by the drift accumulated before the predicted finish.
4. `5b = t(terminal state) − predicted_wall_finish`, and
   `5a = (D_actual − D_planned) / D_planned`, with both durations excluding the
   dispatcher's queueing.

Both the sim and the wall value are retained per task, in
`5a_delta_actual_minus_predicted_s` and `5a_delta_v1_bid_offset_s`, so the size
of the conversion is auditable.

---

## 4. Output

`repeat_N/execution_fidelity_metrics.json` per repeat, and
`execution_fidelity_metrics.json` at the run root after the merge.

The actual finish comes from the task's own terminal transition, the same parse
`analyze_fleet_coordination_4a.py` uses, so 4a and 5a cannot disagree about what
a task did.

---

## 5. Insufficient data

`--api-state` is required. Without it the actual finish instant is unavailable
and the analyser computes no delta rather than falling back to `t_end_physical`,
which records that the robot stopped whether it arrived or gave up.

A task that reached no terminal state produces no figure and is counted in a
separate column.

A task whose `finish_sim` falls outside the recorded `/clock` range is
extrapolated at the locally measured rate and flagged `clock_extrapolated`.

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md),
generated from the per-repeat metrics by
`../summary/show_collection_summary.py`. They are not repeated here, so that one
copy cannot drift from another.

Per-repeat metrics files are the only place a figure is stored. Pooled
per-scenario figures are generated on demand and are not kept on disk.
