# Benchmark procedure - Feature 5: Execution fidelity

Computing the F5 metrics from collected runs. Scenario routes, spawn coordinates
and the collection procedure itself are in
[collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md) and
[collect/hardware/PROCEDURE.md](../../collect/hardware/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §6 (F5). This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 5. Execution fidelity (5a relative duration error, 5b finish-time offset) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| ENV_ID | ENV_01 — Gazebo; ENV_02 — physical TurtleBot3 fleet (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| CONFIG_ID | CONFIG_01 |
| Repeats | 5 per scenario on each platform |

| Metric | Compares |
|---|---|
| F5a — Relative duration error | `100 × (D_actual − D_planned) / D_planned`, both durations excluding dispatcher queueing |
| F5b — Finish-time offset | `t_completed − t_estimated_finish`, a difference of two instants, which includes queueing |

The same per-task analysis runs on the N=1 baselines where those recordings
exist. No separate collection pass is run for this feature.

---

## 2. Input

**Dataset**: `../../data/<platform>/scenarios/<scenario_dir>/run_*`, for
`platform` in `hardware` and `simulation`. The bag and the task-planning metrics
sit in each `repeat_N`; the API-state capture sits at run level.

**Signals read**:

| Signal | Carries |
|---|---|
| `/rmf_task/bid_response` | `proposal.finish_time` and `proposal.expected_robot_name` of the winning bid |
| `/clock` | the (wall, sim) pairs that convert a predicted finish instant to wall time |
| `api_state_attempt<N>.jsonl` | the task's terminal transition, the actual finish instant |
| `repeat_N/task_planning_metrics.json` | each task's robot, `t_award` and `queueing_s` |

**Timing semantics**: `proposal.finish_time` is an absolute `rmf_traffic::Time`
instant on the simulation-clock timeline, not a duration. F5b therefore needs a
conversion onto the wall-time timeline the terminal state is observed on. The
conversion reads the `/clock` samples bracketing the instant, rather than a
single offset sampled at bid time.

`/task_summaries`, which the metric table names, has no publisher in this
deployment; the analysis reads the API-state transitions instead. A run supports
complete F5 analysis only if its bag carries `/clock` and its API-state capture
is present. A bag with no `/clock` was recorded against the system clock, where
the conversion is the identity.

---

## 3. Steps

### Step 1 — Analyse each repeat

Run inside the `open-rmf` container, once per repeat. The API-state attempt
number does not follow the repeat number: take it from `api_state_path` in that
repeat's `task_planning_metrics.json`.

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/simulation/scenarios/bottleneck/run_20261020_0838
REPEAT=$RUN/repeat_1
API=$(python3 -c "import json,os,sys; print(os.path.basename(
  json.load(open(sys.argv[1]))['api_state_path']))" "$REPEAT/task_planning_metrics.json")

python3 $BENCHMARK/analyse/execution_fidelity/analyze_execution_fidelity.py \
  --task-planning-metrics "$REPEAT/task_planning_metrics.json" \
  --bag "$REPEAT/bag" \
  --api-state "$RUN/$API" \
  --scenario "Bottleneck, N=3, CONFIG_01, repeat 1" \
  --output "$REPEAT/execution_fidelity_metrics.json"
```

**Method, per task**:

1. Take the winning `bid_response`, matched by `task_id` **and**
   `proposal.expected_robot_name == task['robot']`. Losing bids appear on the
   same topic and must be excluded.
2. `finish_sim = proposal.finish_time`, in sim-clock seconds.
3. Find the `/clock` samples bracketing `finish_sim` **by their sim value** and
   interpolate the corresponding wall time. This is a direct lookup, not an
   offset: `finish_time` is a future instant and the sim clock falls behind the
   wall clock under three-robot contention, so an offset sampled at bid time is
   already stale by the predicted finish.
4. Read the actual finish from the task's terminal transition in the API-state
   capture. A robot's stop time is not a substitute: it records that the robot
   stopped, whether it arrived or gave up.
5. `D_planned = predicted_wall_finish − t_bid_response`;
   `D_actual = t_completed − t_award`. Each duration starts when its own clock
   starts, so queueing falls outside both.

### Step 2 — Merge the repeats

```bash
python3 $BENCHMARK/analyse/execution_fidelity/merge_repeat_metrics.py \
  --run-dir "$RUN" \
  --scenario "Bottleneck, N=3, CONFIG_01" \
  --output "$RUN/execution_fidelity_metrics.json"
```

The merge pools the per-task values across repeats; it does not average
per-repeat means. A task whose `queueing_s` exceeds `QUEUEING_OUTLIER_S` = 400 s
is excluded and listed under `excluded_tasks` with its figures, so the exclusion
is visible rather than silent.

Repeat both steps per scenario and per platform.

---

## 4. Output

| File | Holds |
|---|---|
| `repeat_N/execution_fidelity_metrics.json` | per-task F5 figures and the timing diagnostics |
| `<run_dir>/execution_fidelity_metrics.json` | the merged run-level figures |

| Key | Holds |
|---|---|
| `5a_duration_error_pct` | F5a, per task and summarised |
| `5a_delta_actual_minus_predicted_s` | **F5b.** The key carries a `5a_` prefix for historical reasons; the metric is F5b |
| `5a_delta_v1_bid_offset_s`, `drift_correction_s` | the same offset computed with a constant bid-time offset, and the difference, so the clock conversion is auditable |
| `clock_extrapolated` | set when the predicted finish fell outside the recorded `/clock` range |
| `n_tasks_with_delta`, `n_tasks_no_terminal_state` | how many tasks produced a figure, and how many did not |

The actual finish is parsed exactly as
`analyze_fleet_coordination_4a.py` parses it, so 4a and 5a cannot disagree about
what a task did.

---

## 5. Insufficient data

| Condition | Treatment |
|---|---|
| No `--api-state` | No delta is computed. `t_end_physical` is not substituted |
| Task reached no terminal state | No figure; counted in `n_tasks_no_terminal_state` |
| No `bid_response` matching the task and its robot | `status: no_matching_bid_response`; no figure is built from a losing bid or another robot's proposal |
| Predicted finish outside the `/clock` range | Extrapolated at the locally measured rate and flagged `clock_extrapolated`; the merge counts these in `n_clock_extrapolated` |
| `queueing_s` above 400 s | Excluded by the merge and listed in `excluded_tasks` |

An unavailable figure is null, never zero, and the scored-task counts accompany
every aggregate.

**Interpretation limit**: F5 measures the agreement between a bid-time estimate
and the observed execution. A large error does not by itself say whether the
cause is the duration model, fleet coordination, Nav2 execution or localisation.

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md) §6 and
[APPENDIX.md](../../APPENDIX.md), generated from the per-repeat metrics by
[`../summary/show_collection_summary.py`](../summary/show_collection_summary.py).
They are not repeated here, so that one copy cannot drift from another.

Before reporting, verify that the expected repeats are represented, that each
task matched a winning bid, that the required terminal transitions are present,
and that any `clock_extrapolated` or excluded tasks remain identifiable.
