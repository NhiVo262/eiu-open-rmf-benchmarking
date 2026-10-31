# Benchmark procedure — Feature 2: Traffic scheduling

Computing the F2 metrics from a collected N=3 run. Scenario routes, spawn
coordinates and the collection procedure itself are in
[collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §F2. This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 2. Traffic scheduling (2a conflict-free scheduling rate, 2b schedule propagation latency) |
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

| Topic | Carries |
|---|---|
| `/rmf_traffic/itinerary_set` | each committed itinerary, with participant and version |
| `/rmf_traffic/negotiation_notice` | one per conflict, with `participants[]` |
| `/rmf_traffic/negotiation_conclusion` | the conflict outcome, with `resolved` |
| `/rmf_traffic/query_update_1` | the mirror update that reflects a committed itinerary |
| `/task_api_requests` | task submissions, the denominator of the feasibility verdict |

`run_simulation_scenarios.sh` runs this feature's per-repeat analyser and the
per-run merge itself, so the steps below are needed only to recompute an existing
run.

---

## 3. Steps

Run inside the `open-rmf` container, once per repeat:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/simulation/scenarios/<scenario_dir>/<run_dir>

python3 $BENCHMARK/analyse/traffic_scheduling/analyze_traffic_scheduling.py \
  --bag "$RUN/repeat_N/bag" \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --scenario "<Scenario>, N=3, CONFIG_01" \
  --output "$RUN/repeat_N/traffic_scheduling_metrics.json"
```

After all five repeats, merge them into one file per run:

```bash
python3 $BENCHMARK/analyse/traffic_scheduling/merge_repeat_metrics.py \
  --run-dir "$RUN" \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --scenario "<Scenario>, N=3, CONFIG_01" \
  --output "$RUN/traffic_scheduling_metrics.json"
```

To check the reconstructed schedule against the recorded itineraries of one
repeat:

```bash
python3 $BENCHMARK/analyse/traffic_scheduling/analyze_traffic_scheduling_reconstruction.py \
  --repeat-dir "$RUN/repeat_N" \
  --output "$RUN/repeat_N/schedule_reconstruction.json"
```

---

## 4. Output

`repeat_N/traffic_scheduling_metrics.json` per repeat, and
`traffic_scheduling_metrics.json` at the run root after the merge.

The feasibility verdict is carried separately from 2a as
`2a_schedule_feasibility_pct`, and `per_conflict[]` carries a per-episode outcome
tag, since the verdict derives from it.

Feasibility classification:

```
Conflict?
  No  -> feasible
  Yes -> negotiation
           -> resolved            -> feasible
           -> forfeited/abandoned -> a participant established a new valid
                                     itinerary within the recovery window?
                                       Yes -> feasible (replan recovery)
                                       No  -> infeasible
```

A task is infeasible only if it was the task active on a robot (per
`/fleet_states`) that participated in a conflict ending unresolved **and**
unrecovered. The recovery window is 5 s; on the current collection the only
abandoned conflict recovered in 0.01 s, so any window from 0.1 s to 30 s gives
the same verdict.

Negotiation outcome counts are not reported here. They belong to F3, which
measures negotiation events and latency.

---

## 5. Insufficient data

A `MirrorUpdate` carrying `is_remedial_update` is excluded from the 2b sample
rather than counted as a slow propagation.

An itinerary with no matching `query_update_1` in the recording contributes to
neither numerator nor denominator of 2b, and is counted in the output so the
sample size stays visible.

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md),
generated from the per-repeat metrics by
`../summary/show_collection_summary.py`. They are not repeated here, so that one
copy cannot drift from another.

Per-repeat metrics files are the only place a figure is stored. Pooled
per-scenario figures are generated on demand and are not kept on disk.
