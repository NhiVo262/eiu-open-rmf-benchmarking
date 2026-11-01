# Benchmark procedure - Feature 3: Negotiation

Computing the F3 metrics from the collected N=3 runs. Scenario routes, spawn
coordinates and the collection procedure itself are in
[collect/simulation/PROCEDURE.md](../../collect/simulation/PROCEDURE.md) and
[collect/hardware/PROCEDURE.md](../../collect/hardware/PROCEDURE.md).

Metric definitions, formulas, observation units and thresholds are in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) §4 (F3). This document covers the
inputs, the steps, the outputs and the handling of insufficient data. It holds no
result figures.

---

## 1. Objective

| Item | Value |
|---|---|
| Feature | 3. Negotiation (3a negotiation events, 3b resolution latency by outcome) |
| Scenarios | Bottleneck, Crossing, Head-on, Shared Lane |
| ENV_ID | ENV_01 — Gazebo; ENV_02 — physical TurtleBot3 fleet (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| CONFIG_ID | CONFIG_01 |
| Repeats | 5 per scenario on each platform |

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
| `/rmf_traffic/negotiation_notice` | one per conflict, with `conflict_version` and `participants[]` |
| `/rmf_traffic/negotiation_conclusion` | the conflict outcome, with `resolved` |
| `/rmf_traffic/negotiation_refusal` | the conflict versions the schedule node refused, which split an abandoned negotiation from a failed search |

**Counting unit.** One event is one conflict episode, counted from
`negotiation_notice`. `negotiation_proposal` and `negotiation_forfeit` are recorded
in the bag but not read: they are steps inside an episode, and a single episode can
carry several of either, so counting them converts steps into events.
`negotiation_conclusion.resolved` is an outcome flag, not a counting unit.

`conflict_version` restarts at 0 in every repeat, because each repeat rebuilds
the schedule node. Counts are therefore summed across repeats and never
de-duplicated by version.

---

## 3. Steps

Run inside the `open-rmf` container, once per repeat:

```bash
BENCHMARK=/home/eiu/rmf_ws/src/benchmark
RUN=$BENCHMARK/data/simulation/scenarios/<scenario_dir>/<run_dir>

python3 $BENCHMARK/analyse/negotiation/analyze_negotiation.py \
  --bag "$RUN/repeat_N/bag" \
  --scenario "<Scenario>, N=3, CONFIG_01" \
  --output "$RUN/repeat_N/negotiation_metrics.json"
```

After all five repeats:

```bash
python3 $BENCHMARK/analyse/negotiation/merge_repeat_metrics.py \
  --run-dir "$RUN" \
  --scenario "<Scenario>, N=3, CONFIG_01" \
  --output "$RUN/negotiation_metrics.json"
```

---

## 4. Output

`repeat_N/negotiation_metrics.json` per repeat, and
`negotiation_metrics.json` at the run root after the merge.

The output carries notices, distinct `conflict_version`s and conclusions as three
separate counts, so a conflict that is noticed twice shows as a difference between
the first two rather than being absorbed into the headline number.

Latency is reported pooled and split by outcome: resolved, abandoned, and abandoned
split again into refused and search failed by whether a `negotiation_refusal` names the
conflict version. An instant abandonment therefore cannot read as fast coordination.

The merged file carries both denominators: `3b_n_latency_samples` is the number of
negotiations a latency was observed for, `3a_n_negotiation` the number that occurred, and
`3b_unconcluded` their difference. A notice that reached no conclusion has no latency and
enters none of the splits; it is not a latency of zero.

---

## 5. Insufficient data

Only a conflict carrying both a notice and a conclusion contributes to 3b. A
notice with no conclusion in the recording is counted in 3a and reported as
incomplete for 3b.

**Precision limit of the current start event.** 3b takes both instants as bag
arrival times. The constant part of the transport delay cancels in the subtraction;
the arrival jitter does not. Most of the distribution sits inside that jitter, and
values at that scale are reported as jitter-limited rather than as precise
measurements.

`NegotiationStatus.msg` (`/rmf_traffic/negotiation_statuses`, and nested inside
`/rmf_traffic/negotiation_states`) carries the schedule node's own `start_time`
and `last_response_time`, which are written in-message and are immune to
recording jitter. Both topics are in the collection script's topic list; a
collection that carries them supports a precise start event. **Not yet
implemented in the analyser.**

---

## 6. Results

Reported figures live in one place, [RESULTS.md](../../RESULTS.md) §4 and
[APPENDIX.md](../../APPENDIX.md), generated from the per-repeat metrics by
`../summary/show_collection_summary.py`. They are not repeated here, so that one
copy cannot drift from another.

Per-repeat metrics files are the only place a figure is stored. Pooled
per-scenario figures are generated on demand and are not kept on disk.
