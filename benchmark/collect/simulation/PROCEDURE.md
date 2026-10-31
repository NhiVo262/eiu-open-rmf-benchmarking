# Collection procedure — simulation (ENV_01: Gazebo fleet)

Collecting benchmark data from the Gazebo fleet. The F1 to F8 metrics are defined in
[METRIC_DESIGN.md](../../METRIC_DESIGN.md) and analysed per feature under
`analyse/<feature>/PROCEDURE.md`. Scenario routes and spawn coordinates are carried by
each runner's scenario table.

## Datasets

| Dataset | Runner | Protocol |
|---|---|---|
| 4× N=3 scenarios | `collect/simulation/run_simulation_scenarios.sh` | 3 concurrent tasks per repeat, 5 repeats |
| 12× matched N=1 baselines | `collect/simulation/run_simulation_baselines.sh` | 1 route per task, `--rounds 1 --repeats 1`, 5 repeats per route, spawn = winning robot's position in the matching N=3 run |

The data lands in `data/simulation/scenarios/` and `data/simulation/baselines/`.

## Scripts

Analysers, in `analyse/task_planning/`:

| File | Purpose |
|---|---|
| `analyze_task_planning.py` | Computes the 1a, 1b and 1c metrics from one bag and its API state capture, makespan measured from arrival to completion |
| `show_negotiation_resolved.py` | Resolved and abandoned negotiation counts |
| `analyze_task_planning_all_runs.py` | Re-runs the analyser over every repeat of one or more collections, `--collection-dir data/<platform>/<scenarios|baselines>`, taking each repeat's API capture and robots from its existing metrics |

Runners, in `collect/`:

| File | Purpose |
|---|---|
| `simulation/run_simulation_baselines.sh` | Collects the 12 matched N=1 baselines |
| `simulation/run_simulation_scenarios.sh` | Collects the 4 N=3 scenarios |
| `rmf/run_benchmark_concurrent.py` | Submits the tasks; called by the baselines and scenarios runners, not run directly |
| `rmf/run_benchmark.py` | Submits one task; called by `run_f4a_state_pilot.sh`, not run directly |
| `rmf/record_api_state.py` | Records the RMF websocket task-state stream for the whole run; called by the runners |
| `rmf/annotate_task_outcomes.py` | Joins a repeat's `repeats_summary.json` with its `task_planning_metrics.json`, so the recorded outcome states what each task did rather than what the runner could infer from release and distance |

## Reproducing the F1 numbers from the data present

Inside the `open-rmf` container, with the workspace sourced so the navigation graph is
found:

```bash
B=/home/eiu/rmf_ws/src/benchmark
python3 $B/analyse/task_planning/analyze_task_planning_all_runs.py \
  --collection-dir $B/data/simulation/scenarios \
  --collection-dir $B/data/simulation/baselines
```

It rewrites each repeat's `task_planning_metrics.json` in place and reports any repeat it
skipped. The F1 tables in `RESULTS.md` §A are computed from those files as
`METRIC_DESIGN.md` §F1 defines: dispersion per route, the batch figure per repeat.

## Collecting a dataset from scratch

1. `docker ps --filter name=open-rmf`; if the container is not running, `docker start open-rmf`.
2. Run one runner:
   ```bash
   docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/collect/simulation/run_simulation_baselines.sh"
   docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/collect/simulation/run_simulation_scenarios.sh"
   ```
3. Check `data/simulation/<collection>/<scenario>/run_<timestamp>_<suffix>/repeat_<n>/task_planning_metrics.json`.

