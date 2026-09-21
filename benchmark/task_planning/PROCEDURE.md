# Task Planning — collection procedure

## Methodology

- **Makespan** is measured from the assigned robot's arrival at the route's
  first waypoint to its last observed movement (not from task submission) —
  robots reset to a fixed spawn point between repeats, not to each route's
  own start, so measuring from submission would count that transit as part
  of makespan. Computed by `analyze_task_planning_concurrent.py
  --nav-graph <world_tb3 nav_graph>`.

## Datasets

All scripts live in `scripts/` (paths below are relative to this file's folder).

| Dataset | Script | Protocol |
|---|---|---|
| Clear Path (N=1) | `scripts/run_clearpath.sh` | 1 route, `--rounds 5 --repeats 1`, 5 repeats |
| 4× N=3 scenarios | `scripts/run_task_planning_scenarios.sh`* | 3 concurrent tasks/repeat, 5 repeats, official runs listed in README |

\* not runnable from this folder alone — see `scripts/` table below.
| 12× matched N=1 baselines | `scripts/run_task_planning_baselines.sh` | 1 route/task, `--rounds 1 --repeats 1`, 5 repeats/route, spawn = winning robot's position in the matching N=3 run |

## scripts/

| File | Purpose |
|---|---|
| `analyze_task_planning_concurrent.py` | Computes 1a/1b/1c metrics from a bag (arrival-based makespan) |
| `run_benchmark_concurrent.py` | Submits tasks for N=3 scenarios and the 12 matched baselines |
| `run_benchmark.py` | Submits tasks for Clear Path (`--rounds N` single-task protocol) |
| `check_negotiation_resolved.py` | Resolved/abandoned negotiation counts, cited in the report |
| `run_clearpath.sh`, `run_task_planning_baselines.sh` | Collect Clear Path / the 12 matched baselines from scratch (self-contained) |
| `analyze_n3_scenarios.sh` | Computes `task_planning_metrics.json` for the 4 N=3 scenarios from the bags already sitting in this folder

## Reproducing the report's numbers (bags already present)

```bash
docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/task_planning/scripts/analyze_n3_scenarios.sh"
```
Then pool `1c_makespan_s` (or `per_task[].makespan_s`) across a dataset's
`repeat_*/task_planning_metrics.json` for the report's mean/stdev/min/max.

## Collecting a dataset from scratch

1. `docker ps --filter name=open-rmf` — if not running, `docker start open-rmf`.
2. Run one script (don't run two at once — each takes exclusive control of Gazebo/Nav2):
   ```bash
   docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/task_planning/scripts/run_clearpath.sh"
   docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/task_planning/scripts/run_task_planning_baselines.sh"
   ```
3. Wait (~5 min/repeat; a full 12-baseline run is hours — launch it in the background).
4. Check `<scenario>/run_<timestamp>/repeat_<n>/task_planning_metrics.json`.

