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
| 12× matched N=1 baselines | `scripts/run_task_planning_baselines.sh` | 1 route/task, `--rounds 1 --repeats 1`, 5 repeats/route, spawn = winning robot's position in the matching N=3 run |

\* not runnable from this folder alone — see `scripts/` tables below.

## scripts/

**Analyze existing data / needed to run a benchmark** 

| File | Purpose |
|---|---|
| `analyze_task_planning_concurrent.py` | Computes 1a/1b/1c metrics from a bag (arrival-based makespan) |
| `check_negotiation_resolved.py` | Resolved/abandoned negotiation counts, cited in the report |
| `analyze_n3_scenarios.sh` | Computes `task_planning_metrics.json` for the 4 N=3 scenarios from the bags already sitting in this folder |
| `run_benchmark_concurrent.py` | Submits tasks — called by `run_task_planning_baselines.sh` / `run_task_planning_scenarios.sh`, not run directly |
| `run_benchmark.py` | Submits tasks — called by `run_clearpath.sh`, not run directly |

**Collect a new, independent dataset from scratch** 

| File | Purpose |
|---|---|
| `run_clearpath.sh` | Collects Clear Path (N=1, no contention) |
| `run_task_planning_baselines.sh` | Collects the 12 matched N=1 baselines |
| `run_task_planning_scenarios.sh` | How the 4 N=3 bags already in this folder were produced. **Not runnable here** — needs `traffic_scheduling/scripts/`, which isn't part of this commit. Kept for provenance; will move once Traffic Scheduling is committed (Milestone 3). |

## Reproducing the report's numbers (bags already present)

```bash
docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/task_planning/scripts/analyze_n3_scenarios.sh"
```
Then pool `1c_makespan_s` (or `per_task[].makespan_s`) across a dataset's
`repeat_*/task_planning_metrics.json` for the report's mean/stdev/min/max.

## Collecting a dataset from scratch

1. `docker ps --filter name=open-rmf` if not running, `docker start open-rmf`.
2. Run one script:
   ```bash
   docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/task_planning/scripts/run_clearpath.sh"
   docker exec open-rmf bash -c "/home/eiu/rmf_ws/src/benchmark/task_planning/scripts/run_task_planning_baselines.sh"
   ```
3. Check `<scenario>/run_<timestamp>/repeat_<n>/task_planning_metrics.json`.

