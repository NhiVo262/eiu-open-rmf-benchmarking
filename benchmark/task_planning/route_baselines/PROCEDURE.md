# Benchmark Procedure — Feature 1: Task Planning
## Route baselines: matched single-robot (N=1) runs for all 12 routes

---

## 1. Objective

| Item | Description |
|---|---|
| Feature | 1. Task planning — makespan baseline only (1c) |
| Purpose | For each of the 12 routes used across Bottleneck/Crossing/Head-on/Shared Lane, run that exact route alone (no traffic) to get its own matched baseline. Compared against the Actual (N=3) makespan of the same route — see `Makespan by route` in the report. |
| Method | Clear-Path-style replication: same methodology as the Clear Path scenario (1 robot, no contention), applied to every other scenario's routes, one robot at a time — per reviewer feedback: "Run [Clear Path] over the same routes the other scenarios use, one robot at a time." |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
| FLEET_ID | N = 1 (`tb3_robot1` only) |
| Repeats | 5 per route |
| Routes | 12 (3 per scenario × 4 scenarios) |

---

## 2. Routes, spawn points, and folder layout

Each route is run and stored independently under `<scenario>/<route_slug>/run_<timestamp>/`. Spawn coordinates below are the same canonical values used in the four official N=3 PROCEDURE.md files (`bottleneck_FLEET0N_TRAF03_CONFIG01`, `crossing_FLEET02_TRAF01_CONFIG01`, `headon_FLEET0N_TRAF04_CONFIG01`, `sharedlane_FLEET0N_TRAF02_CONFIG01`), not the rounded values in the report's Coordinates table.

| Scenario | Route folder | Route (places) | Spawn vertex | Spawn (x, y) |
|---|---|---|---|---|
| bottleneck | `bottleneck1_bottleneck3` | `bottleneck_1 → bottleneck_3` | crossing_1 | (10.498, -6.565) |
| bottleneck | `sharedlane3_loop1` | `sharedlane_3 → loop_1` | crossing_2 | (10.454, -8.209) |
| bottleneck | `bottleneck3_sharedlane3` | `bottleneck_3 → sharedlane_3` | crossing_1 | (10.498, -6.565) |
| crossing | `charger1_bottleneck1` | `charger_1 → bottleneck_1` | charger_1 | (5.368, -6.654) |
| crossing | `crossing1_loop4` | `crossing_1 → loop_4` | crossing_1 | (10.498, -6.565) |
| crossing | `crossing2_bottleneck3` | `crossing_2 → bottleneck_3` | crossing_2 | (10.454, -8.209) |
| headon | `charger1_bottleneck3` | `charger_1 → bottleneck_3` | charger_1 | (5.368, -6.654) |
| headon | `crossing2_bottleneck1` | `crossing_2 → bottleneck_1` | crossing_2 | (10.454, -8.209) |
| headon | `crossing1_bottleneck3` | `crossing_1 → bottleneck_3` | crossing_1 | (10.498, -6.565) |
| sharedlane | `sharedlane1_sharedlane3` | `sharedlane_1 → sharedlane_3` | crossing_1 | (10.498, -6.565) |
| sharedlane | `sharedlane3_sharedlane1` | `sharedlane_3 → sharedlane_1` | crossing_2 | (10.454, -8.209) |
| sharedlane | `charger1_sharedlane2` | `charger_1 → sharedlane_2` | charger_1 | (5.368, -6.654) |

---

## 3. Reproduction command

One invocation of `run_clearpath_route.sh` runs all 5 repeats for one route (unlike the N=3 scripts, where each repeat is a separate invocation).

```bash
bash ~/rmf_ws/src/benchmark/task_planning/scripts/run_clearpath_route.sh \
  <scenario_slug> <route_slug> <x_pose> <y_pose> <place1> <place2> <fixed_wait> <min_dist>
```

Example — Bottleneck's `bottleneck_1 → bottleneck_3`:

```bash
bash ~/rmf_ws/src/benchmark/task_planning/scripts/run_clearpath_route.sh \
  bottleneck bottleneck1_bottleneck3 10.498 -6.565 bottleneck_1 bottleneck_3 300 5
```

| Parameter | Value | Note |
|---|---|---|
| fixed-wait | 300s | Same convention as the four official N=3 PROCEDURE.md files. All 60 recorded repeats (12 routes × 5) completed in ≤227.6s, well under 300s — reproducing with this value does not change any recorded outcome. |
| min-expected-distance-m | 5 | Same convention as the official PROCEDURE.md files. The shortest distance recorded across all 60 repeats was 6.69m, so this threshold does not reclassify any recorded run. |
| repeats | 5 | Hardcoded inside `run_clearpath_route.sh` |
| rounds | 1 | Default (9th positional arg, omitted above) |

The script handles the full lifecycle per route: bring up Gazebo+Nav2, seed AMCL at the spawn pose, start the zenoh bridge, launch RMF core + fleet adapter, wait for robot registration, record the bag, run `run_benchmark.py`, then `analyze_task_planning.py`, and tear everything down — so each route is a clean, independent process (no state carried over from the previous route).

To reproduce all 12 routes, call the command in section 3 once per row of the table in section 2.

---

## 4. One-time setup (same as the other scenarios)

```bash
xhost +local: root
cd ~/eiu_ws/src/scripts && docker compose up -d
docker exec -it open-rmf bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb3_fleet
```

---

## 5. Official results

Baseline (N=1) makespan per route — see the report's "Makespan by route" appendix table for the full n/mean/stdev/min/max, and "Over Baseline by Route" for the comparison against each route's own Actual (N=3) makespan.

Raw data per route: `<scenario>/<route_slug>/run_<timestamp>/bag/`, `task_planning_metrics.json`, `run_benchmark.log`.
