# Benchmark Procedure — Feature 1: Task Planning
## Route baselines: matched single-robot (N=1) runs for all 12 routes

---

## 1. What this is

The report compares each route's Actual (N=3, under traffic) makespan against that *same* route run alone (N=1, no traffic) — its "baseline". This folder holds those 12 baseline runs: every route used across Bottleneck/Crossing/Head-on/Shared Lane, run solo, 5 repeats each.

Unlike the other 4 scenario folders, there is **no multi-terminal bring-up here** — one script (`run_clearpath_route.sh`) does everything for one route end-to-end: launch sim, seed AMCL, start the fleet adapter, record the bag, submit 5 repeats, run the analyzer, tear down. So reproducing all 12 routes is 12 calls to that one script, shown as a copy-paste loop in section 3.

| Item | Value |
|---|---|
| Feature | 1. Task planning — makespan baseline only (1c) |
| FLEET_ID | N = 1 (`tb3_robot1` only) |
| Repeats | 5 per route |
| Routes | 12 (3 per scenario × 4 scenarios) |

---

## 2. The 12 routes

Spawn coordinates are the same canonical values used in the four official N=3 PROCEDURE.md files — **not** the rounded values in the report's Coordinates table.

| Scenario | Route folder | Places (`place1 place2`) | Spawn (`x_pose y_pose`) |
|---|---|---|---|
| bottleneck | `bottleneck1_bottleneck3` | `bottleneck_1 bottleneck_3` | `10.498 -6.565` |
| bottleneck | `sharedlane3_loop1` | `sharedlane_3 loop_1` | `10.454 -8.209` |
| bottleneck | `bottleneck3_sharedlane3` | `bottleneck_3 sharedlane_3` | `10.498 -6.565` |
| crossing | `charger1_bottleneck1` | `charger_1 bottleneck_1` | `5.368 -6.654` |
| crossing | `crossing1_loop4` | `crossing_1 loop_4` | `10.498 -6.565` |
| crossing | `crossing2_bottleneck3` | `crossing_2 bottleneck_3` | `10.454 -8.209` |
| headon | `charger1_bottleneck3` | `charger_1 bottleneck_3` | `5.368 -6.654` |
| headon | `crossing2_bottleneck1` | `crossing_2 bottleneck_1` | `10.454 -8.209` |
| headon | `crossing1_bottleneck3` | `crossing_1 bottleneck_3` | `10.498 -6.565` |
| sharedlane | `sharedlane1_sharedlane3` | `sharedlane_1 sharedlane_3` | `10.498 -6.565` |
| sharedlane | `sharedlane3_sharedlane1` | `sharedlane_3 sharedlane_1` | `10.454 -8.209` |
| sharedlane | `charger1_sharedlane2` | `charger_1 sharedlane_2` | `5.368 -6.654` |

---

## 3. Run all 12 — copy-paste loop

```bash
cd ~/rmf_ws/src/benchmark/task_planning/scripts

# scenario  route_folder              x_pose   y_pose   place1        place2
ROUTES=(
  "bottleneck bottleneck1_bottleneck3   10.498 -6.565  bottleneck_1 bottleneck_3"
  "bottleneck sharedlane3_loop1         10.454 -8.209  sharedlane_3 loop_1"
  "bottleneck bottleneck3_sharedlane3   10.498 -6.565  bottleneck_3 sharedlane_3"
  "crossing   charger1_bottleneck1       5.368 -6.654  charger_1    bottleneck_1"
  "crossing   crossing1_loop4           10.498 -6.565  crossing_1   loop_4"
  "crossing   crossing2_bottleneck3     10.454 -8.209  crossing_2   bottleneck_3"
  "headon     charger1_bottleneck3       5.368 -6.654  charger_1    bottleneck_3"
  "headon     crossing2_bottleneck1     10.454 -8.209  crossing_2   bottleneck_1"
  "headon     crossing1_bottleneck3     10.498 -6.565  crossing_1   bottleneck_3"
  "sharedlane sharedlane1_sharedlane3   10.498 -6.565  sharedlane_1 sharedlane_3"
  "sharedlane sharedlane3_sharedlane1   10.454 -8.209  sharedlane_3 sharedlane_1"
  "sharedlane charger1_sharedlane2       5.368 -6.654  charger_1    sharedlane_2"
)

for r in "${ROUTES[@]}"; do
  bash run_clearpath_route.sh $r 300 5   # 300=fixed-wait(s), 5=min-expected-distance(m)
done
```

One row = one route = one full run of 5 repeats (`--repeats 5` is hardcoded inside the script). To redo a single route, copy its row out and run that one line alone.

`fixed-wait 300` and `min-expected-distance-m 5` are the same convention as the four official N=3 PROCEDURE.md files, not a logged historical value (none was recorded — see note below). Both are safe against the data already collected: the longest of the 60 recorded repeats took 227.6s (< 300s), and the shortest distance was 6.69m (> 5m), so neither threshold would have reclassified any recorded run.

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

Baseline (N=1) makespan per route: see `RESULTS.md` in this folder, or the report's "Makespan by route" appendix table (full n/mean/stdev/min/max) and "Over Baseline by Route" chart (each route's baseline compared to its own Actual/N=3 makespan).

Raw data per route: `<scenario>/<route_folder>/run_<timestamp>/bag/`, `task_planning_metrics.json`, `run_benchmark.log`.
