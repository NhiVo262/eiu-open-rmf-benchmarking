# Benchmark Procedure — Feature 1: Task Planning
## Route baselines: single-robot (N=1) runs, all 12 routes

---

## 1. Objective

| Item | Description |
|---|---|
| Feature | 1. Task planning (1c makespan baseline) |
| Scenario | Route baselines — each of the 12 routes from Bottleneck/Crossing/Head-on/Shared Lane, run alone (N=1) |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
| FLEET_ID | N = 1 (`tb3_robot1`) |
| CONFIG_ID | CONFIG_01 |
| Repeats | 5 per route |

---

## 2. Spawn position & route conventions

| Scenario | Route folder | Route (places) | Spawn vertex | Spawn position |
|---|---|---|---|---|
| Bottleneck | `bottleneck1_bottleneck3` | `bottleneck_1 → bottleneck_3` | crossing_1 | x=10.498, y=-6.565 |
| Bottleneck | `sharedlane3_loop1` | `sharedlane_3 → loop_1` | crossing_2 | x=10.454, y=-8.209 |
| Bottleneck | `bottleneck3_sharedlane3` | `bottleneck_3 → sharedlane_3` | crossing_1 | x=10.498, y=-6.565 |
| Crossing | `charger1_bottleneck1` | `charger_1 → bottleneck_1` | charger_1 | x=5.368, y=-6.654 |
| Crossing | `crossing1_loop4` | `crossing_1 → loop_4` | crossing_1 | x=10.498, y=-6.565 |
| Crossing | `crossing2_bottleneck3` | `crossing_2 → bottleneck_3` | crossing_2 | x=10.454, y=-8.209 |
| Head-on | `charger1_bottleneck3` | `charger_1 → bottleneck_3` | charger_1 | x=5.368, y=-6.654 |
| Head-on | `crossing2_bottleneck1` | `crossing_2 → bottleneck_1` | crossing_2 | x=10.454, y=-8.209 |
| Head-on | `crossing1_bottleneck3` | `crossing_1 → bottleneck_3` | crossing_1 | x=10.498, y=-6.565 |
| Shared Lane | `sharedlane1_sharedlane3` | `sharedlane_1 → sharedlane_3` | crossing_1 | x=10.498, y=-6.565 |
| Shared Lane | `sharedlane3_sharedlane1` | `sharedlane_3 → sharedlane_1` | crossing_2 | x=10.454, y=-8.209 |
| Shared Lane | `charger1_sharedlane2` | `charger_1 → sharedlane_2` | charger_1 | x=5.368, y=-6.654 |

---

## 3. Task / route / repeat configuration

| Parameter | Value |
|---|---|
| Rounds (patrol loops) / repeat | 1 |
| Repeats (independent full re-runs) | 5 |
| fixed-wait | 300s (stop criterion per repeat) |
| min-expected-distance-m | 5 (below this threshold → flagged `short_distance`) |


---

## 4. Terminal step-by-step

```bash
# 0. One-time setup
xhost +local: root
cd ~/eiu_ws/src/scripts && docker compose up -d
docker exec -it open-rmf bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb3_fleet

# 1. Run each route (each call handles bring-up, 5 repeats, analysis, teardown)
cd ~/rmf_ws/src/benchmark/task_planning/scripts

bash run_clearpath_route.sh bottleneck bottleneck1_bottleneck3 10.498 -6.565 bottleneck_1 bottleneck_3 300 5
bash run_clearpath_route.sh bottleneck sharedlane3_loop1       10.454 -8.209 sharedlane_3 loop_1        300 5
bash run_clearpath_route.sh bottleneck bottleneck3_sharedlane3 10.498 -6.565 bottleneck_3 sharedlane_3  300 5
bash run_clearpath_route.sh crossing   charger1_bottleneck1     5.368 -6.654 charger_1    bottleneck_1  300 5
bash run_clearpath_route.sh crossing   crossing1_loop4         10.498 -6.565 crossing_1   loop_4        300 5
bash run_clearpath_route.sh crossing   crossing2_bottleneck3   10.454 -8.209 crossing_2   bottleneck_3  300 5
bash run_clearpath_route.sh headon     charger1_bottleneck3     5.368 -6.654 charger_1    bottleneck_3  300 5
bash run_clearpath_route.sh headon     crossing2_bottleneck1   10.454 -8.209 crossing_2   bottleneck_1  300 5
bash run_clearpath_route.sh headon     crossing1_bottleneck3   10.498 -6.565 crossing_1   bottleneck_3  300 5
bash run_clearpath_route.sh sharedlane sharedlane1_sharedlane3 10.498 -6.565 sharedlane_1 sharedlane_3  300 5
bash run_clearpath_route.sh sharedlane sharedlane3_sharedlane1 10.454 -8.209 sharedlane_3 sharedlane_1  300 5
bash run_clearpath_route.sh sharedlane charger1_sharedlane2     5.368 -6.654 charger_1    sharedlane_2  300 5
```

---

## 5. Official results

Full n/mean/stdev/min/max per route in `RESULTS.md`.

Raw data: `<scenario>/<route_folder>/run_<timestamp>/bag/`, `task_planning_metrics.json`, `run_benchmark.log`.
