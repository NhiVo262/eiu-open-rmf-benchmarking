# Benchmark Procedure — Feature 1: Task Planning
## Scenario: Bottleneck, Fleet size N = 3 (TRAF_03)

---

## 1. Objective

| Item | Description |
|---|---|
| Feature | 1. Task planning (1a dispatch success rate, 1b planning latency, 1c makespan) |
| Scenario | Bottleneck — multiple robots contending for one narrow corridor, only 1 robot at a time |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| TRAF_ID | TRAF_03 — bottleneck (narrow corridor, single-lane mutex) |
| CONFIG_ID | CONFIG_01 — 1 fleet adapter managing the whole fleet |
| Repeats | 5 |

---

## 2. Spawn position & robot route conventions

| Robot | Spawn position (map `world_tb3`) | Route |
|---|---|---|
| `tb3_robot1` | `charger_1` — x=5.368, y=-6.654 | *(does not win any route in this set — see section 3)* |
| `tb3_robot2` | `crossing_1` — x=10.498, y=-6.565 | `bottleneck_1 ↔ bottleneck_3` / `bottleneck_3 ↔ sharedlane_3` |
| `tb3_robot3` | `crossing_2` — x=10.454, y=-8.209 | `sharedlane_3 ↔ loop_1` |

3 routes are chosen so all of them must cross the mutex segment `bottleneck_1 ↔ bottleneck_3`, based
on the graph connectivity `sharedlane_3 <-> bottleneck_1 <-> bottleneck_3 <-> loop_1`:

```
--route bottleneck_1,bottleneck_3   # straight through the mutex segment, both directions
--route sharedlane_3,loop_1         # from the west side, MUST cross bottleneck_1↔bottleneck_3 to reach loop_1
--route bottleneck_3,sharedlane_3   # from the east side, crosses the mutex segment the opposite way
```
---

## 3. Task / route / repeat configuration

| Parameter | Value |
|---|---|
| Routes (submitted concurrently, 1 per repeat) | `bottleneck_1,bottleneck_3` / `sharedlane_3,loop_1` / `bottleneck_3,sharedlane_3` |
| Rounds (patrol loops) / repeat | 1 |
| Repeats (independent full re-runs) | 5 |
| fixed-wait | 300s (stop criterion per repeat) |
| min-expected-distance-m | 5 (below this threshold → flagged `short_distance`, suspected stuck task) |

```bash
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark_concurrent.py \
  --route bottleneck_1,bottleneck_3 \
  --route sharedlane_3,loop_1 \
  --route bottleneck_3,sharedlane_3 \
  --rounds 1 --repeats 5 \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --fixed-wait 300 --min-expected-distance-m 5 \
  --output-dir "$RUN_DIR"
```

> **Note — `tb3_robot1` never wins a bid**: all 3 routes sit on the east side of the map
> (`bottleneck`/`sharedlane_3`/`loop_1`, x≈25–29m), while `tb3_robot1` spawns at `charger_1` (x≈5.4m,
> farthest west). Its bid cost is higher than robot2/robot3 for every route in this set, so it loses
> all 15 bids across the run — a consequence of route design, not a bug (1a still reaches 100%). See
> `RESULTS.md` for the full breakdown.

---

## 4. Terminal step-by-step

```bash
# 0. One-time setup
xhost +local: root                                 # host: allow the container to render GUI
cd ~/eiu_ws/src/scripts && docker compose up -d
docker exec -it open-rmf bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb3_fleet

# 1. Terminal 1 — Zenoh router
zenohd

# 2. Terminal 2 — Simulation (Gazebo + Nav2, all 3 robots, headless)
ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py

# 3. Set the AMCL initial pose for each robot (after T2 is publishing /tf, before Nav2 activates)
ros2 topic pub --once /tb3_robot1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 5.368, y: -6.654, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"

ros2 topic pub --once /tb3_robot2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 10.498, y: -6.565, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"

ros2 topic pub --once /tb3_robot3/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 10.454, y: -8.209, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"

# 4. Terminal 3 — Zenoh bridge
cd ~/rmf_ws/src/tb3_fleet/config/zenoh
./zenoh-bridge-ros2dds -c tb3_multi_zenoh_bridge_ros2dds_client_config.json5

# 5. Terminal 4 — RMF core + Fleet adapter
ros2 launch tb3_fleet tb3_world.launch.py \
  fleet_config_file:=$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/config/fleet/tb3_multi_simulation_config.yaml \
  bidding_time_window:=60.0
# Wait for "Successfully added robot" x3 before continuing.

# 6. Pre-test check
ros2 topic echo /fleet_states --once

# 7. Record rosbag before submitting any task
RUN_DIR=~/rmf_ws/benchmark/task_planning/bottleneck_FLEET0N_TRAF03_CONFIG01/run_$(date -u +%Y%m%d_%H%M)_N3
mkdir -p "$RUN_DIR" && cd "$RUN_DIR"
ros2 bag record -o bag \
  /rmf_task/bid_notice /rmf_task/bid_response \
  /rmf_task/dispatch_request /rmf_task/dispatch_ack \
  /task_api_requests /fleet_states \
  /rmf_traffic/negotiation_notice /rmf_traffic/negotiation_proposal \
  /rmf_traffic/negotiation_conclusion /rmf_traffic/negotiation_rejection \
  /rmf_traffic/negotiation_forfeit \
  /rmf_traffic/blockade_set /rmf_traffic/blockade_ready \
  /rmf_traffic/blockade_reached /rmf_traffic/blockade_release \
  2>&1 | tee rosbag_record.log &

# 8. Run benchmark script (see section 3 for parameters)
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark_concurrent.py \
  --route bottleneck_1,bottleneck_3 \
  --route sharedlane_3,loop_1 \
  --route bottleneck_3,sharedlane_3 \
  --rounds 1 --repeats 5 \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --fixed-wait 300 --min-expected-distance-m 5 \
  --output-dir "$RUN_DIR"

# 9. Stop the rosbag
kill -INT <pid_bag_record>
ros2 bag reindex -s mcap "$RUN_DIR/bag"   # REQUIRED if the bag was killed with -9 instead
ros2 bag info "$RUN_DIR/bag"              # verify the bag is valid

# 10. Evaluate metrics from the rosbag
python3 ~/rmf_ws/benchmark/task_planning/scripts/analyze_task_planning_concurrent.py \
  --bag "$RUN_DIR/bag" \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --scenario "Bottleneck, N=3, TRAF_03, CONFIG_01" \
  --output "$RUN_DIR/task_planning_metrics.json"
```
---

## 5. Official results (`run_20260821_0847_N3`)

| Metric | Target | Result |
|---|---|---|
| 1a. Task dispatch success rate | ≥ 95% | **100%** (15/15) |
| 1b. Planning latency | a few seconds | **1.58 ms** mean (min 0.57ms, max 6.48ms) |
| 1c. Makespan | — | **197.6s ± 69.0s** (min 91.3s, max 336.0s, n=15, 0 unresolved) |

Full breakdown (per-robot makespan, robot1 non-participation note, interpretation) in `RESULTS.md`.

Raw data: `run_20260821_0847_N3/bag/`, `task_planning_metrics.json`, `repeats_summary.json`,
`benchmark_run.log`.
