# Benchmark Procedure — Feature 1: Task Planning
## Scenario: Crossing, Fleet size N = 3 (TRAF_01)

---

## 1. Objective

| Item | Description |
|---|---|
| Feature | 1. Task planning (1a dispatch success rate, 1b planning latency, 1c makespan) |
| Scenario | Crossing — multiple robots on routes that intersect at shared hub vertices |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| TRAF_ID | TRAF_01 — crossing traffic (real negotiation at intersections) |
| CONFIG_ID | CONFIG_01 — 1 fleet adapter managing the whole fleet |
| Repeats | 5 |

---

## 2. Spawn position & robot route conventions

| Robot | Spawn position (map `world_tb3`) | Route |
|---|---|---|
| `tb3_robot1` | `charger_1` — x=5.368, y=-6.654 | `charger_1 ↔ bottleneck_1` |
| `tb3_robot2` | `crossing_1` — x=10.498, y=-6.565 | `crossing_1 ↔ loop_4` |
| `tb3_robot3` | `crossing_2` — x=10.454, y=-8.209 | `crossing_2 ↔ bottleneck_3` |
---

## 3. Task / route / repeat configuration

| Parameter | Value |
|---|---|
| Routes (submitted concurrently, 1 per repeat) | `charger_1,bottleneck_1` / `crossing_1,loop_4` / `crossing_2,bottleneck_3` |
| Rounds (patrol loops) / repeat | 1 |
| Repeats (independent full re-runs) | 5 |
| fixed-wait | 300s (stop criterion per repeat) |
| min-expected-distance-m | 5 (below this threshold → flagged `short_distance`, suspected stuck task) |

```bash
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark_concurrent.py \
  --route charger_1,bottleneck_1 \
  --route crossing_1,loop_4 \
  --route crossing_2,bottleneck_3 \
  --rounds 1 --repeats 1 \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --fixed-wait 300 --min-expected-distance-m 5 \
  --output-dir "$RUN_DIR/repeat_N"
```

---

## 4. Terminal step-by-step

```bash
# 0. One-time setup
xhost +local: root                                 
cd ~/eiu_ws/src/scripts && docker compose up -d
docker exec -it open-rmf bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tb3_fleet

# 1. Terminal 1 — Zenoh router
zenohd

# 2. Terminal 2 — Simulation (Gazebo + Nav2, all 3 robots, headless)
ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py

# 3. Set the AMCL initial pose for each robot 
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

# 5. Terminal 4 — RMF core WITHOUT the fleet adapter (adapter is launched per-repeat in step 8)
ros2 launch tb3_fleet tb3_world.launch.py \
  fleet_config_file:=$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/config/fleet/tb3_multi_simulation_config.yaml \
  bidding_time_window:=60.0 \
  include_fleet_adapter:=false

# 6. Terminal 5 — Fleet adapter
python3 -m tb3_fleet.tb3_fleet_adapter \
  -c $(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/config/fleet/tb3_multi_simulation_config.yaml \
  -n $(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml \
  --zenoh-config $(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/config/zenoh/tb3_fleet_adapter_zenoh_config.json5
# Wait for "Successfully added robot" x3 before continuing.

# 7. Start the rosbag recorder
RUN_DIR=~/rmf_ws/benchmark/task_planning/crossing_FLEET02_TRAF01_CONFIG01/run_$(date -u +%Y%m%d_%H%M)_N3
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

# 8. For EACH of the 5 repeats:
#    a. (repeat 2 onward) kill + relaunch the fleet adapter fresh (repeat step 6's command).
#       If registration fails with "Unable to compute a location on the navigation graph" — a robot
#       is physically wedged — teleport all 3 robots back to their spawn coordinates
#       (`gz service .../set_pose`) and reseed AMCL (step 3) before retrying registration.
#    b. Submit exactly 1 repeat (see section 3 for the full command):
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark_concurrent.py \
  --route charger_1,bottleneck_1 \
  --route crossing_1,loop_4 \
  --route crossing_2,bottleneck_3 \
  --rounds 1 --repeats 1 \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --fixed-wait 300 --min-expected-distance-m 5 \
  --output-dir "$RUN_DIR/repeat_N"

# 9. After all 5 repeats: stop the bag, merge the 5 repeat_N/repeats_summary.json into one
#    repeats_summary.json at the top of $RUN_DIR.
kill -INT <pid_bag_record>
ros2 bag reindex -s mcap "$RUN_DIR/bag"   # REQUIRED if the bag was killed with -9 instead
ros2 bag info "$RUN_DIR/bag"              # verify the bag is valid

# 10. Evaluate metrics from the rosbag
python3 ~/rmf_ws/benchmark/task_planning/scripts/analyze_task_planning_concurrent.py \
  --bag "$RUN_DIR/bag" \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --scenario "Crossing, N=3, TRAF_01, CONFIG_01" \
  --output "$RUN_DIR/task_planning_metrics.json"
```
---

## 5. Official results (`run_20260824_0108_N3`)

| Metric | Target | Result |
|---|---|---|
| 1a. Task dispatch success rate | ≥ 95% | **100%** (15/15) |
| 1b. Planning latency | a few seconds | **4.06 ms** mean (min 2.02ms, max 6.21ms) |
| 1c. Makespan | — | **256.3s ± 69.6s** (min 141.6s, max 315.3s, n=12/15, 3 unresolved) |

Full breakdown (per-robot makespan, negotiation counts, interpretation) in `RESULTS.md`.

Raw data: `run_20260824_0108_N3/bag/`, `task_planning_metrics.json`, `benchmark_run.log`.
