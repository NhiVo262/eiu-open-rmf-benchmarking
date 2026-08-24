# Benchmark Procedure — Feature 1: Task Planning
## Scenario: Clear Path, Fleet size N = 1 (Phase 0 — Baseline)

---

## 1. Objective

| Item | Description |
|---|---|
| Feature | 1. Task planning (1a dispatch success rate, 1b planning latency, 1c makespan) |
| Scenario | Clear path — 1 robot, no other robot to cross paths with |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
| FLEET_ID | FLEET_01 — 1 robot (`tb3_robot1`) |
| TRAF_ID | TRAF_00 — no traffic interaction |
| CONFIG_ID | CONFIG_01 — 1 fleet adapter managing the whole fleet |
| Repeats | 5 |

---

## 2. Spawn position & robot route conventions

| Robot | Spawn position (map `world_tb3`) | Patrol route |
|---|---|---|
| `tb3_robot1` | `charger_1` — x=5.368, y=-6.654 | `charger_1 ↔ crossing_1` (1 direct lane, no intermediate stop) |

---

## 3. Task / route / repeat configuration

| Parameter | Value |
|---|---|
| Route | `charger_1, crossing_1` — patrol, dispatched through a real auction (no hard `-F/-R` assignment) |
| Rounds (patrol loops) / repeat | 5 |
| Repeats (independent full re-runs) | 5 |
| fixed-wait | 220s (measured baseline: 1 task of 5 rounds takes 103–130s; 220s leaves margin) |
| min-expected-distance-m | 25 (below this threshold → flagged `short_distance`, suspected stuck task) |

```bash
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark.py \
  --places charger_1 crossing_1 \
  --rounds 5 \
  --repeats 5 \
  --robot tb3_robot1 \
  --fixed-wait 220 \
  --min-expected-distance-m 25 \
  --output-dir "$RUN_DIR"
```

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

# 2. Terminal 2 — Simulation (Gazebo + Nav2)
ros2 launch tb3_fleet tb3_simulation_nav2.launch.py

# 3. Set the AMCL initial pose (after T2 is publishing /tf, before Nav2 activates)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 5.368, y: -6.654, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"

# 4. Terminal 3 — Zenoh bridge
cd ~/rmf_ws/src/tb3_fleet/config/zenoh
./zenoh-bridge-ros2dds -c tb3_zenoh_bridge_ros2dds_client_config.json5

# 5. Terminal 4 — RMF core + Fleet adapter
ros2 launch tb3_fleet tb3_world.launch.py

# 6. Pre-test check
ros2 topic echo /fleet_states --once

# 7. Record rosbag before assigning the task
RUN_DIR=~/rmf_ws/benchmark/task_planning/clearpath_FLEET01_TRAF00_CONFIG01/run_$(date -u +%Y%m%d_%H%M)
mkdir -p "$RUN_DIR" && cd "$RUN_DIR"
ros2 bag record -o bag \
  /rmf_task/bid_notice /rmf_task/bid_response \
  /rmf_task/dispatch_request /rmf_task/dispatch_ack \
  /task_api_requests /fleet_states \
  2>&1 | tee rosbag_record.log &

# 8. Run benchmark script
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark.py \
  --places charger_1 crossing_1 --rounds 5 --repeats 5 \
  --robot tb3_robot1 --fixed-wait 220 --min-expected-distance-m 25 \
  --output-dir "$RUN_DIR"

# 9. Stop the bag with SIGTERM (SIGINT does not close the .mcap file cleanly under nohup)
kill -TERM <pid_bag_record>
ros2 bag info "$RUN_DIR/bag"          # verify the bag is valid

# 10. Evaluate metrics from the rosbag
python3 ~/rmf_ws/benchmark/task_planning/scripts/analyze_task_planning.py \
  --bag "$RUN_DIR/bag" \
  --robot tb3_robot1 \
  --scenario "Clear path (baseline), N=1, TRAF_00, CONFIG_01" \
  --output "$RUN_DIR/task_planning_metrics.json"
```

---

## 5. Official results (`run_20260819_0800`)

| Metric | Target | Result |
|---|---|---|
| 1a. Task dispatch success rate | ≥ 95% | **100%** (5/5) |
| 1b. Planning latency | a few seconds | **1.80 ms** mean (min 0.60ms, max 2.54ms) |
| 1c. Makespan (N=1, baseline) | reference for other N | **195.8s ± 47.7s** (min 127.5s, max 250.5s) |

Raw data: `run_20260819_0800/bag/`, `task_planning_metrics.json`, `run_benchmark.log`, `rosbag_record.log`.
