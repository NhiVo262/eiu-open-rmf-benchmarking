# Benchmark Procedure — Feature 1: Task Planning
## Scenario: Shared Lane, Fleet size N = 3 (TRAF_02)

---

## 1. Objective

| Item | Description |
|---|---|
| Feature | 1. Task planning (1a dispatch success rate, 1b planning latency, 1c makespan) |
| Scenario | Shared Lane — multiple robots sharing one long corridor, same or opposite direction |
| ENV_ID | ENV_01 — EIU indoor lab (map `world_tb3`) |
| FLEET_ID | N = 3 (`tb3_robot1`, `tb3_robot2`, `tb3_robot3`) |
| TRAF_ID | TRAF_02 — shared lane (shared corridor, not an intersection/mutex) |
| CONFIG_ID | CONFIG_01 — 1 fleet adapter managing the whole fleet |
| Repeats | 5 |

---

## 2. Spawn position & robot route conventions

| Robot | Spawn position (map `world_tb3`) | Route |
|---|---|---|
| `tb3_robot1` | `charger_1` — x=5.368, y=-6.654 | `charger_1 ↔ sharedlane_2` (enters mid-corridor) |
| `tb3_robot2` | `crossing_1` — x=10.498, y=-6.565 | `sharedlane_1 ↔ sharedlane_3` (full corridor, direction A) |
| `tb3_robot3` | `crossing_2` — x=10.454, y=-8.209 | `sharedlane_3 ↔ sharedlane_1` (full corridor, direction B) |

The two full-corridor routes (A/B) run in opposite directions on the same
`sharedlane_1 ↔ sharedlane_2 ↔ sharedlane_3` corridor to create genuine shared-lane contention. The
third route starts at `charger_1` (`tb3_robot1`'s home) instead of repeating a corridor-only route, so
all 3 robots have a chance to win bids instead of robot1 (spawned farthest from the corridor) always
losing on cost.

Robot-to-route assignment above reflects repeat 1's initial bid outcome, not a hard assignment — every
task is dispatched through a real auction (no `-F`/`-R` hard binding); a different robot can win a
given route in later repeats depending on its position at bid time.

---

## 3. Task / route / repeat configuration

| Parameter | Value |
|---|---|
| Routes (submitted concurrently, 1 per repeat) | `sharedlane_1,sharedlane_3` / `sharedlane_3,sharedlane_1` / `charger_1,sharedlane_2` |
| Rounds (patrol loops) / repeat | 1 |
| Repeats (independent full re-runs) | 5 |
| fixed-wait | 300s (stop criterion per repeat) |
| min-expected-distance-m | 5 (below this threshold → flagged `short_distance`, suspected stuck task) |

```bash
python3 ~/rmf_ws/benchmark/task_planning/scripts/run_benchmark_concurrent.py \
  --route sharedlane_1,sharedlane_3 \
  --route sharedlane_3,sharedlane_1 \
  --route charger_1,sharedlane_2 \
  --rounds 1 --repeats 5 \
  --robots tb3_robot1 tb3_robot2 tb3_robot3 \
  --fixed-wait 300 --min-expected-distance-m 5 \
  --output-dir "$RUN_DIR"
```

> **Note — bidding imbalance and makespan outliers**: `tb3_robot2` (spawned closest to the corridor)
> won most bids in the official run (11/15) because its cost (distance) was consistently lower than
> robot1/robot3 for routes repeated many times inside the corridor — a natural consequence of spawn
> position, not a bug (1a still reaches 100%). 2 tasks showed unusually high makespan (488.6s for
> robot1, 1231.9s for robot3) compared to the rest (~108–305s), reflecting the real time cost of
> negotiating/yielding on a shared corridor (`negotiation_forfeit=48`). 2 other tasks had no clear
> completion time detected via `/fleet_states` within the measurement window
> (`n_makespan_unresolved=2`), excluded from 1c. See `RESULTS.md` for the full breakdown.

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

# Repeat step 3 once more (2 passes total) — a single reseed pass is often not enough for AMCL to
# converge tightly before Nav2 lifecycle activation.

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
RUN_DIR=~/rmf_ws/benchmark/task_planning/sharedlane_FLEET0N_TRAF02_CONFIG01/run_$(date -u +%Y%m%d_%H%M)_N3
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
  --route sharedlane_1,sharedlane_3 \
  --route sharedlane_3,sharedlane_1 \
  --route charger_1,sharedlane_2 \
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
  --scenario "Shared Lane, N=3, TRAF_02, CONFIG_01" \
  --output "$RUN_DIR/task_planning_metrics.json"
```

**Infrastructure note**: as with all N≥2 scenarios, official data collection always runs **headless**
(no `use_gzclient`, no `use_rviz`). The GUI is only used for a separate demo-video recording, never as
a data source.

---

## 5. Official results (`run_20260821_0449_N3`)

| Metric | Target | Result |
|---|---|---|
| 1a. Task dispatch success rate | ≥ 95% | **100%** (15/15) |
| 1b. Planning latency | a few seconds | **1.35 ms** mean (min 0.58ms, max 3.28ms) |
| 1c. Makespan | — | **280.1s ± 303.6s** (min 108.2s, max 1231.9s, n=13, 2 unresolved) |

Full breakdown (per-robot makespan, negotiation counts, interpretation) in `RESULTS.md`.

Raw data: `run_20260821_0449_N3/bag/`, `task_planning_metrics.json`, `repeats_summary.json`,
`benchmark_run.log`.
