# Hardware Data Collection Procedure

Platform: ENV_02 — physical TurtleBot3 fleet.
Sequence: N=1 baselines, followed by N=3 scenarios.
Target: five retained repeats per route or scenario.

## 1. Preparation

1. Mark the nine navigation waypoints on the floor. Position each robot with its wheel axis on the specified waypoint and orient it toward the indicated next waypoint. See [SCENARIO_DESIGN.md](../../SCENARIO_DESIGN.md) for routes and initial poses.
2. Deploy the same navigation map to all robots (`~/turtlebot3_ws/maps/`). Verify waypoint and lane reachability using `tour_waypoints.py`.
3. Configure distinct robot ROS domains (4, 5, 6), `rmw_cyclonedds_cpp`, and the CycloneDDS configuration on both ends of the bridges.
4. Confirm robot bring-up, Nav2 lifecycle (`bt_navigator`: active), required topics, and `navigate_to_pose` action availability.
5. Verify battery levels against `HW_MIN_SOC`, which both runners apply.

Build the fleet package in the workstation container:

```bash
docker exec -it open-rmf bash
cd ~/rmf_ws
colcon build --packages-select tb3_fleet --symlink-install
source install/setup.bash
```

## 2. Startup Sequence

1. Start robot bring-up and Nav2 on each turtlebot3; verify `map -> odom` and `odom -> base_footprint` transforms.
2. Start the selected collection runner on the workstation.
3. Once the runner reports that bridge routes are ready, start or restart `zenoh-bridge` on the participating robots.
4. Follow the runner's placement prompt and park each robot on the printed waypoint and heading. Both runners seed AMCL themselves; no initial pose is set by hand.
5. Confirm placement and press Enter to begin the repeat.

The runners perform connectivity, localisation, battery and recording checks before accepting a repeat. At N=1 the seeded pose is compared against the placement and rejected beyond `MAX_SEED_XY_ERR_M` or `MAX_SEED_YAW_ERR_RAD`.

## 3. N=1 Baseline Collection

Collect all 12 matched routes, five repeats each, with one robot:

```bash
cd src/benchmark/collect/hardware
./run_hardware_baselines.sh bottleneck_r1,bottleneck_r2,bottleneck_r3
./run_hardware_baselines.sh crossing_r1,crossing_r2,crossing_r3
./run_hardware_baselines.sh headon_r1,headon_r2,headon_r3
./run_hardware_baselines.sh sharedlane_r1,sharedlane_r2,sharedlane_r3
```

For a single-repeat check:

```bash
TARGET_REPEATS=1 ./run_hardware_baselines.sh crossing_r2
```

Position each robot at its route's start waypoint, as defined in SCENARIO_DESIGN.md.

## 4. N=3 Scenario Collection

Collect five repeats for each scenario with three robots. Hardware collection without an independent position reference is declared with `HW_NO_GROUND_TRUTH=1`:

```bash
export HW_NO_GROUND_TRUTH=1
cd src/benchmark/collect/hardware
./run_hardware_scenarios.sh "Bottleneck"
./run_hardware_scenarios.sh "Crossing"
./run_hardware_scenarios.sh "Head-on"
./run_hardware_scenarios.sh "Shared Lane"
```

## 5. Repeat Acceptance and Outputs

Each runner starts the RMF stack, records the task execution, closes and verifies the rosbag, and generates per-repeat metrics.

| Collection | Required retained data |
|---|---|
| N=1 | Task state, fleet state, tf, tf_static, amcl_pose, cmd_vel, battery state, task-planning metrics |
| N=3 | N=1 signals plus scan, plan, traffic/negotiation topics, and coordination checks |

Resume an interrupted collection:

```bash
HW_RESUME=1 ./run_hardware_scenarios.sh "Bottleneck"
# or
HW_RESUME=1 ./run_hardware_baselines.sh bottleneck_r1,bottleneck_r2,bottleneck_r3
```

## 6. Post-Collection Verification

Verify the number of retained repeats, readable bags, task outcomes, and generated metric files. Run collection summaries as required:

```bash
python3 src/benchmark/analyse/summary/show_baseline_durations.py \
  --tag bottleneck_r1 --tag bottleneck_r2 --tag bottleneck_r3

python3 src/benchmark/analyse/summary/show_collection_summary.py \
  --scenario "Bottleneck=<run_dir>"
```

Ground-truth-dependent analyses are not applicable to hardware collections recorded with `HW_NO_GROUND_TRUTH=1`. Metric definitions and statistical summaries are documented in [METRIC_DESIGN.md](../../METRIC_DESIGN.md), [RESULTS.md](../../RESULTS.md), and [APPENDIX.md](../../APPENDIX.md).

## 7. Collection Parameters

| Parameter | Default | Scope |
|---|---|---|
| `TARGET_REPEATS` | 5 | both |
| `MAX_ATTEMPTS` | 8 | both |
| `HW_RESUME` | 0 | both |
| `HW_OPERATOR_TIMEOUT` | 900 s | both |
| `HW_ROS_DOMAIN_ID` | 12 | workstation |
| `HW_MIN_SOC` | 0.50 | both |
| `AMCL_SETTLE_S` | 30 s | both |
| `SEED_TIMEOUT_S` | 30 s | both |
| `MAX_SEED_XY_ERR_M` | 0.50 | N=1 |
| `MAX_SEED_YAW_ERR_RAD` | 0.35 | N=1 |
| `BIDDING_WINDOW_S` | 5.0 s | N=3 |
| `HW_NO_GROUND_TRUTH` | 0 | N=3; set 1 if no external tracker |