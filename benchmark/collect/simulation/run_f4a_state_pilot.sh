#!/bin/bash


set -o pipefail
WS=/home/eiu/rmf_ws
BENCHMARK_DIR=$WS/src/benchmark
COLLECT_RMF=$BENCHMARK_DIR/collect/rmf
TB3_FLEET_SHARE=$WS/install/tb3_fleet/share/tb3_fleet
ZENOH_BRIDGE=$TB3_FLEET_SHARE/bin/zenoh-bridge-ros2dds
ZENOH_BRIDGE_CONFIG=$TB3_FLEET_SHARE/config/simulation/zenoh_bridge/tb3_zenoh_bridge_single_robot_config_sim.json5
OUTPUT_DIR=$BENCHMARK_DIR/data/simulation/f4a_state_pilot
BASE_RUN_DIR=$OUTPUT_DIR/run_$(date -u +%Y%m%d_%H%M)_N1
mkdir -p "$BASE_RUN_DIR"
LOG=$BASE_RUN_DIR/validate.log
ROBOT=${ROBOT_NAME:-tb3_robot1}
SX=15.092
SY=-5.728   # wp_1
SCENARIO_FLEET_CONFIG=${SCENARIO_FLEET_CONFIG:-$TB3_FLEET_SHARE/config/simulation/fleet/multi_robot_config_sim.yaml}
SESSION_DIR=$BENCHMARK_DIR/.session/simulation
mkdir -p "$SESSION_DIR"
FLEET_CONFIG=$SESSION_DIR/fleet_config_${ROBOT}.yaml
python3 - "$SCENARIO_FLEET_CONFIG" "$ROBOT" "$FLEET_CONFIG" <<'EOPY' || exit 1
import sys
import yaml
src, robot, out = sys.argv[1:4]
cfg = yaml.safe_load(open(src))
robots = cfg['rmf_fleet']['robots']
if robot not in robots:
    raise SystemExit(f'[{robot}] is not declared in {src}')
cfg['rmf_fleet']['robots'] = {robot: robots[robot]}
with open(out, 'w') as f:
    yaml.safe_dump(cfg, f, sort_keys=False)
EOPY


log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$LOG"; }

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

kill_group() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null
    fi
}

teardown() {
    kill_group "$BAG_PID"; kill_group "$FA_PID"; kill_group "$BRIDGE_PID"; kill_group "$SIM_PID"; kill_group "$ZENOHD_PID"
    sleep 6
    kill_group "$BAG_PID"; kill_group "$FA_PID"; kill_group "$BRIDGE_PID"; kill_group "$SIM_PID"; kill_group "$ZENOHD_PID"
    sleep 2
    pkill -9 -f "tb3_world.launch.py" 2>/dev/null
    pkill -9 -f "tb3_fleet.tb3_fleet_adapter" 2>/dev/null
    pkill -9 -f "zenoh-bridge-ros2dds" 2>/dev/null
    pkill -9 -f "tb3_simulation_nav2.launch.py" 2>/dev/null
    pkill -9 -f "ros2 bag record" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "ruby.*gz" 2>/dev/null
    pkill -9 -f "nav2_" 2>/dev/null
    pkill -9 -f "rmf_traffic" 2>/dev/null
    pkill -9 -f "rmf_fleet_adapter" 2>/dev/null
    pkill -9 -f "rmf_task_dispatcher" 2>/dev/null
    pkill -9 -f "rmf_visualization" 2>/dev/null
    pkill -9 -f "building_map_server" 2>/dev/null
    pkill -9 -f "ros_gz_bridge" 2>/dev/null
    pkill -9 -f zenohd 2>/dev/null
    sleep 3
}

max_attempts=6
attempt=0
success=0
while [ "$attempt" -lt "$max_attempts" ]; do
    attempt=$((attempt+1))
    RUN_DIR="$BASE_RUN_DIR/attempt_$attempt"
    mkdir -p "$RUN_DIR"
    log "=== attempt $attempt/$max_attempts: full teardown+bringup ==="
    teardown

    log "1. zenoh router"
    setsid zenohd > "$RUN_DIR/zenohd.log" 2>&1 &
    ZENOHD_PID=$!
    sleep 3

    log "2. simulation (1 robot)"
    setsid ros2 launch tb3_fleet tb3_simulation_nav2.launch.py \
        use_rviz:=False use_gzclient:=False x_pose:=$SX y_pose:=$SY \
        > "$RUN_DIR/simulation.log" 2>&1 &
    SIM_PID=$!
    for i in $(seq 1 45); do

        timeout 10 ros2 topic list 2>/dev/null | grep -q "^/tf$" && break
        sleep 2
    done

    log "seeding AMCL initial pose"
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: $SX, y: $SY, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}" \
        > "$RUN_DIR/initialpose.log" 2>&1
    log "settling AMCL/Nav2 (60s)"
    sleep 60

    log "3. zenoh bridge"
    setsid "$ZENOH_BRIDGE" -c "$ZENOH_BRIDGE_CONFIG" \
        > "$RUN_DIR/zenoh_bridge.log" 2>&1 &
    BRIDGE_PID=$(pgrep -f zenoh-bridge-ros2dds | head -1)
    sleep 3

    log "4. RMF core + fleet adapter"
    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        fleet_config_file:="$FLEET_CONFIG" \
        > "$RUN_DIR/fleet_adapter.log" 2>&1 &
    FA_PID=$!
    ok=0
    for i in $(seq 1 60); do
        grep -q "Successfully added robot" "$RUN_DIR/fleet_adapter.log" 2>/dev/null && { ok=1; break; }
        sleep 2
    done
    if [ "$ok" -ne 1 ]; then
        log "attempt $attempt: fleet adapter never registered robot -- retrying"
        continue
    fi
    log "robot registered"
    sleep 5

    log "5. bag record (all topics)"
    setsid ros2 bag record -a -o "$RUN_DIR/bag" \
        > "$RUN_DIR/rosbag_record.log" 2>&1 &
    BAG_PID=$!
    sleep 3

    log "6. submit ONE task"
    python3 $COLLECT_RMF/run_benchmark.py \
        --places wp_1 wp_9 \
        --rounds 1 --repeats 1 \
        --robot tb3_robot1 \
        --fixed-wait 220 --min-expected-distance-m 3 \
        --output-dir "$RUN_DIR" \
        > "$RUN_DIR/run_benchmark.log" 2>&1
    log "task submission exited $?"

    log "7. stop bag"
    kill -TERM -- "-$BAG_PID" 2>/dev/null
    sleep 6
    kill -TERM -- "-$BAG_PID" 2>/dev/null
    sleep 3
    ros2 bag info "$RUN_DIR/bag" > "$RUN_DIR/bag_info.log" 2>&1

    DIST=$(python3 -c "import json; print(json.load(open('$RUN_DIR/repeats_summary.json'))[0]['total_distance_m'])" 2>/dev/null)
    log "attempt $attempt: total_distance_m=$DIST"
    if python3 -c "exit(0 if float('$DIST' or 0) > 1.0 else 1)" 2>/dev/null; then
        log "attempt $attempt: SUCCESS"
        success=1
        break
    else
        log "attempt $attempt: task did not move -- retrying"
    fi
done

teardown
log "DONE. success=$success final_run_dir=$RUN_DIR"
