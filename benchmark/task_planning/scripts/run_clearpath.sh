#!/bin/bash

set -o pipefail

WS=/home/eiu/rmf_ws
TP_DIR=$WS/src/benchmark/task_planning
SCRIPTS=$TP_DIR/scripts
TS_SCRIPTS=$WS/src/benchmark/traffic_scheduling/scripts
NAV_GRAPH=$WS/install/tb3_fleet/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml
BASE_OUT=$TP_DIR/clearpath_FLEET01_TRAF00_CONFIG01
mkdir -p "$BASE_OUT"
MASTER_LOG=$BASE_OUT/official_run_$(date -u +%Y%m%d_%H%M).log

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$MASTER_LOG"; }

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

ROBOT=tb3_robot1
SX=5.368
SY=-6.654

kill_group() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null
    fi
}

teardown_scenario() {
    log "Tearing down"
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
    log "Teardown complete"
}

bring_up_common() {
    local RUN_DIR="$1" LOGSUFFIX="$2"
    setsid zenohd > "$RUN_DIR/zenohd${LOGSUFFIX}.log" 2>&1 &
    ZENOHD_PID=$!
    sleep 3

    setsid ros2 launch tb3_fleet tb3_simulation_nav2.launch.py \
        use_rviz:=False use_gzclient:=False x_pose:=$SX y_pose:=$SY \
        > "$RUN_DIR/simulation${LOGSUFFIX}.log" 2>&1 &
    SIM_PID=$!

    log "waiting for /tf..."
    local tf_i
    for tf_i in $(seq 1 45); do
        ros2 topic list 2>/dev/null | grep -q "^/tf$" && break
        sleep 2
    done

    # tb3_simulation_nav2.launch.py does not seed AMCL on its own -- explicit
    # /initialpose publish required (see run_task_planning_baselines.sh).
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: $SX, y: $SY, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}" \
        > "$RUN_DIR/initialpose${LOGSUFFIX}.log" 2>&1
    log "settling AMCL/Nav2 (60s)"
    sleep 60

    cd $WS/src/tb3_fleet/config/zenoh
    setsid ./zenoh-bridge-ros2dds -c tb3_zenoh_bridge_ros2dds_client_config.json5 \
        > "$RUN_DIR/zenoh_bridge${LOGSUFFIX}.log" 2>&1 &
    BRIDGE_PID=$!
    cd "$RUN_DIR"
    sleep 3
}

launch_fleet_bundle_and_wait() {
    local RUN_DIR="$1" LOGNAME="$2"
    FA_LOG="$RUN_DIR/$LOGNAME"
    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        > "$FA_LOG" 2>&1 &
    FA_PID=$!

    local reg_i
    for reg_i in $(seq 1 60); do
        if grep -q "Unable to compute a location on the navigation graph" "$FA_LOG" 2>/dev/null; then
            log "WEDGE DETECTED in $LOGNAME"
            return 1
        fi
        if grep -q "Successfully added robot" "$FA_LOG" 2>/dev/null; then
            log "robot registered after ${reg_i}x2s ($LOGNAME)"
            return 0
        fi
        sleep 2
    done
    log "TIMEOUT waiting for registration ($LOGNAME)"
    return 1
}

BAG_TOPICS="/rmf_task/bid_notice /rmf_task/bid_response \
/rmf_task/dispatch_request /rmf_task/dispatch_ack \
/task_api_requests /fleet_states"

RUN_DIR=$BASE_OUT/run_$(date -u +%Y%m%d_%H%M)_RESET_v2
mkdir -p "$RUN_DIR"
log "=== Clear Path v2: charger_1,crossing_1 spawn=($SX,$SY) -> $RUN_DIR ==="

completed=0
attempt=0
max_attempts=15
while [ "$completed" -lt 5 ] && [ "$attempt" -lt "$max_attempts" ]; do
    attempt=$((attempt+1))
    i=$((completed+1))
    log "repeat $i/5 (attempt $attempt): full teardown + bring-up"
    teardown_scenario
    bring_up_common "$RUN_DIR" "_attempt${attempt}"
    if ! launch_fleet_bundle_and_wait "$RUN_DIR" "fleet_adapter_attempt${attempt}.log"; then
        log "repeat $i/5 (attempt $attempt): fleet bundle failed -- STOPPING. $completed repeat(s) kept."
        break
    fi
    sleep 10

    ATTEMPT_DIR="$RUN_DIR/attempt_${attempt}"
    mkdir -p "$ATTEMPT_DIR"
    eval "setsid ros2 bag record -o \"$ATTEMPT_DIR/bag\" $BAG_TOPICS > \"$ATTEMPT_DIR/rosbag_record.log\" 2>&1 &"
    BAG_PID=$!
    sleep 3

    log "repeat $i/5 (attempt $attempt): submitting (rounds=5)"
    timeout 300 python3 $SCRIPTS/run_benchmark.py \
        --places charger_1 crossing_1 --rounds 5 --repeats 1 \
        --robot $ROBOT --fixed-wait 220 --min-expected-distance-m 25 \
        --output-dir "$ATTEMPT_DIR" \
        > "$RUN_DIR/run_benchmark_attempt${attempt}.log" 2>&1
    RC=$?
    log "repeat $i/5 (attempt $attempt): run_benchmark.py exited with code $RC"

    kill_group "$BAG_PID"
    sleep 5
    ros2 bag info "$ATTEMPT_DIR/bag" > "$ATTEMPT_DIR/bag_info.log" 2>&1

    if ! python3 $TS_SCRIPTS/check_physics_glitch.py \
            --bag "$ATTEMPT_DIR/bag" \
            --robots $ROBOT \
            > "$ATTEMPT_DIR/physics_check.log" 2>&1; then
        log "repeat $i/5 (attempt $attempt): PHYSICS GLITCH -- discarding, retry as repeat $i"
        mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_physics_glitch_attempt${attempt}"
        continue
    fi

    python3 $SCRIPTS/analyze_task_planning_concurrent.py \
        --bag "$ATTEMPT_DIR/bag" \
        --robots $ROBOT \
        --scenario "Clear path (baseline, reset-per-repeat), N=1, TRAF_00, CONFIG_01 repeat $i" \
        --nav-graph "$NAV_GRAPH" \
        --output "$ATTEMPT_DIR/task_planning_metrics.json" \
        > "$ATTEMPT_DIR/analyze.log" 2>&1
    log "repeat $i/5 (attempt $attempt): analyze exited with code $?"

    mv "$ATTEMPT_DIR" "$RUN_DIR/repeat_${i}"
    completed=$((completed+1))
done

log "$completed/5 repeats completed after $attempt attempt(s)"
teardown_scenario
log "=== Clear Path v2 DONE: $RUN_DIR ==="
