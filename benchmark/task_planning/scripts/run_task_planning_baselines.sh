#!/bin/bash

set -o pipefail

WS=/home/eiu/rmf_ws
TP_DIR=$WS/src/benchmark/task_planning
SCRIPTS=$TP_DIR/scripts
TS_SCRIPTS=$WS/src/benchmark/traffic_scheduling/scripts
NAV_GRAPH=$WS/install/tb3_fleet/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml
BASE_OUT=$TP_DIR/baselines_matched
mkdir -p "$BASE_OUT"
MASTER_LOG=$BASE_OUT/official_run_$(date -u +%Y%m%d_%H%M).log

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$MASTER_LOG"; }

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

ROBOT=tb3_robot1

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
    local RUN_DIR="$1" LOGSUFFIX="$2" SX="$3" SY="$4"
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

run_baseline_route() {
    local SCEN_LABEL="$1" ROUTE="$2" SX="$3" SY="$4" DIRNAME="$5"
    local RUN_DIR="$BASE_OUT/$DIRNAME/run_$(date -u +%Y%m%d_%H%M)_N1"
    mkdir -p "$RUN_DIR"
    log "=== [$SCEN_LABEL] baseline for route [$ROUTE] spawn=($SX,$SY) -> $RUN_DIR ==="

    local completed=0 attempt=0 max_attempts=12
    while [ "$completed" -lt 5 ] && [ "$attempt" -lt "$max_attempts" ]; do
        attempt=$((attempt+1))
        local i=$((completed+1))
        log "[$SCEN_LABEL] repeat $i/5 (attempt $attempt): full teardown + bring-up"
        teardown_scenario
        bring_up_common "$RUN_DIR" "_attempt${attempt}" "$SX" "$SY"
        if ! launch_fleet_bundle_and_wait "$RUN_DIR" "fleet_adapter_attempt${attempt}.log"; then
            log "[$SCEN_LABEL] repeat $i/5 (attempt $attempt): fleet bundle failed -- STOPPING. $completed repeat(s) kept."
            break
        fi
        sleep 10

        local ATTEMPT_DIR="$RUN_DIR/attempt_${attempt}"
        mkdir -p "$ATTEMPT_DIR"
        eval "setsid ros2 bag record -o \"$ATTEMPT_DIR/bag\" $BAG_TOPICS > \"$ATTEMPT_DIR/rosbag_record.log\" 2>&1 &"
        BAG_PID=$!
        sleep 3

        log "[$SCEN_LABEL] repeat $i/5 (attempt $attempt): submitting"
        timeout 400 python3 $SCRIPTS/run_benchmark_concurrent.py \
            --route "$ROUTE" \
            --rounds 1 --repeats 1 \
            --robots $ROBOT \
            --fixed-wait 300 --min-expected-distance-m 2 \
            --output-dir "$ATTEMPT_DIR" \
            > "$RUN_DIR/run_benchmark_attempt${attempt}.log" 2>&1
        RC=$?
        log "[$SCEN_LABEL] repeat $i/5 (attempt $attempt): run_benchmark_concurrent.py exited with code $RC"

        kill_group "$BAG_PID"
        sleep 5
        ros2 bag info "$ATTEMPT_DIR/bag" > "$ATTEMPT_DIR/bag_info.log" 2>&1

        if ! python3 $TS_SCRIPTS/check_physics_glitch.py \
                --bag "$ATTEMPT_DIR/bag" \
                --robots $ROBOT \
                > "$ATTEMPT_DIR/physics_check.log" 2>&1; then
            log "[$SCEN_LABEL] repeat $i/5 (attempt $attempt): PHYSICS GLITCH -- discarding, retry as repeat $i"
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_physics_glitch_attempt${attempt}"
            continue
        fi

        python3 $SCRIPTS/analyze_task_planning_concurrent.py \
            --bag "$ATTEMPT_DIR/bag" \
            --robots $ROBOT \
            --scenario "$SCEN_LABEL baseline repeat $i" \
            --nav-graph "$NAV_GRAPH" \
            --output "$ATTEMPT_DIR/task_planning_metrics.json" \
            > "$ATTEMPT_DIR/analyze.log" 2>&1
        log "[$SCEN_LABEL] repeat $i/5 (attempt $attempt): analyze exited with code $?"

        mv "$ATTEMPT_DIR" "$RUN_DIR/repeat_${i}"
        completed=$((completed+1))
    done

    log "[$SCEN_LABEL] $completed/5 repeats completed after $attempt attempt(s)"
    teardown_scenario
    log "=== [$SCEN_LABEL] DONE: $RUN_DIR ==="
}

run_baseline_route "Bottleneck bottleneck_1-bottleneck_3" "bottleneck_1,bottleneck_3" 10.498 -6.565 "bottleneck_r1"
run_baseline_route "Bottleneck sharedlane_3-loop_1"       "sharedlane_3,loop_1"       10.454 -8.209 "bottleneck_r2"
run_baseline_route "Bottleneck bottleneck_3-sharedlane_3" "bottleneck_3,sharedlane_3" 10.498 -6.565 "bottleneck_r3"
run_baseline_route "Crossing charger_1-bottleneck_1"      "charger_1,bottleneck_1"    5.368  -6.654 "crossing_r1"
run_baseline_route "Crossing crossing_1-loop_4"           "crossing_1,loop_4"         10.498 -6.565 "crossing_r2"
run_baseline_route "Crossing crossing_2-bottleneck_3"     "crossing_2,bottleneck_3"   10.454 -8.209 "crossing_r3"
run_baseline_route "Headon charger_1-bottleneck_3"        "charger_1,bottleneck_3"    5.368  -6.654 "headon_r1"
run_baseline_route "Headon crossing_1-bottleneck_3"       "crossing_1,bottleneck_3"   10.498 -6.565 "headon_r2"
run_baseline_route "Headon crossing_2-bottleneck_1"       "crossing_2,bottleneck_1"   10.454 -8.209 "headon_r3"
run_baseline_route "SharedLane sharedlane_1-sharedlane_3" "sharedlane_1,sharedlane_3" 10.498 -6.565 "sharedlane_r1"
run_baseline_route "SharedLane sharedlane_3-sharedlane_1" "sharedlane_3,sharedlane_1" 10.498 -6.565 "sharedlane_r2"
run_baseline_route "SharedLane charger_1-sharedlane_2"    "charger_1,sharedlane_2"    5.368  -6.654 "sharedlane_r3"

log "########## ALL 12 MATCHED BASELINES COMPLETE ##########"
