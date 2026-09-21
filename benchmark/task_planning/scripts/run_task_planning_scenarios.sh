#!/bin/bash

set -o pipefail

WS=/home/eiu/rmf_ws
TS_DIR=$WS/src/benchmark/traffic_scheduling
SCRIPTS=$WS/src/benchmark/task_planning/scripts
TS_SCRIPTS=$TS_DIR/scripts
MASTER_LOG=$TS_DIR/official_run_$(date -u +%Y%m%d_%H%M).log

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$MASTER_LOG"; }

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

BAG_TOPICS="/rmf_task/bid_notice /rmf_task/bid_response \
/rmf_task/dispatch_request /rmf_task/dispatch_ack \
/task_api_requests /fleet_states \
/rmf_traffic/negotiation_notice /rmf_traffic/negotiation_proposal \
/rmf_traffic/negotiation_conclusion /rmf_traffic/negotiation_rejection \
/rmf_traffic/negotiation_forfeit \
/rmf_traffic/itinerary_set /rmf_traffic/query_update_1 \
/rmf_traffic/participants \
/rmf_traffic/blockade_set /rmf_traffic/blockade_ready \
/rmf_traffic/blockade_reached /rmf_traffic/blockade_release"

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
    pkill -9 -f "tb3_multi_simulation_nav2.launch.py" 2>/dev/null
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
    pkill -9 -f "tb3_fleet.set_initial_pose" 2>/dev/null
    pkill -9 -f zenohd 2>/dev/null
    sleep 3
    log "Teardown complete"
}

bring_up_common() {
    local RUN_DIR="$1" LOGSUFFIX="$2"
    setsid zenohd > "$RUN_DIR/zenohd${LOGSUFFIX}.log" 2>&1 &
    ZENOHD_PID=$!
    sleep 3

    setsid ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py \
        use_rviz:=False use_gzclient:=False \
        > "$RUN_DIR/simulation${LOGSUFFIX}.log" 2>&1 &
    SIM_PID=$!

    log "waiting for /tf..."
    local tf_i
    for tf_i in $(seq 1 45); do
        ros2 topic list 2>/dev/null | grep -q "^/tf$" && break
        sleep 2
    done
    log "settling AMCL/Nav2 (60s)"
    sleep 60

    cd $WS/src/tb3_fleet/config/zenoh
    setsid ./zenoh-bridge-ros2dds -c tb3_multi_zenoh_bridge_ros2dds_client_config.json5 \
        > "$RUN_DIR/zenoh_bridge${LOGSUFFIX}.log" 2>&1 &
    BRIDGE_PID=$!
    cd "$RUN_DIR"
    sleep 3
}

launch_fleet_bundle_and_wait() {
    local RUN_DIR="$1" LOGNAME="$2"
    FA_LOG="$RUN_DIR/$LOGNAME"
    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        fleet_config_file:=$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/config/fleet/tb3_multi_simulation_config.yaml \
        bidding_time_window:=60.0 \
        > "$FA_LOG" 2>&1 &
    FA_PID=$!

    local reg_i
    for reg_i in $(seq 1 60); do
        if grep -q "Unable to compute a location on the navigation graph" "$FA_LOG" 2>/dev/null; then
            log "WEDGE DETECTED in $LOGNAME (navigation-graph localization failure)"
            return 1
        fi
        COUNT=$(grep -c "Successfully added robot" "$FA_LOG" 2>/dev/null)
        COUNT=${COUNT:-0}
        if [ "$COUNT" -ge 3 ] 2>/dev/null; then
            log "all 3 robots registered after ${reg_i}x2s ($LOGNAME)"
            return 0
        fi
        sleep 2
    done
    log "TIMEOUT waiting for registration ($LOGNAME)"
    return 1
}
run_scenario_repeat_loop() {
    local NAME="$1" DIRNAME="$2" R1="$3" R2="$4" R3="$5"
    local RUN_DIR="$TS_DIR/$DIRNAME/run_$(date -u +%Y%m%d_%H%M)_N3"
    mkdir -p "$RUN_DIR"
    log "=== [$NAME] (full stack restart before every repeat) starting -> $RUN_DIR ==="

    local completed=0
    local attempt=0
    local max_attempts=15
    while [ "$completed" -lt 5 ] && [ "$attempt" -lt "$max_attempts" ]; do
        attempt=$((attempt+1))
        local i=$((completed+1))
        log "[$NAME] repeat $i/5 (attempt $attempt): full teardown + bring-up"
        teardown_scenario
        bring_up_common "$RUN_DIR" "_attempt${attempt}"
        if ! launch_fleet_bundle_and_wait "$RUN_DIR" "fleet_adapter_attempt${attempt}.log"; then
            log "[$NAME] repeat $i/5 (attempt $attempt): fleet bundle failed to come up -- STOPPING this scenario's loop (no guessed recovery attempted). $completed repeat(s) completed so far are kept."
            break
        fi
        sleep 10

        local ATTEMPT_DIR="$RUN_DIR/attempt_${attempt}"
        mkdir -p "$ATTEMPT_DIR"
        eval "setsid ros2 bag record -o \"$ATTEMPT_DIR/bag\" $BAG_TOPICS > \"$ATTEMPT_DIR/rosbag_record.log\" 2>&1 &"
        BAG_PID=$!
        sleep 3

        log "[$NAME] repeat $i/5 (attempt $attempt): submitting"
        timeout 400 python3 $SCRIPTS/run_benchmark_concurrent.py \
            --route "$R1" --route "$R2" --route "$R3" \
            --rounds 1 --repeats 1 \
            --robots tb3_robot1 tb3_robot2 tb3_robot3 \
            --fixed-wait 300 --min-expected-distance-m 5 \
            --output-dir "$ATTEMPT_DIR" \
            > "$RUN_DIR/run_benchmark_attempt${attempt}.log" 2>&1
        RC=$?
        log "[$NAME] repeat $i/5 (attempt $attempt): run_benchmark_concurrent.py exited with code $RC"

        kill_group "$BAG_PID"
        sleep 5
        ros2 bag info "$ATTEMPT_DIR/bag" > "$ATTEMPT_DIR/bag_info.log" 2>&1
        if ! python3 $TS_SCRIPTS/check_physics_glitch.py \
                --bag "$ATTEMPT_DIR/bag" \
                --robots tb3_robot1 tb3_robot2 tb3_robot3 \
                > "$ATTEMPT_DIR/physics_check.log" 2>&1; then
            log "[$NAME] repeat $i/5 (attempt $attempt): PHYSICS GLITCH detected (see $ATTEMPT_DIR/physics_check.log) -- discarding this attempt, will retry as repeat $i"
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_physics_glitch_attempt${attempt}"
            continue
        fi

        python3 $TS_SCRIPTS/analyze_traffic_scheduling.py \
            --bag "$ATTEMPT_DIR/bag" \
            --robots tb3_robot1 tb3_robot2 tb3_robot3 \
            --scenario "$NAME, N=3, CONFIG_01, repeat $i" \
            --output "$ATTEMPT_DIR/traffic_scheduling_metrics.json" \
            > "$ATTEMPT_DIR/analyze.log" 2>&1
        log "[$NAME] repeat $i/5 (attempt $attempt): analyze exited with code $?"

        mv "$ATTEMPT_DIR" "$RUN_DIR/repeat_${i}"
        if [ "$RC" -ne 0 ]; then
            log "[$NAME] repeat $i/5 (attempt $attempt): kept despite run_benchmark_concurrent.py exit code $RC (physics-clean; a non-zero exit here is a real outcome, e.g. the outer 'timeout 400' firing, not simulator noise -- only physics glitches get retried)"
        fi
        completed=$((completed+1))
    done

    log "[$NAME] $completed/5 repeats completed after $attempt attempt(s)"

    python3 -c "
import json, glob, os, re
run_dir = '$RUN_DIR'
files = glob.glob(os.path.join(run_dir, 'repeat_*', 'repeats_summary.json'))
files.sort(key=lambda p: int(re.search(r'repeat_(\d+)', p).group(1)))
merged = []
for f in files:
    merged.extend(json.load(open(f)))
json.dump(merged, open(os.path.join(run_dir, 'repeats_summary.json'), 'w'), indent=2)
print(f'merged {len(files)} repeat dir(s) -> {len(merged)} total task entries')
" | tee -a "$MASTER_LOG"

    python3 $TS_SCRIPTS/merge_repeat_metrics.py \
        --run-dir "$RUN_DIR" \
        --scenario "$NAME, N=3, CONFIG_01" \
        --robots tb3_robot1 tb3_robot2 tb3_robot3 \
        > "$RUN_DIR/merge_metrics.log" 2>&1
    log "[$NAME] merge_repeat_metrics exited with code $?"

    teardown_scenario
    log "=== [$NAME] DONE: $RUN_DIR ==="
}

SCENARIOS="${1:-Bottleneck,Crossing,Head-on,Shared Lane}"
want() { [[ ",$SCENARIOS," == *",$1,"* ]]; }

log "########## CORRECTED TRAFFIC SCHEDULING RUN (full stack restart per repeat, all 4 scenarios): scenarios=[$SCENARIOS] ##########"

if want "Bottleneck"; then
    run_scenario_repeat_loop "Bottleneck" "bottleneck_FLEET0N_TRAF03_CONFIG01" \
        "bottleneck_1,bottleneck_3" "sharedlane_3,loop_1" "bottleneck_3,sharedlane_3"
fi

if want "Crossing"; then
    run_scenario_repeat_loop "Crossing" "crossing_FLEET02_TRAF01_CONFIG01" \
        "charger_1,bottleneck_1" "crossing_1,loop_4" "crossing_2,bottleneck_3"
fi

if want "Head-on"; then
    run_scenario_repeat_loop "Head-on" "headon_FLEET0N_TRAF04_CONFIG01" \
        "charger_1,bottleneck_3" "crossing_1,bottleneck_3" "crossing_2,bottleneck_1"
fi

if want "Shared Lane"; then
    run_scenario_repeat_loop "Shared Lane" "sharedlane_FLEET0N_TRAF02_CONFIG01" \
        "sharedlane_1,sharedlane_3" "sharedlane_3,sharedlane_1" "charger_1,sharedlane_2"
fi

log "########## RUN COMPLETE [$SCENARIOS] ##########"
