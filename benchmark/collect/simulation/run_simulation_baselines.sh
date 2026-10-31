#!/bin/bash


set -o pipefail

WS=/home/eiu/rmf_ws
BENCHMARK_DIR=$WS/src/benchmark
COLLECT_RMF=$BENCHMARK_DIR/collect/rmf
COLLECT_SIM=$BENCHMARK_DIR/collect/simulation
ANALYSE_TASK_PLANNING=$BENCHMARK_DIR/analyse/task_planning
ANALYSE_TRAFFIC_SCHEDULING=$BENCHMARK_DIR/analyse/traffic_scheduling
TB3_FLEET_SHARE=$WS/install/tb3_fleet/share/tb3_fleet
NAV_GRAPH=$TB3_FLEET_SHARE/maps/world_tb3/nav_graphs/0.yaml
ZENOH_BRIDGE=$TB3_FLEET_SHARE/bin/zenoh-bridge-ros2dds
ZENOH_BRIDGE_CONFIG=$TB3_FLEET_SHARE/config/simulation/zenoh_bridge/tb3_zenoh_bridge_multi_robot_config_sim.json5
OUTPUT_DIR=$BENCHMARK_DIR/data/simulation/baselines
TARGET_REPEATS=${TARGET_REPEATS:-5}
mkdir -p "$OUTPUT_DIR"
MASTER_LOG=$OUTPUT_DIR/official_run_$(date -u +%Y%m%d_%H%M).log

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$MASTER_LOG"; }

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

ROBOT=${ROBOT_NAME:-tb3_robot1}
SCENARIO_FLEET_CONFIG=${SCENARIO_FLEET_CONFIG:-$TB3_FLEET_SHARE/config/simulation/fleet/multi_robot_config_sim.yaml}
FLEET_CONFIG=${FLEET_CONFIG:-}
SEED_TIMEOUT_S=${SEED_TIMEOUT_S:-30}
REACHABILITY_TIMEOUT_S=${REACHABILITY_TIMEOUT_S:-120}
AMCL_SETTLE_S=${AMCL_SETTLE_S:-30}
MAX_SEED_XY_ERR_M=${MAX_SEED_XY_ERR_M:-0.50}
MAX_SEED_YAW_ERR_RAD=${MAX_SEED_YAW_ERR_RAD:-0.35}
GZ_WORLD=${GZ_WORLD:-sim_world}
COLLECT_HW=$BENCHMARK_DIR/collect/hardware
SESSION_DIR=$BENCHMARK_DIR/.session/simulation
mkdir -p "$SESSION_DIR"

fleet_config_for_robot() {
    local out="$SESSION_DIR/fleet_config_${ROBOT}.yaml"
    python3 - "$SCENARIO_FLEET_CONFIG" "$ROBOT" "$out" <<'EOPY' || return 1
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
    printf '%s' "$out"
}

robot_reachable() {
    local deadline=$((SECONDS + REACHABILITY_TIMEOUT_S)) missing
    while [ "$SECONDS" -lt "$deadline" ]; do
        missing=""
        timeout 5 ros2 topic echo --once --qos-reliability best_effort \
            "/$ROBOT/scan" >/dev/null 2>&1 || missing="$missing $ROBOT/scan"
        timeout 5 ros2 topic echo --once "/$ROBOT/tf" >/dev/null 2>&1 \
            || missing="$missing $ROBOT/tf"
        [ -z "$missing" ] && { log "Robot [$ROBOT] publishes scan and tf."; return 0; }
        log "Waiting for robot streams, missing:$missing"
        sleep 5
    done
    log "ERROR: robot streams not reachable after ${REACHABILITY_TIMEOUT_S}s, missing:$missing"
    return 1
}

seed_accepted() {
    local want_x="$1" want_y="$2" want_yaw="$3" report
    report=$(timeout 15 ros2 topic echo --once "/$ROBOT/amcl_pose" 2>/dev/null | python3 -c "
import math
import sys

import yaml

# ros2 topic echo writes a stream of YAML documents separated by ---, so the whole
# stream is loaded and the first message taken.
msg = next((d for d in yaml.safe_load_all(sys.stdin.read()) if d), None)
if not msg:
    raise SystemExit('no amcl_pose')
pose = msg['pose']['pose']
q = pose['orientation']
yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                 1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
want_x, want_y, want_yaw = (float(a) for a in sys.argv[1:4])
d_xy = math.hypot(pose['position']['x'] - want_x, pose['position']['y'] - want_y)
d_yaw = abs((yaw - want_yaw + math.pi) % (2 * math.pi) - math.pi)
print(f'{d_xy:.3f} {d_yaw:.3f}')
" "$want_x" "$want_y" "$want_yaw")
    if [ -z "$report" ]; then
        log "WARNING: could not read amcl_pose from [$ROBOT]; the seeded pose was not confirmed."
        return 0
    fi
    set -- $report
    log "Seed check for [$ROBOT]: off by $1 m and $2 rad."
    if awk -v v="$1" -v lim="$MAX_SEED_XY_ERR_M" 'BEGIN { exit !(v > lim) }' \
       || awk -v v="$2" -v lim="$MAX_SEED_YAW_ERR_RAD" 'BEGIN { exit !(v > lim) }'; then
        log "ERROR: AMCL did not settle on the seeded pose (tolerance ${MAX_SEED_XY_ERR_M} m, ${MAX_SEED_YAW_ERR_RAD} rad)."
        return 1
    fi
    return 0
}

start_pose() {
    local start="$1" route="$2"
    python3 -m tb3_fleet.set_initial_pose \
        --robot-namespace unused --nav-graph "$NAV_GRAPH" \
        --waypoint "$start" --route-end "${route##*,}" --print-pose
}

kill_group() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null
    fi
}

teardown_scenario() {
    log "Tearing down"
    kill_group "$BAG_PID"; kill_group "$GT_PID"; kill_group "$FA_PID"; kill_group "$BRIDGE_PID"; kill_group "$SIM_PID"; kill_group "$ZENOHD_PID"
    sleep 6
    kill_group "$BAG_PID"; kill_group "$FA_PID"; kill_group "$BRIDGE_PID"; kill_group "$SIM_PID"; kill_group "$ZENOHD_PID"
    sleep 2
    pkill -9 -f "tb3_world.launch.py" 2>/dev/null
    pkill -9 -f "tb3_fleet.tb3_fleet_adapter" 2>/dev/null
    pkill -9 -f "zenoh-bridge-ros2dds" 2>/dev/null
    pkill -9 -f "tb3_simulation_nav2.launch.py" 2>/dev/null
    pkill -9 -f "ros2 bag record" 2>/dev/null
    pkill -9 -f "record_api_state.py" 2>/dev/null
    pkill -9 -f "record_ground_truth_poses.py" 2>/dev/null
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


place_robot() {
    local px="$1" py="$2" pyaw="$3" qz qw
    read -r qz qw < <(python3 -c "
import math, sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2), math.cos(yaw / 2))
" "$pyaw")
    gz service -s "/world/$GZ_WORLD/set_pose" \
        --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
        --req "name: \"$ROBOT\", position: {x: $px, y: $py, z: 0.01}, orientation: {x: 0, y: 0, z: $qz, w: $qw}" \
        >> "$SESSION_DIR/set_pose.log" 2>&1 \
        || log "WARNING: could not place [$ROBOT] at ($px, $py); see $SESSION_DIR/set_pose.log"
    sleep 3
}

seed_initial_pose() {
    local RUN_DIR="$1" LOGSUFFIX="$2"
    python3 -m tb3_fleet.set_initial_pose \
        --robot-namespace "$ROBOT" --nav-graph "$NAV_GRAPH" \
        --waypoint "$START_WP" --route-end "${ROUTE_END}" \
        --timeout-sec "$SEED_TIMEOUT_S" \
        > "$RUN_DIR/initialpose${LOGSUFFIX}.log" 2>&1
}

wait_for_amcl_transform() {
    local deadline=$((SECONDS + AMCL_SETTLE_S)) seen=""
    while [ -z "$seen" ] && [ "$SECONDS" -lt "$deadline" ]; do
        seen=$(timeout 5 ros2 topic echo "/$ROBOT/tf" 2>/dev/null | grep -m1 "frame_id: map")
    done
    if [ -z "$seen" ]; then
        log "WARNING: no map -> odom from AMCL within ${AMCL_SETTLE_S}s on [$ROBOT]."
    else
        log "Robot [$ROBOT] publishes map -> odom."
    fi
    return 0
}

bring_up_common() {
    local RUN_DIR="$1" LOGSUFFIX="$2"
    setsid zenohd > "$RUN_DIR/zenohd${LOGSUFFIX}.log" 2>&1 &
    ZENOHD_PID=$!
    sleep 3

    TB3_SIM_ROBOTS="$ROBOT" DISPLAY= setsid ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py \
        use_rviz:=False use_gzclient:=False \
        > "$RUN_DIR/simulation${LOGSUFFIX}.log" 2>&1 &
    SIM_PID=$!

    log "waiting for /$ROBOT/tf..."
    local tf_i
    for tf_i in $(seq 1 45); do
        timeout 10 ros2 topic list 2>/dev/null | grep -q "^/$ROBOT/tf$" && break
        sleep 2
    done

    setsid "$ZENOH_BRIDGE" -c "$ZENOH_BRIDGE_CONFIG" \
        > "$RUN_DIR/zenoh_bridge${LOGSUFFIX}.log" 2>&1 &
    BRIDGE_PID=$!
    sleep 3
}

launch_fleet_bundle_and_wait() {
    local RUN_DIR="$1" LOGNAME="$2"
    FA_LOG="$RUN_DIR/$LOGNAME"
    setsid python3 $COLLECT_RMF/record_api_state.py \
        --out "$RUN_DIR/api_state_attempt${attempt}.jsonl" --port "$API_PORT" \
        > "$RUN_DIR/api_state_logger_attempt${attempt}.log" 2>&1 &
    API_PID=$!
    sleep 3

    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        fleet_config_file:="$FLEET_CONFIG" \
        server_uri:=ws://127.0.0.1:$API_PORT \
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

API_PORT=${API_PORT:-8411}

BAG_TOPICS=(
    /dispatch_states /fleet_states /mutex_group_request /mutex_group_states
    /rmf_task/bid_notice /rmf_task/bid_response /rmf_task/dispatch_ack
    /rmf_task/dispatch_request /rmf_traffic/blockade_reached /rmf_traffic/blockade_ready
    /rmf_traffic/blockade_release /rmf_traffic/blockade_set /rmf_traffic/itinerary_clear
    /rmf_traffic/itinerary_delay /rmf_traffic/itinerary_extend
    /rmf_traffic/itinerary_reached /rmf_traffic/itinerary_set /rmf_traffic/negotiation_ack
    /rmf_traffic/negotiation_conclusion /rmf_traffic/negotiation_forfeit
    /rmf_traffic/negotiation_notice /rmf_traffic/negotiation_proposal
    /rmf_traffic/negotiation_refusal /rmf_traffic/negotiation_rejection
    /rmf_traffic/negotiation_repeat /rmf_traffic/negotiation_states
    /rmf_traffic/negotiation_statuses /rmf_traffic/participants /rmf_traffic/query_update_1
    /rmf_traffic/schedule_startup /task_api_requests /task_api_responses /lane_states /clock
    /$ROBOT/amcl_pose /$ROBOT/odom /$ROBOT/scan /$ROBOT/battery_state /$ROBOT/tf
    /$ROBOT/tf_static /$ROBOT/cmd_vel /$ROBOT/plan
)

ROUTES="${1:-all}"
want_route() { [ "$ROUTES" = "all" ] || [[ ",$ROUTES," == *",$1,"* ]]; }

run_baseline_route() {
    local SCEN_LABEL="$1" ROUTE="$2" START_WP="$3" DIRNAME="$4"
    want_route "$DIRNAME" || return 0
    local ROUTE_END="${ROUTE##*,}"
    local SX SY SYAW
    read -r SX SY SYAW < <(start_pose "$START_WP" "$ROUTE")
    local RUN_DIR="$OUTPUT_DIR/$DIRNAME/run_$(date -u +%Y%m%d_%H%M)"
    mkdir -p "$RUN_DIR"
    log "=== [$SCEN_LABEL] baseline for route [$ROUTE], start [$START_WP] at ($SX, $SY) yaw $SYAW -> $RUN_DIR ==="

    local completed=0 attempt=0 max_attempts=12
    while [ "$completed" -lt "$TARGET_REPEATS" ] && [ "$attempt" -lt "$max_attempts" ]; do
        attempt=$((attempt+1))
        local i=$((completed+1))
        log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): full teardown + bring-up"
        teardown_scenario
        bring_up_common "$RUN_DIR" "_attempt${attempt}"
        if ! robot_reachable; then
            log "[$SCEN_LABEL] repeat $i (attempt $attempt): robot streams absent -- discarding this attempt."
            continue
        fi
        place_robot "$SX" "$SY" "$SYAW"
        seed_initial_pose "$RUN_DIR" "_attempt${attempt}"
        wait_for_amcl_transform
        if ! seed_accepted "$SX" "$SY" "$SYAW"; then
            log "[$SCEN_LABEL] repeat $i (attempt $attempt): AMCL did not settle on the seeded pose -- discarding this attempt."
            continue
        fi
        if ! launch_fleet_bundle_and_wait "$RUN_DIR" "fleet_adapter_attempt${attempt}.log"; then
            log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): fleet bundle failed -- STOPPING. $completed repeat(s) kept."
            break
        fi
        sleep 10

        local ATTEMPT_DIR="$RUN_DIR/attempt_${attempt}"
        mkdir -p "$ATTEMPT_DIR"
        local window_start window_end
        window_start=$(date +%s)
        setsid ros2 bag record -o "$ATTEMPT_DIR/bag" "${BAG_TOPICS[@]}" \
            > "$ATTEMPT_DIR/rosbag_record.log" 2>&1 &
        BAG_PID=$!

        setsid python3 $COLLECT_SIM/record_ground_truth_poses.py \
            --out "$ATTEMPT_DIR/ground_truth_poses.jsonl" --hz 10 \
            --match "^$ROBOT$" \
            > "$ATTEMPT_DIR/ground_truth_poses.log" 2>&1 &
        GT_PID=$!
        sleep 3

        log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): submitting"
        timeout 720 python3 $COLLECT_RMF/run_benchmark_concurrent.py \
            --route "$ROUTE" \
            --rounds 1 --repeats 1 \
            --robots $ROBOT \
            --fixed-wait 600 --min-expected-distance-m 2 \
            --output-dir "$ATTEMPT_DIR" \
            > "$RUN_DIR/run_benchmark_attempt${attempt}.log" 2>&1
        RC=$?
        log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): run_benchmark_concurrent.py exited with code $RC"

        sleep 10
        window_end=$(date +%s)
        kill_group "$BAG_PID"
        kill_group "$GT_PID"
        sleep 5
        ros2 bag info "$ATTEMPT_DIR/bag" > "$ATTEMPT_DIR/bag_info.log" 2>&1

        python3 $COLLECT_HW/check_hardware_preflight.py \
            --repeat-dir "$ATTEMPT_DIR" \
            --robots $ROBOT \
            --attempt "$attempt" \
            --window-start "$window_start" --window-end "$window_end" \
            > "$ATTEMPT_DIR/preflight.log" 2>&1
        PRE_RC=$?
        if [ "$PRE_RC" -ne 0 ]; then
            log "[$SCEN_LABEL] repeat $i (attempt $attempt): preflight exited $PRE_RC, see $ATTEMPT_DIR/preflight.log -- discarding."
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_preflight_attempt${attempt}"
            continue
        fi

        if ! python3 $COLLECT_SIM/check_physics_glitch.py \
                --bag "$ATTEMPT_DIR/bag" \
                --robots $ROBOT \
                > "$ATTEMPT_DIR/physics_check.log" 2>&1; then
            log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): PHYSICS GLITCH -- discarding, retry as repeat $i"
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_physics_glitch_attempt${attempt}"
            continue
        fi

        python3 $ANALYSE_TASK_PLANNING/analyze_task_planning.py \
            --bag "$ATTEMPT_DIR/bag" \
            --api-state "$RUN_DIR/api_state_attempt${attempt}.jsonl" \
            --robots $ROBOT \
            --scenario "$SCEN_LABEL baseline repeat $i" \
            --nav-graph "$NAV_GRAPH" \
            --output "$ATTEMPT_DIR/task_planning_metrics.json" \
            > "$ATTEMPT_DIR/analyze.log" 2>&1
        log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): analyze exited with code $?"

        python3 $COLLECT_RMF/annotate_task_outcomes.py \
            --repeats-summary "$ATTEMPT_DIR/repeats_summary.json" \
            --task-planning-metrics "$ATTEMPT_DIR/task_planning_metrics.json" \
            >> "$ATTEMPT_DIR/analyze.log" 2>&1
        log "[$SCEN_LABEL] repeat $i/$TARGET_REPEATS (attempt $attempt): outcome annotation exited with code $?"

        mv "$ATTEMPT_DIR" "$RUN_DIR/repeat_${i}"
        completed=$((completed+1))
    done

    log "[$SCEN_LABEL] $completed/$TARGET_REPEATS repeats completed after $attempt attempt(s)"
    teardown_scenario
    log "=== [$SCEN_LABEL] DONE: $RUN_DIR ==="
}

ROUTES_TABLE=(
"Bottleneck wp_2-wp_5|wp_2,wp_5|wp_2|bottleneck_r1"
"Bottleneck wp_3-wp_4|wp_3,wp_4|wp_3|bottleneck_r2"
"Bottleneck wp_4-wp_3|wp_4,wp_3|wp_4|bottleneck_r3"
"Crossing wp_2-wp_6|wp_2,wp_6|wp_2|crossing_r1"
"Crossing wp_6-wp_9|wp_6,wp_9|wp_6|crossing_r2"
"Crossing wp_7-wp_8|wp_7,wp_8|wp_7|crossing_r3"
"Headon wp_2-wp_5|wp_2,wp_5|wp_2|headon_r1"
"Headon wp_5-wp_2|wp_5,wp_2|wp_5|headon_r2"
"Headon wp_3-wp_7|wp_3,wp_7|wp_3|headon_r3"
"SharedLane wp_2-wp_7|wp_2,wp_7|wp_2|sharedlane_r1"
"SharedLane wp_4-wp_5|wp_4,wp_5|wp_4|sharedlane_r2"
"SharedLane wp_6-wp_3|wp_6,wp_3|wp_6|sharedlane_r3"
)

if [ -z "$FLEET_CONFIG" ]; then
    FLEET_CONFIG=$(fleet_config_for_robot)
    if [ -z "$FLEET_CONFIG" ] || [ ! -f "$FLEET_CONFIG" ]; then
        log "ERROR: could not derive a one-robot fleet config for [$ROBOT] from [$SCENARIO_FLEET_CONFIG]."
        exit 1
    fi
    log "Robot [$ROBOT], fleet config derived from [$SCENARIO_FLEET_CONFIG]."
fi

for entry in "${ROUTES_TABLE[@]}"; do
    IFS='|' read -r label route start dirname <<< "$entry"
    run_baseline_route "$label" "$route" "$start" "$dirname"
done

log "########## ALL 12 MATCHED BASELINES COMPLETE ##########"
