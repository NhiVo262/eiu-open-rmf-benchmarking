#!/bin/bash
#
# Collects the single-robot hardware baseline: one contention-free run of each route,
# repeated TARGET_REPEATS times.
#
# Runs on the workstation. The robot runs its own bringup, Nav2 and zenoh-bridge-ros2dds on
# its own ROS_DOMAIN_ID; the two sides meet over zenoh. The runner seeds AMCL from the
# navigation graph before every repeat.
#
#   workstation  ./run_hardware_baselines.sh [route[,route...]|all]
#   robot        ./zenoh-bridge-ros2dds -c ~/turtlebot3_ws/config/tb3_zenoh_robotN_config_hw.json5
#
# The script pauses for the robot to be placed on the route's start waypoint, then seeds AMCL itself and waits for map -> odom.
#
#   TARGET_REPEATS   repeats per route (default 5)
#   HW_RESUME=1      continue the newest run directory instead of starting a new one
#   RUN_DIR          write into this directory instead of choosing one
#   HW_VIZ=1         launch RViz with the fleet adapter
#   HW_UNATTENDED=1  do not pause for the operator between repeats
#   MAX_ATTEMPTS     give up on a route after this many attempts (default 8)
#   HW_MIN_SOC       refuse to start a repeat below this charge, 0..1 (default 0.50)
#   REACHABILITY_TIMEOUT_S  seconds to wait for the robot's streams before discarding
#                    the attempt (default 120)

set -o pipefail

WORKSPACE=${WORKSPACE:-/home/eiu/rmf_ws}
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WORKSPACE/install/setup.bash" >/dev/null 2>&1

export ROS_DOMAIN_ID=${HW_ROS_DOMAIN_ID:-12}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

BENCHMARK_DIR=$WORKSPACE/src/benchmark
COLLECT_HW=$BENCHMARK_DIR/collect/hardware
COLLECT_RMF=$BENCHMARK_DIR/collect/rmf
ANALYSE_TASK_PLANNING=$BENCHMARK_DIR/analyse/task_planning
TB3_FLEET_SHARE=$WORKSPACE/install/tb3_fleet/share/tb3_fleet
ZENOH_BRIDGE=$TB3_FLEET_SHARE/bin/zenoh-bridge-ros2dds
ZENOH_BRIDGE_CONFIG=$COLLECT_HW/zenoh_bridge_workstation.json5
OUTPUT_DIR=$BENCHMARK_DIR/data/hardware/baselines
SESSION_DIR=$BENCHMARK_DIR/.session/hardware   

NAV_GRAPH=${NAV_GRAPH:-$TB3_FLEET_SHARE/maps/world_tb3/nav_graphs/0.yaml}
FLEET_CONFIG=${FLEET_CONFIG:-}
SCENARIO_FLEET_CONFIG=${SCENARIO_FLEET_CONFIG:-$TB3_FLEET_SHARE/config/hardware/fleet/multi_robot_config_hw.yaml}

ROBOT_NAME=${ROBOT_NAME:-tb3_robot1}
FLEET_NAME=${FLEET_NAME:-tb3_fleet}
API_SERVER_PORT=${API_SERVER_PORT:-8411}
TARGET_REPEATS=${TARGET_REPEATS:-5}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-8}
HW_UNATTENDED=${HW_UNATTENDED:-0}
HW_OPERATOR_TIMEOUT=${HW_OPERATOR_TIMEOUT:-900}
HW_RESUME=${HW_RESUME:-0}
RUN_DIR=${RUN_DIR:-}
HW_VIZ=${HW_VIZ:-0}

ZENOH_ROUTER_PORT=7447
ZENOH_BRIDGE_COMM=zenoh-bridge-ro          
REGISTRATION_TIMEOUT_S=${REGISTRATION_TIMEOUT_S:-120}
TASK_TIMEOUT_S=720
POST_MOTION_RECORD_S=10
HW_MIN_SOC=${HW_MIN_SOC:-0.50}
REACHABILITY_TIMEOUT_S=${REACHABILITY_TIMEOUT_S:-120}
ACTION_ROUTE_TIMEOUT_S=${ACTION_ROUTE_TIMEOUT_S:-30}
MAX_SEED_XY_ERR_M=${MAX_SEED_XY_ERR_M:-0.50}
MAX_SEED_YAW_ERR_RAD=${MAX_SEED_YAW_ERR_RAD:-0.35}
SEED_TIMEOUT_S=${SEED_TIMEOUT_S:-30}
AMCL_SETTLE_S=${AMCL_SETTLE_S:-30}

[ "$HW_VIZ" = "1" ] && HEADLESS=false || HEADLESS=true

API_LOGGER_PID=""; FLEET_ADAPTER_PID=""; BAG_RECORDER_PID=""
CURRENT_ATTEMPT_DIR=""

LOGGER_NAME=hardware_baseline_collector
mkdir -p "$OUTPUT_DIR" "$SESSION_DIR"
LOG_FILE=$OUTPUT_DIR/hardware_baselines_$(date -u +%Y%m%d_%H%M).log
HOLDER_PIDFILE=$SESSION_DIR/hold_robot_topics_$ROBOT_NAME.pid
HOLDER_LOG=$SESSION_DIR/hold_robot_topics_$ROBOT_NAME.log

log() {
    local severity="$1"; shift
    printf '[%s] [%s] [%s]: %s\n' \
        "$severity" "$(date +%s.%N)" "$LOGGER_NAME" "$*" | tee -a "$LOG_FILE"
}
log_info()  { log INFO "$@"; }
log_warn()  { log WARN "$@"; }
log_error() { log ERROR "$@"; }

BAG_TOPICS=(
    /dispatch_states /fleet_states /mutex_group_request /mutex_group_states
    /rmf_task/bid_notice /rmf_task/bid_response /rmf_task/dispatch_ack
    /rmf_task/dispatch_request
    /rmf_traffic/blockade_reached /rmf_traffic/blockade_ready
    /rmf_traffic/blockade_release /rmf_traffic/blockade_set
    /rmf_traffic/itinerary_clear /rmf_traffic/itinerary_delay
    /rmf_traffic/itinerary_extend /rmf_traffic/itinerary_reached
    /rmf_traffic/itinerary_set
    /rmf_traffic/negotiation_ack /rmf_traffic/negotiation_conclusion
    /rmf_traffic/negotiation_forfeit /rmf_traffic/negotiation_notice
    /rmf_traffic/negotiation_proposal /rmf_traffic/negotiation_refusal
    /rmf_traffic/negotiation_rejection /rmf_traffic/negotiation_repeat
    /rmf_traffic/negotiation_states /rmf_traffic/negotiation_statuses
    /rmf_traffic/participants /rmf_traffic/query_update_1
    /rmf_traffic/schedule_startup
    /task_api_requests /task_api_responses
    "/$ROBOT_NAME/amcl_pose" "/$ROBOT_NAME/battery_state" "/$ROBOT_NAME/cmd_vel"
    "/$ROBOT_NAME/tf" "/$ROBOT_NAME/tf_static"
)

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

ROUTE_FILTER="${1:-all}"

require_scripts() {
    local missing=() path
    for path in "$COLLECT_HW/check_zenoh_routes.py" \
                "$COLLECT_HW/stop_robot_navigation.py" \
                "$COLLECT_HW/hold_robot_topics.py" \
                "$COLLECT_RMF/record_api_state.py" \
                "$COLLECT_RMF/run_benchmark_concurrent.py" \
                "$ANALYSE_TASK_PLANNING/analyze_task_planning.py"; do
        [ -f "$path" ] || missing+=("$path")
    done
    if [ ${#missing[@]} -gt 0 ]; then
        log_error "preflight: missing required scripts: ${missing[*]}"
        return 1
    fi
    return 0
}

# kill_group: terminate the process group led by pid.
kill_group() {
    local pid="$1"
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && kill -TERM -- "-$pid" 2>/dev/null
    return 0
}

stop_robot_navigation() {
    [ -n "$FLEET_CONFIG" ] && [ -f "$FLEET_CONFIG" ] || return 0
    python3 "$COLLECT_HW/stop_robot_navigation.py" \
        --fleet-config "$FLEET_CONFIG" --robots "$ROBOT_NAME" \
        >> "$SESSION_DIR/stop_robot_navigation.log" 2>&1 \
        && log_info "navigation: stop command completed for robot=$ROBOT_NAME" \
        || log_warn "navigation: stop not confirmed for robot=$ROBOT_NAME; log=$SESSION_DIR/stop_robot_navigation.log"
}

teardown() {
    local pattern
    local patterns=(tb3_world.launch.py tb3_fleet.tb3_fleet_adapter "ros2 bag record"
                    record_api_state.py rmf_traffic rmf_fleet_adapter rmf_task_dispatcher
                    rmf_visualization building_map_server)
    log_info "lifecycle: stopping workstation processes"
    stop_robot_navigation
    kill_group "$BAG_RECORDER_PID"; kill_group "$API_LOGGER_PID"
    kill_group "$FLEET_ADAPTER_PID"
    BAG_RECORDER_PID=""; API_LOGGER_PID=""; FLEET_ADAPTER_PID=""
    sleep 6
    for pattern in "${patterns[@]}"; do pkill -TERM -f "$pattern" 2>/dev/null; done
    sleep 3
    for pattern in "${patterns[@]}"; do pkill -9 -f "$pattern" 2>/dev/null; done
    return 0
}

# on_signal: discard the recording in progress, stop the stack and exit.
on_signal() {
    trap - INT TERM
    log_warn "lifecycle: received interrupt; stopping collection"
    if [ -n "$CURRENT_ATTEMPT_DIR" ] && [ -d "$CURRENT_ATTEMPT_DIR" ]; then
        mv "$CURRENT_ATTEMPT_DIR" \
           "$(dirname "$CURRENT_ATTEMPT_DIR")/discarded_$(basename "$CURRENT_ATTEMPT_DIR")"
        log_warn "collection: current attempt marked discarded"
    fi
    teardown
    exit 130
}
trap on_signal INT TERM

# bridge_pids: pids of the workstation zenoh bridge.
bridge_pids() {
    ps -eo pid,comm | awk -v comm="$ZENOH_BRIDGE_COMM" '$2 == comm { print $1 }'
}

# zenoh_up: start the zenoh router and the workstation bridge unless they already run.
zenoh_up() {
    if ! ss -ltn 2>/dev/null | grep -q ":$ZENOH_ROUTER_PORT "; then
        setsid zenohd > "$SESSION_DIR/zenohd.log" 2>&1 &
        sleep 3
        if ! ss -ltn 2>/dev/null | grep -q ":$ZENOH_ROUTER_PORT "; then
            log_error "zenoh: router startup failed; log=$SESSION_DIR/zenohd.log"
            return 1
        fi
        log_info "zenoh: router ready; port=$ZENOH_ROUTER_PORT"
    fi
    if [ -z "$(bridge_pids)" ]; then
        setsid "$ZENOH_BRIDGE" -c "$ZENOH_BRIDGE_CONFIG" \
            > "$SESSION_DIR/zenoh_bridge_workstation.log" 2>&1 &
        sleep 5
        if [ -z "$(bridge_pids)" ]; then
            log_error "zenoh: workstation bridge startup failed; log=$SESSION_DIR/zenoh_bridge_workstation.log"
            return 1
        fi
        log_info "zenoh: workstation bridge ready"
    fi
    return 0
}

# holder_pids: pids of the topic holders running for ROBOT_NAME.
holder_pids() {
    pgrep -f "hold_robot_topics\.py --robot $ROBOT_NAME\$" 2>/dev/null
}


hold_topics_up() {
    local pid stray
    pid=$(cat "$HOLDER_PIDFILE" 2>/dev/null)
    for stray in $(holder_pids); do
        [ "$stray" = "$pid" ] && continue
        kill -9 "$stray" 2>/dev/null
        log_warn "zenoh: terminated stale topic holder; robot=$ROBOT_NAME pid=$stray"
    done
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        return 0
    fi
    setsid python3 "$COLLECT_HW/hold_robot_topics.py" --robot "$ROBOT_NAME" \
        > "$HOLDER_LOG" 2>&1 &
    echo $! > "$HOLDER_PIDFILE"
    sleep 3
    if ! kill -0 "$(cat "$HOLDER_PIDFILE")" 2>/dev/null; then
        log_error "zenoh: topic holder startup failed; log=$HOLDER_LOG"
        return 1
    fi
    log_info "zenoh: topic holder ready; robot=$ROBOT_NAME"
    return 0
}


restart_topic_holder() {
    local pid
    pid=$(cat "$HOLDER_PIDFILE" 2>/dev/null)
    [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null
    rm -f "$HOLDER_PIDFILE"
    log_info "zenoh: restarting topic holder; robot=$ROBOT_NAME"
    hold_topics_up
}


rebuild_bridge_routes() {
    local pid
    log_warn "zenoh: robot topics unavailable; restarting workstation bridge"
    for pid in $(bridge_pids); do kill -9 "$pid" 2>/dev/null; done
    pid=$(cat "$HOLDER_PIDFILE" 2>/dev/null)
    [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null
    rm -f "$HOLDER_PIDFILE"
    sleep 3
    zenoh_up || return 1
    hold_topics_up || return 1
    sleep 5
    return 0
}

missing_streams() {
    local missing=""
    timeout 5 ros2 topic echo --once --qos-reliability best_effort \
        "/$ROBOT_NAME/scan" >/dev/null 2>&1 || missing="$missing $ROBOT_NAME/scan"
    timeout 5 ros2 topic echo --once "/$ROBOT_NAME/tf" >/dev/null 2>&1 \
        || missing="$missing $ROBOT_NAME/tf"
    echo "$missing"
}

robot_reachable() {
    local deadline=$((SECONDS + REACHABILITY_TIMEOUT_S))
    local missing restarted=0 rebuilt=0
    missing=$(missing_streams)
    while [ -n "$missing" ] && [ "$SECONDS" -lt "$deadline" ]; do
        log_info "preflight: waiting for robot topics; missing=$missing"
        if [ "$restarted" -eq 0 ]; then
            restart_topic_holder
            restarted=1
        elif [ "$rebuilt" -eq 0 ]; then
            rebuild_bridge_routes || return 1
            rebuilt=1
        fi
        sleep 5
        missing=$(missing_streams)
    done
    if [ -n "$missing" ]; then
        log_error "preflight: robot topics unavailable; timeout=${REACHABILITY_TIMEOUT_S}s missing=$missing"
        log_error "preflight: robot topic check failed after bridge recovery"
        log_error "preflight: verify bringup and Zenoh bridge; robot=$ROBOT_NAME"
        return 1
    fi
    log_info "preflight: required topics available; robot=$ROBOT_NAME topics=scan,tf"
    return 0
}

nav_action_served() {
    local out rc deadline
    deadline=$((SECONDS + ACTION_ROUTE_TIMEOUT_S))
    while :; do
        out=$(python3 "$COLLECT_HW/check_zenoh_routes.py" \
              --fleet-config "$FLEET_CONFIG" --robots "$ROBOT_NAME" \
              2>> "$SESSION_DIR/check_zenoh_routes.log")
        rc=$?
        # The checker reports a robot by printing it, so an empty report and a crash both
        # print nothing; only the exit status separates them.
        [ "$rc" -eq 0 ] && [ -z "$out" ] && break
        [ "$SECONDS" -ge "$deadline" ] && break
        log_info "preflight: waiting for NavigateToPose action; robot=$ROBOT_NAME"
        sleep 5
    done
    if [ "$rc" -eq 0 ] && [ -z "$out" ]; then
        log_info "preflight: NavigateToPose action available; robot=$ROBOT_NAME"
        return 0
    fi
    if [ -n "$out" ]; then
        set -- $out
        case "$2" in
        robot) log_error "preflight: NavigateToPose action unavailable; robot=$ROBOT_NAME timeout=${ACTION_ROUTE_TIMEOUT_S}s" ;;
        *)     log_error "preflight: action route unavailable; robot=$ROBOT_NAME timeout=${ACTION_ROUTE_TIMEOUT_S}s log_dir=$SESSION_DIR" ;;
        esac
    else
        log_error "preflight: action route check failed; exit_code=$rc"
    fi
    log_error "preflight: action route log=$SESSION_DIR/check_zenoh_routes.log"
    return 1
}

battery_ok() {
    # percentage from the OpenCR board arrives on either the 0..1 or the 0..100 scale.
    local soc
    soc=$(timeout 10 ros2 topic echo --once --field percentage \
          "/$ROBOT_NAME/battery_state" 2>/dev/null | head -1 | tr -d '[:space:]')
    if [ -z "$soc" ]; then
        log_warn "preflight: battery state unavailable; robot=$ROBOT_NAME (charge unverified)"
        return 0
    fi
    soc=$(awk -v v="$soc" 'BEGIN { printf "%.3f", (v > 1.0 ? v / 100.0 : v) }')
    log_info "preflight: battery state; robot=$ROBOT_NAME soc=$soc"
    if awk -v v="$soc" -v floor="$HW_MIN_SOC" 'BEGIN { exit !(v < floor) }'; then
        log_error "preflight: battery below minimum; robot=$ROBOT_NAME soc=$soc minimum=$HW_MIN_SOC"
        return 1
    fi
    return 0
}

fleet_config_for_robot() {
    local out="$SESSION_DIR/fleet_config_${ROBOT_NAME}.yaml"
    python3 - "$SCENARIO_FLEET_CONFIG" "$ROBOT_NAME" "$out" <<'EOPY' || return 1
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


start_pose() {
    local start="$1" route="$2"
    python3 -m tb3_fleet.set_initial_pose \
        --robot-namespace unused \
        --nav-graph "$NAV_GRAPH" \
        --waypoint "$start" \
        --route-end "${route##*,}" \
        --print-pose
}

# first_hop: echo the neighbour the robot sets off towards, which is what the operator
# aims it at. Reads the same graph function the seed derives its heading from.
first_hop() {
    local start="$1" route="$2"
    python3 -c "
import sys
from tb3_fleet.set_initial_pose import first_hop
print(first_hop(sys.argv[1], sys.argv[2], sys.argv[3]) or sys.argv[2])
" "$NAV_GRAPH" "$start" "${route##*,}"
}


place_robot() {
    local repeat="$1" start="$2" route="$3"
    [ "$HW_UNATTENDED" = "1" ] && return 0
    local px py pyaw faces
    read -r px py pyaw < <(start_pose "$start" "$route")
    faces=$(first_hop "$start" "$route")
    log_info "operator: robot placement required; robot=$ROBOT_NAME repeat=$repeat"
    log_info "operator: place robot at waypoint=$start map_x=$px map_y=$py yaw=$pyaw rad; heading_toward=$faces"
    log_info "operator: press ENTER to publish the initial pose"
    read -r -t "$HW_OPERATOR_TIMEOUT" _ \
        || log_warn "operator: input timeout (${HW_OPERATOR_TIMEOUT}s); continuing"
    return 0
}


seed_initial_pose() {
    local start="$1" route="$2"
    python3 -m tb3_fleet.set_initial_pose \
        --robot-namespace "$ROBOT_NAME" \
        --nav-graph "$NAV_GRAPH" \
        --waypoint "$start" \
        --route-end "${route##*,}" \
        --timeout-sec "$SEED_TIMEOUT_S" \
        >> "$SESSION_DIR/set_initial_pose.log" 2>&1
    if [ "$?" -ne 0 ]; then
        log_error "localization: initial pose publication failed; robot=$ROBOT_NAME"
        log_error "localization: check bridge subscription to initialpose; log=$SESSION_DIR/set_initial_pose.log"
        log_error "localization: required subscriber pattern=.*/initialpose"
        return 1
    fi
    log_info "localization: initial pose published; robot=$ROBOT_NAME source=navigation_graph"
    return 0
}

wait_for_amcl_transform() {
    local deadline seen=""
    deadline=$((SECONDS + AMCL_SETTLE_S))
    while [ -z "$seen" ] && [ "$SECONDS" -lt "$deadline" ]; do
        seen=$(timeout 5 ros2 topic echo "/$ROBOT_NAME/tf" 2>/dev/null \
               | grep -m1 "frame_id: map")
    done
    if [ -z "$seen" ]; then
        log_warn "localization: map->odom transform not observed; robot=$ROBOT_NAME timeout=${AMCL_SETTLE_S}s"
        log_warn "localization: AMCL transform unavailable; verify initial pose and lifecycle state"
        return 0
    fi
    log_info "localization: map->odom transform available; robot=$ROBOT_NAME"
    return 0
}

# Validate the AMCL estimate against the configured initial pose.
seed_accepted() {
    local start="$1" route="$2"
    local px py pyaw report
    read -r px py pyaw < <(start_pose "$start" "$route")
    report=$(timeout 15 ros2 topic echo --once "/$ROBOT_NAME/amcl_pose" 2>/dev/null \
             | python3 -c "
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
print(f'{d_xy:.3f} {d_yaw:.3f} {yaw:.3f}')
" "$px" "$py" "$pyaw")
    if [ -z "$report" ]; then
        log_warn "localization: AMCL pose unavailable; robot=$ROBOT_NAME (initial pose unverified)"
        log_warn "localization: pose check returned no data"
        return 0
    fi
    set -- $report
    log_info "localization: initial pose check; robot=$ROBOT_NAME xy_error=$1 m yaw_error=$2 rad amcl_yaw=$3 rad seed_yaw=$pyaw rad"
    if awk -v v="$1" -v lim="$MAX_SEED_XY_ERR_M" 'BEGIN { exit !(v > lim) }' \
       || awk -v v="$2" -v lim="$MAX_SEED_YAW_ERR_RAD" 'BEGIN { exit !(v > lim) }'; then
        log_error "localization: initial pose outside tolerance; max_xy=${MAX_SEED_XY_ERR_M}m max_yaw=${MAX_SEED_YAW_ERR_RAD}rad"
        log_error "localization: verify placement; robot=$ROBOT_NAME waypoint=$start x=$px y=$py heading_toward=$(first_hop "$start" "$route")"
        return 1
    fi
    return 0
}

start_fleet_adapter() {
    local adapter_log="$1"
    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        fleet_config_file:="$FLEET_CONFIG" \
        use_sim_time:=false \
        headless:="$HEADLESS" \
        server_uri:="ws://127.0.0.1:$API_SERVER_PORT" \
        > "$adapter_log" 2>&1 &
    FLEET_ADAPTER_PID=$!
}

wait_for_registration() {
    local adapter_log="$1" i
    for i in $(seq 1 $((REGISTRATION_TIMEOUT_S / 2))); do
        if grep -q "Successfully added robot \[$ROBOT_NAME\]" "$adapter_log" 2>/dev/null; then
            log_info "fleet: robot registered; fleet=$FLEET_NAME robot=$ROBOT_NAME elapsed=$((i * 2))s"
            return 0
        fi
        grep -q "Unable to compute a location on the navigation graph" \
            "$adapter_log" 2>/dev/null && break
        kill -0 "$FLEET_ADAPTER_PID" 2>/dev/null || break
        sleep 2
    done
    return 1
}


record_and_run() {
    local attempt_dir="$1" route="$2" run_dir="$3" attempt="$4"
    mkdir -p "$attempt_dir"
    setsid ros2 bag record -o "$attempt_dir/bag" "${BAG_TOPICS[@]}" \
        > "$attempt_dir/rosbag_record.log" 2>&1 &
    BAG_RECORDER_PID=$!
    sleep 3

    timeout "$TASK_TIMEOUT_S" python3 \
        "$COLLECT_RMF/run_benchmark_concurrent.py" \
        --route "$route" --rounds 1 --repeats 1 --robots "$ROBOT_NAME" \
        --fixed-wait 600 --min-expected-distance-m 2 --output-dir "$attempt_dir" \
        --api-state "$run_dir/api_state_attempt${attempt}.jsonl" \
        > "$run_dir/run_benchmark_attempt${attempt}.log" 2>&1
    log_info "task: benchmark runner finished; exit_code=$?"

    # Keep recording until RobotState releases the task_id, which F4a reads as completion.
    sleep "$POST_MOTION_RECORD_S"
    kill_group "$BAG_RECORDER_PID"; BAG_RECORDER_PID=""
    sleep 5

    ros2 bag info "$attempt_dir/bag" > "$attempt_dir/bag_info.log" 2>&1

    grep -oE "^Messages:[[:space:]]+[0-9]+" "$attempt_dir/bag_info.log" \
        | grep -oE "[0-9]+" | head -1 > "$attempt_dir/bag_message_count"
}

run_route() {
    local label="$1" route="$2" start="$3" dirname="$4"
    [ "$ROUTE_FILTER" = "all" ] || [[ ",$ROUTE_FILTER," == *",$dirname,"* ]] || return 0

    local run_dir="$RUN_DIR"
    if [ -z "$run_dir" ] && [ "$HW_RESUME" = "1" ]; then
        run_dir=$(find "$OUTPUT_DIR/$dirname" -maxdepth 1 -type d -name 'run_*' \
                  2>/dev/null | sort | tail -1)
    fi
    : "${run_dir:=$OUTPUT_DIR/$dirname/run_$(date -u +%Y%m%d_%H%M)}"
    mkdir -p "$run_dir"

    local completed
    completed=$(find "$run_dir" -maxdepth 1 -type d -name 'repeat_[0-9]*' 2>/dev/null | wc -l)
    if [ "$completed" -ge "$TARGET_REPEATS" ]; then
        log_info "collection: route already complete; route=$dirname repeats=$completed/$TARGET_REPEATS"
        return 0
    fi
    log_info "collection: route started; route=$dirname waypoints=$route start=$start repeats=$completed/$TARGET_REPEATS output=$run_dir"

    local attempt=0 attempt_id repeat attempt_dir message_count adapter_log analysis_rc
    attempt_id=$(find "$run_dir" -maxdepth 1 -name 'api_state_attempt*.jsonl' 2>/dev/null \
                 | sed 's/.*attempt\([0-9]\{1,\}\)\.jsonl.*/\1/' | sort -n | tail -1)
    attempt_id=${attempt_id:-0}

    while [ "$completed" -lt "$TARGET_REPEATS" ] && [ "$attempt" -lt "$MAX_ATTEMPTS" ]; do
        attempt=$((attempt + 1)); attempt_id=$((attempt_id + 1)); repeat=$((completed + 1))
        log_info "collection: starting repeat; route=$dirname repeat=$repeat/$TARGET_REPEATS attempt=$attempt"

        teardown
        zenoh_up || return 1
        hold_topics_up || return 1
        robot_reachable || continue
        nav_action_served || continue
        place_robot "$repeat" "$start" "$route"
        seed_initial_pose "$start" "$route" || continue
        wait_for_amcl_transform
        seed_accepted "$start" "$route" || continue
        battery_ok || continue

        setsid python3 "$COLLECT_RMF/record_api_state.py" \
            --out "$run_dir/api_state_attempt${attempt_id}.jsonl" --port "$API_SERVER_PORT" \
            > "$run_dir/api_state_logger_attempt${attempt_id}.log" 2>&1 &
        API_LOGGER_PID=$!
        sleep 3

        adapter_log="$run_dir/fleet_adapter_attempt${attempt_id}.log"
        start_fleet_adapter "$adapter_log"
        if ! wait_for_registration "$adapter_log"; then
            log_warn "fleet: robot registration failed; fleet=$FLEET_NAME robot=$ROBOT_NAME log=$adapter_log"
            continue
        fi
        sleep 10

        attempt_dir="$run_dir/attempt_$(date -u +%H%M%S)"
        CURRENT_ATTEMPT_DIR="$attempt_dir"
        log_info "task: submitting route; waypoints=$route repeat=$repeat"
        record_and_run "$attempt_dir" "$route" "$run_dir" "$attempt_id"
        CURRENT_ATTEMPT_DIR=""
        message_count=$(cat "$attempt_dir/bag_message_count" 2>/dev/null)

        case "$message_count" in
        ''|*[!0-9]*) log_warn "rosbag2: invalid message count; repeat=$repeat value=$message_count (discarded)"
                     mv "$attempt_dir" "$run_dir/discarded_$(basename "$attempt_dir")"
                     continue ;;
        esac

        if [ "$message_count" -lt 1 ]; then
            log_warn "rosbag2: empty or unreadable recording; repeat=$repeat (discarded)"
            mv "$attempt_dir" "$run_dir/discarded_$(basename "$attempt_dir")"
            continue
        fi

        python3 "$ANALYSE_TASK_PLANNING/analyze_task_planning.py" \
            --bag "$attempt_dir/bag" --robots "$ROBOT_NAME" \
            --api-state "$run_dir/api_state_attempt${attempt_id}.jsonl" \
            --scenario "$label hardware baseline repeat $repeat" \
            --nav-graph "$NAV_GRAPH" \
            --output "$attempt_dir/task_planning_metrics.json" \
            > "$attempt_dir/analyze.log" 2>&1
        analysis_rc=$?
        log_info "analysis: task planning finished; repeat=$repeat exit_code=$analysis_rc"
        if [ "$analysis_rc" -ne 0 ]; then
            log_warn "analysis: task planning failed; repeat=$repeat (discarded)"
            mv "$attempt_dir" "$run_dir/discarded_analysis_$(basename "$attempt_dir")"
            continue
        fi

        python3 "$COLLECT_RMF/annotate_task_outcomes.py" \
            --repeats-summary "$attempt_dir/repeats_summary.json" \
            --task-planning-metrics "$attempt_dir/task_planning_metrics.json" \
            >> "$attempt_dir/analyze.log" 2>&1
        analysis_rc=$?
        log_info "analysis: outcome annotation finished; repeat=$repeat exit_code=$analysis_rc"
        if [ "$analysis_rc" -ne 0 ]; then
            log_warn "analysis: outcome annotation failed; repeat=$repeat (discarded)"
            mv "$attempt_dir" "$run_dir/discarded_analysis_$(basename "$attempt_dir")"
            continue
        fi

        mv "$attempt_dir" "$run_dir/repeat_${repeat}"
        completed=$((completed + 1))
    done

    log_info "collection: route finished; route=$dirname repeats=$completed/$TARGET_REPEATS attempts=$attempt"
    teardown
    return 0
}

require_scripts || exit 1

log_info "collection: starting hardware N=1 baselines; routes=$ROUTE_FILTER robot=$ROBOT_NAME"
log_info "preflight: minimum battery soc=$HW_MIN_SOC"
if [ -z "$FLEET_CONFIG" ]; then
    mkdir -p "$SESSION_DIR"
    FLEET_CONFIG=$(fleet_config_for_robot)
    if [ -z "$FLEET_CONFIG" ] || [ ! -f "$FLEET_CONFIG" ]; then
        log_error "configuration: failed to generate single-robot fleet config; robot=$ROBOT_NAME source="
        log_error "$SCENARIO_FLEET_CONFIG"
        exit 1
    fi
    log_info "configuration: single-robot fleet config ready; robot=$ROBOT_NAME source=$SCENARIO_FLEET_CONFIG"
fi
log_info "configuration: fleet_config=$FLEET_CONFIG"
log_info "configuration: nav_graph=$NAV_GRAPH"

for entry in "${ROUTES_TABLE[@]}"; do
    IFS='|' read -r label route start dirname <<< "$entry"
    run_route "$label" "$route" "$start" "$dirname"
done

log_info "collection: finished hardware N=1 baselines; routes=$ROUTE_FILTER"