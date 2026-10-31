#!/bin/bash
#
# Collects the four benchmark scenarios on three physical TurtleBot3, repeating
# each scenario TARGET_REPEATS times.
#
# Bring-up and Nav2 run on each robot; this script starts neither. On the workstation it
# starts the zenoh router and bridge, the RMF core and the fleet adapter, records the bag
#
# Before every repeat it checks that each robot publishes scan and tf, and that
# each charge is at or above HW_MIN_SOC, then pauses for the operator to reposition the
# robots.
#
# Each robot runs on its own ROS_DOMAIN_ID with its own zenoh-bridge-ros2dds, which keeps
# the three stacks apart at the DDS layer. 
#
#   HW_NO_GROUND_TRUTH    1 collects with no external tracker: the pose comes from
#                         AMCL
#   TARGET_REPEATS        repeats per scenario (default 5)
#   MAX_ATTEMPTS          give up on a scenario after this many attempts (default 8)
#   HW_MIN_SOC            refuse to start a repeat below this charge (default 0.50)
#   AMCL_SETTLE_S         seconds to wait for a pose from each robot after the seed
#                         (default 30); AMCL converges on motion, not on this wait
#   REACHABILITY_TIMEOUT_S  seconds to wait for every robot stream to arrive before
#                         discarding the attempt (default 120)
#   SEED_TIMEOUT_S        seconds to wait for AMCL to receive the initial pose (default 30)
#   BIDDING_WINDOW_S      auction window per task (default 5.0; RMF's own default is 2.0)
#   FIXED_WAIT_S          upper bound on one repeat in seconds (default 300)
#   N3_REPORT_ONLY=1      count a repeat even when it measured fewer than three robots
#   HW_RESUME=1           continue the newest run directory of each scenario
#   HW_UNATTENDED=1       do not pause for the operator between repeats
#   HW_OPERATOR_TIMEOUT   seconds to wait at the operator prompt (default 900)
#   HW_ROS_DOMAIN_ID      ROS domain of the workstation stack (default 12)
#   API_SERVER_PORT       port the websocket state recorder listens on (default 8411)
#   RUN_SUFFIX            suffix appended to the run directory name, for a run kept
#                         apart from the collection; empty by default
#   FLEET_CONFIG          fleet config the adapter is launched with
#   NAV_GRAPH             navigation graph the routes are read from
#

set -o pipefail

WS=${WS:-/home/eiu/rmf_ws}
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1

export ROS_DOMAIN_ID=${HW_ROS_DOMAIN_ID:-12}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

BENCHMARK_DIR=$WS/src/benchmark
COLLECT_HW=$BENCHMARK_DIR/collect/hardware
COLLECT_RMF=$BENCHMARK_DIR/collect/rmf
ANALYSE_TASK_PLANNING=$BENCHMARK_DIR/analyse/task_planning
ANALYSE_TRAFFIC_SCHEDULING=$BENCHMARK_DIR/analyse/traffic_scheduling
OUTPUT_DIR=$BENCHMARK_DIR/data/hardware/scenarios
TB3_FLEET_SHARE=$WS/install/tb3_fleet/share/tb3_fleet
NAV_GRAPH=${NAV_GRAPH:-$TB3_FLEET_SHARE/maps/world_tb3/nav_graphs/0.yaml}
ZENOH_BRIDGE=$TB3_FLEET_SHARE/bin/zenoh-bridge-ros2dds
ZENOH_BRIDGE_CONFIG=$COLLECT_HW/zenoh_bridge_workstation.json5
ZENOH_ROUTER_CONFIG=$TB3_FLEET_SHARE/config/hardware/zenoh_router/tb3_zenoh_router_config_hw.json5
FLEET_CONFIG=${FLEET_CONFIG:-$TB3_FLEET_SHARE/config/hardware/fleet/multi_robot_config_hw.yaml}
LOCK_FILE=$BENCHMARK_DIR/.session/hardware.pid
SESSION_DIR=$BENCHMARK_DIR/.session/hardware   
ZENOH_ROUTER_PORT=7447
ZENOH_BRIDGE_COMM=zenoh-bridge-ro          

ROBOTS="tb3_robot1 tb3_robot2 tb3_robot3"
API_SERVER_PORT=${API_SERVER_PORT:-8411}
TARGET_REPEATS=${TARGET_REPEATS:-5}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-8}
RUN_SUFFIX=${RUN_SUFFIX:-}
RUN_TAIL=${RUN_SUFFIX:+_$RUN_SUFFIX}
HW_MIN_SOC=${HW_MIN_SOC:-0.50}
HW_UNATTENDED=${HW_UNATTENDED:-0}
HW_OPERATOR_TIMEOUT=${HW_OPERATOR_TIMEOUT:-900}
HW_RESUME=${HW_RESUME:-0}
HW_NO_GROUND_TRUTH=${HW_NO_GROUND_TRUTH:-0}
REGISTRATION_TIMEOUT_S=${REGISTRATION_TIMEOUT_S:-120}
AMCL_SETTLE_S=${AMCL_SETTLE_S:-30}
REACHABILITY_TIMEOUT_S=${REACHABILITY_TIMEOUT_S:-120}
ACTION_ROUTE_TIMEOUT_S=${ACTION_ROUTE_TIMEOUT_S:-30}
STOP_CONFIRM_TIMEOUT_S=${STOP_CONFIRM_TIMEOUT_S:-60}
SEED_TIMEOUT_S=${SEED_TIMEOUT_S:-30}
BIDDING_WINDOW_S=${BIDDING_WINDOW_S:-5.0}
FIXED_WAIT_S=${FIXED_WAIT_S:-300}
N3_GATE_ARGS=""
[ "${N3_REPORT_ONLY:-0}" = "1" ] && N3_GATE_ARGS="--report-only"

BAG_PID=""; GT_PID=""; API_LOGGER_PID=""

LOGGER_NAME=hardware_scenario_collector
LOG_FILE=$OUTPUT_DIR/hardware_scenarios_$(date -u +%Y%m%d_%H%M).log
mkdir -p "$OUTPUT_DIR" "$BENCHMARK_DIR/.session" "$SESSION_DIR"

log() {
    local severity="$1"; shift
    printf '[%s] [%s] [%s]: %s\n' \
        "$severity" "$(date +%s.%N)" "$LOGGER_NAME" "$*" | tee -a "$LOG_FILE"
}
log_info()  { log INFO "$@"; }
log_warn()  { log WARN "$@"; }
log_error() { log ERROR "$@"; }

# Topics recorded per repeat. The RMF topics match the simulation scenarios; the robot
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
    /rmf_traffic/schedule_startup /task_api_requests /task_api_responses
    /lane_states
    "/tb3_robot1/amcl_pose" "/tb3_robot2/amcl_pose" "/tb3_robot3/amcl_pose"
    "/tb3_robot1/battery_state" "/tb3_robot2/battery_state" "/tb3_robot3/battery_state"
    "/tb3_robot1/cmd_vel" "/tb3_robot2/cmd_vel" "/tb3_robot3/cmd_vel"
    "/tb3_robot1/tf" "/tb3_robot2/tf" "/tb3_robot3/tf"
    "/tb3_robot1/tf_static" "/tb3_robot2/tf_static" "/tb3_robot3/tf_static"
    "/tb3_robot1/scan" "/tb3_robot2/scan" "/tb3_robot3/scan"
    "/tb3_robot1/plan" "/tb3_robot2/plan" "/tb3_robot3/plan"
)

# Scenario table: label | directory | route 1 | route 2 | route 3 | lanes closed | faces.

SCENARIOS_TABLE=(
"Bottleneck|bottleneck|wp_2,wp_5|wp_3,wp_4|wp_4,wp_3|wp_6,wp_7|wp_3,wp_2,wp_2"
"Crossing|crossing|wp_2,wp_6|wp_6,wp_9|wp_7,wp_8||wp_4,wp_7,wp_6"
"Head-on|headon|wp_2,wp_5|wp_5,wp_2|wp_3,wp_7||wp_3,wp_3,wp_5"
"Shared Lane|sharedlane|wp_2,wp_7|wp_4,wp_5|wp_6,wp_3||wp_3,wp_2,wp_4"
)

set_placement() {
    local s1="$1" s2="$2" s3="$3" faces="$4"
    local starts=("${s1%%,*}" "${s2%%,*}" "${s3%%,*}")
    local f k
    IFS=',' read -r -a f <<< "$faces"
    PARKING=()
    for k in 0 1 2; do
        PARKING+=("${starts[$k]},${f[$k]}")
    done
}

PARKING=()

SCENARIO_FILTER="${1:-Bottleneck,Crossing,Head-on,Shared Lane}"


acquire_lock() {
    local holder
    if [ -f "$LOCK_FILE" ]; then
        holder=$(cat "$LOCK_FILE" 2>/dev/null)
        if [ -n "$holder" ] && kill -0 "$holder" 2>/dev/null; then
            log_error "zenoh: another hardware run is active, pid [$holder]. Stop it, or remove $LOCK_FILE if it is stale."
            return 1
        fi
        log_warn "zenoh: taking over a stale lock from pid [$holder]."
    fi
    echo $$ > "$LOCK_FILE"
    trap release_lock EXIT
    return 0
}

release_lock() { [ "$(cat "$LOCK_FILE" 2>/dev/null)" = "$$" ] && rm -f "$LOCK_FILE"; }


# require_scripts: abort unless every script this runner invokes is present.
require_scripts() {
    local missing=() path
    for path in "$COLLECT_HW/check_hardware_preflight.py" \
                "$COLLECT_HW/merge_repeat_summaries.py" \
                "$COLLECT_HW/check_zenoh_routes.py" \
                "$COLLECT_HW/record_ground_truth_poses_hw.py" \
                "$COLLECT_RMF/run_benchmark_concurrent.py" \
                "$ANALYSE_TASK_PLANNING/analyze_task_planning.py" \
                "$ANALYSE_TRAFFIC_SCHEDULING/analyze_traffic_scheduling.py" \
                "$ANALYSE_TRAFFIC_SCHEDULING/merge_repeat_metrics.py" \
                "$COLLECT_RMF/record_api_state.py"; do
        [ -f "$path" ] || missing+=("$path")
    done
    if [ ${#missing[@]} -gt 0 ]; then
        log_error "collection: missing script(s): ${missing[*]}"
        return 1
    fi
    return 0
}

# kill_group: terminate a process group started with setsid, by its leader pid.
kill_group() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null
    fi
}

# bridge_pids: pids of the workstation zenoh bridge.
bridge_pids() {
    ps -eo pid,comm | awk -v comm="$ZENOH_BRIDGE_COMM" '$2 == comm { print $1 }'
}

# zenoh_up: start the zenoh router and the workstation bridge unless they already run.
zenoh_up() {
    if ! ss -ltn 2>/dev/null | grep -q ":$ZENOH_ROUTER_PORT "; then
        setsid zenohd -c "$ZENOH_ROUTER_CONFIG" > "$SESSION_DIR/zenohd.log" 2>&1 &
        sleep 3
        if ! ss -ltn 2>/dev/null | grep -q ":$ZENOH_ROUTER_PORT "; then
            log_error "zenoh: router [zenohd] did not start; see [$SESSION_DIR/zenohd.log]."
            return 1
        fi
        log_info "zenoh: started router [zenohd] on port [$ZENOH_ROUTER_PORT]."
    fi
    if [ -z "$(bridge_pids)" ]; then
        setsid "$ZENOH_BRIDGE" -c "$ZENOH_BRIDGE_CONFIG" \
            > "$SESSION_DIR/zenoh_bridge_workstation.log" 2>&1 &
        sleep 5
        if [ -z "$(bridge_pids)" ]; then
            log_error "zenoh: workstation [zenoh-bridge-ros2dds] did not start; see [$SESSION_DIR/zenoh_bridge_workstation.log]."
            return 1
        fi
        log_info "zenoh: started workstation [zenoh-bridge-ros2dds]."
    fi
    return 0
}

# hold_topics_up: keep one subscriber per robot topic alive for the session, leaving the
hold_topics_up() {
    local r pidfile
    for r in $ROBOTS; do
        pidfile=$SESSION_DIR/hold_robot_topics_$r.pid
        if [ -f "$pidfile" ] && kill -0 "$(cat "$pidfile")" 2>/dev/null; then
            continue
        fi
        setsid python3 "$COLLECT_HW/hold_robot_topics.py" --robot "$r" \
            > "$SESSION_DIR/hold_robot_topics_$r.log" 2>&1 &
        echo $! > "$pidfile"
        sleep 3
        if ! kill -0 "$(cat "$pidfile")" 2>/dev/null; then
            log_error "zenoh: topic holder for [$r] did not start; see [$SESSION_DIR/hold_robot_topics_$r.log]."
            return 1
        fi
    done
    log_info "zenoh: holding the bridge routes open for 3 robots."
    return 0
}

# stop_robot_navigation: cancel every Nav2 goal on every robot.
#
stop_robot_navigation() {
    python3 "$COLLECT_HW/stop_robot_navigation.py" \
        --fleet-config "$FLEET_CONFIG" --robots $ROBOTS \
        >> "$SESSION_DIR/stop_robot_navigation.log" 2>&1 \
        && log_info "Navigation stopped on 3 robots." \
        || log_warn "Navigation not confirmed stopped; see [$SESSION_DIR/stop_robot_navigation.log]."
}

# teardown: stop the RMF stack and the recorder. The zenoh router, the workstation bridge,
teardown() {
    log_info "collection: stopping the workstation stack."
    stop_robot_navigation
    kill_group "$BAG_PID"; kill_group "$GT_PID"; kill_group "$FA_PID"
    sleep 6
    kill_group "$BAG_PID"; kill_group "$GT_PID"; kill_group "$FA_PID"
    sleep 2
    pkill -9 -f "tb3_fleet.tb3_fleet_adapter" 2>/dev/null
    pkill -9 -f "ros2 bag record" 2>/dev/null
    pkill -9 -f "record_api_state.py" 2>/dev/null
    pkill -9 -f "record_ground_truth_poses_hw.py" 2>/dev/null
    pkill -9 -f "rmf_traffic" 2>/dev/null
    pkill -9 -f "rmf_fleet_adapter" 2>/dev/null
    pkill -9 -f "rmf_task_dispatcher" 2>/dev/null
    pkill -9 -f "rmf_visualization" 2>/dev/null
    pkill -9 -f "building_map_server" 2>/dev/null
    pkill -9 -f "tb3_fleet.set_initial_pose" 2>/dev/null
    sleep 3
    log_info "collection: workstation stack stopped."
}


# missing_streams: echo the robot streams that carry no sample right now, as they arrive
missing_streams() {
    local missing=""
    local r
    for r in $ROBOTS; do
        timeout 5 ros2 topic echo --once --qos-reliability best_effort \
            "/$r/scan" >/dev/null 2>&1 || missing="$missing $r/scan"
        timeout 5 ros2 topic echo --once "/$r/tf" >/dev/null 2>&1 \
            || missing="$missing $r/tf"
    done
    echo "$missing"
}

# nav_action_served: report every robot that is not serving navigate_to_pose, as
nav_action_served() {
    local out rc
    out=$(python3 "$COLLECT_HW/check_zenoh_routes.py" \
          --fleet-config "$FLEET_CONFIG" --robots $ROBOTS \
          2>> "$SESSION_DIR/check_zenoh_routes.log")
    rc=$?
    if [ "$rc" -ne 0 ] && [ -z "$out" ]; then
        echo "checker exited $rc without reporting"
        return 0
    fi
    printf '%s' "$out"
}

# restart_topic_holder: replace one robot's topic holder.
restart_topic_holder() {
    local r="$1" pidfile=$SESSION_DIR/hold_robot_topics_$1.pid
    local pid
    pid=$(cat "$pidfile" 2>/dev/null)
    [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null
    rm -f "$pidfile"
    log_info "zenoh: replacing the topic holder for [$r] to rebuild its bridge routes."
    hold_topics_up
}

# rebuild_bridge_routes: replace the workstation bridge and every topic holder.
rebuild_bridge_routes() {
    local pid r
    log_warn "zenoh: streams still absent after replacing every holder; rebuilding the workstation bridge."
    for pid in $(bridge_pids); do kill -9 "$pid" 2>/dev/null; done
    for r in $ROBOTS; do
        pid=$(cat "$SESSION_DIR/hold_robot_topics_$r.pid" 2>/dev/null)
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null
        rm -f "$SESSION_DIR/hold_robot_topics_$r.pid"
    done
    sleep 3
    zenoh_up || return 1
    hold_topics_up || return 1
    sleep 5
    return 0
}

robots_reachable() {
    local deadline=$((SECONDS + REACHABILITY_TIMEOUT_S))
    local missing restarted="" rebuilt=0 r
    missing=$(missing_streams)
    while [ -n "$missing" ] && [ "$SECONDS" -lt "$deadline" ]; do
        log_info "zenoh: waiting for robot streams, missing:$missing"
        for r in $ROBOTS; do
            case "$missing" in *"$r/"*) ;; *) continue ;; esac
            case " $restarted " in *" $r "*) continue ;; esac
            restart_topic_holder "$r"
            restarted="$restarted $r"
        done
        sleep 5
        missing=$(missing_streams)
        local pending=""
        for r in $ROBOTS; do
            case "$missing" in *"$r/"*) ;; *) continue ;; esac
            case " $restarted " in *" $r "*) continue ;; esac
            pending="$pending $r"
        done
        if [ -n "$missing" ] && [ "$rebuilt" -eq 0 ] && [ -z "$pending" ]; then
            rebuild_bridge_routes || return 1
            rebuilt=1
            missing=$(missing_streams)
        fi
    done

    if [ -n "$missing" ]; then
        log_error "zenoh: robot streams not reachable after ${REACHABILITY_TIMEOUT_S}s, missing:$missing"
        log_error "navigation: on every robot, check bring-up and Nav2 on that robot's own ROS_DOMAIN_ID with RMW_IMPLEMENTATION [$RMW_IMPLEMENTATION]."
        log_error "zenoh: on this machine, check the router, the workstation bridge and the route holders; their logs are in [$SESSION_DIR]."
        log_error "zenoh: a bridge restarted on a robot retires its routes and announces them again, so do not restart one while this check runs."
        return 1
    fi

    local unserved
    deadline=$((SECONDS + ACTION_ROUTE_TIMEOUT_S))
    unserved=$(nav_action_served)
    while [ -n "$unserved" ] && [ "$SECONDS" -lt "$deadline" ]; do
        log_info "navigation: waiting for navigate_to_pose to be announced by:$(echo "$unserved" | awk '{ printf " %s", $1 }')"
        sleep 5
        unserved=$(nav_action_served)
    done
    if [ -n "$unserved" ]; then
        log_error "navigation: not serving navigate_to_pose after ${ACTION_ROUTE_TIMEOUT_S}s; the fleet adapter cannot command these robots:"
        echo "$unserved" | while read -r r verdict; do
            case "$verdict" in
            robot) log_error "  [$r] publishes its topics but serves no action. Restart Nav2 on that robot; this machine cannot supply it." ;;
            *)     log_error "  [$r] announces nothing. Check its bridge and the link; logs are in [$SESSION_DIR]." ;;
            esac
        done
        log_error "zenoh: detail is in [$SESSION_DIR/check_zenoh_routes.log]."
        return 1
    fi

    log_info "preflight: topics=/scan,/tf action=NavigateToPose robots=3/3."
    return 0
}

# battery_ok: read each robot's /battery_state and reject a charge below HW_MIN_SOC.
battery_ok() {
    local bad=""
    local r
    for r in $ROBOTS; do
        local soc
        soc=$(timeout 10 ros2 topic echo --once --field percentage \
              "/$r/battery_state" 2>/dev/null | head -1 | tr -d '[:space:]')
        if [ -z "$soc" ]; then
            log_warn "collection: robot [$r] publishes no battery_state, charge not verified."
            continue
        fi
        soc=$(awk -v v="$soc" 'BEGIN { printf "%.3f", (v > 1.0 ? v / 100.0 : v) }')
        log_info "collection: robot [$r] state of charge [$soc]."
        if awk -v v="$soc" -v floor="$HW_MIN_SOC" 'BEGIN { exit !(v < floor) }'; then
            bad="$bad $r($soc)"
        fi
    done
    if [ -n "$bad" ]; then
        log_error "collection: charge below HW_MIN_SOC [$HW_MIN_SOC] on:$bad. Charge these robots before continuing."
        return 1
    fi
    return 0
}

# flush_stdin: discard input queued before the prompt.
flush_stdin() {
    local junk
    while read -r -t 0 junk 2>/dev/null; do
        read -r -t 1 junk 2>/dev/null || break
    done
}

# start_pose: echo "x y yaw" for one "waypoint,facing" pair, read from the navigation
start_pose() {
    local pair="$1"
    python3 -m tb3_fleet.set_initial_pose \
        --robot-namespace unused \
        --nav-graph "$NAV_GRAPH" \
        --waypoint "${pair%%,*}" \
        --toward "${pair##*,}" \
        --print-pose
}

# seed_initial_poses: publish an AMCL initial pose for every robot, from the navigation
seed_initial_poses() {
    local r pair rc i=0
    local failed=""
    for r in $ROBOTS; do
        pair=${PARKING[$i]}
        i=$((i+1))
        python3 -m tb3_fleet.set_initial_pose \
            --robot-namespace "$r" \
            --nav-graph "$NAV_GRAPH" \
            --waypoint "${pair%%,*}" \
            --toward "${pair##*,}" \
            --timeout-sec "$SEED_TIMEOUT_S" \
            >> "$SESSION_DIR/set_initial_pose.log" 2>&1
        rc=$?
        [ "$rc" -ne 0 ] && failed="$failed $r"
    done
    if [ -n "$failed" ]; then
        log_error "localization: seeding the initial pose failed for:$failed"
        log_error "zenoh: see [$SESSION_DIR/set_initial_pose.log]. The robot bridges must allow"
        log_error "localization: [.*/initialpose] under subscribers for the seed to reach AMCL."
        return 1
    fi
    log_info "localization: seeded the initial pose of 3 robots from the navigation graph."
    return 0
}

manage_lane_request() {
    local action="$1" pair="$2"
    [ -z "$pair" ] && return 0
    local done_word=closed
    [ "$action" = open ] && done_word=opened
    python3 -m rmf_demos_fleet_adapter.manage_lane "$action" \
        "${pair%%,*}" "${pair##*,}" --bidir --nav_graph "$NAV_GRAPH" \
        >> "$SESSION_DIR/manage_lane.log" 2>&1 \
        && log_info "Lane ${pair} ${done_word} for this scenario." \
        || log_warn "Lane ${action} for ${pair} failed; see [$SESSION_DIR/manage_lane.log]."
}

# require_tty: refuse to run without a terminal.
require_tty() {
    if [ ! -t 0 ]; then
        log_error "collection: stdin is not a terminal. Run this from an interactive shell, or set HW_UNATTENDED=1."
        return 1
    fi
    return 0
}

# wait_for_operator: pause for the operator to place the three robots on their start
wait_for_operator() {
    local scenario="$1" repeat="$2"
    shift 2
    if [ "$HW_UNATTENDED" = "1" ]; then
        log_info "operator: unattended mode enabled; prompt skipped."
        return 0
    fi
    flush_stdin
    local i=1 waypoint
    log_info "operator: placement requested; scenario=$scenario repeat=$repeat."
    for waypoint in "$@"; do
        local pose
        pose=$(start_pose "$waypoint")
        log_info "operator: robot=tb3_robot${i} waypoint=${waypoint%%,*} position=($(echo "$pose" | awk '{print $1", "$2}')) facing=${waypoint##*,} yaw=$(echo "$pose" | awk '{print $3}') rad."
        i=$((i+1))
    done
    log_info "operator: press ENTER to publish initial poses to AMCL."
    read -r -t "$HW_OPERATOR_TIMEOUT" _ && return 0
    log_warn "operator: prompt timeout=${HW_OPERATOR_TIMEOUT}s; proceeding."
    return 0
}

# ------------------------------------------------------------------- bring-up
# wait_for_amcl_transform: wait until AMCL on every robot broadcasts map -> odom.

wait_for_amcl_transform() {
    local r deadline seen silent=""
    for r in $ROBOTS; do
        deadline=$((SECONDS + AMCL_SETTLE_S))
        seen=""
        while [ -z "$seen" ] && [ "$SECONDS" -lt "$deadline" ]; do
            seen=$(timeout 5 ros2 topic echo "/$r/tf" 2>/dev/null \
                   | grep -m1 "frame_id: map")
        done
        [ -z "$seen" ] && silent="$silent $r"
    done
    if [ -n "$silent" ]; then
        log_warn "localization: no map -> odom from AMCL within ${AMCL_SETTLE_S}s on:$silent"
        log_warn "localization: the seed did not reach AMCL, or AMCL is not running on that robot."
        return 0
    fi
    log_info "localization: map->odom available; robots=3/3."
    return 0
}

# start_rmf_and_wait: start the websocket recorder and the RMF core, then
start_rmf_and_wait() {
    local run_dir="$1" log_name="$2"
    FA_LOG="$run_dir/$log_name"
    setsid python3 $COLLECT_RMF/record_api_state.py \
        --out "$run_dir/api_state_attempt${attempt_id}.jsonl" --port "$API_SERVER_PORT" \
        > "$run_dir/api_state_logger_attempt${attempt_id}.log" 2>&1 &
    API_PID=$!
    sleep 3

    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        fleet_config_file:="$FLEET_CONFIG" \
        bidding_time_window:="$BIDDING_WINDOW_S" \
        use_sim_time:=false \
        server_uri:=ws://127.0.0.1:$API_SERVER_PORT \
        > "$FA_LOG" 2>&1 &
    FA_PID=$!

    local deadline=$((SECONDS + REGISTRATION_TIMEOUT_S)) waited=0
    while [ "$SECONDS" -lt "$deadline" ]; do
        if grep -q "Unable to compute a location on the navigation graph" "$FA_LOG" 2>/dev/null; then
            log_error "navigation: navigation-graph failure in [$log_name]: a robot could not be placed on a lane."
            return 1
        fi
        COUNT=$(grep -c "Successfully added robot" "$FA_LOG" 2>/dev/null)
        COUNT=${COUNT:-0}
        if [ "$COUNT" -ge 3 ] 2>/dev/null; then
            log_info "fleet: registered robots=3/3 elapsed=${waited}s."
            return 0
        fi
        if ! kill -0 "$FA_PID" 2>/dev/null; then
            log_error "fleet: the fleet stack exited after ${waited}s with $COUNT of 3 robots added; see [$log_name]."
            return 1
        fi
        sleep 2
        waited=$((waited + 2))
    done
    log_error "fleet: robots did not join the fleet within ${REGISTRATION_TIMEOUT_S}s; $COUNT of 3 added. See [$log_name]."
    return 1
}

# run_scenario: collect TARGET_REPEATS repeats of one scenario. Each
run_scenario() {
    local scenario="$1" dirname="$2" places1="$3" places2="$4" places3="$5"
    local closed_lanes="$6" faces="$7"
    [[ ",$SCENARIO_FILTER," == *",$scenario,"* ]] || return 0

    local run_dir=""
    if [ "$HW_RESUME" = "1" ]; then
        run_dir=$(find "$OUTPUT_DIR/$dirname" -maxdepth 1 -type d -name "run_*${RUN_TAIL}" \
                  2>/dev/null | sort | tail -1)
    fi
    : "${run_dir:=$OUTPUT_DIR/$dirname/run_$(date -u +%Y%m%d_%H%M)${RUN_TAIL}}"
    mkdir -p "$run_dir"

    local completed attempt attempt_id analysis_rc abort_scenario=0
    completed=$(find "$run_dir" -maxdepth 1 -type d -name 'repeat_[0-9]*' 2>/dev/null | wc -l)
    if [ "$completed" -ge "$TARGET_REPEATS" ]; then
        log_info "collection: scenario [$scenario] already holds $completed of $TARGET_REPEATS repeats."
        return 0
    fi
    log_info "collection: scenario=$scenario status=started existing_repeats=$completed output=$run_dir."
    if [ "$HW_NO_GROUND_TRUTH" = "1" ]; then
        printf '%s\n' \
            '{' \
            '  "external_tracker": false,' \
            '  "pose_source": "amcl",' \
            '  "unavailable_metrics": ["F7", "localisation gate"]' \
            '}' > "$run_dir/ground_truth.json"
    fi

    attempt=0
    attempt_id=$(find "$run_dir" -maxdepth 1 \
                      \( -name '*attempt[0-9]*' -o -name 'attempt_[0-9]*' \) 2>/dev/null \
                 | sed -n 's/.*attempt_\{0,1\}\([0-9]\{1,\}\).*/\1/p' | sort -n | tail -1)
    attempt_id=${attempt_id:-0}
    while [ "$completed" -lt "$TARGET_REPEATS" ] && [ "$attempt" -lt "$MAX_ATTEMPTS" ]; do
        attempt=$((attempt+1)); attempt_id=$((attempt_id + 1))
        local i=$((completed+1))
        log_info "collection: scenario=$scenario repeat=$i/$TARGET_REPEATS attempt=$attempt."
        teardown
        zenoh_up || return 1
        hold_topics_up || return 1

        if ! robots_reachable; then
            log_warn "collection: a robot is unreachable, discarding attempt $attempt before placing the robots."
            continue
        fi

        set_placement "$places1" "$places2" "$places3" "$faces"
        wait_for_operator "$scenario" "$i" "${PARKING[@]}"

        if ! seed_initial_poses; then
            log_warn "localization: initial pose not seeded, discarding attempt $attempt. Nothing was recorded."
            continue
        fi

        wait_for_amcl_transform

        log_info "preflight: checking battery state; minimum_soc=$HW_MIN_SOC."
        if ! battery_ok; then
            log_warn "collection: charge too low, discarding attempt $attempt. Repeat $i will be retried."
            continue
        fi

        if ! start_rmf_and_wait "$run_dir" "fleet_adapter_attempt${attempt_id}.log"; then
            log_warn "fleet: the RMF stack did not come up, discarding attempt $attempt. $completed of $TARGET_REPEATS repeats done."
            continue
        fi
        sleep 5
        manage_lane_request close "$closed_lanes"

        local attempt_dir="$run_dir/attempt_${attempt_id}"
        mkdir -p "$attempt_dir"
        setsid ros2 bag record -o "$attempt_dir/bag" "${BAG_TOPICS[@]}" \
            > "$attempt_dir/rosbag_record.log" 2>&1 &
        BAG_PID=$!

        log_info "task: submitting scenario [$scenario] for repeat $i."
        local window_start window_end
        window_start=$(date +%s)
        timeout 720 python3 $COLLECT_RMF/run_benchmark_concurrent.py \
            --route "$places1" --route "$places2" --route "$places3" \
            --rounds 1 --repeats 1 \
            --robots $ROBOTS \
            --fixed-wait "$FIXED_WAIT_S" --min-expected-distance-m 5 \
            --api-state "$run_dir/api_state_attempt${attempt_id}.jsonl" \
            --output-dir "$attempt_dir" \
            > "$run_dir/run_benchmark_attempt${attempt_id}.log" 2>&1
        runner_rc=$?
        window_end=$(date +%s)
        log_info "task: task runner exited $runner_rc."

        sleep 10
        kill_group "$BAG_PID"
        kill_group "$GT_PID"
        sleep 5
        ros2 bag info "$attempt_dir/bag" > "$attempt_dir/bag_info.log" 2>&1
        bag_info_rc=$?

        bag_messages=$(grep -oE "^Messages:[[:space:]]+[0-9]+" "$attempt_dir/bag_info.log" \
            | grep -oE "[0-9]+" | head -1)
        bag_messages=${bag_messages:-0}
        if [ "$bag_info_rc" -ne 0 ] || [ "$bag_messages" -lt 1 ]; then
            log_warn "recording: bag empty or unreadable, exit $bag_info_rc with $bag_messages message(s), discarding attempt $attempt."
            mv "$attempt_dir" "$run_dir/discarded_bad_bag_attempt${attempt_id}"
            continue
        fi

        PRE_GT_ARG=""
        [ "$HW_NO_GROUND_TRUTH" = "1" ] && PRE_GT_ARG="--no-ground-truth"
        python3 $COLLECT_HW/check_hardware_preflight.py \
            --repeat-dir "$attempt_dir" \
            --robots $ROBOTS \
            --attempt "$attempt_id" $PRE_GT_ARG \
            --window-start "$window_start" --window-end "$window_end" \
            > "$attempt_dir/hardware_preflight.log" 2>&1
        PRE_RC=$?
        if [ "$PRE_RC" -eq 1 ]; then
            log_warn "preflight: failed, see $attempt_dir/hardware_preflight.log, discarding attempt $attempt."
            mv "$attempt_dir" "$run_dir/discarded_preflight_attempt${attempt_id}"
            continue
        elif [ "$PRE_RC" -eq 2 ]; then
            log_error "recording: preflight could not run: the ground-truth file is missing or empty. Stopping scenario [$scenario]."
            mv "$attempt_dir" "$run_dir/discarded_no_ground_truth_attempt${attempt_id}"
            break
        elif [ "$PRE_RC" -ne 0 ]; then
            log_warn "preflight: exited $PRE_RC, which is neither keep nor a known failure; the repeat is unchecked. Discarding attempt $attempt."
            mv "$attempt_dir" "$run_dir/discarded_preflight_rc${PRE_RC}_attempt${attempt_id}"
            continue
        fi

        python3 $ANALYSE_TRAFFIC_SCHEDULING/analyze_traffic_scheduling.py \
            --bag "$attempt_dir/bag" \
            --robots $ROBOTS \
            --scenario "$scenario, N=3, CONFIG_01, HARDWARE, repeat $i" \
            --output "$attempt_dir/traffic_scheduling_metrics.json" \
            > "$attempt_dir/analyze.log" 2>&1
        analysis_rc=$?
        log_info "analysis: traffic scheduling for repeat $i exited $analysis_rc."
        if [ "$analysis_rc" -ne 0 ]; then
            log_warn "analysis: traffic scheduling failed, so the repeat has no conflict metrics. Discarding attempt $attempt."
            mv "$attempt_dir" "$run_dir/discarded_analysis_attempt${attempt_id}"
            continue
        fi

        python3 $ANALYSE_TASK_PLANNING/analyze_task_planning.py \
            --bag "$attempt_dir/bag" \
            --api-state "$run_dir/api_state_attempt${attempt_id}.jsonl" \
            --robots $ROBOTS \
            --scenario "$scenario, N=3, CONFIG_01, HARDWARE, repeat $i" \
            --nav-graph "$NAV_GRAPH" \
            --output "$attempt_dir/task_planning_metrics.json" \
            >> "$attempt_dir/analyze.log" 2>&1
        analysis_rc=$?
        log_info "task: task-planning analysis for repeat $i exited $analysis_rc."
        if [ "$analysis_rc" -ne 0 ]; then
            log_warn "task: task-planning analysis failed, so the repeat has no task metrics and the gate has nothing to read. Discarding attempt $attempt."
            mv "$attempt_dir" "$run_dir/discarded_analysis_attempt${attempt_id}"
            continue
        fi

        python3 $COLLECT_RMF/annotate_task_outcomes.py \
            --repeats-summary "$attempt_dir/repeats_summary.json" \
            --task-planning-metrics "$attempt_dir/task_planning_metrics.json" \
            >> "$attempt_dir/analyze.log" 2>&1
        analysis_rc=$?
        log_info "task: outcome annotation for repeat $i exited $analysis_rc."
        if [ "$analysis_rc" -ne 0 ]; then
            log_warn "task: outcome annotation failed, so each task's terminal status was not written into the summary. Discarding attempt $attempt."
            mv "$attempt_dir" "$run_dir/discarded_analysis_attempt${attempt_id}"
            continue
        fi

        python3 "$COLLECT_HW/check_n3_coordination.py" \
            --task-planning-metrics "$attempt_dir/task_planning_metrics.json" \
            --traffic-scheduling-metrics "$attempt_dir/traffic_scheduling_metrics.json" \
            --api-state "$run_dir/api_state_attempt${attempt_id}.jsonl" \
            --nav-graph "$NAV_GRAPH" \
            --window-start "$window_start" --window-end "$window_end" \
            --robots $ROBOTS $N3_GATE_ARGS \
            2>&1 | tee -a "$attempt_dir/n3_coordination.log" | tee -a "$LOG_FILE"
        n3_rc=${PIPESTATUS[0]}
        if [ "$n3_rc" -ne 0 ]; then
            mv "$attempt_dir" "$run_dir/not_n3_attempt_${attempt_id}"
            log_warn "collection: attempt $attempt recorded but is not an N=3 sample; kept as [not_n3_attempt_${attempt_id}] and not counted. See its n3_coordination.log."
            continue
        fi

        mv "$attempt_dir" "$run_dir/repeat_${i}"
        if [ "$runner_rc" -ne 0 ]; then
            log_warn "preflight: scenario [$scenario] repeat $i kept with task runner exit $runner_rc; the preflight passed."
        fi
        completed=$((completed+1))
    done

    log_info "collection: scenario=$scenario status=complete repeats=$completed/$TARGET_REPEATS attempts=$attempt."

    python3 "$COLLECT_HW/merge_repeat_summaries.py" --run-dir "$run_dir" \
        2>&1 | tee -a "$LOG_FILE"
    log_info "analysis: repeat summary merge for scenario [$scenario] exited ${PIPESTATUS[0]}."

    python3 $ANALYSE_TRAFFIC_SCHEDULING/merge_repeat_metrics.py \
        --run-dir "$run_dir" \
        --scenario "$scenario, N=3, CONFIG_01, HARDWARE" \
        --robots $ROBOTS \
        > "$run_dir/merge_metrics.log" 2>&1
    log_info "analysis: repeat metrics merge for scenario [$scenario] exited $?."

    teardown
    log_info "collection: scenario [$scenario] written to [$run_dir]."

    if [ "$abort_scenario" -ne 0 ]; then
        log_error "collection: scenario [$scenario] stopped early; what was collected has been merged."
        return 1
    fi
}

require_scripts || exit 1
acquire_lock || exit 1
if [ "$HW_UNATTENDED" != "1" ] && ! require_tty; then
    exit 1
fi

log_info "collection: hardware N=3 collection started; scenarios=$SCENARIO_FILTER robots=3."
log_info "preflight: fleet config [$FLEET_CONFIG]."
log_info "navigation: navigation graph [$NAV_GRAPH]."

for entry in "${SCENARIOS_TABLE[@]}"; do
    IFS='|' read -r label dirname places1 places2 places3 closed_lanes faces <<< "$entry"
    run_scenario "$label" "$dirname" "$places1" "$places2" "$places3" "$closed_lanes" "$faces"
done

log_info "collection: hardware N=3 scenarios finished; selection=$SCENARIO_FILTER."