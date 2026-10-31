#!/bin/bash
#
# Collects the four N=3 scenarios in Gazebo, repeating each scenario TARGET_REPEATS
# times, into data/simulation/scenarios/.
#
# Each repeat starts from a full teardown and bring-up of the zenoh router, the
# simulation, the bridge, the RMF core and the fleet adapter. An attempt whose stack
# fails to come up, whose bag is unreadable, or in which the physics engine displaced a
# robot is discarded and retried.
#
#   $1                scenarios to collect, comma-separated (default all four)
#   TARGET_REPEATS    repeats per scenario (default 5)
#   MAX_ATTEMPTS      give up on a scenario after this many attempts (default 15)
#   RUN_SUFFIX        suffix appended to the run directory name, for a run kept apart
#                     from the collection; empty by default
#   API_PORT          port the RMF websocket state stream is read on (default 8411)
#   SIM_RESUME=1      continue the newest run directory of each scenario
#   N3_REPORT_ONLY=1  count a repeat even when it measured fewer than three robots
#   AMCL_SETTLE_S     seconds to wait for map -> odom after the seed (default 30)
#   SEED_TIMEOUT_S    seconds to wait for AMCL to receive the initial pose (default 30)
#   REACHABILITY_TIMEOUT_S
#                     seconds to wait for every robot stream to arrive (default 120)
#   MAX_SEED_XY_ERR_M / MAX_SEED_YAW_ERR_RAD
#                     how far AMCL may settle from the seeded pose (default 0.50, 0.35)
#
# The placement, the seeding, the lane closure and the N=3 gate are the ones
# run_hardware_scenarios.sh applies, so the two platforms collect the same thing.

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
OUTPUT_DIR=$BENCHMARK_DIR/data/simulation/scenarios
mkdir -p "$OUTPUT_DIR" "$BENCHMARK_DIR/.session"
MASTER_LOG=$OUTPUT_DIR/official_run_$(date -u +%Y%m%d_%H%M).log

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$MASTER_LOG"; }

# Refuse to start while another instance is live. Two instances share one Gazebo,
# Nav2 and RMF stack. The lock holds the holder's pid, so a file left behind by a
# killed run is ignored.
LOCK=$BENCHMARK_DIR/.session/simulation.pid
if [ -f "$LOCK" ]; then
    OTHER=$(cat "$LOCK" 2>/dev/null)
    if [ -n "$OTHER" ] && kill -0 "$OTHER" 2>/dev/null; then
        log "REFUSING TO START: another run is already active (pid $OTHER)."
        log "Stop it first, or remove $LOCK if you are certain it is stale."
        exit 1
    fi
    log "found a stale lock from pid $OTHER, taking over"
fi
echo $$ > "$LOCK"
release_lock() { [ "$(cat "$LOCK" 2>/dev/null)" = "$$" ] && rm -f "$LOCK"; }
trap release_lock EXIT

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

API_PORT=${API_PORT:-8411}
TARGET_REPEATS=${TARGET_REPEATS:-5}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-15}
SIM_RESUME=${SIM_RESUME:-0}
AMCL_SETTLE_S=${AMCL_SETTLE_S:-30}
SEED_TIMEOUT_S=${SEED_TIMEOUT_S:-30}
REACHABILITY_TIMEOUT_S=${REACHABILITY_TIMEOUT_S:-120}
MAX_SEED_XY_ERR_M=${MAX_SEED_XY_ERR_M:-0.50}
MAX_SEED_YAW_ERR_RAD=${MAX_SEED_YAW_ERR_RAD:-0.35}
FLEET_CONFIG=${FLEET_CONFIG:-$TB3_FLEET_SHARE/config/simulation/fleet/multi_robot_config_sim.yaml}
ROBOTS="tb3_robot1 tb3_robot2 tb3_robot3"
GZ_WORLD=${GZ_WORLD:-sim_world}
SESSION_DIR=$BENCHMARK_DIR/.session/simulation
mkdir -p "$SESSION_DIR"
COLLECT_HW=$BENCHMARK_DIR/collect/hardware
N3_GATE_ARGS=""
[ "${N3_REPORT_ONLY:-0}" = "1" ] && N3_GATE_ARGS="--report-only"
# Where each robot stands at the start of a repeat, as "waypoint,facing". Filled by
# set_placement from the scenario row.
PARKING=()
# Kept equal to the hardware runner so the two platforms dispatch alike: one auction at a
# time means three tasks are awarded one window apart, and a long window staggers the
# concurrency these scenarios exist to measure.
BIDDING_WINDOW_S=${BIDDING_WINDOW_S:-5.0}
FIXED_WAIT_S=${FIXED_WAIT_S:-300}
RUN_SUFFIX=${RUN_SUFFIX:-}
# The path already states the platform and the collection, so the name carries only
# the timestamp unless a run is deliberately set apart.
RUN_TAIL=${RUN_SUFFIX:+_$RUN_SUFFIX}

# Topics recorded per repeat. The RMF topics match the hardware collections; /clock
# carries the simulated clock, which exists only here.
#
# Per-robot streams, namespaced per robot:
#   amcl_pose      AMCL output
#   odom           wheel odometry
#   scan           laser ranges, read for 7c
#   battery_state  state of charge
#   tf, tf_static  transform tree, the source of the adapter's reported pose
#   cmd_vel        commanded velocity
#   plan           the global path Nav2 chose
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
    /tb3_robot1/amcl_pose /tb3_robot2/amcl_pose /tb3_robot3/amcl_pose /tb3_robot1/odom
    /tb3_robot2/odom /tb3_robot3/odom /tb3_robot1/scan /tb3_robot2/scan /tb3_robot3/scan
    /tb3_robot1/battery_state /tb3_robot2/battery_state /tb3_robot3/battery_state
    /tb3_robot1/tf /tb3_robot2/tf /tb3_robot3/tf /tb3_robot1/tf_static /tb3_robot2/tf_static
    /tb3_robot3/tf_static /tb3_robot1/cmd_vel /tb3_robot2/cmd_vel /tb3_robot3/cmd_vel
    /tb3_robot1/plan /tb3_robot2/plan /tb3_robot3/plan
)

kill_group() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null
    fi
}

# robots_reachable: wait until every robot publishes the streams the fleet adapter reads.
#
# Replaces a fixed settling sleep: Gazebo, Nav2 and AMCL come up at a pace that varies
# with host load, so a fixed wait is either wasted time or a discarded attempt. This is
# the check run_hardware_scenarios.sh makes before it places the robots.
robots_reachable() {
    local deadline=$((SECONDS + REACHABILITY_TIMEOUT_S)) missing r
    while [ "$SECONDS" -lt "$deadline" ]; do
        missing=""
        for r in $ROBOTS; do
            timeout 5 ros2 topic echo --once --qos-reliability best_effort \
                "/$r/scan" >/dev/null 2>&1 || missing="$missing $r/scan"
            timeout 5 ros2 topic echo --once "/$r/tf" >/dev/null 2>&1 \
                || missing="$missing $r/tf"
        done
        [ -z "$missing" ] && { log "All 3 robots publish scan and tf."; return 0; }
        log "Waiting for robot streams, missing:$missing"
        sleep 5
    done
    log "ERROR: robot streams not reachable after ${REACHABILITY_TIMEOUT_S}s, missing:$missing"
    return 1
}

# require_scripts: abort unless every script this runner invokes is present.
require_scripts() {
    local missing=() path
    for path in "$COLLECT_HW/check_hardware_preflight.py" \
                "$COLLECT_HW/check_n3_coordination.py" \
                "$COLLECT_HW/merge_repeat_summaries.py" \
                "$COLLECT_HW/stop_robot_navigation.py" \
                "$COLLECT_SIM/check_physics_glitch.py" \
                "$COLLECT_SIM/record_ground_truth_poses.py" \
                "$COLLECT_RMF/record_api_state.py" \
                "$COLLECT_RMF/run_benchmark_concurrent.py" \
                "$COLLECT_RMF/annotate_task_outcomes.py" \
                "$ANALYSE_TASK_PLANNING/analyze_task_planning.py" \
                "$ANALYSE_TRAFFIC_SCHEDULING/analyze_traffic_scheduling.py"; do
        [ -f "$path" ] || missing+=("$path")
    done
    if [ ${#missing[@]} -gt 0 ]; then
        log "ERROR: missing script(s): ${missing[*]}"
        return 1
    fi
    return 0
}

# start_pose: echo "x y yaw" for one "waypoint,facing" pair, read from the navigation
# graph. The heading is the bearing of the lane towards the named neighbour, so no angle
# is set by eye.
start_pose() {
    local pair="$1"
    python3 -m tb3_fleet.set_initial_pose \
        --robot-namespace unused --nav-graph "$NAV_GRAPH" \
        --waypoint "${pair%%,*}" --toward "${pair##*,}" --print-pose
}

# set_placement: fill PARKING with "<start>,<faces>" per robot, from the scenario row.
# Robot k starts on route k's first waypoint, facing the second vertex of that route's
# shortest path. SCENARIO_DESIGN.md section 2.2 specifies the placement and what it means
# for the per-robot figures.
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

# place_robots: teleport each robot onto its start waypoint in Gazebo.
#
# This is what the operator does by hand on the physical fleet. Moving the model is not
# enough on its own: AMCL still believes the robot is where it was, so seed_initial_poses
# must follow.
place_robots() {
    local r pair px py pyaw qz qw k=0
    for r in $ROBOTS; do
        pair=${PARKING[$k]}; k=$((k+1))
        read -r px py pyaw < <(start_pose "$pair")
        read -r qz qw < <(python3 -c "
import math, sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2), math.cos(yaw / 2))
" "$pyaw")
        gz service -s "/world/$GZ_WORLD/set_pose" \
            --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
            --req "name: \"$r\", position: {x: $px, y: $py, z: 0.01}, orientation: {x: 0, y: 0, z: $qz, w: $qw}" \
            >> "$SESSION_DIR/set_pose.log" 2>&1 \
            || log "WARNING: could not place [$r] at ${pair%%,*}; see $SESSION_DIR/set_pose.log"
    done
    sleep 3
    log "Placed 3 robots on their route starts."
}

# seed_initial_poses: publish an AMCL initial pose for every robot, from the navigation
# graph. The pose comes from the same graph the routes are planned on, so a heading cannot
# be set by eye and no robot is left on a pose nobody recorded.
seed_initial_poses() {
    local r pair rc k=0 failed=""
    for r in $ROBOTS; do
        pair=${PARKING[$k]}; k=$((k+1))
        python3 -m tb3_fleet.set_initial_pose \
            --robot-namespace "$r" --nav-graph "$NAV_GRAPH" \
            --waypoint "${pair%%,*}" --toward "${pair##*,}" \
            --timeout-sec "$SEED_TIMEOUT_S" \
            >> "$SESSION_DIR/set_initial_pose.log" 2>&1
        rc=$?
        [ "$rc" -ne 0 ] && failed="$failed $r"
    done
    if [ -n "$failed" ]; then
        log "ERROR: seeding the initial pose failed for:$failed (see $SESSION_DIR/set_initial_pose.log)"
        return 1
    fi
    log "Seeded the initial pose of 3 robots from the navigation graph."
    return 0
}

# wait_for_amcl_transform: wait until every robot publishes map -> odom, which is the
# signal that AMCL has taken the seed. The adapter reads the robot pose from <robot>/tf
# and Nav2 refuses a goal it cannot place the robot for, so the workload cannot start
# before this.
wait_for_amcl_transform() {
    local r deadline seen silent=""
    for r in $ROBOTS; do
        deadline=$((SECONDS + AMCL_SETTLE_S))
        seen=""
        while [ -z "$seen" ] && [ "$SECONDS" -lt "$deadline" ]; do
            # Read the match from the output, not the exit status: grep -m1 closes the
            # pipe and ros2 topic echo then dies of SIGPIPE.
            seen=$(timeout 5 ros2 topic echo "/$r/tf" 2>/dev/null | grep -m1 "frame_id: map")
        done
        [ -z "$seen" ] && silent="$silent $r"
    done
    if [ -n "$silent" ]; then
        log "WARNING: no map -> odom from AMCL within ${AMCL_SETTLE_S}s on:$silent"
        return 0
    fi
    log "All 3 robots publish map -> odom."
    return 0
}

# seed_accepted: whether AMCL settled on the pose that was seeded, for every robot.
#
# map -> odom appearing only proves AMCL is running, not that it kept the seed. AMCL
# corrects the seed against the laser, so a robot placed facing the wrong way converges on
# its true heading and RMF then plans from a pose the route does not start at: it issues an
# in-place rotation that Nav2 accepts and does not finish, and the attempt burns the whole
# submitter timeout before anything says why.
seed_accepted() {
    local r pair px py pyaw report bad="" k=0
    for r in $ROBOTS; do
        pair=${PARKING[$k]}; k=$((k+1))
        read -r px py pyaw < <(start_pose "$pair")
        report=$(timeout 15 ros2 topic echo --once "/$r/amcl_pose" 2>/dev/null | python3 -c "
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
" "$px" "$py" "$pyaw")
        if [ -z "$report" ]; then
            log "WARNING: could not read amcl_pose from [$r]; the seeded pose was not confirmed."
            continue
        fi
        set -- $report
        log "Seed check for [$r]: off by $1 m and $2 rad."
        if awk -v v="$1" -v lim="$MAX_SEED_XY_ERR_M" 'BEGIN { exit !(v > lim) }' \
           || awk -v v="$2" -v lim="$MAX_SEED_YAW_ERR_RAD" 'BEGIN { exit !(v > lim) }'; then
            bad="$bad $r"
        fi
    done
    if [ -n "$bad" ]; then
        log "ERROR: AMCL did not settle on the seeded pose for:$bad (tolerance ${MAX_SEED_XY_ERR_M} m, ${MAX_SEED_YAW_ERR_RAD} rad)"
        return 1
    fi
    return 0
}

# manage_lane_request: close or open the lane a scenario names, after the fleet adapter is
# up. A lane request is a volatile topic and the adapter is the only subscriber, so one
# published while no adapter runs reaches nobody and the scenario runs with the lane open.
manage_lane_request() {
    local action="$1" pair="$2"
    [ -z "$pair" ] && return 0
    local done_word=closed
    [ "$action" = open ] && done_word=opened
    python3 -m rmf_demos_fleet_adapter.manage_lane "$action" \
        "${pair%%,*}" "${pair##*,}" --bidir --nav_graph "$NAV_GRAPH" \
        >> "$SESSION_DIR/manage_lane.log" 2>&1 \
        && log "Lane ${pair} ${done_word} for this scenario." \
        || log "WARNING: lane ${action} for ${pair} failed; see $SESSION_DIR/manage_lane.log"
}

# stop_robot_navigation: cancel every Nav2 goal on every robot.
#
# Nav2 does not learn that the fleet adapter is gone, so a repeat torn down with a goal
# still active leaves that goal held. The next attempt's adapter then sends a goal to a
# Nav2 still serving the old one. Teardown cancels before it kills the adapter, while the
# zenoh session the cancel travels on is still up.
stop_robot_navigation() {
    [ -f "$FLEET_CONFIG" ] || return 0
    python3 "$COLLECT_HW/stop_robot_navigation.py" \
        --fleet-config "$FLEET_CONFIG" --robots $ROBOTS \
        >> "$SESSION_DIR/stop_robot_navigation.log" 2>&1 \
        && log "Navigation stopped on 3 robots." \
        || log "WARNING: navigation not confirmed stopped; see $SESSION_DIR/stop_robot_navigation.log"
}

teardown_scenario() {
    log "Tearing down"
    stop_robot_navigation
    kill_group "$BAG_PID"; kill_group "$FA_PID"; kill_group "$BRIDGE_PID"; kill_group "$SIM_PID"; kill_group "$ZENOHD_PID"
    sleep 6
    kill_group "$BAG_PID"; kill_group "$FA_PID"; kill_group "$BRIDGE_PID"; kill_group "$SIM_PID"; kill_group "$ZENOHD_PID"
    sleep 2
    pkill -9 -f "tb3_world.launch.py" 2>/dev/null
    pkill -9 -f "tb3_fleet.tb3_fleet_adapter" 2>/dev/null
    pkill -9 -f "zenoh-bridge-ros2dds" 2>/dev/null
    pkill -9 -f "tb3_multi_simulation_nav2.launch.py" 2>/dev/null
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

    # Render headless on the DRM render node instead of through the X server. Ogre2
    # reaches the integrated GPU directly that way and holds real time for three
    # lidars at 10 Hz; the same GPU driven over GLX on DISPLAY drops the simulation
    # to 0.15 of real time, which takes the recorded sampling rate with it.
    DISPLAY= setsid ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py \
        use_rviz:=False use_gzclient:=False \
        > "$RUN_DIR/simulation${LOGSUFFIX}.log" 2>&1 &
    SIM_PID=$!

    # Nav2 runs one stack per robot under the robot's namespace, so the transform tree
    # appears as /<robot>/tf. Waiting on the first robot's own topic returns as soon as a
    # stack is up, rather than running out a fixed wait on a topic this launch never
    # creates.
    local first_robot=${ROBOTS%% *}
    log "waiting for /$first_robot/tf..."
    local tf_i
    for tf_i in $(seq 1 45); do
        # Bounded: ros2 topic list blocks indefinitely when DDS discovery stalls,
        # and an unbounded call here holds the whole collection instead of letting
        # the wait expire and the attempt be retried.
        timeout 10 ros2 topic list 2>/dev/null | grep -q "^/$first_robot/tf$" && break
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
    # Listen for rmf_websocket's BroadcastClient, which connects out to server_uri
    # and pushes the live task-state stream as JSON. That stream is the only place
    # RMF reports a task reaching "completed".
    setsid python3 $COLLECT_RMF/record_api_state.py \
        --out "$RUN_DIR/api_state_attempt${attempt_id}.jsonl" --port "$API_PORT" \
        > "$RUN_DIR/api_state_logger_attempt${attempt_id}.log" 2>&1 &
    API_PID=$!
    sleep 3

    setsid ros2 launch tb3_fleet tb3_world.launch.py \
        fleet_config_file:=$TB3_FLEET_SHARE/config/simulation/fleet/multi_robot_config_sim.yaml \
        bidding_time_window:="$BIDDING_WINDOW_S" \
        server_uri:=ws://127.0.0.1:$API_PORT \
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
# next_attempt_id: the highest attempt number any artefact in the run directory uses,
# plus one.
next_attempt_id() {
    local run_dir="$1" highest
    highest=$(find "$run_dir" -maxdepth 1 \
                   \( -name '*attempt[0-9]*' -o -name 'attempt_[0-9]*' \) 2>/dev/null \
              | sed -n 's/.*attempt_\{0,1\}\([0-9]\{1,\}\).*/\1/p' | sort -n | tail -1)
    echo $(( ${highest:-0} + 1 ))
}

# attempt_names_free: whether no artefact of that attempt number exists yet.
attempt_names_free() {
    local run_dir="$1" id="$2" name
    for name in "api_state_attempt${id}.jsonl" "api_state_logger_attempt${id}.log" \
                "fleet_adapter_attempt${id}.log" "run_benchmark_attempt${id}.log" \
                "simulation_attempt${id}.log" "zenohd_attempt${id}.log" \
                "attempt_${id}"; do
        [ -e "$run_dir/$name" ] && return 1
    done
    return 0
}

run_scenario_repeat_loop() {
    local NAME="$1" DIRNAME="$2" R1="$3" R2="$4" R3="$5" CLOSED="$6" FACES="$7"
    [[ ",$SCENARIO_FILTER," == *",$NAME,"* ]] || return 0

    set_placement "$R1" "$R2" "$R3" "$FACES"

    local RUN_DIR=""
    if [ "$SIM_RESUME" = "1" ]; then
        RUN_DIR=$(find "$OUTPUT_DIR/$DIRNAME" -maxdepth 1 -type d -name "run_*${RUN_TAIL}" \
                  2>/dev/null | sort | tail -1)
    fi
    : "${RUN_DIR:=$OUTPUT_DIR/$DIRNAME/run_$(date -u +%Y%m%d_%H%M)${RUN_TAIL}}"
    mkdir -p "$RUN_DIR"

    local completed
    completed=$(find "$RUN_DIR" -maxdepth 1 -type d -name 'repeat_[0-9]*' 2>/dev/null | wc -l)
    if [ "$completed" -ge "$TARGET_REPEATS" ]; then
        log "[$NAME] already holds $completed of $TARGET_REPEATS repeats."
        return 0
    fi
    log "=== [$NAME] -> $RUN_DIR with $completed repeat(s) present ==="

    local attempt=0 attempt_id
    attempt_id=$(next_attempt_id "$RUN_DIR")
    while [ "$completed" -lt "$TARGET_REPEATS" ] && [ "$attempt" -lt "$MAX_ATTEMPTS" ]; do
        attempt=$((attempt+1))
        while ! attempt_names_free "$RUN_DIR" "$attempt_id"; do
            log "[$NAME] attempt id $attempt_id is already used in $RUN_DIR, taking the next."
            attempt_id=$((attempt_id+1))
        done
        local i=$((completed+1))
        log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): full teardown + bring-up"
        teardown_scenario
        bring_up_common "$RUN_DIR" "_attempt${attempt_id}"
        if ! robots_reachable; then
            log "[$NAME] repeat $i (attempt $attempt): robot streams absent -- discarding this attempt."
            continue
        fi
        place_robots
        if ! seed_initial_poses; then
            log "[$NAME] repeat $i (attempt $attempt): seeding failed -- discarding this attempt."
            continue
        fi
        wait_for_amcl_transform
        if ! seed_accepted; then
            log "[$NAME] repeat $i (attempt $attempt): AMCL did not settle on the seeded pose -- discarding this attempt."
            continue
        fi

        if ! launch_fleet_bundle_and_wait "$RUN_DIR" "fleet_adapter_attempt${attempt_id}.log"; then
            # Retry the repeat. The failure precedes the bag and the recorders, so no
            # data has been written, and the next iteration opens with a full teardown
            # and rebuild of the stack.
            log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): fleet bundle failed to come up -- discarding this attempt and retrying (full teardown + rebuild). $completed/$TARGET_REPEATS repeats done so far."
            continue
        fi
        sleep 10
        # After the fleet adapter is up, not before: the adapter is the only subscriber of
        # a lane request, and it reads the graph fresh on every launch, so every lane
        # starts open and only this call cuts the one the scenario names.
        manage_lane_request close "$CLOSED"

        local window_start window_end
        window_start=$(date +%s)

        local ATTEMPT_DIR="$RUN_DIR/attempt_${attempt_id}"
        mkdir -p "$ATTEMPT_DIR"
        setsid ros2 bag record -o "$ATTEMPT_DIR/bag" "${BAG_TOPICS[@]}" \
            > "$ATTEMPT_DIR/rosbag_record.log" 2>&1 &
        BAG_PID=$!

        # Record Gazebo's physics poses. /fleet_states carries AMCL estimates rather
        # than true position, and ros_gz_bridge drops the entity names from Pose_V, so
        # this stream is taken from Gazebo directly.
        setsid python3 $COLLECT_SIM/record_ground_truth_poses.py \
            --out "$ATTEMPT_DIR/ground_truth_poses.jsonl" --hz 10 \
            > "$ATTEMPT_DIR/ground_truth_poses.log" 2>&1 &
        GT_PID=$!
        sleep 3

        log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): submitting"
        # Submit the three patrols and wait up to 600 s for every task to be released,
        # under a 720 s hard timeout. The wait is an upper bound: it returns as soon as
        # the last task is released. The longest route observed under N=3 contention
        # needs about 380 s.
        timeout 720 python3 $COLLECT_RMF/run_benchmark_concurrent.py \
            --route "$R1" --route "$R2" --route "$R3" \
            --rounds 1 --repeats 1 \
            --robots tb3_robot1 tb3_robot2 tb3_robot3 \
            --fixed-wait "$FIXED_WAIT_S" --min-expected-distance-m 5 \
            --output-dir "$ATTEMPT_DIR" \
            > "$RUN_DIR/run_benchmark_attempt${attempt_id}.log" 2>&1
        RC=$?
        log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): run_benchmark_concurrent.py exited with code $RC"

        # Record past the last task's physical end, so that the release of
        # RobotState.task_id, which follows t_end_physical by 0.1 s to 0.5 s and is what
        # F4a reads as completion, lands inside the bag.
        sleep 10
        window_end=$(date +%s)
        kill_group "$BAG_PID"
        kill_group "$GT_PID"
        sleep 5
        ros2 bag info "$ATTEMPT_DIR/bag" > "$ATTEMPT_DIR/bag_info.log" 2>&1
        BAG_INFO_RC=$?

        # Discard the attempt unless ros2 bag info reads the bag and reports at least
        # one message. The physics check below falls back to ground truth when there is
        # no readable bag, so it does not catch an empty or truncated recording.
        BAG_MSGS=$(grep -oE "^Messages:[[:space:]]+[0-9]+" "$ATTEMPT_DIR/bag_info.log" \
            | grep -oE "[0-9]+" | head -1)
        BAG_MSGS=${BAG_MSGS:-0}
        if [ "$BAG_INFO_RC" -ne 0 ] || [ "$BAG_MSGS" -lt 1 ]; then
            log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): BAG UNREADABLE OR EMPTY (ros2 bag info rc=$BAG_INFO_RC, messages=$BAG_MSGS; see $ATTEMPT_DIR/bag_info.log) -- discarding this attempt, will retry as repeat $i"
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_bad_bag_attempt${attempt_id}"
            continue
        fi

        # The same per-repeat gate run_hardware_scenarios.sh applies: stream delivery
        # against the nominal rates, how long the position reference saw each robot, how
        # far it travelled and what its charge did. Gazebo is the position reference here,
        # so the check reads the same ground_truth_poses.jsonl the tracker writes.
        python3 $COLLECT_HW/check_hardware_preflight.py \
            --repeat-dir "$ATTEMPT_DIR" \
            --robots $ROBOTS \
            --attempt "$attempt_id" \
            --window-start "$window_start" --window-end "$window_end" \
            > "$ATTEMPT_DIR/preflight.log" 2>&1
        PRE_RC=$?
        if [ "$PRE_RC" -eq 1 ]; then
            log "[$NAME] repeat $i (attempt $attempt): preflight failed, see $ATTEMPT_DIR/preflight.log -- discarding."
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_preflight_attempt${attempt}"
            continue
        elif [ "$PRE_RC" -ne 0 ]; then
            log "[$NAME] repeat $i (attempt $attempt): preflight could not run (rc=$PRE_RC), see $ATTEMPT_DIR/preflight.log -- discarding."
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_preflight_attempt${attempt}"
            continue
        fi

        if ! python3 $COLLECT_SIM/check_physics_glitch.py \
                --bag "$ATTEMPT_DIR/bag" \
                --robots tb3_robot1 tb3_robot2 tb3_robot3 \
                > "$ATTEMPT_DIR/physics_check.log" 2>&1; then
            log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): PHYSICS GLITCH detected (see $ATTEMPT_DIR/physics_check.log) -- discarding this attempt, will retry as repeat $i"
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_physics_glitch_attempt${attempt}"
            continue
        fi

        python3 $ANALYSE_TRAFFIC_SCHEDULING/analyze_traffic_scheduling.py \
            --bag "$ATTEMPT_DIR/bag" \
            --robots tb3_robot1 tb3_robot2 tb3_robot3 \
            --scenario "$NAME, N=3, CONFIG_01, repeat $i" \
            --output "$ATTEMPT_DIR/traffic_scheduling_metrics.json" \
            > "$ATTEMPT_DIR/analyze.log" 2>&1
        ANALYSIS_RC=$?
        log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): traffic-scheduling analyze exited $ANALYSIS_RC"
        if [ "$ANALYSIS_RC" -ne 0 ]; then
            log "[$NAME] repeat $i (attempt $attempt): no traffic metrics, so the gate has nothing to read -- discarding."
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_analysis_attempt${attempt}"
            continue
        fi

        # Also compute the task-planning metrics, which carry the per-task timings that
        # F5 reads.
        python3 $ANALYSE_TASK_PLANNING/analyze_task_planning.py \
            --bag "$ATTEMPT_DIR/bag" \
            --api-state "$RUN_DIR/api_state_attempt${attempt}.jsonl" \
            --robots tb3_robot1 tb3_robot2 tb3_robot3 \
            --scenario "$NAME, N=3, CONFIG_01, repeat $i" \
            --nav-graph "$NAV_GRAPH" \
            --output "$ATTEMPT_DIR/task_planning_metrics.json" \
            >> "$ATTEMPT_DIR/analyze.log" 2>&1
        ANALYSIS_RC=$?
        log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): task-planning analyze exited $ANALYSIS_RC"
        if [ "$ANALYSIS_RC" -ne 0 ]; then
            log "[$NAME] repeat $i (attempt $attempt): no task metrics, so the gate has nothing to read -- discarding."
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_analysis_attempt${attempt}"
            continue
        fi

        python3 $COLLECT_RMF/annotate_task_outcomes.py \
            --repeats-summary "$ATTEMPT_DIR/repeats_summary.json" \
            --task-planning-metrics "$ATTEMPT_DIR/task_planning_metrics.json" \
            >> "$ATTEMPT_DIR/analyze.log" 2>&1
        ANALYSIS_RC=$?
        log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): outcome annotation exited $ANALYSIS_RC"
        if [ "$ANALYSIS_RC" -ne 0 ]; then
            log "[$NAME] repeat $i (attempt $attempt): each task's terminal status was not written into the summary -- discarding."
            mv "$ATTEMPT_DIR" "$RUN_DIR/discarded_analysis_attempt${attempt}"
            continue
        fi

        # A complete recording is not yet an N=3 sample: the dispatcher may have left a
        # robot idle, or no conflict may have arisen. The repeat is kept either way; it
        # only counts towards TARGET_REPEATS when it measured all three robots. This is
        # the gate run_hardware_scenarios.sh applies, so the two platforms admit the same
        # repeats.
        python3 "$COLLECT_HW/check_n3_coordination.py" \
            --task-planning-metrics "$ATTEMPT_DIR/task_planning_metrics.json" \
            --traffic-scheduling-metrics "$ATTEMPT_DIR/traffic_scheduling_metrics.json" \
            --api-state "$RUN_DIR/api_state_attempt${attempt}.jsonl" \
            --nav-graph "$NAV_GRAPH" \
            --window-start "$window_start" --window-end "$window_end" \
            --robots $ROBOTS $N3_GATE_ARGS \
            2>&1 | tee -a "$ATTEMPT_DIR/n3_coordination.log" | tee -a "$MASTER_LOG"
        N3_RC=${PIPESTATUS[0]}
        if [ "$N3_RC" -ne 0 ]; then
            mv "$ATTEMPT_DIR" "$RUN_DIR/not_n3_attempt_${attempt}"
            log "[$NAME] repeat $i (attempt $attempt): recorded but not an N=3 sample; kept as [not_n3_attempt_${attempt}] and not counted."
            continue
        fi

        mv "$ATTEMPT_DIR" "$RUN_DIR/repeat_${i}"
        if [ "$RC" -ne 0 ]; then
            log "[$NAME] repeat $i/$TARGET_REPEATS (attempt $attempt): kept despite run_benchmark_concurrent.py exit code $RC (physics-clean; a non-zero exit here is a real outcome, e.g. the outer 'timeout 720' firing, not simulator noise -- only physics glitches get retried)"
        fi
        completed=$((completed+1))
    done

    log "[$NAME] $completed/$TARGET_REPEATS repeats completed after $attempt attempt(s)"

    # The same merge the hardware runner performs, so both platforms produce one
    # repeats_summary.json per run with the same shape.
    python3 "$COLLECT_HW/merge_repeat_summaries.py" --run-dir "$RUN_DIR" \
        2>&1 | tee -a "$MASTER_LOG"

    python3 $ANALYSE_TRAFFIC_SCHEDULING/merge_repeat_metrics.py \
        --run-dir "$RUN_DIR" \
        --scenario "$NAME, N=3, CONFIG_01" \
        --robots tb3_robot1 tb3_robot2 tb3_robot3 \
        > "$RUN_DIR/merge_metrics.log" 2>&1
    log "[$NAME] merge_repeat_metrics exited with code $?"

    teardown_scenario
    log "=== [$NAME] DONE: $RUN_DIR ==="
}

# One row per scenario, seven pipe-separated fields, identical to the table
# run_hardware_scenarios.sh collects from:
#
#   1  scenario label
#   2  data directory name
#   3  route 1, as "start,goal"
#   4  route 2
#   5  route 3
#   6  lane to close, as "a,b"; empty for no closure
#   7  neighbour each route's start faces, in route order
#
# Robot k starts on route k's first waypoint, facing field 7's vertex k, fixed for every
# repeat. Measurement objectives, waypoint roles and the contention each scenario creates
# are specified in SCENARIO_DESIGN.md.
SCENARIOS_TABLE=(
"Bottleneck|bottleneck|wp_2,wp_5|wp_3,wp_4|wp_4,wp_3|wp_6,wp_7|wp_3,wp_2,wp_2"
"Crossing|crossing|wp_2,wp_6|wp_6,wp_9|wp_7,wp_8||wp_4,wp_7,wp_6"
"Head-on|headon|wp_2,wp_5|wp_5,wp_2|wp_3,wp_7||wp_3,wp_3,wp_5"
"Shared Lane|sharedlane|wp_2,wp_7|wp_4,wp_5|wp_6,wp_3||wp_3,wp_2,wp_4"
)

SCENARIO_FILTER="${1:-Bottleneck,Crossing,Head-on,Shared Lane}"

require_scripts || exit 1

log "########## SIMULATION N=3 COLLECTION: scenarios=[$SCENARIO_FILTER] ##########"
log "Fleet config [$FLEET_CONFIG]."
log "Navigation graph [$NAV_GRAPH]."

for entry in "${SCENARIOS_TABLE[@]}"; do
    IFS='|' read -r label dirname r1 r2 r3 closed faces <<< "$entry"
    run_scenario_repeat_loop "$label" "$dirname" "$r1" "$r2" "$r3" "$closed" "$faces"
done

log "########## RUN COMPLETE [$SCENARIO_FILTER] ##########"
