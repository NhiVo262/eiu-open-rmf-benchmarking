#!/bin/bash
set -o pipefail

SCENARIO_SLUG="$1"
ROUTE_SLUG="$2"
X_POSE="$3"
Y_POSE="$4"
PLACE1="$5"
PLACE2="$6"
FIXED_WAIT="$7"
MIN_DIST="$8"
ROUNDS="${9:-1}"

WS=/home/eiu/rmf_ws
SCRIPTS=$WS/src/benchmark/task_planning/scripts
RUN_DIR=$WS/src/benchmark/task_planning/route_baselines/$SCENARIO_SLUG/$ROUTE_SLUG/run_$(date -u +%Y%m%d_%H%M)
mkdir -p "$RUN_DIR"
LOG=$RUN_DIR/orchestration.log
echo "$RUN_DIR" > $WS/run_dir_current.txt

log() { echo "[$(date -u +%H:%M:%S)] $*" | tee -a "$LOG"; }

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

kill_group() {
    local pid="$1"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null
    fi
}

cleanup() {
    log "Tearing down: bag, fleet adapter/RMF, zenoh-bridge, simulation, zenohd"
    kill_group "$BAG_PID"
    kill_group "$FA_PID"
    kill_group "$BRIDGE_PID"
    kill_group "$SIM_PID"
    kill_group "$ZENOHD_PID"
    sleep 6
    kill_group "$BAG_PID"
    kill_group "$FA_PID"
    kill_group "$BRIDGE_PID"
    kill_group "$SIM_PID"
    kill_group "$ZENOHD_PID"
    sleep 2
    # Safety net for anything that still escaped its process group.
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
    sleep 2
    log "Teardown complete"
}
trap cleanup EXIT

log "=== Route $SCENARIO_SLUG/$ROUTE_SLUG: spawn=($X_POSE,$Y_POSE) places=[$PLACE1 $PLACE2] fixed_wait=$FIXED_WAIT ==="

log "Starting zenohd"
setsid zenohd > "$RUN_DIR/zenohd.log" 2>&1 &
ZENOHD_PID=$!
sleep 3

log "Launching simulation (Gazebo headless + Nav2), spawn=($X_POSE,$Y_POSE)"
setsid ros2 launch tb3_fleet tb3_simulation_nav2.launch.py \
    use_rviz:=False use_gzclient:=False \
    x_pose:=$X_POSE y_pose:=$Y_POSE autostart:=False \
    > "$RUN_DIR/simulation.log" 2>&1 &
SIM_PID=$!

log "Waiting for /tf to be published (up to 90s)..."
for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "^/tf$"; then
        log "/tf is up after ${i}x2s"
        break
    fi
    sleep 2
done

log "Bringing up localization (AMCL) -- does not depend on the map frame, so no race here"
ros2 service call /lifecycle_manager_localization/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' \
    >> "$RUN_DIR/lifecycle_bringup.log" 2>&1
sleep 3

log "Seeding AMCL initial pose at ($X_POSE,$Y_POSE)"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: $X_POSE, y: $Y_POSE, z: 0.0}, orientation: {w: 1.0}}, \
covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}" \
>> "$RUN_DIR/initialpose.log" 2>&1

log "Waiting for AMCL to actually publish map->odom (up to 30s)..."
for i in $(seq 1 15); do
    if timeout 1 ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1; then
        log "map->odom transform is up after ${i}x2s"
        break
    fi
    sleep 2
done

log "Bringing up navigation (planner/controller/bt_navigator) now that the map frame exists"
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}' \
    >> "$RUN_DIR/lifecycle_bringup.log" 2>&1
sleep 5

log "Starting zenoh-bridge"
cd $WS/src/tb3_fleet/config/zenoh
setsid ./zenoh-bridge-ros2dds -c tb3_zenoh_bridge_ros2dds_client_config.json5 \
    > "$RUN_DIR/zenoh_bridge.log" 2>&1 &
BRIDGE_PID=$!
cd "$RUN_DIR"
sleep 3

log "Launching RMF core + fleet adapter"
setsid ros2 launch tb3_fleet tb3_world.launch.py \
    > "$RUN_DIR/fleet_adapter.log" 2>&1 &
FA_PID=$!

log "Waiting for 'Successfully added robot' (up to 90s)..."
FOUND=0
for i in $(seq 1 45); do
    if grep -q "Successfully added robot" "$RUN_DIR/fleet_adapter.log" 2>/dev/null; then
        log "Robot registered after ${i}x2s"
        FOUND=1
        break
    fi
    sleep 2
done
if [ "$FOUND" -eq 0 ]; then
    log "ERROR: robot never registered, aborting this route"
    exit 1
fi
log "Letting the robot's navigation stack settle for 20s before submitting tasks"
sleep 20

log "Starting rosbag record"
cd "$RUN_DIR"
setsid ros2 bag record -o bag \
    /rmf_task/bid_notice /rmf_task/bid_response \
    /rmf_task/dispatch_request /rmf_task/dispatch_ack \
    /task_api_requests /fleet_states \
    > "$RUN_DIR/rosbag_record.log" 2>&1 &
BAG_PID=$!
sleep 3
log "Bag recording, pid=$BAG_PID"

log "Running run_benchmark.py: places=[$PLACE1 $PLACE2] rounds=$ROUNDS repeats=5 fixed_wait=$FIXED_WAIT"
python3 $SCRIPTS/run_benchmark.py \
    --places $PLACE1 $PLACE2 --rounds $ROUNDS --repeats 5 \
    --robot tb3_robot1 --fixed-wait $FIXED_WAIT --min-expected-distance-m $MIN_DIST \
    --output-dir "$RUN_DIR" \
    > "$RUN_DIR/run_benchmark.log" 2>&1
log "run_benchmark.py exited with code $?"

log "Stopping rosbag (SIGTERM)"
kill_group "$BAG_PID"
sleep 5
ros2 bag info "$RUN_DIR/bag" > "$RUN_DIR/bag_info.log" 2>&1
log "Bag info written"

log "Running analyze_task_planning.py"
python3 $SCRIPTS/analyze_task_planning.py \
    --bag "$RUN_DIR/bag" \
    --robot tb3_robot1 \
    --scenario "Clear path replication of $SCENARIO_SLUG/$ROUTE_SLUG ($PLACE1,$PLACE2), N=1" \
    --output "$RUN_DIR/task_planning_metrics.json" \
    > "$RUN_DIR/analyze.log" 2>&1
log "analyze_task_planning.py exited with code $?"

log "=== DONE: $RUN_DIR ==="
