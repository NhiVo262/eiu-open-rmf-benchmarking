#!/bin/bash
# Computes task_planning_metrics.json for the 4 N=3 scenarios from the bags
# already copied into this folder (bottleneck/crossing/headon/sharedlane
# _FLEET.../run_*_N3/repeat_*/bag/) -- self-contained within task_planning/,
# no dependency on traffic_scheduling/ (which isn't committed alongside this
# folder). Use this to reproduce the report's N=3 numbers from the data
# that's already here; use scripts/run_task_planning_scenarios.sh only if
# you need to re-run the simulation and collect fresh bags (that script
# does depend on traffic_scheduling/scripts/ -- see PROCEDURE.md).
set -o pipefail

WS=/home/eiu/rmf_ws
TP_DIR=$WS/src/benchmark/task_planning
NAV_GRAPH=$WS/install/tb3_fleet/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source $WS/install/setup.bash >/dev/null 2>&1

SCENARIOS=(
  "bottleneck_FLEET0N_TRAF03_CONFIG01:Bottleneck"
  "crossing_FLEET02_TRAF01_CONFIG01:Crossing"
  "headon_FLEET0N_TRAF04_CONFIG01:Head-on"
  "sharedlane_FLEET0N_TRAF02_CONFIG01:Shared Lane"
)

for entry in "${SCENARIOS[@]}"; do
    DIRNAME="${entry%%:*}"
    LABEL="${entry##*:}"
    RUN_DIR=$(ls -d "$TP_DIR/$DIRNAME"/run_*_N3 2>/dev/null | head -1)
    if [ -z "$RUN_DIR" ]; then
        echo "SKIP [$LABEL]: no run_*_N3 folder found under $DIRNAME/"
        continue
    fi
    for REPEAT_DIR in "$RUN_DIR"/repeat_*; do
        [ -d "$REPEAT_DIR/bag" ] || continue
        i=$(basename "$REPEAT_DIR" | grep -oE '[0-9]+$')
        echo "[$LABEL] repeat $i: analyzing $REPEAT_DIR/bag"
        python3 "$TP_DIR/scripts/analyze_task_planning_concurrent.py" \
            --bag "$REPEAT_DIR/bag" \
            --robots tb3_robot1 tb3_robot2 tb3_robot3 \
            --scenario "$LABEL, N=3, CONFIG_01, repeat $i" \
            --nav-graph "$NAV_GRAPH" \
            --output "$REPEAT_DIR/task_planning_metrics.json" \
            > "$REPEAT_DIR/analyze_task_planning.log" 2>&1
        echo "[$LABEL] repeat $i: exit code $?"
    done
done

echo "DONE. Pool per_task[].makespan_s (or the 1c_makespan_s field) across each"
echo "scenario's repeat_*/task_planning_metrics.json for the report's mean/stdev/min/max."
