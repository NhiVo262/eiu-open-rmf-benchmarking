# Open-RMF Benchmarking

This EIU × ARTC project evaluates an Open-RMF deployment with TurtleBot3 robots in
simulation and on hardware, using matched workloads and test conditions.

## Contents

1. [Overview](#overview)
2. [Problem statement and objectives](#problem-statement-and-objectives)
3. [Benchmark scope: F1–F8](#benchmark-scope-f1f8)
4. [Project contributions](#project-contributions)
5. [System architecture](#system-architecture)
6. [Technology stack](#technology-stack)
7. [Repository structure](#repository-structure)
8. [Requirements and setup](#requirements-and-setup)
9. [Running the simulation demo](#running-the-simulation-demo)
10. [Running benchmarks and reproducing results](#running-benchmarks-and-reproducing-results)
11. [Recorded data and results](#recorded-data-and-results)
12. [Known limitations](#known-limitations)
13. [Documentation and future work](#documentation-and-future-work)

## Overview

Open-RMF assigns work to robot fleets and coordinates their use of shared space. Each
robot executes its assigned plan through its navigation, localisation and control stack.
This project integrates three TurtleBot3 robots with Open-RMF, Nav2, Zenoh
and evaluates the resulting system.

Experiments cover a simulation fleet and a physical fleet. Collection procedures,
measurement methods and results are documented in [benchmark/](benchmark/ARCHITECTURE.md).

## Problem statement and objectives

Task completion time, execution fidelity and physical separation depend on both RMF
coordination and robot execution. Evaluation therefore requires measurements of task
planning, traffic scheduling and robot behaviour under defined test conditions.

The objective is to evaluate F1–F8 under specified workloads, compare simulation with
hardware where measurements are comparable, and identify performance and measurement
limitations.
Selected conditions are matched: navigation graph, routes, initial placement, fleet
profile, controller and velocity caps. Other differences and measurement constraints are
recorded in [SYSTEM_UNDER_TEST.md](benchmark/SYSTEM_UNDER_TEST.md).

## Benchmark scope: F1–F8

| Feature | Evaluation | Scope of the available evidence |
|---|---|---|
| F1 — Task planning | Dispatch success, bid latency, queueing and makespan | Three concurrent tasks; placement largely determines assignment |
| F2 — Traffic scheduling | Conflict-free itinerary entry, conflict outcomes and schedule propagation | Four scenarios at N=3; schedule quantities describe RMF's core |
| F3 — Negotiation | Notice counts, outcomes and latency by outcome | Resolved negotiations, refusals and failed searches at N=3 |
| F4 — Fleet coordination | Task completion, utilisation and throughput scaling | Retained workloads and route-matched N=1 references |
| F5 — Execution fidelity | Planned versus actual task duration and finish time | Scenario and baseline tasks on both platforms |
| F6 — Congestion handling | Time without a motion command during task assignment; episode duration | Per-route N=3 versus N=1 |
| F7 — Safety and collision avoidance | Separation based on reported poses and supplementary lidar evidence | Gazebo reference in simulation; no external position reference on hardware |
| F8 — Scalability | Planning latency and batch throughput across fleet sizes | N=1 and N=3 |

Formulas, observation units, timers, denominators and deviations from the original
requirements are defined in [METRIC_DESIGN.md](benchmark/METRIC_DESIGN.md).

## Project contributions

- **Fleet integration:** an Open-RMF EasyFullControl adapter that sends Nav2 goals and
  reads robot telemetry over Zenoh, with simulation and hardware configurations.
- **Experimental workflow:** scenario runners, state recording and per-repeat checks for the two platforms.
- **Measurement pipeline:** metric definitions, per-feature analysers.
- **Cross-platform evaluation:** route-matched single-robot baselines and multi-robot
  results, with documented coverage, measurement limitations and case studies.

## System architecture

```text
TASK COORDINATION

                     run_benchmark_concurrent.py
                                  |
                 ROS 2 task API: task_api_requests,
                task_api_responses, /dispatch_states
                                  v
                         Open-RMF dispatcher
                                  |  bid notice / bid response
                                  v
                      Fleet adapter (tb3_fleet)
                 - EasyFullControl
                 - Tb3RobotAdapter x N, in-process
                 - traffic participant, negotiation
                 - one shared Zenoh client session

COMMUNICATION

                     zenohd (workstation, :7447)
                                  |
        +----------------------+----------------------+
        |                      |                      |
   SIMULATION              HARDWARE               RECORDING
   one bridge,          one bridge per           workstation
   workstation        robot, on the robot          bridge
        |                      |                      |
   ROS 2 / DDS            ROS 2 / DDS          workstation DDS
        |                      |
   Nav2 + AMCL            Nav2 + AMCL
        |                      |
  ros_gz_bridge         robot bring-up
        |                      |
     Gazebo               TurtleBot3

DATA COLLECTION

  ROS 2 topics      -->  rosbag2 (MCAP)
  RMF task states   -->  record_api_state.py (WebSocket) -> JSONL
  Gazebo poses      -->  ground-truth JSONL

DATA ANALYSIS

  MCAP + JSONL  ->  analysers  ->  metrics JSON  ->  RESULTS.md / APPENDIX.md
```

The dispatcher auctions tasks among fleets. The fleet adapter bids, manages task execution
and converts navigation callbacks into Nav2 goals. It holds the fleet's traffic participant
and takes part in negotiation. The per-robot adapters run inside the fleet-adapter process
and share its single Zenoh client session.

Transport differs by platform. In simulation one workstation bridge serves all three
robots, and Nav2 reaches Gazebo through `ros_gz_bridge`. On hardware each robot runs its
own bridge, robot bring-up supplies TF, `scan` and `battery_state`, and the hardware
runners start a second workstation bridge that returns robot topics to workstation DDS
for recording. TF and battery state reach the adapter as Zenoh subscriptions; reported
pose is derived from the `map` to `base_footprint` transform.

RMF pushes task-state updates over WebSocket to `record_api_state.py`, which writes JSONL.
See the [fleet package documentation](tb3_fleet/README.md) for transport details.

## Technology stack

| Layer | Components |
|---|---|
| Middleware | ROS 2 Jazzy, CycloneDDS |
| Coordination | Open-RMF task, traffic and fleet-adapter libraries |
| Navigation | Nav2, AMCL, DWB local planner |
| Simulation | Gazebo and TurtleBot3 simulation resources |
| Hardware | Three TurtleBot3 Burger robots |
| Robot transport | Zenoh 1.5.0, `zenohd`, `zenoh-bridge-ros2dds` |
| Recording | rosbag2/MCAP, RMF WebSocket state updates in JSONL, Gazebo ground truth |
| Analysis | Python scripts for metrics, diagnostics and summaries |
| Environment | [Dockerfile](scripts/Dockerfile), [Compose configuration](scripts/docker-compose.yaml) |

## Repository structure

```text
src/                      
├── tb3_fleet/                # Project fleet integration
│   ├── config/               # simulation/ and hardware/ configurations
│   ├── launch/               # RMF and simulation launch files
│   ├── maps/                 # Building-map source, occupancy map and assets
│   └── tb3_fleet/            # Adapters, TF and initial-pose helpers
├── benchmark/
│   ├── collect/              # rmf/, simulation/ and hardware/ runners and recorders
│   ├── analyse/              # Feature analysers, validity checks and summaries
│   ├── data/                 # simulation/ and hardware/ datasets
│   └── results/              # Report figures and other cited outputs
├── scripts/                  # Container, dependency and network configuration
             
              
```

The build generates the Gazebo world and navigation graph from
[world_tb3.building.yaml](tb3_fleet/maps/turtlebot3_world/world_tb3.building.yaml).
See [benchmark architecture](benchmark/ARCHITECTURE.md) for the detailed artefact layout.

## Requirements and setup

### Host and workspace

Dependencies are specified in
[Dockerfile](scripts/Dockerfile) and [requirements.txt](scripts/requirements.txt).
ROS Jazzy runs inside the container.

### Start the open-rmf container

Bring the stack up on the host, in a terminal that stays open.
[launch_docker.sh](scripts/launch_docker.sh) grants the container X11 access, runs
`docker compose up` in the foreground, and on `Ctrl+C` revokes that access and runs
`docker compose down`:

```bash
cd ~/eiu_ws/src/scripts
bash launch_docker.sh
```

The Compose file defines the services `openRMF-Benchmarking`, whose container is named
`open-rmf`, and `openRMF-vda5050`. Without the helper script, start the benchmarking
service alone and grant display access separately:

```bash
cd ~/eiu_ws/src/scripts
xhost +SI:localuser:root
docker compose up -d openRMF-Benchmarking
```

Enter the running container from any other host terminal:

```bash
docker exec -it open-rmf bash
```

### Install the source dependencies

This repository contains `tb3_fleet/`, `benchmark/` and `scripts/`. The container image
supplies the ROS and Open-RMF dependencies at the versions pinned in
[Dockerfile](scripts/Dockerfile). 

### Build the project packages

```bash
colcon build --packages-select tb3_fleet
source install/setup.bash
```

```bash
fleet_share="$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet"
test -f "$fleet_share/maps/world_tb3/nav_graphs/0.yaml"
test -x "$fleet_share/bin/zenoh-bridge-ros2dds"
```

## Running the simulation demo

In **each** terminal, enter the container and source the same workspace:

```bash
docker exec -it open-rmf bash
source /opt/ros/jazzy/setup.bash
source /home/eiu/rmf_ws/install/setup.bash
export ROS_DOMAIN_ID=4
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/eiu/rmf_ws
```
### 1. Zenoh router

```bash
zenohd
```

### 2.Launch the simulation environment and Nav2

```bash
TB3_SIM_ROBOTS=tb3_robot1,tb3_robot2,tb3_robot3 \
  ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py \
  use_gzclient:=true use_rviz:=true
```

Wait for Nav2 lifecycle activation in each namespace. The `use_gzclient` and `use_rviz`
arguments are set here for observation; benchmark runners normally operate headless.

### 3. Robot-side Zenoh bridge

Once the simulation is publishing robot telemetry:

```bash
fleet_share="$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet"
"$fleet_share/bin/zenoh-bridge-ros2dds" \
  -c "$fleet_share/config/simulation/zenoh_bridge/tb3_zenoh_bridge_multi_robot_config_sim.json5"
```

### 4. Task-state recording

Create a new capture file and retain its printed path:

```bash
demo_capture="$(mktemp /tmp/rmf-demo.XXXXXX)"
printf 'Demo task-state capture: %s\n' "$demo_capture"
python3 src/benchmark/collect/rmf/record_api_state.py \
  --host 127.0.0.1 --port 8411 --out "$demo_capture"
```

The listener records RMF task-state updates in JSONL format.

### 5. RMF core and fleet adapter

```bash
fleet_share="$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet"
ros2 launch tb3_fleet tb3_world.launch.py \
  fleet_config_file:="$fleet_share/config/simulation/fleet/multi_robot_config_sim.yaml" \
  use_sim_time:=true bidding_time_window:=5.0 \
  server_uri:=ws://127.0.0.1:8411
```

Wait for registration of `tb3_robot1`, `tb3_robot2` and `tb3_robot3`. In the task terminal,
inspect the fleet before submitting work:

```bash
ros2 topic echo /fleet_states --once
```

Expect a fleet named `tb3_fleet` with state entries for all three robots. If registration
or telemetry is missing, check the simulation, bridge, domain and adapter logs first.

### 6. Submit a patrol

Submit a task through the fleet auction:

```bash
ros2 run rmf_demos_tasks dispatch_patrol --use_sim_time -p wp_2 wp_5
```

Alternatively, request a particular robot:

```bash
ros2 run rmf_demos_tasks dispatch_patrol --use_sim_time \
  -p wp_2 wp_5 -F tb3_fleet -R tb3_robot1
```
## Running benchmarks and reproducing results

### Collect new simulation data

Runners manage startup, robot placement, seeding, recording and validation. Stop the
manual demo and run one collection at a time. Inside the container, after sourcing the
workspace:

```bash
cd /home/eiu/rmf_ws/src/benchmark/collect/simulation
bash run_simulation_scenarios.sh "Head-on"
```

Omit the scenario argument to run all four scenarios. To collect the N=1 baseline
routes:

```bash
cd /home/eiu/rmf_ws/src/benchmark/collect/simulation
bash run_simulation_baselines.sh
```

Outputs are stored in run and repeat directories under
`benchmark/data/simulation/scenarios/` or `benchmark/data/simulation/baselines/`.
Verify coverage using retained repeat counts, acceptance-check logs and task outcomes.
For full procedures, see:

- [Simulation collection](benchmark/collect/simulation/PROCEDURE.md).
- [Hardware collection](benchmark/collect/hardware/PROCEDURE.md), including robot setup,
  communication, initialisation and platform-specific checks.

### Inspect existing results 

Run the following commands on existing JSON metrics, inside the container from
`/home/eiu/rmf_ws`:

```bash
python3 src/benchmark/analyse/summary/show_collection_summary.py \
  --scenario "Crossing=src/benchmark/data/hardware/scenarios/crossing/run_20261018_0639"
```

The output contains per-feature tables. Column labels distinguish pooled statistics
from means of per-repeat statistics.

To recompute F8 planning latency growth and load the existing throughput summary:

```bash
analysis_dir="$(mktemp -d /tmp/rmf-analysis.XXXXXX)"
python3 src/benchmark/analyse/summary/analyze_scalability.py \
  --platform-dir src/benchmark/data/simulation \
  --output "$analysis_dir/scalability_metrics.json"
printf 'Analysis output: %s/scalability_metrics.json\n' "$analysis_dir"
```

Expect N=1 and N=3 planning latency statistics and growth values on stdout, and a JSON file
containing `8a_as_specified`, `8a_first_task`, `8a_per_scenario` and `8b`. The `8b` section
reads the supplied `data/simulation/fleet_scaling_metrics.json`. Recomputing that input is
covered in the [fleet coordination procedure](benchmark/analyse/fleet_coordination/PROCEDURE.md).

Raw bag reanalysis is documented in the per-feature procedures. Some batch scripts
overwrite per-repeat outputs; use a separate dataset copy to preserve cited results.

## Recorded data and results

| Artefact | Purpose |
|---|---|
| `repeat_N/bag/` | Recorded ROS streams in MCAP |
| `repeat_N/ground_truth_poses.jsonl` | Gazebo reference poses in the simulation dataset |
| `repeat_N/<feature>_metrics.json` | Derived values |
| `repeat_N/*.log` | Recording, analysis and admission-check output |
| `run_*/api_state_attempt<N>.jsonl` | RMF state capture |
| `run_*/*.log` | Simulation, adapter, transport and task-runner logs |

| Scenario | Workload intent |
|---|---|
| Bottleneck | Bidirectional routes sharing a short edge, with one alternative edge closed |
| Crossing | Intersection traffic and opposing travel on a shared edge |
| Head-on | Opposing routes with an alternative path around the graph |
| Shared Lane | Routes sharing an edge in the same direction |

Full routes, initial poses and closures are defined in
[SCENARIO_DESIGN.md](benchmark/SCENARIO_DESIGN.md). Event counts and timelines identify
the traffic interactions observed in each repeat.

The [evaluation report](benchmark/RESULTS.md) documents these principal findings:

- Bid and schedule propagation latencies are low relative to task execution times.
- The duration model underestimates task execution time, including in single-robot baselines.
- Multi-robot delays and waiting times vary across scenarios and routes.
- Negotiation latency depends on outcome; rapid refusals differ from successful resolutions.
- Separation estimates based on RMF poses can differ substantially from reference measurements.

## Known limitations

- Each scenario or baseline route has five repeats. Comparisons are descriptive;
  statistical significance has not been established.
- N=1 and N=3 are the evaluated fleet sizes. Two points do not establish a scaling law
  or the fleet's operating limit.
- Robots start at their routes' initial waypoints, constraining task allocation.
  Unconstrained assignment and load balancing remain untested.
- Retained repeats require all dispatched tasks to complete. Their completion fraction
  describes the selected dataset, not all-attempt reliability.
- Hardware has no independent position reference. Reported pose and lidar evidence have
  different validity limits; simulation validation does not establish hardware safety.
- Localisation, environment, state persistence and sensing differ between platforms
  despite matched routes and selected settings.

## Documentation and future work

| Document | Coverage |
|---|---|
| [Fleet integration](tb3_fleet/README.md) | Package resources and Zenoh transport |
| [System under test](benchmark/SYSTEM_UNDER_TEST.md) | Measured configuration and platform differences |
| [Metric definitions](benchmark/METRIC_DESIGN.md) | Formulas, sources, timers and denominators |
| [Scenario design](benchmark/SCENARIO_DESIGN.md) | Graph, routes, placement and workload limits |
| [Benchmark architecture](benchmark/ARCHITECTURE.md) | Collection-to-analysis contracts and file layout |
| [Results and discussion](benchmark/RESULTS.md) | Findings, diagnostics and case studies |
| [Simulation procedure](benchmark/collect/simulation/PROCEDURE.md) | Simulation collection and analysis |
| [Hardware procedure](benchmark/collect/hardware/PROCEDURE.md) | Physical setup, collection and troubleshooting |

Further work includes additional fleet sizes and workloads, an independent hardware
position reference, experiments with unconstrained task allocation, and controlled tests
to separate duration model error from navigation and traffic effects.
