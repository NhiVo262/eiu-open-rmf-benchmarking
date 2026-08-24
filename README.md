# EIU-OPEN-RMF-BENCHMARKING

## Overview

Workspace for benchmarking Open-RMF with a 3-robot TurtleBot3 fleet, as part of the EIU x ARTC project. It combines the upstream Open-RMF stack with a custom fleet integration package (`tb3_fleet`) that connects TurtleBot3 (Nav2) to Open-RMF, enabling multi-robot task dispatch and the benchmarking test plan.

## Packages

| Package | Role |
|---|---|
| **`tb3_fleet`** | Custom integration package: fleet adapter, robot adapter, world/nav-graph maps, launch files for simulation and RMF core. See [Architecture](#architecture) and [Configuration](#configuration). |
| `scripts` | Docker build/run setup for the environment (see [Setup](#setup)). |

## Repository Layout

```
eiu_ws/src/
├── tb3_fleet/                  # Main project package (see below)
└── scripts/                    # Docker environment setup
```

### `tb3_fleet/` layout

```
tb3_fleet/
├── config/
│   ├── fleet/                       # tb3_simulation_config.yaml (1 robot), tb3_multi_simulation_config.yaml (3 robots)
│   ├── nav2/nav2_params.yaml        # Shared Nav2 params for all 3 robots
│   ├── multi_robot_bridge/          # ros_gz_bridge topic config, 1 file per robot (tb3_robotN_bridge.yaml)
│   ├── multi_robot_models/          # Spawned robot SDF, 1 file per robot (tb3_robotN.sdf)
│   ├── zenoh/                       # Zenoh bridge/adapter configs (single- and multi-robot)
│   └── rviz2_config.rviz
├── launch/
│   ├── tb3_simulation_nav2.launch.py       # 1-robot Gazebo + Nav2 bringup
│   ├── tb3_multi_simulation_nav2.launch.py # 3-robot Gazebo + Nav2 + tf_aggregator
│   └── tb3_world.launch.py                 # RMF core (dispatcher, schedule, fleet adapter)
├── maps/turtlebot3_world/
│   ├── world_tb3.building.yaml      # Building map (rmf_traffic_editor)
│   ├── map.yaml / map.pgm           # Occupancy grid for Nav2 AMCL
│   └── models/                      # Furniture models placed in the world
├── tb3_fleet/
│   ├── tb3_fleet_adapter.py         # Fleet adapter entry point
│   ├── tb3_robot_adapter.py         # Robot adapter: nav, stop, battery, pose (via Zenoh + Nav2)
│   ├── robot_adapter.py             # Abstract base class
│   └── tf_aggregator.py             # Republishes each robot's /tf onto the shared /tf, frame-prefixed (RViz only)
└── scripts/patch_world.py           # Patches the sensor plugin into the generated world
```

## Architecture

```
 rmf_demos_tasks (CLI)  /  rmf-web (Dashboard)
              │  ApiRequest (task_api_requests)
              ▼
     rmf_task_dispatcher (rmf_task_ros2)
              │  BidNotice (bidding, N-second window)
              ▼
     tb3_fleet_adapter  ◄──── EasyFullControl (rmf_adapter)
              │  BidProposal (cost estimate from nav graph)
              ▼
     DispatchCommand (award) ──► navigate() / stop() callbacks
              │
              ▼
     tb3_robot_adapter × 3  ──►  zenohd (router, tcp/7447)  ◄──  zenoh-bridge-ros2dds (1 shared process)
     (one per robot)                                              │  bridges /tb3_robotN/tf, .../battery_state,
                                                                    │  .../navigate_to_pose for all 3 robots
                                                                    ▼
                                                           Nav2 action server / TF / battery_state
                                                           (per robot, in Gazebo)
```

- **`rmf_task_dispatcher`** runs the bidding: broadcasts a `BidNotice`, fleet adapters reply with a `BidProposal`, then awards the task via `DispatchCommand` when the window closes.
- **`tb3_fleet_adapter`** registers the fleet via `rmf_adapter.easy_full_control`, computes the Gazebo↔RMF transform, and spins one `Tb3RobotAdapter` per configured robot (3 in the multi-robot config).
- **`tb3_robot_adapter`** talks to each robot's Nav2 action server and TF/battery topics **over Zenoh**, not native ROS2 — this lets the fleet adapter run outside the robots' ROS domain.
- Each robot's native topics are already namespaced (`/tb3_robot1/tf`, etc.), so the multi-robot Zenoh bridge is a **single shared process** — not one per robot. See [Zenoh setup](#zenoh--router-and-bridge).
- **`tf_aggregator.py`** (one instance per robot, launched automatically) republishes each robot's TF onto the shared `/tf` with prefixed frame IDs — for RViz visualization only, not part of task dispatch.
- Traffic scheduling/conflict resolution between robots is handled by `rmf_traffic_schedule` / `rmf_traffic_blockade`, independent of the fleet adapter.

## Setup

### Requirements

- Docker
- ROS2 **Jazzy** (via the pinned container image below)
- Base image pinned by digest in `scripts/Dockerfile`: `osrf/ros:jazzy-desktop@sha256:1d6f898b6ab77636c40f26298070ad3de5a9e06f0a71cf9ab066fd6b7838f151`

### Environment (Docker)

Two containers built from the same image, defined in `scripts/docker-compose.yaml`:

| Container | Purpose | ROS_DOMAIN_ID |
|---|---|---|
| `open-rmf` | Main Open-RMF benchmarking stack | 1 |
| `vda5050` | VDA5050 adapter work (separate track) | 2 |

```bash
cd ~/eiu_ws/src/scripts
docker compose up -d
docker exec -it open-rmf bash
```

### Build

```bash
cd ~/rmf_ws
colcon build --packages-select tb3_fleet
source install/setup.bash
```

## Configuration

### Fleet config

`config/fleet/` has two variants: `tb3_simulation_config.yaml` (1 robot) and `tb3_multi_simulation_config.yaml` (3 robots).

Key sections (same in both):
- **`rmf_fleet`** — velocity/acceleration limits, footprint, battery model, `task_capabilities`, `finishing_request`, `recharge_threshold`.
- **`rmf_fleet.robots.<name>`** — `charger`, `initial_map`, `map_frame`/`robot_frame`, `init_timeout_sec`.
- **`reference_coordinates`** — Gazebo↔RMF coordinate transform, as matching point pairs.
- **`fleet_manager`** — legacy connection info, unused.

### Zenoh — router and bridge

`tb3_robot_adapter.py` never touches native ROS2 topics — everything goes through Zenoh. Three processes, none started by any `ros2 launch` file except the fleet adapter:

| Process | Role | Started by |
|---|---|---|
| `zenohd` | Zenoh router, `tcp/7447`. | manual |
| `zenoh-bridge-ros2dds` | Bridges robots' native `tf`/`tf_static`/`battery_state`/`navigate_to_pose` into Zenoh. Multi-robot config bridges all 3 robots through **one** process (topics already namespaced per robot). | manual |
| `tb3_fleet_adapter.py` | Zenoh client on the RMF side, config via `--zenoh-config`. | `tb3_world.launch.py` (automatic) |

Configs in `config/zenoh/`: `tb3_multi_zenoh_bridge_ros2dds_client_config.json5` (3-robot, shared), `tb3_zenoh_bridge_ros2dds_client_config.json5` (1-robot), `tb3_fleet_adapter_zenoh_config.json5` (fleet-adapter side).

```bash
zenohd

cd ~/rmf_ws/src/tb3_fleet/config/zenoh
./zenoh-bridge-ros2dds -c tb3_multi_zenoh_bridge_ros2dds_client_config.json5
```

### Launch procedure (4 terminals)

**Terminal 1 — Zenoh router (same for both):**
```bash
zenohd
```

**Terminal 2 — Simulation + Nav2:**
```bash
# 1 robot
ros2 launch tb3_fleet tb3_simulation_nav2.launch.py

# 3 robots
ros2 launch tb3_fleet tb3_multi_simulation_nav2.launch.py
```

**Terminal 3 — Zenoh bridge (once Terminal 2 is publishing `tf`/`battery_state`):**
```bash
cd ~/rmf_ws/src/tb3_fleet/config/zenoh

# 1 robot
./zenoh-bridge-ros2dds -c tb3_zenoh_bridge_ros2dds_client_config.json5

# 3 robots
./zenoh-bridge-ros2dds -c tb3_multi_zenoh_bridge_ros2dds_client_config.json5
```

**Terminal 4 — RMF core + Fleet adapter:**
```bash
# 1 robot — fleet_config_file defaults to the 1-robot yaml, no override needed
ros2 launch tb3_fleet tb3_world.launch.py

# 3 robots — must override fleet_config_file
ros2 launch tb3_fleet tb3_world.launch.py \
  fleet_config_file:=$(ros2 pkg prefix tb3_fleet)/share/tb3_fleet/config/fleet/tb3_multi_simulation_config.yaml \
  bidding_time_window:=60.0
```

> The fleet adapter reads each robot's TF/pose through the Zenoh bridge during init (`init_timeout_sec`), so terminals 1–3 must be up first, in order.

### Dispatching tasks

```bash
# Via auction — RMF picks the best-suited robot
ros2 run rmf_demos_tasks dispatch_patrol -p charger_1 crossing_1

# Direct — target a specific robot
ros2 run rmf_demos_tasks dispatch_patrol -p charger_1 crossing_1 -F tb3_fleet -R tb3_robot1
```

Available waypoints (`maps/turtlebot3_world/`world_tb3.building.yaml)
