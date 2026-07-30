#EIU-OPEN-RMF-BENCHMARKING

## Overview

Workspace for benchmarking Open-RMF with a TurtleBot3 fleet, as part of the EIU x ARTC project. It combines the upstream Open-RMF stack with a custom fleet integration package (`tb3_fleet`) that connects TurtleBot3 (Nav2) to Open-RMF, enabling task dispatch, and providing the basis for the benchmarking test plan.

## Packages

| Package | Role |
|---|---|
| **`tb3_fleet`** | Custom integration package: fleet adapter, robot adapter, world/nav-graph maps, launch files for simulation and RMF core. Project-specific — see [Architecture](#architecture) and [Configuration](#configuration) below. |
| `free_fleet` | Zenoh-based bridge library/messages used by `tb3_fleet_adapter` to communicate with the robot's Nav2 stack (goal send/cancel/result, TF, battery state) without requiring a shared ROS domain. |
| `rmf_demos` | Upstream Open-RMF demo package; provides the `rmf_demos_tasks` CLI tools used to dispatch tasks (`dispatch_patrol`, `dispatch_go_to_place`, `cancel_task`, etc.). |
| `rmf_traffic_editor` | Editor and tools (`rmf_building_map_tools`) used to author the building map and generate the Gazebo world and nav graph from it. |
| `rmf_visualization` | RViz plugins and visualizer nodes (fleet states, floorplans, nav graphs, schedule) used by `tb3_fleet/launch/rmf_visualization.launch.xml`. |
| `rmf-web` | Open-RMF web dashboard and API server, for submitting/monitoring tasks and fleet status from a browser. |
| `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_simulations` | Upstream TurtleBot3 packages: robot description, messages, and Gazebo simulation assets. |
| `scripts` | Docker build/run setup for the environment (see [Setup](#setup)). |

## Repository Layout

```
eiu_ws/src/
├── tb3_fleet/                  # Main project package (see below)
├── free_fleet/                 # Zenoh bridge for Nav2 robots
├── rmf_demos/                  # Task dispatch CLI tools
├── rmf_traffic_editor/         # Map authoring + world/nav-graph generation
├── rmf_visualization/          # RViz visualization plugins
├── rmf-web/                    # Web dashboard + API server
├── turtlebot3*/                # Upstream TurtleBot3 packages
└── scripts/                    # Docker environment setup
```

### `tb3_fleet/` layout

```
tb3_fleet/
├── config/
│   ├── fleet/tb3_simulation_config.yaml   # Fleet config: robot, transform, charger...
│   ├── zenoh/                             # Zenoh bridge config (robot <-> fleet adapter)
│   └── rviz2_config.rviz
├── launch/
│   ├── tb3_simulation_nav2.launch.py      # Gazebo + spawn robot + Nav2 bringup
│   ├── tb3_world.launch.py                # RMF core (dispatcher, schedule, fleet adapter)
│   ├── rmf_visualization.launch.xml
│   └── ...
├── maps/tb3_world/
│   ├── tb3_world.building.yaml            # Building map (rmf_traffic_editor)
│   ├── nav_graphs/0.yaml                  # Nav graph (waypoints, lanes)
│   └── tb3_world.world                    # Gazebo world (generated from building.yaml)
├── tb3_fleet/
│   ├── tb3_fleet_adapter.py               # Fleet adapter entry point
│   ├── tb3_robot_adapter.py               # Robot adapter: nav, stop, battery, pose (via Zenoh + Nav2)
│   └── robot_adapter.py                   # Abstract base class
└── scripts/patch_world.py                 # Patches the sensor plugin into the world after generation
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
     tb3_robot_adapter  ──Zenoh──►  Nav2 action server (robot)
              │                          │
              ▼                          ▼
     TF / battery_state  ◄──Zenoh──  robot / Gazebo sim
```

- **`rmf_task_dispatcher`** runs the bidding process: it broadcasts a `BidNotice`, fleet adapters reply with a `BidProposal` (cost), and after the bidding window closes it awards the task to the winning fleet via `DispatchCommand`.
- **`tb3_fleet_adapter`** (`tb3_fleet_adapter.py`) registers the fleet with `rmf_adapter.easy_full_control`, computes the Gazebo↔RMF coordinate transform, and spins one `Tb3RobotAdapter` per configured robot.
- **`tb3_robot_adapter.py`** implements the `navigate`/`stop`/`execute_action` callbacks. It talks to the robot's Nav2 action server and TF/battery topics **over Zenoh** rather than native ROS2 topics — this is what lets the fleet adapter run outside the robot's ROS domain (e.g. in a separate Docker container or over the network).
- Traffic scheduling and conflict resolution between robots is handled internally by `rmf_traffic_schedule` / `rmf_traffic_blockade` (from `rmf_traffic_ros2`), independent of the fleet adapter.

## Setup

### Requirements

- Docker with X11 forwarding (for Gazebo/RViz GUI)
- ROS2 **Jazzy** (provided via the pinned container image, see below)

### Environment (Docker)

The environment runs inside two containers built from the same image, defined in `scripts/docker-compose.yaml`:

| Container | Purpose | ROS_DOMAIN_ID |
|---|---|---|
| `open-rmf` | Main Open-RMF benchmarking stack | 1 |
| `vda5050` | VDA5050 adapter work (separate track) | 2 |

```bash
cd ~/eiu_ws/src/scripts
docker compose up -d
docker exec -it open-rmf bash
```

The image is pinned to a fixed tag (`eiu-artc/open-rmf:jazzy-20260622`) to keep the RMF/Nav2/Gazebo versions reproducible across machines. `${HOME}/eiu_ws` on the host is mounted to `/home/eiu/rmf_ws` in the container.

### Build

Inside the container:

```bash
cd ~/rmf_ws
colcon build --packages-select tb3_fleet
source install/setup.bash
```

## Configuration

### Fleet config — `tb3_fleet/config/fleet/tb3_simulation_config.yaml`

Key sections:

- **`rmf_fleet`** — fleet-wide parameters: velocity/acceleration limits, footprint, battery model, `task_capabilities` (e.g. `loop: true`), `finishing_request` (behavior after a task completes), `recharge_threshold`.
- **`rmf_fleet.robots.<name>`** — per-robot settings: `charger`, `navigation_stack` (1 or 2), `initial_map`, `map_frame`/`robot_frame`, `init_timeout_sec`.
- **`reference_coordinates`** — the Gazebo↔RMF coordinate transform, defined as matching point pairs per map. **Must be accurate**, or the planner will compute incorrect costs or fail to snap the robot onto the nav graph.
- **`fleet_manager`** — legacy fleet manager connection info (host/port/credentials).

### Zenoh config — `tb3_fleet/config/zenoh/`

Defines the Zenoh session used by the fleet adapter to reach the robot's Nav2 action server, TF, and battery topics without a shared ROS domain.

### Launch-time flags to avoid

- **Do not pass `--use_sim_time`** when submitting tasks via `rmf_demos_tasks`, and **do not pass the `-sim`** flag to the fleet adapter in `tb3_world.launch.py`. The task-dispatch pipeline (`rmf_task_dispatcher`, `tb3_fleet_adapter`) is configured to run on **wall time**. If `/clock` isn't published yet when these run on sim time, the bidding timer (`bidding_time_window`) freezes and tasks silently never get dispatched, even though bidding itself appears to proceed normally in the logs.

### Launch procedure (2 terminals)

**Terminal 1 — Simulation + Nav2:**
```bash
ros2 launch tb3_fleet tb3_simulation_nav2.launch.py
```

**Terminal 2 — RMF core + Fleet adapter (start after Terminal 1 is ready):**
```bash
ros2 launch tb3_fleet tb3_world.launch.py
```

> The fleet adapter needs to read the robot's TF/pose during initialization (`init_timeout_sec`, default 30s), so Terminal 1 must be up first.

### Dispatching tasks

```bash
# Via auction — RMF picks the best-suited robot
ros2 run rmf_demos_tasks dispatch_patrol -p wp3 wp4

# Direct — target a specific robot
ros2 run rmf_demos_tasks dispatch_patrol -p wp3 wp4 -F tb3_fleet -R tb3_robot1
```

Available waypoints (see `maps/tb3_world/nav_graphs/0.yaml`): `wp3`, `wp4`, `wp5`, `wp6`, `wp2_parking`, `wp1_charging`, `initial_wp`.
