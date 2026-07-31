#EIU-OPEN-RMF-BENCHMARKING

## Overview

Workspace for benchmarking Open-RMF with a TurtleBot3 fleet, as part of the EIU x ARTC project. It combines the upstream Open-RMF stack with a custom fleet integration package (`tb3_fleet`) that connects TurtleBot3 (Nav2) to Open-RMF, enabling task dispatch, and providing the basis for the benchmarking test plan.

## Packages

| Package | Role |
|---|---|
| **`tb3_fleet`** | Custom integration package: fleet adapter, robot adapter, world/nav-graph maps, launch files for simulation and RMF core. Project-specific — see [Architecture](#architecture) and [Configuration](#configuration) below.
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
     tb3_robot_adapter  ─────►  zenohd (router, tcp/7447)  ◄─────  zenoh-bridge-ros2dds
     (zenoh client)                                                (zenoh client, namespace /tb3_robot1)
                                                                          │  bridges native ROS2 topics
                                                                          ▼
                                                                 Nav2 action server / TF / battery_state
                                                                 (robot or Gazebo sim)
```

- **`rmf_task_dispatcher`** runs the bidding process: it broadcasts a `BidNotice`, fleet adapters reply with a `BidProposal` (cost), and after the bidding window closes it awards the task to the winning fleet via `DispatchCommand`.
- **`tb3_fleet_adapter`** (`tb3_fleet_adapter.py`) registers the fleet with `rmf_adapter.easy_full_control`, computes the Gazebo↔RMF coordinate transform, and spins one `Tb3RobotAdapter` per configured robot.
- **`tb3_robot_adapter.py`** implements the `navigate`/`stop`/`execute_action` callbacks. It talks to the robot's Nav2 action server and TF/battery topics **over Zenoh** rather than native ROS2 topics — this is what lets the fleet adapter run outside the robot's ROS domain (e.g. in a separate Docker container or over the network). See [Zenoh setup](#zenoh--router-and-bridge) below — neither the router nor the bridge is started automatically by any launch file, they must be run manually.
- Traffic scheduling and conflict resolution between robots is handled internally by `rmf_traffic_schedule` / `rmf_traffic_blockade` (from `rmf_traffic_ros2`), independent of the fleet adapter.

## Setup

### Requirements

- Docker 
- ROS2 **Jazzy** (provided via the pinned container image, see below)
- The container image **must be pinned to a fixed digest**. The digest currently pinned is:
  ```
  sha256:d1b0540be35cd81afb913d28fd4c11397f1e783ff3f1669f0343cd062cfdf202
  ```

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

- **`rmf_fleet`** — fleet-wide parameters: velocity/acceleration limits, footprint, battery model, `task_capabilities`, `finishing_request` (behavior after a task completes), `recharge_threshold`.
- **`rmf_fleet.robots.<name>`** — per-robot settings: `charger`, `initial_map`, `map_frame`/`robot_frame`, `init_timeout_sec`.
- **`reference_coordinates`** — the Gazebo↔RMF coordinate transform, defined as matching point pairs per map. 
- **`fleet_manager`** — legacy fleet manager connection info (host/port/credentials).

### Zenoh — router and bridge

`tb3_robot_adapter.py` never touches the robot's Nav2 action server or TF/battery topics through native ROS2 — everything goes through Zenoh. This requires **three** separate processes, none of which are started by any `ros2 launch` file in this repo — they must each be run manually (or wrapped in your own launch/systemd setup):

| Process | Role | Started by |
|---|---|---|
| `zenohd` | Zenoh **router** — the rendezvous point both clients below connect to. Default listen port `tcp/7447`. | manual |
| `zenoh-bridge-ros2dds` | Zenoh **client** on the robot side. Bridges the robot's *native* ROS2 topics/actions (`tf`, `tf_static`, `battery_state`, `navigate_to_pose` action) into Zenoh, under the namespace configured in `ros2dds.namespace` (`/tb3_robot1`). | manual |
| `tb3_fleet_adapter.py` | Zenoh **client** on the RMF side (Python `zenoh` session, config passed via `--zenoh-config`). Subscribes to `tb3_robot1/tf`, `tb3_robot1/battery_state`, and sends nav goals as Zenoh queries under `tb3_robot1/navigate_to_pose/...`. | `tb3_world.launch.py` (automatic — see below) |

Config files, in `tb3_fleet/config/zenoh/`:

- **`tb3_zenoh_bridge_ros2dds_client_config.json5`** — bridge-side config. Key fields:
  - `ros2dds.namespace: "/tb3_robot1"` — prefix applied to every bridged topic; must match the robot name used in `tb3_simulation_config.yaml` and in `dispatch_patrol -R <robot>`.
  - `ros2dds.allow` — explicit allow-list of what gets bridged (only `tf`, `tf_static`, `battery_state` as publishers, and the `navigate_to_pose` action). Anything not listed here is invisible to the fleet adapter.
  - `connect.endpoints: ["tcp/127.0.0.1:7447"]` — the router address. For a real robot on separate hardware, change this to the router's actual reachable IP.
- **`tb3_fleet_adapter_zenoh_config.json5`** — fleet-adapter-side config, same router endpoint, `mode: "client"`.

#### Commands

Run in this order — the router must be up before either client connects, and the robot's native ROS2 topics (from `tb3_simulation_nav2.launch.py`, or the real robot's Nav2 stack) must be publishing before the bridge has anything to bridge:

```bash
# 1. Start the Zenoh router
zenohd

# 2. Start the ROS2↔Zenoh bridge (run where the robot's native ROS2 topics are visible —
#    inside the sim container for Gazebo, or on the robot's Raspberry Pi for real hardware).
#    The bridge binary ships locally in this repo, so run it from that directory:
cd ~/rmf_ws/src/tb3_fleet/config/zenoh
./zenoh-bridge-ros2dds -c tb3_zenoh_bridge_ros2dds_client_config.json5
```

`tb3_fleet_adapter.py` (the third Zenoh client) does **not** need a separate command — it's launched as part of `tb3_world.launch.py` (see [Launch procedure](#launch-procedure-2-terminals) below), which passes `tb3_fleet_adapter_zenoh_config.json5` via `--zenoh-config` automatically.

### Launch-time flags to avoid

- **Do not pass `--use_sim_time`** when submitting tasks via `rmf_demos_tasks`, and **do not pass the `-sim`** flag to the fleet adapter in `tb3_world.launch.py`. The task-dispatch pipeline (`rmf_task_dispatcher`, `tb3_fleet_adapter`) is configured to run on **wall time**. If `/clock` isn't published yet when these run on sim time, the bidding timer (`bidding_time_window`) freezes and tasks silently never get dispatched, even though bidding itself appears to proceed normally in the logs.

### Launch procedure (4 terminals)

**Terminal 1 — Zenoh router:**
```bash
zenohd
```

**Terminal 2 — Simulation + Nav2:**
```bash
ros2 launch tb3_fleet tb3_simulation_nav2.launch.py
```

**Terminal 3 — Zenoh bridge (start once Terminal 2 is publishing `tf`/`battery_state`):**
```bash
cd ~/rmf_ws/src/tb3_fleet/config/zenoh
./zenoh-bridge-ros2dds -c tb3_zenoh_bridge_ros2dds_client_config.json5
```

**Terminal 4 — RMF core + Fleet adapter (start after Terminal 3 is bridging):**
```bash
ros2 launch tb3_fleet tb3_world.launch.py
```

> The fleet adapter needs to read the robot's TF/pose **through the Zenoh bridge** during initialization (`init_timeout_sec`, default 30s), so terminals 1–3 must all be up first, in that order.

### Dispatching tasks

```bash
# Via auction — RMF picks the best-suited robot
ros2 run rmf_demos_tasks dispatch_patrol -p wp3 wp4

# Direct — target a specific robot
ros2 run rmf_demos_tasks dispatch_patrol -p wp3 wp4 -F tb3_fleet -R tb3_robot1
```

Available waypoints (see `maps/tb3_world/nav_graphs/0.yaml`): `wp3`, `wp4`, `wp5`, `wp6`, `wp2_parking`, `wp1_charging`, `initial_wp`.
