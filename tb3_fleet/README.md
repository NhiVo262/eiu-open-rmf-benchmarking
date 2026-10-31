# TurtleBot3 Fleet Integration

This package integrates three TurtleBot3 robots with Open-RMF in simulation and
on hardware. It includes the fleet adapter, launch files and platform-specific
configuration.
## Platform configuration

Configuration is held in two places. `config/` in this package is
version-controlled and read on the workstation.
### In this package, read on the workstation

```text
config/
  simulation/
    fleet/multi_robot_config_sim.yaml                          RMF fleet, 3 robots
    nav2_params.yaml                                           Nav2 parameters
    gz_bridge/tb3_robotN.yaml                                  ros_gz_bridge topic mapping
    models/tb3_robotN.sdf                                      Gazebo robot model
    zenoh_bridge/tb3_zenoh_bridge_single_robot_config_sim.json5  Bridge for 1 robot
    zenoh_bridge/tb3_zenoh_bridge_multi_robot_config_sim.json5   Bridge for 3 robots
  hardware/
    fleet/multi_robot_config_hw.yaml                           RMF fleet, 3 robots       [workstation]
    zenoh_router/tb3_zenoh_router_config_hw.json5               Workstation router        [workstation]
    zenoh_bridge/tb3_zenoh_bridge_robotN_config_hw.json5        Bridge, one file per robot [robot]
    autostart/systemd/bringup.service                           Bring-up systemd unit      [robot]
    autostart/systemd/nav2.service                              Nav2 systemd unit          [robot]
    autostart/systemd/zenoh-bridge.service                      Robot bridge systemd unit  [robot]
    autostart/run-bringup.sh                                    Launched by bringup.service [robot]
    autostart/run-nav2.sh                                       Launched by nav2.service   [robot]
    autostart/env.sh                                            Environment the units source [robot]
```

Everything under `simulation/` is read on the workstation. Under `hardware/`,
`[workstation]` marks a file the RMF side reads directly, and `[robot]` marks a
file that is installed onto hardware.

The baseline runners, `run_simulation_baselines.sh` and
`run_hardware_baselines.sh`, derive single-robot fleet configurations from the
corresponding three-robot configurations at runtime.

Nav2 parameters are included only for simulation. On hardware, Nav2 runs on each
robot using parameters from `turtlebot3_navigation2`.

### On each robot, set up on the hardware itself

Bring-up and Nav2 run as systemd units on the hardware. The units marked `[robot]` above
are their version-controlled source; each robot runs an installed copy under
`~/turtlebot3_ws/`, with `@USER@` substituted.

```text
~/turtlebot3_ws/
  autostart/
    install.sh                                  Installs the units into /etc/systemd/system
    env.sh                                      ROS_DOMAIN_ID, RMW_IMPLEMENTATION, NHATBOT_MAP
    run-bringup.sh                              ros2 launch turtlebot3_bringup robot.launch.py
    run-nav2.sh                                 ros2 launch turtlebot3_navigation2 navigation2.
    systemd/nhatbot-bringup.service             OpenCR and LiDAR bring-up
    systemd/nhatbot-nav2.service                Nav2, after bring-up
  config/
    tb3_zenoh_bridge_robotN_config_hw.json5     This robot's bridge configuration
    zenoh-bridge-ros2dds                        arm64 build of the bridge
```

# Zenoh network

The transport architecture uses two components:

| Component | Location | Configuration |
|---|---|---|
| Zenoh router, `zenohd` | Workstation running the RMF core and fleet adapter | `config/hardware/zenoh_router/` for hardware scenario runs |
| `zenoh-bridge-ros2dds`, simulation | Workstation, one instance for all robots | `config/simulation/zenoh_bridge/tb3_zenoh_bridge_multi_robot_config_sim.json5` |
| `zenoh-bridge-ros2dds`, hardware | Each robot, one instance per robot | `config/hardware/zenoh_bridge/tb3_zenoh_bridge_robotN_config_hw.json5`, with N = 1, 2, 3 |

The fleet adapter connects as a Zenoh client. Its session settings are defined
in the `zenoh:` section of the fleet configuration.

The bridge configurations expose robot topics and the `navigate_to_pose` action
over Zenoh. On hardware, robot N runs its bridge with
`tb3_zenoh_bridge_robotN_config_hw.json5`, which sets the namespace
`/tb3_robotN` and connects to the workstation router at
`tcp/192.168.10.65:7447`.

`nhatbot-zenoh-bridge.service` in this package runs that bridge as a systemd
service.

Hardware benchmark runners also start a workstation bridge configured by
`src/benchmark/collect/hardware/zenoh_bridge_workstation.json5`. This bridge
makes robot topics available on workstation DDS for recording and analysis.
The fleet adapter reads telemetry directly from Zenoh.

`tb3_zenoh_bridge_multi_robot_config_sim.json5` leaves the bridge namespace unset
because the simulated robot topics are already namespaced.

## Launch files

```text
launch/
  rmf/tb3_world.launch.py                       RMF core, visualiser and fleet adapter
  simulation/tb3_simulation_nav2.launch.py       Gazebo, Nav2 and 1 robot
  simulation/tb3_multi_simulation_nav2.launch.py Gazebo, Nav2 and 3 robots
```

`tb3_world.launch.py` supports both platforms. Select the fleet configuration
with `fleet_config_file`; the default is the simulation configuration. Set
`use_sim_time:=false` for hardware.

`tb3_multi_simulation_nav2.launch.py` starts the robots listed in
`TB3_SIM_ROBOTS`, a comma-separated list of robot names. The simulation baseline
runner uses this option for single-robot runs.

## Package components

| Component | Purpose |
|---|---|
| `tb3_fleet/robot_adapter.py` | Robot adapter interface |
| `tb3_fleet/tb3_robot_adapter.py` | TurtleBot3 adapter implementation |
| `tb3_fleet/tb3_fleet_adapter.py` | Fleet adapter node |
| `tb3_fleet/set_initial_pose.py`, `tb3_fleet/tf_aggregator.py` | Initial pose and TF helpers used by the simulation launch files |
| `tb3_fleet/nav2_tb3_get_tf.py`, `tb3_fleet/nav2_tb3_send_navigate_to_pose.py` | Manual TF queries and navigation requests over Zenoh |
| `maps/` | Map sources and generated simulation resources |
| `scripts/patch_world.py` | Gazebo world modifications applied by CMake |
| `tests/` | Unit tests for the robot adapter and initial pose helper |

## Map sources and generated resources

Two directories represent the same laboratory environment:

| Directory | Origin | Contents |
|---|---|---|
| `maps/turtlebot3_world/` | Version-controlled source | `world_tb3.building.yaml` for the traffic editor; `map.yaml`, `map.pgm` and `map.png` for the SLAM map used on both platforms; `models/` for Gazebo assets |
| `maps/world_tb3/` | Generated during the build | `world_tb3.world`, `models/`, `nav_graphs/0.yaml` |

`building_map_generator` derives the output directory name from the
`.building.yaml` filename. Keeping the source directory separate prevents the
source and generated `models/` directories from merging during installation.

