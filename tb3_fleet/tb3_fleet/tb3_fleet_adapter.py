#!/usr/bin/env python3

import argparse
import sys
import time
import threading
import gc
import tracemalloc

from tb3_fleet.tb3_robot_adapter import Tb3RobotAdapter
import nudged
import rclpy
import rclpy.node
from rclpy.parameter import Parameter
import rmf_adapter
from rmf_adapter import Adapter, Transformation
import rmf_adapter.easy_full_control as rmf_easy
import rmf_adapter.fleet_update_handle as rmf_fleet
from tf2_ros import Buffer
import yaml
import zenoh

CANARY_PERIOD_S = 1.0
CANARY_JITTER_WARN_S = 0.5
MEMORY_CHECK_PERIOD_S = 20.0
MEMORY_TRACE_DEPTH = 15
MEMORY_TOP_STATS = 8
CYCLE_LOG_INTERVAL = 50
SLOW_CYCLE_MULTIPLIER = 3


# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------
def compute_transforms(level, coords, node=None):
    """Get transforms between RMF and robot coordinates."""
    rmf_coords = coords['rmf']
    robot_coords = coords['robot']
    tf = nudged.estimate(rmf_coords, robot_coords)
    if node:
        mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
        node.get_logger().info(
            f'Transformation error estimate for {level}: {mse}'
        )

    return Transformation(
        tf.get_rotation(),
        tf.get_scale(),
        tf.get_translation()
    )


def update_robot(robot: Tb3RobotAdapter):
    robot_pose = robot.get_pose()
    if robot_pose is None:
        robot.node.get_logger().info(f'Failed to get pose of robot [{robot.name}]')
        return

    state = rmf_easy.RobotState(
        robot.get_map_name(),
        robot_pose,
        robot.get_battery_soc()
    )
    robot.update(state)


class CanaryWatchdog:

    def __init__(self, node, period_s: float = CANARY_PERIOD_S,
                 jitter_warn_s: float = CANARY_JITTER_WARN_S):
        self._node = node
        self._period_s = period_s
        self._jitter_warn_s = jitter_warn_s
        self._last_tick = time.monotonic()
        node.create_timer(period_s, self._on_tick)

    def _on_tick(self):
        now = time.monotonic()
        jitter = now - self._last_tick - self._period_s
        self._last_tick = now
        if jitter > self._jitter_warn_s:
            self._node.get_logger().warn(
                f'Canary heartbeat late by {jitter:.2f}s (tick interval '
                f'{self._period_s:.1f}s).'
            )
        else:
            self._node.get_logger().debug(f'Canary heartbeat ok, jitter={jitter:.3f}s.')


def _get_process_rss_mb() -> float:
    with open('/proc/self/status') as f:
        for line in f:
            if line.startswith('VmRSS:'):
                return int(line.split()[1]) / 1024.0
    return -1.0


def _run_memory_watchdog(node, period_s: float = MEMORY_CHECK_PERIOD_S):
    while rclpy.ok():
        time.sleep(period_s)
        rss_mb = _get_process_rss_mb()
        current, peak = tracemalloc.get_traced_memory()
        n_objects = len(gc.get_objects())
        node.get_logger().info(
            f'Memory: rss={rss_mb:.1f}MiB, tracemalloc current='
            f'{current/1e6:.1f}MB peak={peak/1e6:.1f}MB, gc_objects={n_objects}.'
        )
        try:
            snapshot = tracemalloc.take_snapshot()
            for i, stat in enumerate(snapshot.statistics('lineno')[:MEMORY_TOP_STATS]):
                node.get_logger().info(f'Memory top allocation #{i}: {stat}')
        except Exception as e:
            node.get_logger().warn(f'Failed to take tracemalloc snapshot: {e}')


def _start_health_checks(node):
    tracemalloc.start(MEMORY_TRACE_DEPTH)
    CanaryWatchdog(node)
    threading.Thread(target=_run_memory_watchdog, args=(node,), daemon=True).start()


class CycleStats:
    def __init__(self, target_period_s: float, log_interval: int = CYCLE_LOG_INTERVAL,
                 slow_multiplier: float = SLOW_CYCLE_MULTIPLIER):
        self.target_period_s = target_period_s
        self._log_interval = log_interval
        self._slow_threshold_s = target_period_s * slow_multiplier
        self.count = 0
        self.total_s = 0.0
        self.max_s = 0.0

    def record(self, node, cycle_dt: float):
        self.count += 1
        self.total_s += cycle_dt
        self.max_s = max(self.max_s, cycle_dt)

        if cycle_dt > self._slow_threshold_s:
            node.get_logger().warn(
                f'Update cycle took {cycle_dt:.2f}s, expected {self.target_period_s:.2f}s.'
            )
        if self.count % self._log_interval == 0:
            avg = self.total_s / self.count
            node.get_logger().info(
                f'Update loop: {self.count} cycles, avg={avg:.3f}s, '
                f'max={self.max_s:.3f}s, target={self.target_period_s:.3f}s.'
            )


def _run_update_loop(robots: dict, update_period: float, node):
    stats = CycleStats(update_period)
    while rclpy.ok():
        t0 = node.get_clock().now()
        for robot in robots.values():
            update_robot(robot)
        cycle_dt = (node.get_clock().now() - t0).nanoseconds / 1e9
        stats.record(node, cycle_dt)
        time.sleep(max(0.0, update_period - cycle_dt))


# ------------------------------------------------------------------------------
# Fleet setup
# ------------------------------------------------------------------------------
def _accept_action(description: dict):
    confirm = rmf_fleet.Confirmation()
    confirm.accept()
    return confirm


def _register_plugin_actions(fleet_handle, plugin_config, node):
    if plugin_config is None:
        return
    for plugin_name, plugin_data in plugin_config.items():
        plugin_actions = plugin_data.get('actions')
        if not plugin_actions:
            node.get_logger().warn(
                f'No action provided for plugin [{plugin_name}]! Fleet '
                f'[{fleet_handle.fleet_name}] will not bid on tasks submitted '
                f'with actions associated with this plugin unless the action '
                f'is registered as a performable action for this fleet by '
                f'the user.'
            )
            continue
        for action in plugin_actions:
            fleet_handle.more().add_performable_action(action, _accept_action)


def _create_robot_adapters(fleet_config, config_yaml, node, zenoh_session,
                            fleet_handle, tf_buffer) -> dict:
    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config_yaml = config_yaml['rmf_fleet']['robots'][robot_name]
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = Tb3RobotAdapter(
            robot_name,
            robot_config,
            robot_config_yaml,
            node,
            zenoh_session,
            fleet_handle,
            fleet_config,
            tf_buffer
        )
    return robots


# ------------------------------------------------------------------------------
# Fleet adapter
# ------------------------------------------------------------------------------
def start_fleet_adapter(
    config_path: str,
    nav_graph_path: str,
    zenoh_config_path: str | None,
    server_uri: str | None,
    use_sim_time: bool
):
    print('Starting fleet adapter...')

    rmf_adapter.init_rclcpp()

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    _start_health_checks(node)

    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    assert adapter, (
        'Unable to initialize fleet adapter. '
        'Please ensure RMF Schedule Node is running'
    )

    # Enable sim time for testing offline
    if use_sim_time:
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    fleet_config.server_uri = server_uri

    # Configure the transforms between robot and RMF frames
    for level, coords in config_yaml['reference_coordinates'].items():
        tf = compute_transforms(level, coords, node)
        fleet_config.add_robot_coordinates_transformation(level, tf)

    fleet_handle = adapter.add_easy_fleet(fleet_config)
    assert fleet_handle is not None, (
        'Failed to create EasyFullControl fleet, please verify that the '
        'fleet config is valid.'
    )

    zenoh_config = zenoh.Config.from_file(zenoh_config_path) \
        if zenoh_config_path is not None else zenoh.Config()
    zenoh_session = zenoh.open(zenoh_config)
    tf_buffer = Buffer()

    _register_plugin_actions(fleet_handle, config_yaml.get('plugins'), node)

    robots = _create_robot_adapters(
        fleet_config, config_yaml, node, zenoh_session, fleet_handle, tf_buffer
    )

    update_frequency = config_yaml['rmf_fleet'].get('robot_state_update_frequency', 10.0)
    update_period = 1.0 / update_frequency
    update_thread = threading.Thread(
        target=_run_update_loop, args=(robots, update_period, node), daemon=True
    )
    update_thread.start()
    node.get_logger().info(f'Started update thread with period {update_period:.3f}s')

    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    try:
        rclpy_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy_executor.shutdown()
        zenoh_session.close()


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='fleet_adapter',
        description='Configure and spin up the fleet adapter')
    parser.add_argument('-c', '--config_file', type=str, required=True,
                        help='Path to the config.yaml file')
    parser.add_argument('-n', '--nav_graph', type=str, required=True,
                        help='Path to the nav_graph for this fleet adapter')
    parser.add_argument('-s', '--server_uri', type=str, required=False,
                        default='',
                        help='URI of the api server to transmit state and '
                             'task information.')
    parser.add_argument('-sim', '--use_sim_time',
                        type=lambda v: str(v).lower() in ('true', '1', 'yes'),
                        default=False,
                        help="Use sim time ('true'/'false'), default: false. "
                             "Takes a value so a launch file can pass its own "
                             "use_sim_time LaunchConfiguration through directly.")
    parser.add_argument(
        '--zenoh-config',
        type=str,
        help='Path to custom zenoh configuration file to be used, if not '
        'provided the default config will be used'
    )
    args = parser.parse_args(args_without_ros[1:])

    start_fleet_adapter(
        config_path=args.config_file,
        nav_graph_path=args.nav_graph,
        zenoh_config_path=args.zenoh_config
        if args.zenoh_config != '' else None,
        server_uri=args.server_uri if args.server_uri != '' else None,
        use_sim_time=args.use_sim_time
    )

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
    