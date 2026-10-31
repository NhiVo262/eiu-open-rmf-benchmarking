#!/usr/bin/env python3

import argparse
import json
import sys
import time
import threading
from concurrent.futures import ThreadPoolExecutor
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
from rclpy.qos import qos_profile_system_default
from rmf_fleet_msgs.msg import ClosedLanes, LaneRequest
from tf2_ros import Buffer
import yaml
import zenoh

CANARY_PERIOD_S = 1.0
CANARY_JITTER_WARN_S = 0.5
MEMORY_CHECK_PERIOD_S = 20.0
MEMORY_TRACE_DEPTH = 15
MEMORY_TOP_STATS = 8
CYCLE_LOG_INTERVAL = 50
UPDATE_FAILURE_LOG_EVERY = 100
# At 10 Hz these are 1 s of silence before alarming, then once a minute.
POSE_FAILURES_BEFORE_ALARM = 10
POSE_FAILURE_REPEAT_EVERY = 600
UPDATE_THREAD_JOIN_TIMEOUT_S = 5.0
STATE_PREPARE_TIMEOUT_S = 1.0
SLOW_CYCLE_MULTIPLIER = 3
# Real-time step the update loop re-reads the simulated clock at while waiting out a
# period. Well under the 0.1 s period, so the pacing error it leaves is below one cycle.
SIM_CLOCK_POLL_S = 0.005


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


def prepare_robot_state(robot: Tb3RobotAdapter):
    """Read the robot's current state. Runs on a worker thread."""
    robot_pose = robot.get_pose()
    if robot_pose is None:
        return None
    return rmf_easy.RobotState(
        robot.get_map_name(),
        robot_pose,
        robot.get_battery_soc()
    )


def commit_robot_state(robot: Tb3RobotAdapter, state):
    """Hand a prepared state to RMF, or account for a missing one."""
    if state is None:
        robot.pose_failures += 1
        if (robot.pose_failures == POSE_FAILURES_BEFORE_ALARM
                or robot.pose_failures % POSE_FAILURE_REPEAT_EVERY == 0):
            robot.report_pose_unavailable(robot.pose_failures)
        return
    if robot.pose_failures:
        robot.report_pose_recovered(robot.pose_failures)
        robot.pose_failures = 0
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


def _start_health_checks(node, enable_memory_watchdog: bool = False):
    """Canary always; the memory watchdog only when asked for."""
    CanaryWatchdog(node)
    if not enable_memory_watchdog:
        return
    tracemalloc.start(MEMORY_TRACE_DEPTH)
    node.get_logger().warn(
        'Memory watchdog enabled: expect ~7% of a core and tracemalloc '
        'overhead on every allocation.'
    )
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


def _loop_pacing(node, use_sim_time, stop_event):
    """Return (now, wait) for the update loop's clock.

    Under simulated time both come from the node's clock, so one cycle is one period of
    simulated time, as it is one period of real time on hardware. Pacing a simulated run on
    the wall clock pushes a state update every 0.1 s of real time while the robot advances
    through less than that in simulated time, which hands RMF more updates per metre
    travelled than the physical fleet produces. The wait polls the clock in short real
    sleeps rather than blocking on it, so it still returns when stop_event is set and
    cannot hang if /clock stops arriving.
    """
    if not use_sim_time:
        def wait(seconds):
            if stop_event is not None:
                stop_event.wait(seconds)
            else:
                time.sleep(seconds)

        return time.monotonic, wait

    clock = node.get_clock()

    def now():
        return clock.now().nanoseconds / 1e9

    def wait(seconds):
        target = now() + seconds
        while now() < target:
            if stop_event is not None and stop_event.is_set():
                return
            time.sleep(SIM_CLOCK_POLL_S)

    return now, wait


def _run_update_loop(robots: dict, update_period: float, node, fleet_handle=None,
                     reassign_interval: float = 60.0, stop_event=None,
                     use_sim_time: bool = False):
    """Push every robot's state to RMF, forever, at update_period."""
    stats = CycleStats(update_period)
    now, wait = _loop_pacing(node, use_sim_time, stop_event)
    last_reassign = now()
    failures = {}
    pool = ThreadPoolExecutor(
        max_workers=max(1, len(robots)),
        thread_name_prefix='robot_state'
    )

    def note_failure(name, stage, e):
        failures[name] = failures.get(name, 0) + 1
        # Throttled: a robot that keeps failing would otherwise fill the log
        # with one traceback per cycle, ten times a second.
        if failures[name] == 1 or failures[name] % UPDATE_FAILURE_LOG_EVERY == 0:
            node.get_logger().error(
                f'Failed to {stage} robot [{name}] '
                f'({failures[name]} consecutive): {type(e).__name__}: {e}'
            )

    try:
        while rclpy.ok() and not (stop_event is not None and stop_event.is_set()):
            t0 = now()

            pending = {
                name: pool.submit(prepare_robot_state, robot)
                for name, robot in robots.items()
            }
            prepared = {}
            for name, future in pending.items():
                try:
                    prepared[name] = future.result(timeout=STATE_PREPARE_TIMEOUT_S)
                except Exception as e:
                    prepared[name] = None
                    note_failure(name, 'read', e)

            for name, robot in robots.items():
                try:
                    commit_robot_state(robot, prepared.get(name))
                    if prepared.get(name) is not None:
                        failures[name] = 0
                except Exception as e:
                    note_failure(name, 'update', e)

            try:
                if (fleet_handle is not None
                        and t0 - last_reassign > reassign_interval):
                    fleet_handle.more().reassign_dispatched_tasks()
                    last_reassign = t0
            except Exception as e:
                node.get_logger().error(f'reassign_dispatched_tasks failed: {e}')

            cycle_dt = now() - t0
            stats.record(node, cycle_dt)
            wait(max(0.0, update_period - cycle_dt))
    finally:
        pool.shutdown(wait=False, cancel_futures=True)


def _accept_action(description: dict):
    confirm = rmf_fleet.Confirmation()
    confirm.accept()
    return confirm


def _subscribe_lane_requests(fleet_handle, fleet_name, node):
    """Apply lane open and close requests, and report which lanes are held closed.

    EasyFullControl does not read lane_closure_requests; the handling lives in each
    fleet adapter, as it does in rmf_demos_fleet_adapter. Without this a request is
    published, the sender exits successfully and no lane ever closes, so a scenario that
    names a closed lane runs with the graph intact and measures something else.

    Returns the subscription, which the caller must keep a reference to.
    """
    closed_lanes = set()
    closed_lanes_pub = node.create_publisher(
        ClosedLanes, 'closed_lanes', qos_profile_system_default)

    def on_request(msg):
        if msg.fleet_name and msg.fleet_name != fleet_name:
            return
        if msg.open_lanes:
            fleet_handle.more().open_lanes(list(msg.open_lanes))
            closed_lanes.difference_update(msg.open_lanes)
        if msg.close_lanes:
            fleet_handle.more().close_lanes(list(msg.close_lanes))
            closed_lanes.update(msg.close_lanes)
        node.get_logger().info(
            f'[{fleet_name}] lanes now closed: {sorted(closed_lanes)}')
        closed_lanes_pub.publish(
            ClosedLanes(fleet_name=fleet_name, closed_lanes=sorted(closed_lanes)))

    return node.create_subscription(
        LaneRequest, 'lane_closure_requests', on_request,
        qos_profile=qos_profile_system_default)


def _register_plugin_actions(fleet_handle, fleet_name, plugin_config, node):
    if plugin_config is None:
        return
    for plugin_name, plugin_data in plugin_config.items():
        plugin_actions = plugin_data.get('actions')
        if not plugin_actions:
            node.get_logger().warn(
                f'No action provided for plugin [{plugin_name}]! Fleet '
                f'[{fleet_name}] will not bid on tasks submitted '
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


def zenoh_config(settings):
    """Return a zenoh Config with the fleet config's zenoh settings applied.

    Each key is a zenoh config path, such as mode or connect/endpoints, and its value is
    inserted as JSON. An absent or empty section leaves the zenoh defaults in place.
    """
    config = zenoh.Config()
    for key, value in (settings or {}).items():
        config.insert_json5(key, json.dumps(value))
    return config


def start_fleet_adapter(
    config_path: str,
    nav_graph_path: str,
    server_uri: str | None,
    use_sim_time: bool
):
    print('Starting fleet adapter...')

    rmf_adapter.init_rclcpp()

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    # Not assert: assertions vanish under python -O, and these three are the
    # conditions that make everything after them meaningless.
    if not fleet_config:
        raise RuntimeError(f'Failed to parse config file [{config_path}]')

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    _start_health_checks(node)

    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    if adapter is None:
        raise RuntimeError(
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
    if fleet_handle is None:
        raise RuntimeError(
            'Failed to create EasyFullControl fleet, please verify that the '
            'fleet config is valid.'
        )

    zenoh_session = zenoh.open(zenoh_config(config_yaml.get('zenoh')))
    tf_buffer = Buffer()

    _register_plugin_actions(fleet_handle, fleet_name, config_yaml.get('plugins'), node)
    # Held for the life of the adapter: a subscription that is not referenced is
    # collected and the fleet silently stops honouring lane closures.
    node.lane_request_sub = _subscribe_lane_requests(fleet_handle, fleet_name, node)

    robots = _create_robot_adapters(
        fleet_config, config_yaml, node, zenoh_session, fleet_handle, tf_buffer
    )

    update_frequency = config_yaml['rmf_fleet'].get('robot_state_update_frequency', 10.0)
    if not isinstance(update_frequency, (int, float)) or update_frequency <= 0:
        raise ValueError(
            f'robot_state_update_frequency must be a positive number, got '
            f'[{update_frequency!r}]'
        )
    update_period = 1.0 / update_frequency
    stop_event = threading.Event()
    update_thread = threading.Thread(
        target=_run_update_loop,
        args=(robots, update_period, node, fleet_handle, 60.0, stop_event,
              use_sim_time),
        daemon=True
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
        stop_event.set()
        update_thread.join(timeout=UPDATE_THREAD_JOIN_TIMEOUT_S)
        if update_thread.is_alive():
            node.get_logger().warn(
                f'Update thread did not stop within '
                f'{UPDATE_THREAD_JOIN_TIMEOUT_S:.1f}s; shutting down anyway.'
            )
        node.destroy_node()
        rclpy_executor.shutdown()
        zenoh_session.close()


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
    args = parser.parse_args(args_without_ros[1:])

    start_fleet_adapter(
        config_path=args.config_file,
        nav_graph_path=args.nav_graph,
        server_uri=args.server_uri if args.server_uri != '' else None,
        use_sim_time=args.use_sim_time
    )

    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
    
