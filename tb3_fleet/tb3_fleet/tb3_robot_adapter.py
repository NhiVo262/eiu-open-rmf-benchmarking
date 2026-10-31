#!/usr/bin/env python3

import datetime
import math
import threading
import time
from typing import Annotated

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.ros2_types import (
    ActionMsgs_CancelGoal_Request,
    ActionMsgs_CancelGoal_Response,
    ActionMsgs_GoalInfo,
    UUID,
    GeometryMsgs_Point,
    GeometryMsgs_Pose,
    GeometryMsgs_PoseStamped,
    GeometryMsgs_Quaternion,
    GoalStatus,
    Header,
    NavigateToPose_GetResult_Request,
    NavigateToPose_GetResult_Response,
    NavigateToPose_SendGoal_Request,
    NavigateToPose_SendGoal_Response,
    SensorMsgs_BatteryState,
    TFMessage,
    Time,
)

from .robot_adapter import ExecutionHandle, RobotAdapter

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
import rmf_adapter.easy_full_control as rmf_easy
import rmf_adapter.plan as plan
from rmf_adapter.robot_update_handle import ActivityIdentifier, Tier
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import zenoh


def namespacify(base_name: str, namespace: str, delimiter: str = '/') -> str:
    return base_name if not namespace else f'{namespace}{delimiter}{base_name}'


NAV_SEND_MAX_ATTEMPTS = 5
# Seconds to wait for a reply from the robot before retrying a send_goal or a
# cancel_goal query.
NAV_QUERY_TIMEOUT_S = 5.0
NAV_SEND_RETRY_PERIOD_S = 1.0
NAV_RESULT_TIMEOUT_S = 600.0
# Consecutive Nav2 goals that may end in a non-terminal failure before the command is
# reported to RMF as failed. Without a bound, a robot that cannot reach any goal replans
# indefinitely and the repeat ends on the runner's timeout rather than on a result.
NAV_MAX_CONSECUTIVE_REPLANS = 3
# Delay before each replan, indexed by the consecutive failure count. The last entry
# applies to every further failure.
NAV_REPLAN_BACKOFF_S = (0.0, 5.0, 15.0)
# action_msgs/srv/CancelGoal return codes.
CANCEL_RETURN_CODES = {
    0: 'ERROR_NONE',
    1: 'ERROR_REJECTED',
    2: 'ERROR_UNKNOWN_GOAL_ID',
    3: 'ERROR_GOAL_TERMINATED',
}
GRAPH_MERGE_MULTIPLIERS = (1.0, 2.0, 3.0, 5.0, 10.0)
LOST_REPORT_PERIOD_S = 10.0
LOST_CHECK_PERIOD_S = 1.0
# Mirrors tb3_fleet_adapter; kept here so the miss count can be reported
# as a duration without the robot adapter importing the loop module.
POSE_FAILURES_BEFORE_ALARM = 10
DEFAULT_UPDATE_PERIOD_S = 0.1
MAX_TRANSFORM_AGE_S = 5.0


def make_nav2_cancel_all_goals_request():
    return ActionMsgs_CancelGoal_Request(
        goal_info=ActionMsgs_GoalInfo(
            UUID(uuid=[0] * 16),
            Time(sec=0, nanosec=0)
        )
    )


def make_nav2_cancel_goal_request(goal_id):
    """Cancel one goal by id, so a cancel issued for a superseded goal cannot terminate
    the goal that replaced it."""
    return ActionMsgs_CancelGoal_Request(
        goal_info=ActionMsgs_GoalInfo(
            UUID(uuid=list(goal_id)),
            Time(sec=0, nanosec=0)
        )
    )


class Nav2TfHandler:

    def __init__(self, robot_name, zenoh_session, tf_buffer, node,
                 robot_frame='base_footprint', map_frame='map',
                 max_transform_age_s=MAX_TRANSFORM_AGE_S):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.map_frame = map_frame
        self.max_transform_age_s = max_transform_age_s

        self.tf_sub = self.zenoh_session.declare_subscriber(
            namespacify('tf', self.robot_name),
            self._on_tf
        )

    def _on_tf(self, sample: zenoh.Sample):
        try:
            transform = TFMessage.deserialize(sample.payload.to_bytes())
        except Exception as e:
            self.node.get_logger().debug(
                f'Failed to deserialize TF payload: {type(e)}: {e}'
            )
            return
        for zt in transform.transforms:
            t = transform_stamped_to_ros2_msg(zt)
            t.header.frame_id = namespacify(zt.header.frame_id, self.robot_name)
            t.child_frame_id = namespacify(zt.child_frame_id, self.robot_name)

            self.tf_buffer.set_transform(
                t, f'{self.robot_name}_TfListener')

    def transform_age_s(self, transform) -> float | None:
        """Seconds between the transform's stamp and now, on the node's clock."""
        stamp = transform.header.stamp
        stamped_s = stamp.sec + stamp.nanosec * 1e-9
        if stamped_s <= 0.0:
            return None
        now = self.node.get_clock().now()
        return now.nanoseconds / 1e9 - stamped_s

    def get_transform(self) -> TransformStamped | None:

        t0 = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform(
                namespacify(self.map_frame, self.robot_name),
                namespacify(self.robot_frame, self.robot_name),
                rclpy.time.Time()
            )
            dt = time.monotonic() - t0
            if dt > 0.05:
                self.node.get_logger().warn(
                    f'lookup_transform took {dt:.3f}s for [{self.robot_name}].'
                )
            age = self.transform_age_s(transform)
            if age is not None and age > self.max_transform_age_s:
                self.node.get_logger().warn(
                    f'Newest transform for [{self.robot_name}] is {age:.1f}s old '
                    f'(limit {self.max_transform_age_s:.1f}s); treating the pose '
                    f'as unavailable rather than reporting a stale one.'
                )
                return None
            return transform
        except Exception as err:
            self.node.get_logger().info(
                f'Unable to get transform between {self.robot_frame} '
                f'and {self.map_frame}: {type(err)}: {err}'
            )
        return None


class Tb3RobotAdapter(RobotAdapter):

    def __init__(
        self,
        name: str,
        configuration,
        robot_config_yaml,
        node,
        zenoh_session,
        fleet_handle,
        fleet_config: rmf_easy.FleetConfiguration | None,
        tf_buffer
    ):
        RobotAdapter.__init__(self, name, node, fleet_handle)

        self.configuration = configuration
        self.robot_config_yaml = robot_config_yaml
        self.zenoh_session = zenoh_session
        self.fleet_config = fleet_config
        self.tf_buffer = tf_buffer

        self.exec_handle: ExecutionHandle | None = None
        self.map_name = self.robot_config_yaml['initial_map']
        default_map_frame = 'map'
        default_robot_frame = 'base_footprint'
        self.map_frame = self.robot_config_yaml.get('map_frame', default_map_frame)
        self.robot_frame = self.robot_config_yaml.get('robot_frame', default_robot_frame)

        self.battery_soc = 1.0
        self.replan_counts = 0
        self.nav_issue_ticket = None
        # consecutive cycles where get_pose() returned nothing
        self.pose_failures = 0
        self.pose_issue_ticket = None

        self._nav_result_lock = threading.Lock()
        self._nav_result_goal_id = None
        self._nav_result_status = None

        self._pending_replan = False
        self._command_id = 0
        self._latest_goal_id = None
        self._replanned_for_goal_id = None
        # Wall time a deferred replan becomes due; see _handle_failed_goal.
        self._replan_due_at = 0.0
        # set when a newer command supersedes one still being retried
        self._cancel_nav_send = threading.Event()

        self._last_pose = None
        self._lane_segments = None
        self._merge_lane_distance = None
        self._merge_waypoint_distance = None
        self._min_lane_length = None
        self._lost_since = None
        self._lost_reported_at = 0.0
        self._lost_checked_at = 0.0

        self.tf_handler = Nav2TfHandler(
            self.name,
            self.zenoh_session,
            self.tf_buffer,
            self.node,
            robot_frame=self.robot_frame,
            map_frame=self.map_frame,
            max_transform_age_s=self.robot_config_yaml.get(
                'max_transform_age_sec', MAX_TRANSFORM_AGE_S)
        )

        self.battery_state_sub = self.zenoh_session.declare_subscriber(
            namespacify('battery_state', self.name),
            self._on_battery_state
        )
        time.sleep(3)

        # Initialize robot
        init_timeout_sec = self.robot_config_yaml.get('init_timeout_sec', 30)
        self.node.get_logger().info(f'Initializing robot [{self.name}]...')

        init_robot_pose = None
        deadline = time.time() + init_timeout_sec
        while time.time() < deadline:
            init_robot_pose = self.get_pose()
            if init_robot_pose is not None:
                break
            time.sleep(1.0)

        if init_robot_pose is None:
            error_message = f'Timeout trying to initialize robot [{self.name}]'
            self.node.get_logger().error(error_message)
            raise RuntimeError(error_message)

        state = rmf_easy.RobotState(
            self.get_map_name(),
            init_robot_pose,
            self.get_battery_soc()
        )

        if self.fleet_handle is None:
            self.node.get_logger().warn(
                f'Fleet unavailable, skipping adding robot [{self.name}] to fleet.'
            )
            return

        self.update_handle = self.fleet_handle.add_robot(
            self.name,
            state,
            self.configuration,
            rmf_easy.RobotCallbacks(
                lambda destination, execution: self.navigate(destination, execution),
                lambda activity: self.stop(activity),
                lambda category, description, execution: self.execute_action(category, description, execution)
            )
        )
        if not self.update_handle:
            error_message = f'Failed to add robot [{self.name}] to fleet.'
            self.node.get_logger().error(error_message)
            raise RuntimeError(error_message)
        self._want_debug_positions = self.robot_config_yaml.get(
            'debug_positions', False)

    def _log(self, level: str, msg: str):
        """Log to the ROS logger and into RMF's robot log."""
        logger = self.node.get_logger()
        if level == 'error':
            logger.error(msg)
        elif level == 'warning':
            logger.warn(msg)
        else:
            logger.info(msg)
        handle = self.update_handle
        if handle is None:
            return
        try:
            getattr(handle.more(), f'log_{level}')(msg)
        except Exception as e:
            self.node.get_logger().debug(f'could not forward log to RMF: {e}')

    def report_pose_unavailable(self, consecutive: int):
        """Tell RMF the robot's position is no longer being refreshed."""
        seconds = consecutive * self._update_period_estimate()
        self._log(
            'error',
            f'No pose for [{self.name}] for {consecutive} consecutive updates '
            f'(~{seconds:.0f}s). RMF is still holding the last position it was '
            f'given, which is no longer being confirmed.'
        )
        if self.pose_issue_ticket is None:
            self.pose_issue_ticket = self.create_nav_issue_ticket(
                'localization',
                f'No pose available for {consecutive} consecutive updates'
            )

    def report_pose_recovered(self, consecutive: int):
        seconds = consecutive * self._update_period_estimate()
        if consecutive >= POSE_FAILURES_BEFORE_ALARM:
            self._log(
                'info',
                f'Pose for [{self.name}] is being reported again after '
                f'{consecutive} missed updates (~{seconds:.0f}s).'
            )
        if self.pose_issue_ticket is not None:
            try:
                self.pose_issue_ticket.resolve({})
            except Exception as e:
                self.node.get_logger().debug(f'could not resolve pose issue: {e}')
            self.pose_issue_ticket = None

    def _update_period_estimate(self) -> float:
        """Seconds per update cycle, for turning a miss count into a duration."""
        frequency = self.robot_config_yaml.get('robot_state_update_frequency')
        if isinstance(frequency, (int, float)) and frequency > 0:
            return 1.0 / frequency
        return DEFAULT_UPDATE_PERIOD_S

    def _on_battery_state(self, sample: zenoh.Sample):
        battery_state = SensorMsgs_BatteryState.deserialize(
            sample.payload.to_bytes()
        )
        # The board reports the charge on either the 0..1 or the 0..100 scale; RMF takes
        # a fraction, and rejects the whole robot state update when it falls outside
        # [0, 1]. A freshly charged pack reads slightly above full, so the fraction is
        # clamped: without it the robot reports no state at all and receives no task.
        soc = battery_state.percentage
        soc = soc / 100.0 if soc > 1.0 else soc
        self.battery_soc = min(max(soc, 0.0), 1.0)

    def get_battery_soc(self) -> float:
        return self.battery_soc

    def get_map_name(self) -> str:
        return self.map_name

    def get_pose(self) -> Annotated[list[float], 3] | None:
        transform = self.tf_handler.get_transform()
        if transform is None:
            self.node.get_logger().info(
                f'Unable to get transform for robot [{self.name}].'
            )
            return None

        orientation = euler_from_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        robot_pose = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            orientation[2]
        ]
        self._last_pose = robot_pose
        return robot_pose

    def _has_plan_starts(self, pose) -> bool:
        graph = self.fleet_config.graph
        position = np.array([pose[0], pose[1], pose[2]])
        now = datetime.datetime.now()
        for multiplier in GRAPH_MERGE_MULTIPLIERS:
            starts = plan.compute_plan_starts(
                graph,
                self.map_name,
                position,
                now,
                max_merge_waypoint_distance=self._merge_waypoint_distance,
                max_merge_lane_distance=multiplier * self._merge_lane_distance,
                min_lane_length=self._min_lane_length,
            )
            if len(starts) > 0:
                return True
        return False

    def _make_lane_segments(self):

        if self.fleet_config is None:
            return []
        graph = self.fleet_config.graph
        segments = []
        for i in range(graph.num_lanes):
            lane = graph.get_lane(i)
            w0 = graph.get_waypoint(lane.entry.waypoint_index)
            w1 = graph.get_waypoint(lane.exit.waypoint_index)
            if w0.map_name != self.map_name or w1.map_name != self.map_name:
                continue
            segments.append((float(w0.location[0]), float(w0.location[1]),
                             float(w1.location[0]), float(w1.location[1])))
        return segments

    def _distance_to_nearest_lane(self, x: float, y: float) -> float:
        best = float('inf')
        for x0, y0, x1, y1 in self._lane_segments:
            dx, dy = x1 - x0, y1 - y0
            length_sq = dx * dx + dy * dy
            if length_sq < 1e-12:
                d = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
            else:
                t = ((x - x0) * dx + (y - y0) * dy) / length_sq
                t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                d = ((x - x0 - t * dx) ** 2 + (y - y0 - t * dy) ** 2) ** 0.5
            if d < best:
                best = d
        return best

    def _update_lost_state(self):
        if self.update_handle is None or self.fleet_config is None:
            return
        if self._lane_segments is None:
            self._lane_segments = self._make_lane_segments()
            self._merge_lane_distance = self._get_nav_param(
                'max_merge_lane_distance', 0.3)
            self._merge_waypoint_distance = self._get_nav_param(
                'max_merge_waypoint_distance', 1e-3)
            self._min_lane_length = self._get_nav_param('min_lane_length', 1e-8)
            if self._want_debug_positions:
                try:
                    self.update_handle.more().unstable_debug_positions(True)
                    self._log(
                        'warning',
                        f'[{self.name}] unstable_debug_positions is ON; expect '
                        f'heavy stdout from RMF on every position update.'
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f'could not enable debug_positions for [{self.name}]: {e}'
                    )
            self._log(
                'info',
                f'[{self.name}] lost-state check active: merge budget '
                f'{self._merge_lane_distance:.3f}m x up to {max(GRAPH_MERGE_MULTIPLIERS):.0f} '
                f'= {self._merge_lane_distance * max(GRAPH_MERGE_MULTIPLIERS):.2f}m, '
                f'{len(self._lane_segments)} lanes on map [{self.map_name}].'
            )

        pose = self._last_pose
        if pose is None:
            return

        now = time.monotonic()
        if now - self._lost_checked_at < LOST_CHECK_PERIOD_S:
            return
        self._lost_checked_at = now

        if not self._has_plan_starts(pose):
            if self._lost_since is None:
                self._lost_since = now
                self._lost_reported_at = 0.0
            if now - self._lost_reported_at >= LOST_REPORT_PERIOD_S:
                self._lost_reported_at = now
                elapsed = now - self._lost_since
                distance = self._distance_to_nearest_lane(pose[0], pose[1])
                self._log(
                    'error',
                    f'RMF cannot place [{self.name}] on the graph: the reported pose '
                    f'is {distance:.2f}m from the nearest lane on map [{self.map_name}], '
                    f'past the {self._merge_lane_distance * max(GRAPH_MERGE_MULTIPLIERS):.2f}m '
                    f'merge budget, so it has no location for this robot and every '
                    f'plan attempt fails (lost for {elapsed:.0f}s).'
                )
        elif self._lost_since is not None:
            elapsed = now - self._lost_since
            self._lost_since = None
            self._log(
                'info',
                f'RMF can place [{self.name}] on the graph again after {elapsed:.0f}s '
                f'off it.'
            )

    def _get_nav_param(self, name: str, default: float) -> float:
        try:
            return getattr(self.update_handle, name)()
        except Exception as e:
            self.node.get_logger().debug(
                f'could not read {name}, using RMF default {default}: {e}'
            )
            return default

    def _await_nav_result(self, nav_goal_id):
        req = NavigateToPose_GetResult_Request(goal_id=nav_goal_id)

        deadline = time.time() + NAV_RESULT_TIMEOUT_S
        while time.time() < deadline:
            if self._latest_goal_id != nav_goal_id:
                return
            with self._nav_result_lock:
                if self._nav_result_goal_id not in (None, nav_goal_id) or (
                    self._nav_result_goal_id == nav_goal_id and self._nav_result_status is not None
                ):
                    return
            try:
                replies = self.zenoh_session.get(
                    namespacify('navigate_to_pose/_action/get_result', self.name),
                    payload=req.serialize(),
                    timeout=10.0,
                )
                got_valid = False
                for reply in replies:
                    if reply.ok is None:
                        continue
                    try:
                        rep = NavigateToPose_GetResult_Response.deserialize(
                            reply.ok.payload.to_bytes()
                        )
                    except Exception:
                        continue
                    with self._nav_result_lock:
                        self._nav_result_goal_id = nav_goal_id
                        self._nav_result_status = rep.status
                    got_valid = True
                    break
                if got_valid:
                    return
            except Exception as e:
                self.node.get_logger().info(f'Error while awaiting nav result, retrying: {e}')
            time.sleep(0.5)
        self._log(
            'error',
            f'Gave up waiting for the Nav2 result of goal {nav_goal_id} on '
            f'[{self.name}] after {NAV_RESULT_TIMEOUT_S:.0f}s.'
        )

    def _is_navigation_done(self, nav_handle: ExecutionHandle) -> bool:
        if nav_handle.goal_id is None:
            return True

        with self._nav_result_lock:
            if self._nav_result_goal_id != nav_handle.goal_id:
                return False
            status = self._nav_result_status

        if status is None:
            return False

        match status:
            case GoalStatus.STATUS_EXECUTING.value | \
                 GoalStatus.STATUS_ACCEPTED.value | \
                 GoalStatus.STATUS_CANCELING.value:
                return False
            case GoalStatus.STATUS_SUCCEEDED.value:
                self.node.get_logger().info(f'Navigation goal {nav_handle.goal_id} reached')
                if self.nav_issue_ticket is not None:
                    msg = {}
                    self.nav_issue_ticket.resolve(msg)
                    self.nav_issue_ticket = None
                return True
            case GoalStatus.STATUS_CANCELED.value:
                self.node.get_logger().warn(
                    f'Navigation goal {nav_handle.goal_id} was cancelled unexpectedly '
                    f'-- requesting replan instead of reporting it as reached'
                )
                if self._replanned_for_goal_id != nav_handle.goal_id:
                    self._replanned_for_goal_id = nav_handle.goal_id
                    self.nav_issue_ticket = self.create_nav_issue_ticket(
                        'navigation',
                        'Navigate to pose was cancelled unexpectedly',
                        nav_handle.goal_id
                    )
                    self.replan_counts += 1
                    self.update_handle.more().replan()
                return False
            case GoalStatus.STATUS_ABORTED.value:
                self._handle_failed_goal(nav_handle, status, 'Nav2 aborted the goal')
                return False
            case GoalStatus.STATUS_UNKNOWN.value:
                self._handle_failed_goal(
                    nav_handle, status, 'Nav2 reports the goal as unknown')
                return False
            case _:
                self._handle_failed_goal(
                    nav_handle, status, f'unhandled Nav2 goal status [{status}]')
                return False

    def _handle_failed_goal(self, nav_handle: ExecutionHandle, status, reason):
        """Replan after a goal that ended without reaching its destination, with a delay
        that grows with the consecutive failure count.

        EasyFullControl has no way to report a command as failed: CommandExecution offers
        finished, okay and override_schedule, and finished would tell RMF the robot
        arrived. Upstream free_fleet therefore replans without a limit, and so does this.
        NAV_MAX_CONSECUTIVE_REPLANS only marks where the failure stops being transient:
        past it the backoff stays at its longest and the log is raised to error, so a
        robot that cannot move is visible in the record without the adapter inventing an
        outcome RMF never reported.
        """
        if self._replanned_for_goal_id == nav_handle.goal_id:
            return
        self._replanned_for_goal_id = nav_handle.goal_id
        self.replan_counts += 1

        if self.replan_counts == NAV_MAX_CONSECUTIVE_REPLANS + 1:
            self._log(
                'error',
                f'[{self.name}] {reason}; {self.replan_counts} consecutive failures '
                f'without reaching a goal. Still replanning, now at the longest backoff.'
            )

        backoff = NAV_REPLAN_BACKOFF_S[
            min(self.replan_counts - 1, len(NAV_REPLAN_BACKOFF_S) - 1)]
        self._log(
            'warning',
            f'Nav2 ended goal {nav_handle.goal_id} for [{self.name}] with status '
            f'[{status}] instead of succeeding ({reason}); asking RMF to replan after '
            f'{backoff:.0f}s (replan {self.replan_counts} since this robot last reached '
            f'a goal).'
        )
        self.nav_issue_ticket = self.create_nav_issue_ticket(
            'navigation', f'Navigate to pose result status [{status}]: {reason}',
            nav_handle.goal_id)
        # Deferred rather than slept on: this runs on the update loop, which serves
        # every robot, so blocking here would stall the whole fleet's telemetry.
        self._pending_replan = True
        self._replan_due_at = time.time() + backoff

    def create_nav_issue_ticket(self, category, msg, nav_goal_id=None):
        if self.update_handle is None:
            return None
        tier = Tier.Error
        detail = {'nav_goal_id': f'{nav_goal_id}', 'message': msg}
        nav_issue_ticket = self.update_handle.more().create_issue(tier, category, detail)
        return nav_issue_ticket

    def update(self, state: rmf_easy.RobotState):
        if self.update_handle is None:
            return

        if self._pending_replan and time.time() >= self._replan_due_at:
            self._pending_replan = False
            self.update_handle.more().replan()

        self._update_lost_state()

        activity_identifier = None
        exec_handle = self.exec_handle
        if exec_handle:
            if exec_handle.execution and exec_handle.goal_id and self._is_navigation_done(exec_handle):
                exec_handle.execution.finished()
                exec_handle.execution = None
                exec_handle.goal_id = None
                self.replan_counts = 0
            activity_identifier = exec_handle.activity

        self.update_handle.update(state, activity_identifier)

    def _handle_navigate_to_pose(
        self,
        map_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        nav_handle: ExecutionHandle
    ):
        if map_name != self.map_name:
            self.replan_counts += 1
            self._log(
                'warning',
                f'[{self.name}] was asked to navigate on map [{map_name}] but is on '
                f'[{self.map_name}]; this fleet has no way to change map, so the '
                f'command cannot be carried out (asking RMF to replan).'
            )
            nav_handle.set_goal_id(None)
            self._pending_replan = True
            return

        time_now = self.node.get_clock().now().seconds_nanoseconds()
        stamp = Time(sec=time_now[0], nanosec=time_now[1])
        header = Header(stamp=stamp, frame_id=self.map_frame)
        position = GeometryMsgs_Point(x=x, y=y, z=z)
        quat = quaternion_from_euler(0, 0, yaw)
        orientation = GeometryMsgs_Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        pose = GeometryMsgs_Pose(position=position, orientation=orientation)
        pose_stamped = GeometryMsgs_PoseStamped(header=header, pose=pose)

        nav_goal_id = np.random.randint(0, 255, size=(16)).astype('uint8').tolist()
        self._command_id += 1
        command_id = self._command_id
        self._latest_goal_id = nav_goal_id
        req = NavigateToPose_SendGoal_Request(
            goal_id=nav_goal_id,
            pose=pose_stamped,
            behavior_tree=''
        )

        def _send_once():

            t0_send_goal = time.monotonic()
            replies = self.zenoh_session.get(
                namespacify('navigate_to_pose/_action/send_goal', self.name),
                payload=req.serialize(),
                timeout=NAV_QUERY_TIMEOUT_S,
            )
            for reply in replies:
                try:
                    rep = NavigateToPose_SendGoal_Response.deserialize(reply.ok.payload.to_bytes())
                except Exception as e:
                    self.node.get_logger().warn(
                        f'send_goal reply for [{self.name}] did not decode, '
                        f'{len(reply.ok.payload.to_bytes())} bytes: {type(e).__name__}: {e}'
                    )
                    continue

                if rep.accepted:
                    dt_send_goal = time.monotonic() - t0_send_goal
                    self.node.get_logger().info(
                        f'send_goal round-trip for [{self.name}]: {dt_send_goal:.3f}s.'
                    )
                    self.node.get_logger().info(
                        f'[{self.name}] command {command_id} accepted by Nav2 '
                        f'(goal {nav_goal_id})'
                    )
                    with self._nav_result_lock:
                        self._nav_result_goal_id = None
                        self._nav_result_status = None
                    nav_handle.set_goal_id(nav_goal_id)
                    threading.Thread(
                        target=self._await_nav_result,
                        args=(nav_goal_id,),
                        daemon=True,
                    ).start()
                    return 'SUCCESS'
                return 'REJECTED'
            return 'NO_REPLY'

        def _dispatch():
            released = False
            try:
                for attempt in range(1, NAV_SEND_MAX_ATTEMPTS + 1):
                    if nav_handle.execution is None:
                        break       # superseded or stopped while we were retrying
                    outcome = _send_once()
                    if outcome == 'SUCCESS':
                        released = True
                        return
                    if outcome == 'REJECTED':
                        self._log(
                            'warning',
                            f'Nav2 rejected command {command_id} for [{self.name}]; '
                            f'asking RMF to replan.'
                        )
                        break
                    if attempt == NAV_SEND_MAX_ATTEMPTS:
                        self._log(
                            'error',
                            f'No reply from Nav2 for command {command_id} on '
                            f'[{self.name}] after {NAV_SEND_MAX_ATTEMPTS} attempts; '
                            f'the navigation command never reached the robot.'
                        )
                    else:
                        self.node.get_logger().warn(
                            f'send_goal for command {command_id} on [{self.name}] '
                            f'got no valid reply '
                            f'(attempt {attempt}/{NAV_SEND_MAX_ATTEMPTS}).'
                        )
                    if self._cancel_nav_send.wait(NAV_SEND_RETRY_PERIOD_S):
                        break

                self.replan_counts += 1
                if self.update_handle:
                    self._pending_replan = True
            finally:
                if not released:
                    nav_handle.set_goal_id(None)

        self._cancel_nav_send.clear()
        threading.Thread(target=_dispatch, daemon=True).start()

    def _log_command(self, destination: rmf_easy.Destination):
        """Record one line per navigation command, at the moment RMF issues it.

        A command is classified by the destination's graph index rather than by its
        distance: RMF reasons about lanes, so a command that names the waypoint the robot
        already occupies leaves it where it is, while any other waypoint requires motion,
        whether it comes from a task or from yielding during a responsive wait. The task
        id separates the two sources: responsive wait carries a wait.<robot>.<fleet>-<n>
        id, a dispatched task carries its own.

        The line is the input to deciding which commands may be answered without driving;
        the adapter does not act on it.
        """
        pose = self._last_pose
        task_id = None
        if self.update_handle is not None:
            try:
                task_id = self.update_handle.more().current_task_id()
            except Exception as e:
                task_id = f'unavailable ({e})'
        dxy = dyaw = None
        if pose is not None:
            dxy = math.hypot(destination.position[0] - pose[0],
                             destination.position[1] - pose[1])
            dyaw = (destination.position[2] - pose[2] + math.pi) % (2 * math.pi) - math.pi
        previous = self.exec_handle
        self._log(
            'info',
            'command [%s] task=[%s] destination=[%s] graph_index=%s map=[%s] '
            'pose=%s d_xy=%s d_yaw=%s previous_goal=%s'
            % (self.name, task_id, destination.name or '', destination.graph_index,
               destination.map,
               'none' if pose is None else '(%.3f, %.3f, %.3f)' % tuple(pose[:3]),
               'n/a' if dxy is None else '%.3f' % dxy,
               'n/a' if dyaw is None else '%.3f' % dyaw,
               'none' if previous is None else previous.goal_id)
        )

    def navigate(self, destination: rmf_easy.Destination, execution: rmf_easy.CommandExecution):
        self._cancel_nav_send.set()
        self._request_stop(self.exec_handle)
        dock = destination.dock
        if dock is not None:
            self.node.get_logger().info(
                f'[{self.name}] was asked to dock at [{dock}]; this fleet has no '
                f'docking procedure, navigating to the dock position instead.'
            )

        self._log_command(destination)
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} '
            f'on map [{destination.map}]'
        )
        self.exec_handle = ExecutionHandle(execution)
        self._handle_navigate_to_pose(
            destination.map,
            destination.position[0],
            destination.position[1],
            0.0,
            destination.position[2],
            self.exec_handle
        )

    def _request_stop(self, exec_handle: ExecutionHandle):
        if exec_handle is not None:
            with exec_handle.mutex:
                if exec_handle.goal_id is not None:
                    self._handle_stop_navigation(exec_handle.goal_id)

    def _handle_stop_navigation(self, goal_id=None):
        """Cancel the named goal, or every goal when none is named, and return the
        action server's return code as a string.

        Returns 'NO_REPLY' when the robot did not answer within the query timeout, and
        'UNDECODABLE' when a reply arrived but could not be read. A cancel that is not
        confirmed is not a cancel: the caller must not assume the goal has stopped.
        """
        if goal_id is None:
            req = make_nav2_cancel_all_goals_request()
            target = 'all goals'
        else:
            req = make_nav2_cancel_goal_request(goal_id)
            target = f'goal {list(goal_id)}'

        replies = self.zenoh_session.get(
            namespacify('navigate_to_pose/_action/cancel_goal', self.name),
            payload=req.serialize(),
            timeout=NAV_QUERY_TIMEOUT_S,
        )
        for reply in replies:
            try:
                rep = ActionMsgs_CancelGoal_Response.deserialize(
                    reply.ok.payload.to_bytes())
            except Exception as e:
                self._log(
                    'warning',
                    f'Cancel reply for [{self.name}] ({target}) could not be read: {e}'
                )
                return 'UNDECODABLE'
            code = CANCEL_RETURN_CODES.get(rep.return_code, str(rep.return_code))
            canceling = len(rep.goals_canceling)
            level = 'info' if rep.return_code == 0 else 'warning'
            self._log(
                level,
                f'Cancel of {target} on [{self.name}] returned [{code}], '
                f'{canceling} goal(s) canceling.'
            )
            return code

        self._log(
            'warning',
            f'No reply to the cancel of {target} on [{self.name}] within '
            f'{NAV_QUERY_TIMEOUT_S}s; the goal may still be running.'
        )
        return 'NO_REPLY'

    def stop(self, activity: ActivityIdentifier):
        self._cancel_nav_send.set()
        exec_handle = self.exec_handle
        if exec_handle is None:
            return

        if exec_handle.execution is not None and activity.is_same(exec_handle.activity):
            self._request_stop(exec_handle)
            self.exec_handle = None

    def execute_action(self, category, description, execution):
        msg = (f'Action [{category}] is not supported by robot [{self.name}]; '
               f'this fleet adapter implements navigation only.')
        self._log('error', msg)
        execution.error(msg)
        execution.finished()
