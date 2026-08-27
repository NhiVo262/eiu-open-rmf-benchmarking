#!/usr/bin/env python3

from typing import Annotated
import threading
import time

from free_fleet.convert import transform_stamped_to_ros2_msg
from free_fleet.ros2_types import (
    ActionMsgs_CancelGoal_Request,
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
from rmf_adapter.robot_update_handle import ActivityIdentifier, Tier
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import zenoh


def namespacify(base_name: str, namespace: str, delimiter: str = '/') -> str:
    return base_name if not namespace else f'{namespace}{delimiter}{base_name}'


def make_nav2_cancel_all_goals_request():
    return ActionMsgs_CancelGoal_Request(
        goal_info=ActionMsgs_GoalInfo(
            UUID(uuid=[0] * 16),
            Time(sec=0, nanosec=0)
        )
    )


class Nav2TfHandler:

    def __init__(self, robot_name, zenoh_session, tf_buffer, node,
                 robot_frame='base_footprint', map_frame='map'):
        self.robot_name = robot_name
        self.zenoh_session = zenoh_session
        self.node = node
        self.tf_buffer = tf_buffer
        self.robot_frame = robot_frame
        self.map_frame = map_frame

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

    def get_transform(self) -> TransformStamped | None:
        # lookup_transform() is a LOCAL lookup on tf_buffer (filled
        # asynchronously by the Zenoh subscriber in _on_tf above), not a
        # network call. If this is slow (>50ms), tf_buffer/tf2 itself has a
        # problem (e.g. the buffer is too large, or the thread is fighting
        # for the GIL) -- it's not a network delay.
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

        self._nav_result_lock = threading.Lock()
        self._nav_result_goal_id = None
        self._nav_result_status = None

        self._pending_replan = False
        self._latest_goal_id = None
        self._replanned_for_goal_id = None

        self.tf_handler = Nav2TfHandler(
            self.name,
            self.zenoh_session,
            self.tf_buffer,
            self.node,
            robot_frame=self.robot_frame,
            map_frame=self.map_frame
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

    def _on_battery_state(self, sample: zenoh.Sample):
        battery_state = SensorMsgs_BatteryState.deserialize(
            sample.payload.to_bytes()
        )
        self.battery_soc = battery_state.percentage

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
        return robot_pose

    def _await_nav_result(self, nav_goal_id):
        req = NavigateToPose_GetResult_Request(goal_id=nav_goal_id)
        deadline = time.time() + 300.0
        while time.time() < deadline:
            if self._latest_goal_id != nav_goal_id:
                # A newer goal has already been dispatched (e.g. from a
                # replan) -- exit immediately, don't send any more Zenoh
                # queries for this now-stale goal.
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
        self.node.get_logger().warn(f'Timed out awaiting nav result for goal {nav_goal_id}')

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
                self.node.get_logger().info(f'Navigation goal {nav_handle.goal_id} was cancelled')
                return True
            case _:
                # request_replan() is the one RobotUpdateHandle mutator that
                # does not hop onto RMF's own worker thread internally
                # (unlike update_position/update_battery_soc/etc, verified in
                # rmf_ros2 source) -- it is called here directly from our own
                # update_thread instead. _nav_result_status never changes on
                # its own once _await_nav_result has exited, so without this
                # per-goal_id guard this branch would call it every update
                # cycle (e.g. 10Hz) indefinitely for the same stuck goal.
                # Only issuing it once per goal_id keeps that exposure to a
                # single call instead of an unbounded retry storm.
                if self._replanned_for_goal_id != nav_handle.goal_id:
                    self._replanned_for_goal_id = nav_handle.goal_id
                    self.nav_issue_ticket = self.create_nav_issue_ticket(
                        'navigation',
                        f'Navigate to pose result status [{status}]',
                        nav_handle.goal_id
                    )
                    self.replan_counts += 1
                    self.update_handle.more().replan()
                return False

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

        if self._pending_replan:
            self._pending_replan = False
            self.update_handle.more().replan()

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
            if self.update_handle:
                self.update_handle.more().replan()
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
        self._latest_goal_id = nav_goal_id
        req = NavigateToPose_SendGoal_Request(
            goal_id=nav_goal_id,
            pose=pose_stamped,
            behavior_tree=''
        )

        def _dispatch():
            t0_send_goal = time.monotonic()
            replies = self.zenoh_session.get(
                namespacify('navigate_to_pose/_action/send_goal', self.name),
                payload=req.serialize(),
                timeout=2.0,
            )
            for reply in replies:
                try:
                    rep = NavigateToPose_SendGoal_Response.deserialize(reply.ok.payload.to_bytes())
                    if rep.accepted:
                        dt_send_goal = time.monotonic() - t0_send_goal
                        self.node.get_logger().info(
                            f'send_goal round-trip for [{self.name}]: {dt_send_goal:.3f}s.'
                        )
                        self.node.get_logger().info(f'Navigation goal {nav_goal_id} accepted')
                        with self._nav_result_lock:
                            self._nav_result_goal_id = None
                            self._nav_result_status = None
                        nav_handle.set_goal_id(nav_goal_id)
                        threading.Thread(
                            target=self._await_nav_result,
                            args=(nav_goal_id,),
                            daemon=True,
                        ).start()
                        return

                    self.node.get_logger().warn(f'send_goal for [{self.name}] was rejected.')
                    self.replan_counts += 1
                    if self.update_handle:
                        self._pending_replan = True
                    nav_handle.set_goal_id(None)
                    return
                except Exception as e:
                    self.node.get_logger().debug(
                        f'Failed to handle send_goal reply for [{self.name}]: {e}'
                    )
                    continue

            # No reply accepted the goal (Zenoh timeout with zero replies, or
            # every reply above failed to parse). nav_handle.mutex starts
            # pre-locked and is otherwise only released on the accepted-goal
            # path above -- release it here too, or the next _request_stop()
            # on this handle would block forever.
            self.node.get_logger().warn(f'send_goal for [{self.name}] got no valid reply.')
            self.replan_counts += 1
            if self.update_handle:
                self._pending_replan = True
            nav_handle.set_goal_id(None)

        threading.Thread(target=_dispatch, daemon=True).start()

    def navigate(self, destination: rmf_easy.Destination, execution: rmf_easy.CommandExecution):
        self._request_stop(self.exec_handle)
        self.node.get_logger().info(
            f'Commanding [{self.name}] to navigate to {destination.position} on map [{destination.map}]'
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
                    self._handle_stop_navigation()

    def _handle_stop_navigation(self):
        req = make_nav2_cancel_all_goals_request()
        self.zenoh_session.get(
            namespacify('navigate_to_pose/_action/cancel_goal', self.name),
            payload=req.serialize(),
            timeout=2.0,
        )

    def stop(self, activity: ActivityIdentifier):
        exec_handle = self.exec_handle
        if exec_handle is None:
            return

        if exec_handle.execution is not None and activity.is_same(exec_handle.activity):
            self._request_stop(exec_handle)
            self.exec_handle = None

    def execute_action(self, category, description, execution):
        self.node.get_logger().warn(
            f'Action [{category}] is not supported for robot [{self.name}].'
        )
        execution.finished()
