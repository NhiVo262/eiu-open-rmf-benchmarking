"""Behaviour of the adapter's Nav2 goal-failure and cancel branches.

Runs inside the open-rmf container, where rmf_adapter, free_fleet and zenoh are
importable. The methods under test are exercised on an instance built without the
constructor, so no zenoh session, ROS node or robot is required.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from free_fleet.ros2_types import (  # noqa: E402
    ActionMsgs_CancelGoal_Response,
    ActionMsgs_GoalInfo,
    GoalStatus,
    Time,
    UUID,
)

from tb3_fleet import tb3_robot_adapter as tra  # noqa: E402


class FakePayload:
    def __init__(self, payload):
        self._payload = payload

    def to_bytes(self):
        return self._payload


class FakeOk:
    def __init__(self, payload):
        self.payload = FakePayload(payload)


class FakeReply:
    """One zenoh query reply, shaped as reply.ok.payload.to_bytes()."""

    def __init__(self, payload):
        self.ok = FakeOk(payload)


class FakeSession:
    """Returns a fixed list of replies for every get(), and records the payload sent."""

    def __init__(self, replies):
        self._replies = replies
        self.requests = []

    def get(self, key, payload=None, timeout=None):
        self.requests.append((key, payload))
        return list(self._replies)


class FakeExecution:
    def __init__(self):
        self.errors = []
        self.finished_count = 0

    def error(self, msg):
        self.errors.append(msg)

    def finished(self):
        self.finished_count += 1


def cancel_response(return_code, n_canceling=0):
    goals = [ActionMsgs_GoalInfo(UUID(uuid=[0] * 16), Time(sec=0, nanosec=0))
             for _ in range(n_canceling)]
    return ActionMsgs_CancelGoal_Response(
        return_code=return_code, goals_canceling=goals).serialize()


def make_adapter(session):
    """An adapter instance with only the attributes these branches read."""
    robot = tra.Tb3RobotAdapter.__new__(tra.Tb3RobotAdapter)
    robot.name = 'tb3_robot_test'
    robot.zenoh_session = session
    robot.logged = []
    robot._log = lambda level, msg: robot.logged.append((level, msg))
    robot.replan_counts = 0
    robot._replanned_for_goal_id = None
    robot._pending_replan = False
    robot._replan_due_at = 0.0
    robot.nav_issue_ticket = None
    robot.create_nav_issue_ticket = lambda category, msg, nav_goal_id=None: None
    return robot


class StopNavigationTest(unittest.TestCase):
    """_handle_stop_navigation reports what the action server returned."""

    def test_accepted(self):
        robot = make_adapter(FakeSession([FakeReply(cancel_response(0, 1))]))
        self.assertEqual(robot._handle_stop_navigation([1] * 16), 'ERROR_NONE')

    def test_rejected(self):
        robot = make_adapter(FakeSession([FakeReply(cancel_response(1))]))
        self.assertEqual(robot._handle_stop_navigation([1] * 16), 'ERROR_REJECTED')

    def test_unknown_goal(self):
        robot = make_adapter(FakeSession([FakeReply(cancel_response(2))]))
        self.assertEqual(robot._handle_stop_navigation([1] * 16), 'ERROR_UNKNOWN_GOAL_ID')

    def test_no_reply(self):
        """A cancel that was not answered is not reported as a successful cancel."""
        robot = make_adapter(FakeSession([]))
        self.assertEqual(robot._handle_stop_navigation([1] * 16), 'NO_REPLY')

    def test_undecodable_reply(self):
        robot = make_adapter(FakeSession([FakeReply(b'not a cancel response')]))
        self.assertEqual(robot._handle_stop_navigation([1] * 16), 'UNDECODABLE')

    def test_named_goal_is_sent(self):
        """A cancel for one goal must not be sent as a cancel-all."""
        session = FakeSession([FakeReply(cancel_response(0))])
        robot = make_adapter(session)
        robot._handle_stop_navigation([7] * 16)
        sent = tra.ActionMsgs_CancelGoal_Request.deserialize(session.requests[0][1])
        self.assertEqual(list(sent.goal_info.goal_id.uuid), [7] * 16)

    def test_cancel_all_when_no_goal_named(self):
        session = FakeSession([FakeReply(cancel_response(0))])
        robot = make_adapter(session)
        robot._handle_stop_navigation()
        sent = tra.ActionMsgs_CancelGoal_Request.deserialize(session.requests[0][1])
        self.assertEqual(list(sent.goal_info.goal_id.uuid), [0] * 16)


class FailedGoalTest(unittest.TestCase):
    """_handle_failed_goal bounds the replan loop and defers each replan."""

    def setUp(self):
        self.session = FakeSession([FakeReply(cancel_response(0))])
        self.robot = make_adapter(self.session)
        self.now = 1000.0
        tra.time.time = lambda: self.now

    def _handle(self, goal_id):
        handle = tra.ExecutionHandle.__new__(tra.ExecutionHandle)
        handle.goal_id = goal_id
        handle.execution = FakeExecution()
        return handle

    def test_backoff_grows_with_consecutive_failures(self):
        """Delays follow NAV_REPLAN_BACKOFF_S: 0 s, then 5 s, then 15 s."""
        for n, expected in enumerate(tra.NAV_REPLAN_BACKOFF_S, start=1):
            handle = self._handle([n] * 16)
            self.robot._handle_failed_goal(
                handle, GoalStatus.STATUS_ABORTED.value, 'aborted')
            self.assertTrue(self.robot._pending_replan)
            self.assertAlmostEqual(
                self.robot._replan_due_at, self.now + expected, places=6)
            self.assertEqual(handle.execution.finished_count, 0)
            self.robot._pending_replan = False

    def test_replan_is_deferred_not_slept(self):
        """The call returns without advancing time; update() fires the replan later."""
        handle = self._handle([1] * 16)
        before = self.now
        self.robot._handle_failed_goal(
            handle, GoalStatus.STATUS_ABORTED.value, 'aborted')
        self.assertEqual(self.now, before)
        self.assertTrue(self.robot._pending_replan)

    def test_command_fails_once_the_limit_is_exceeded(self):
        """The fourth consecutive failure reports the command failed instead of replanning."""
        for n in range(1, tra.NAV_MAX_CONSECUTIVE_REPLANS + 1):
            self.robot._handle_failed_goal(
                self._handle([n] * 16), GoalStatus.STATUS_ABORTED.value, 'aborted')
            self.robot._pending_replan = False
        handle = self._handle([99] * 16)
        execution = handle.execution
        self.robot._handle_failed_goal(
            handle, GoalStatus.STATUS_ABORTED.value, 'aborted')
        self.assertFalse(self.robot._pending_replan)
        self.assertEqual(execution.finished_count, 1)
        self.assertEqual(len(execution.errors), 1)
        # Cleared so a later update cannot finish the same execution twice.
        self.assertIsNone(handle.execution)

    def test_terminal_failure_cancels_the_goal(self):
        """The outstanding Nav2 goal is cancelled before the command is failed."""
        for n in range(1, tra.NAV_MAX_CONSECUTIVE_REPLANS + 1):
            self.robot._handle_failed_goal(
                self._handle([n] * 16), GoalStatus.STATUS_ABORTED.value, 'aborted')
            self.robot._pending_replan = False
        self.session.requests.clear()
        self.robot._handle_failed_goal(
            self._handle([99] * 16), GoalStatus.STATUS_ABORTED.value, 'aborted')
        self.assertEqual(len(self.session.requests), 1)

    def test_same_goal_is_counted_once(self):
        """Repeated reports of one failed goal must not inflate the failure count."""
        handle = self._handle([5] * 16)
        for _ in range(3):
            self.robot._handle_failed_goal(
                handle, GoalStatus.STATUS_ABORTED.value, 'aborted')
        self.assertEqual(self.robot.replan_counts, 1)


if __name__ == '__main__':
    unittest.main()
