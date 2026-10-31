#!/usr/bin/env python3

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml


DEFAULT_TOUR = ['wp_1', 'wp_2', 'wp_3', 'wp_5', 'wp_7', 'wp_9', 'wp_7',
                'wp_6', 'wp_8', 'wp_6', 'wp_4', 'wp_2', 'wp_1']


def load_waypoints(nav_graph_path):
    with open(nav_graph_path) as f:
        data = yaml.safe_load(f)
    out = {}
    for level in data.get('levels', {}).values():
        for v in level.get('vertices', []):
            params = v[2] if len(v) > 2 else {}
            if params.get('name'):
                out[params['name']] = (v[0], v[1])
        break
    return out


class Tour(Node):

    def __init__(self, frame):
        super().__init__('tour_waypoints')
        self.frame = frame
        self.pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     2.0 * math.atan2(p.orientation.z, p.orientation.w),
                     math.sqrt(max(msg.pose.covariance[0], 0.0)),
                     math.sqrt(max(msg.pose.covariance[7], 0.0)))

    def go(self, x, y, timeout_s):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if result.done():
                return result.result().status
        handle.cancel_goal_async()
        return None

    def settle(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--nav-graph', required=True)
    ap.add_argument('--frame', default='map')
    ap.add_argument('--tour', nargs='+', default=DEFAULT_TOUR,
                    help='waypoint names in visiting order')
    ap.add_argument('--goal-timeout-s', type=float, default=120.0)
    ap.add_argument('--settle-s', type=float, default=3.0,
                    help='seconds to let AMCL update after the robot stops')
    ap.add_argument('--no-pause', action='store_true',
                    help='do not wait for the operator between waypoints')
    args = ap.parse_args(args_without_ros[1:])

    wp = load_waypoints(args.nav_graph)
    missing = [n for n in args.tour if n not in wp]
    if missing:
        raise SystemExit('not in the graph: %s' % ', '.join(missing))

    node = Tour(args.frame)
    if not node.client.wait_for_server(timeout_sec=20.0):
        raise SystemExit('navigate_to_pose action server did not appear')

    rows = []
    try:
        for i, name in enumerate(args.tour, 1):
            x, y = wp[name]
            print('\n[%d/%d] %s  ->  (%.3f, %.3f)' % (i, len(args.tour), name, x, y), flush=True)
            started = time.time()
            status = node.go(x, y, args.goal_timeout_s)
            node.settle(args.settle_s)
            took = time.time() - started
            verdict = {GoalStatus.STATUS_SUCCEEDED: 'reached',
                       GoalStatus.STATUS_ABORTED: 'ABORTED',
                       GoalStatus.STATUS_CANCELED: 'canceled'}.get(status, 'TIMEOUT/none')
            if node.pose is None:
                print('   %s in %.0fs, but no amcl_pose was received' % (verdict, took))
                rows.append((name, verdict, None, None, None))
                continue
            px, py, yaw, sx, sy = node.pose
            err = math.hypot(px - x, py - y)
            print('   %s in %3.0fs | stopped at (%.3f, %.3f) | error %.3f m | sd %.2f/%.2f m'
                  % (verdict, took, px, py, err, sx, sy), flush=True)
            rows.append((name, verdict, err, sx, sy))
            if not args.no_pause and i < len(args.tour):
                input('   look at the robot, then press ENTER to continue ')
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        print('\n%-7s %-12s %9s %9s' % ('wp', 'outcome', 'error', 'sd_x'))
        for name, verdict, err, sx, _ in rows:
            print('%-7s %-12s %9s %9s'
                  % (name, verdict,
                     '-' if err is None else '%.3f m' % err,
                     '-' if sx is None else '%.2f m' % sx))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
