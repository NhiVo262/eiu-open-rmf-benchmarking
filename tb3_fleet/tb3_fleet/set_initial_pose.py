#!/usr/bin/env python3

import argparse
import heapq
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml


# Matches AMCL's default initial covariance in nav2_params.yaml (position
# +/-0.5m std dev, yaw +/-~0.44rad std dev) -- keep in sync if that changes.
DEFAULT_COVARIANCE = [
    0.25, 0, 0, 0, 0, 0,
    0, 0.25, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0.06,
]


def load_waypoints(nav_graph_path):
    """name -> (x, y), from a building_map_generator nav_graph YAML's vertices list."""
    with open(nav_graph_path) as f:
        data = yaml.safe_load(f)
    coords = {}
    for level in data.get('levels', {}).values():
        for v in level.get('vertices', []):
            params = v[2] if len(v) > 2 else {}
            name = params.get('name')
            if name:
                coords[name] = (v[0], v[1])
    return coords


def first_hop(nav_graph_path, start, end):
    """Name of the second waypoint on the shortest path from start to end, which is the
    neighbour the robot sets off towards. Returns None when the two are the same."""
    data = yaml.safe_load(open(nav_graph_path))
    index = {}
    coords = {}
    for level in data.get('levels', {}).values():
        for i, v in enumerate(level.get('vertices', [])):
            coords[i] = (v[0], v[1])
            params = v[2] if len(v) > 2 else {}
            if params.get('name'):
                index[params['name']] = i
        adjacency = {}
        for lane in level.get('lanes', []):
            a, b = lane[0], lane[1]
            adjacency.setdefault(a, []).append(b)
            adjacency.setdefault(b, []).append(a)
        break
    if start not in index or end not in index:
        raise SystemExit(f'waypoint [{start}] or [{end}] is not in {nav_graph_path}')
    s, t = index[start], index[end]
    if s == t:
        return None
    name_of = {i: n for n, i in index.items()}
    dist = {s: 0.0}
    previous = {}
    queue = [(0.0, s)]
    while queue:
        d, u = heapq.heappop(queue)
        if u == t:
            break
        if d > dist.get(u, float('inf')):
            continue
        for w in adjacency.get(u, []):
            nd = d + math.dist(coords[u], coords[w])
            if nd < dist.get(w, float('inf')):
                dist[w] = nd
                previous[w] = u
                heapq.heappush(queue, (nd, w))
    if t not in dist:
        raise SystemExit(f'no path from [{start}] to [{end}] in {nav_graph_path}')
    path = [t]
    while path[-1] != s:
        path.append(previous[path[-1]])
    path.reverse()
    return name_of.get(path[1])


def pose_from_graph(nav_graph_path, waypoint, toward=None):
    """Return (x, y, yaw) for a start waypoint, with yaw pointing along the lane toward
    the named neighbour.

    Deriving the heading from the graph keeps the seeded pose and the route the robot is
    about to drive in one place. A waypoint with more than one outgoing lane has no single
    heading, so --toward names which lane the robot faces.
    """
    coords = load_waypoints(nav_graph_path)
    if waypoint not in coords:
        raise SystemExit(f'waypoint [{waypoint}] is not in {nav_graph_path}')
    x, y = coords[waypoint]
    if toward is None:
        return x, y, 0.0
    if toward not in coords:
        raise SystemExit(f'waypoint [{toward}] is not in {nav_graph_path}')
    tx, ty = coords[toward]
    return x, y, math.atan2(ty - y, tx - x)


class InitialPoseSeeder(Node):

    def __init__(self, robot_namespace: str, x: float, y: float, yaw: float):
        super().__init__(f'set_initial_pose_{robot_namespace}')
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, f'/{robot_namespace}/initialpose', 1)
        self._msg = PoseWithCovarianceStamped()
        self._msg.header.frame_id = 'map'
        self._msg.pose.pose.position.x = x
        self._msg.pose.pose.position.y = y
        self._msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self._msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._msg.pose.covariance = DEFAULT_COVARIANCE

    def seed(self, timeout_sec: float, retry_period_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._msg)
            if self._pub.get_subscription_count() > 0:
                time.sleep(0.2)
                self._msg.header.stamp = self.get_clock().now().to_msg()
                self._pub.publish(self._msg)
                return True
            time.sleep(retry_period_sec)
        return False


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='set_initial_pose',
        description='Seed AMCL initial pose for one robot, retrying until AMCL is subscribed.')
    parser.add_argument('--robot-namespace', '-n', required=True, type=str)
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--yaw', default=0.0, type=float)
    parser.add_argument('--nav-graph', type=str,
                        help='nav_graph YAML the waypoint coordinates are read from')
    parser.add_argument('--waypoint', type=str,
                        help='start waypoint name; replaces --x and --y')
    parser.add_argument('--route-end', type=str,
                        help='the far end of the route this robot starts on; the heading '
                             'becomes the first hop of that route, so the seeded pose '
                             'faces the way the robot will set off')
    parser.add_argument('--toward', type=str,
                        help='neighbouring waypoint the robot faces; the yaw is the '
                             'bearing of that lane. Required where the start waypoint '
                             'has more than one outgoing lane.')
    parser.add_argument('--timeout-sec', default=30.0, type=float)
    parser.add_argument('--retry-period-sec', default=0.5, type=float)
    parser.add_argument('--print-pose', action='store_true',
                        help='print "x y yaw" and exit without publishing, so a caller '
                             'can show the same pose it will seed')
    args = parser.parse_args(args_without_ros[1:])

    if args.waypoint is not None:
        if args.nav_graph is None:
            parser.error('--waypoint requires --nav-graph')
        toward = args.toward
        if toward is None and args.route_end is not None:
            toward = first_hop(args.nav_graph, args.waypoint, args.route_end)
        x, y, yaw = pose_from_graph(args.nav_graph, args.waypoint, toward)
    elif args.x is None or args.y is None:
        parser.error('give either --waypoint with --nav-graph, or both --x and --y')
    else:
        x, y, yaw = args.x, args.y, args.yaw

    if args.print_pose:
        print(f'{x:.4f} {y:.4f} {yaw:.4f}')
        rclpy.try_shutdown()
        return

    node = InitialPoseSeeder(args.robot_namespace, x, y, yaw)
    try:
        node.get_logger().info(
            f'Seeding [{args.robot_namespace}] at ({x:.3f}, {y:.3f}) '
            f'yaw {yaw:.3f} rad.')
        ok = node.seed(args.timeout_sec, args.retry_period_sec)
        if not ok:
            node.get_logger().warn(
                f'AMCL for [{args.robot_namespace}] never subscribed within '
                f'{args.timeout_sec}s -- initial pose was not confirmed delivered.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
