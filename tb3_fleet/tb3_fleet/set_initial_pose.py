#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


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


class InitialPoseSeeder(Node):
    """Publishes /<robot_namespace>/initialpose repeatedly until AMCL is
    subscribed, or a timeout elapses.

    AMCL's own subscription only exists once its lifecycle node has finished
    configuring, which happens some seconds after Nav2 bringup starts. A
    single --once publish sent too early is silently dropped -- this is why
    the per-scenario procedure docs describe re-sending the initialpose
    command by hand when a robot ends up unlocalized.
    """

    def __init__(self, robot_namespace: str, x: float, y: float, yaw: float):
        super().__init__(f'set_initial_pose_{robot_namespace}')
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, f'/{robot_namespace}/initialpose', 1)
        self._msg = PoseWithCovarianceStamped()
        self._msg.header.frame_id = 'map'
        self._msg.pose.pose.position.x = x
        self._msg.pose.pose.position.y = y
        self._msg.pose.pose.orientation.z = 0.0
        self._msg.pose.pose.orientation.w = 1.0
        self._msg.pose.covariance = DEFAULT_COVARIANCE

    def seed(self, timeout_sec: float, retry_period_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._msg)
            if self._pub.get_subscription_count() > 0:
                # AMCL is listening -- publish once more so it definitely
                # sees a message after subscribing, then stop.
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
    parser.add_argument('--x', required=True, type=float)
    parser.add_argument('--y', required=True, type=float)
    parser.add_argument('--yaw', default=0.0, type=float)
    parser.add_argument('--timeout-sec', default=30.0, type=float)
    parser.add_argument('--retry-period-sec', default=0.5, type=float)
    args = parser.parse_args(args_without_ros[1:])

    node = InitialPoseSeeder(args.robot_namespace, args.x, args.y, args.yaw)
    try:
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
