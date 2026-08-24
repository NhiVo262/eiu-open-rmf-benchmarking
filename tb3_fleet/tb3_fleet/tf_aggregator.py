#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfAggregator(Node):

    def __init__(self, robot_namespace: str):
        super().__init__(f'tf_aggregator_{robot_namespace}')
        self._prefix = f'{robot_namespace}/'

        tf_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        tf_static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._tf_pub = self.create_publisher(TFMessage, '/tf', tf_qos)
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', tf_static_qos)

        self.create_subscription(
            TFMessage, f'/{robot_namespace}/tf', self._on_tf, tf_qos)
        self.create_subscription(
            TFMessage, f'/{robot_namespace}/tf_static', self._on_tf_static, tf_static_qos)

    def _prefixed(self, frame_id: str) -> str:
        if frame_id == 'map':
            return frame_id
        if frame_id.startswith(self._prefix):
            return frame_id
        return self._prefix + frame_id

    def _rewrite(self, msg: TFMessage) -> TFMessage:
        for t in msg.transforms:
            t.header.frame_id = self._prefixed(t.header.frame_id)
            t.child_frame_id = self._prefixed(t.child_frame_id)
        return msg

    def _on_tf(self, msg: TFMessage):
        self._tf_pub.publish(self._rewrite(msg))

    def _on_tf_static(self, msg: TFMessage):
        self._tf_static_pub.publish(self._rewrite(msg))


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog='tf_aggregator',
        description='Gop TF rieng cua 1 robot (namespace) vao /tf, /tf_static dung chung, giu frame map chung.')
    parser.add_argument('--robot-namespace', '-n', required=True, type=str)
    args = parser.parse_args(args_without_ros[1:])

    node = TfAggregator(args.robot_namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv)
