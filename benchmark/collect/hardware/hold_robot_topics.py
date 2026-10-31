#!/usr/bin/env python3


import argparse
import signal
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_msgs.msg import TFMessage

ROBOT_TOPICS = [
    ('tf', TFMessage, False, True),
    ('tf_static', TFMessage, True, True),
    ('amcl_pose', PoseWithCovarianceStamped, False, True),
    ('cmd_vel', Twist, False, True),
    ('battery_state', BatteryState, False, True),
    ('scan', LaserScan, False, False),
    ('plan', Path, False, True),
]


REPORT_PERIOD_S = 60.0


class TopicHolder(Node):

    def __init__(self, robot):
        super().__init__('hold_robot_topics')
        self.period = {}
        self.total = {}
        for suffix, msg_type, latched, reliable in ROBOT_TOPICS:
            topic = f'/{robot}/{suffix}'
            self.period[topic] = 0
            self.total[topic] = 0
            self.create_subscription(msg_type, topic, self._make_counter(topic),
                                     self._qos(latched, reliable))
        self.create_timer(REPORT_PERIOD_S, self._report)
        self.get_logger().info(
            f'Holding {len(ROBOT_TOPICS)} topics for robot [{robot}].')

    @staticmethod
    def _qos(latched, reliable):
        return QoSProfile(
            history=History.KEEP_LAST, depth=1,
            reliability=Reliability.RELIABLE if reliable else Reliability.BEST_EFFORT,
            durability=Durability.TRANSIENT_LOCAL if latched else Durability.VOLATILE,
        )

    def _make_counter(self, topic):
        def callback(_):
            self.period[topic] += 1
            self.total[topic] += 1
        return callback

    def _report(self):
        counts = ', '.join(f'{topic.rsplit("/", 1)[-1]} {n}'
                           for topic, n in self.period.items())
        message = f'last {REPORT_PERIOD_S:.0f}s: {counts}.'
        never = sorted(topic for topic, n in self.total.items() if n == 0)
        if never:
            message += ' Never delivered: ' + ', '.join(never) + '.'
        self.get_logger().info(message)
        for topic in self.period:
            self.period[topic] = 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='tb3_robot1',
                        help='Robot whose bridge routes are held open')
    args = parser.parse_args()

    rclpy.init()
    node = TopicHolder(args.robot)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
