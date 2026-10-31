#!/usr/bin/env python3

import argparse
import json
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

DEFAULT_MAX_STALE_S = 0.5

MSG_TYPES = {
    'PoseStamped': PoseStamped,
    'PoseWithCovarianceStamped': PoseWithCovarianceStamped,
    'Odometry': Odometry,
    'TransformStamped': TransformStamped,
    'TFMessage': TFMessage,
}


def _pose_of(msg):
    """(x, y, z, qz, qw, stamp) from any of the accepted message types."""
    if isinstance(msg, PoseStamped):
        p, o, h = msg.pose.position, msg.pose.orientation, msg.header
    elif isinstance(msg, PoseWithCovarianceStamped):
        p, o, h = (msg.pose.pose.position, msg.pose.pose.orientation,
                   msg.header)
    elif isinstance(msg, Odometry):
        p, o, h = (msg.pose.pose.position, msg.pose.pose.orientation,
                   msg.header)
    elif isinstance(msg, TransformStamped):
        p, o, h = (msg.transform.translation, msg.transform.rotation,
                   msg.header)
    else:
        return None
    return (p.x, p.y, p.z, o.z, o.w,
            h.stamp.sec + h.stamp.nanosec * 1e-9)


class GroundTruthRecorder(Node):
    def __init__(self, args):
        super().__init__('hw_ground_truth_recorder')
        self.args = args
        self.lock = threading.Lock()
        # latest[name] = (x, y, z, qz, qw, stamp)
        self.latest = {}
        self.counts = {}
        self.robots = []
        self.dx, self.dy = args.frame_offset

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)

        for spec in args.pose_topic or []:
            name, topic, type_name = self._parse_topic_spec(spec)
            self.robots.append(name)
            self.create_subscription(
                MSG_TYPES[type_name], topic,
                lambda msg, n=name: self._on_pose(n, msg), qos)
            self.get_logger().info(
                'robot %s <- %s (%s)' % (name, topic, type_name))

        if args.tf:
            self.tf_pairs = {}
            for spec in args.tf:
                name, parent, child = spec.split(':')
                self.tf_pairs[(parent, child)] = name
                self.robots.append(name)
                self.get_logger().info(
                    'robot %s <- tf %s -> %s' % (name, parent, child))
            self.create_subscription(TFMessage, args.tf_topic,
                                     self._on_tf, qos)

        if not self.robots:
            raise SystemExit(
                'no source given: pass --pose-topic and/or --tf. Nothing would '
                'be recorded, and an empty ground-truth file makes every '
                'position metric silently unavailable rather than failing.')

        self.out = open(args.out, 'a')
        self.n_lines = 0
        self.n_incomplete = 0
        self.timer = self.create_timer(1.0 / args.hz, self._emit)

    @staticmethod
    def _parse_topic_spec(spec):
        """name:topic[:MsgType]; the type defaults to PoseStamped."""
        parts = spec.split(':')
        if len(parts) == 2:
            return parts[0], parts[1], 'PoseStamped'
        if len(parts) == 3:
            if parts[2] not in MSG_TYPES:
                raise SystemExit(
                    'unknown message type %r; accepted: %s'
                    % (parts[2], ', '.join(sorted(MSG_TYPES))))
            return parts[0], parts[1], parts[2]
        raise SystemExit('--pose-topic wants name:topic[:MsgType], got %r' % spec)

    def _on_pose(self, name, msg):
        if isinstance(msg, TFMessage):
            return self._on_tf(msg)
        pose = _pose_of(msg)
        if pose is None:
            return
        self._store(name, pose)

    def _on_tf(self, msg):
        for tf in msg.transforms:
            key = (tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/'))
            name = self.tf_pairs.get(key)
            if name is None:
                continue
            self._store(name, _pose_of(tf))

    def _store(self, name, pose):
        x, y, z, qz, qw, stamp = pose
        # A stamp of 0 is a header the publisher never filled; arrival time is used instead.
        if stamp <= 0.0:
            stamp = time.time()
            if not getattr(self, '_warned_stamp', False):
                self._warned_stamp = True
                self.get_logger().warn(
                    'source for %s publishes an empty header stamp; falling '
                    'back to arrival time, so the staleness check measures '
                    'transport delay rather than measurement age' % name)
        with self.lock:
            self.latest[name] = (x + self.dx, y + self.dy, z, qz, qw, stamp)
            self.counts[name] = self.counts.get(name, 0) + 1

    def _emit(self):
        now = time.time()
        robots = {}
        with self.lock:
            for name, (x, y, z, qz, qw, stamp) in self.latest.items():
                if now - stamp > self.args.max_stale:
                    continue
                robots[name] = {'x': x, 'y': y, 'z': z, 'qz': qz, 'qw': qw,
                                'stamp': stamp}
        if not robots:
            return
        if len(robots) < len(self.robots):
            self.n_incomplete += 1
        # sim_t is the emit instant on the system clock; each robot's measurement stamp
        # stays inside that robot's entry.
        self.out.write(json.dumps({'sim_t': now, '_recv_wall': now,
                                   'robots': robots}) + '\n')
        self.out.flush()
        self.n_lines += 1

    def report(self):
        expected = self.n_lines or 1
        print('wrote %d samples at %.1f Hz; %d lines were missing at least one '
              'robot (%.1f%%)'
              % (self.n_lines, self.args.hz, self.n_incomplete,
                 100.0 * self.n_incomplete / expected), file=sys.stderr)
        for name in sorted(self.robots):
            print('  %-12s %6d source messages' % (name, self.counts.get(name, 0)),
                  file=sys.stderr)
            if not self.counts.get(name):
                print('  %-12s NEVER SEEN -- every ground-truth metric for this '
                      'robot is unavailable for this repeat' % name,
                      file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='Where to write the JSONL output')
    ap.add_argument('--pose-topic', action='append', metavar='NAME:TOPIC[:TYPE]',
                    help='repeat once per robot. TYPE is one of %s '
                         '(default PoseStamped).' % ', '.join(sorted(MSG_TYPES)))
    ap.add_argument('--tf', action='append', metavar='NAME:PARENT:CHILD',
                    help='read the pose from a TF pair instead, e.g. '
                         'tb3_robot1:map:tb3_robot1_tag. This is the form '
                         'apriltag_ros publishes in.')
    ap.add_argument('--tf-topic', default='/tf',
                    help='Topic the TF pairs given by --tf are read from')
    ap.add_argument('--hz', type=float, default=10.0,
                    help='emit rate; 10 Hz matches /fleet_states, which is what '
                         'makes F7b event counts comparable between the two '
                         'streams (see the module docstring)')
    ap.add_argument('--max-stale', type=float, default=DEFAULT_MAX_STALE_S,
                    help='drop a robot from a line once its latest pose is '
                         'older than this, rather than repeating it as if '
                         'current (default %(default)s s)')
    ap.add_argument('--frame-offset', type=float, nargs=2, default=(0.0, 0.0),
                    metavar=('DX', 'DY'),
                    help='added to every x,y. Use only for a MEASURED rigid '
                         'offset between the tracker frame and the map frame; '
                         'an unmeasured offset would be reported as a constant '
                         'localisation error on every robot.')
    args = ap.parse_args()

    rclpy.init()
    node = GroundTruthRecorder(args)

    stop = threading.Event()

    def handler(*_):
        stop.set()
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    while rclpy.ok() and not stop.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.out.close()
    node.report()
    sys.stderr.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
