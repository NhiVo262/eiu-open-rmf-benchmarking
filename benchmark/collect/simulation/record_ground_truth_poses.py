#!/usr/bin/env python3

import argparse
import json
import os
import re
import signal
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

STOP = False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True, help='Where to write the JSONL output')
    ap.add_argument('--world', default='sim_world', help='Gazebo world name in the pose topic')
    ap.add_argument('--match', default=r'^tb3_robot\d+$',
                    help='regex an entity name must fully match to be recorded')

    ap.add_argument('--rename-to', default=None,
                    help='record the matched entity under this name instead '
                         '(e.g. burger -> tb3_robot1 for single-robot runs)')

    ap.add_argument('--hz', type=float, default=10.0,
                    help='downsample to this rate to match /fleet_states '
                         '(source is ~50Hz); 0 = keep all')
    args = ap.parse_args()

    keep = re.compile(args.match)
    min_dt = 1.0 / args.hz if args.hz > 0 else 0.0
    out = open(args.out, 'a')
    last_emit = [0.0]
    n = [0]

    def cb(msg: Pose_V):
        sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if min_dt and (sim_t - last_emit[0]) < min_dt:
            return
        last_emit[0] = sim_t
        robots = {}
        for p in msg.pose:
            if not keep.match(p.name):
                continue
            robots[args.rename_to or p.name] = {
                'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                'qz': p.orientation.z, 'qw': p.orientation.w,
            }
        if not robots:
            return
        out.write(json.dumps({'sim_t': sim_t, '_recv_wall': time.time(),
                              'robots': robots}) + '\n')
        out.flush()
        n[0] += 1

    node = Node()
    topic = f'/world/{args.world}/dynamic_pose/info'
    if not node.subscribe(Pose_V, topic, cb):
        print(f'failed to subscribe to {topic}', file=sys.stderr)
        return 1
    print(f'recording {topic} -> {args.out} (match={args.match}, {args.hz}Hz)',
          file=sys.stderr)

    def stop(*_):
        global STOP
        STOP = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    while not STOP:
        time.sleep(0.2)
    out.close()
    print(f'wrote {n[0]} samples', file=sys.stderr)
    sys.stderr.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
