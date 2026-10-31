#!/usr/bin/env python3

import argparse
import os
import json
import math
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

MAX_PLAUSIBLE_SPEED_MPS = 8.0


GROUND_TRUTH_MAX_SPEED_MPS = 1.0


def check_ground_truth(path, max_speed):
    """Speed check on Gazebo's own poses. Returns (checked, glitches)."""
    frames = {}
    with open(path) as f:
        for line in f:
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            frames[o['sim_t']] = {k: (v['x'], v['y']) for k, v in o['robots'].items()}
    if len(frames) < 2:
        return False, []
    per = {}
    for t in sorted(frames):
        for nm, p in frames[t].items():
            per.setdefault(nm, []).append((t, p))
    glitches = []
    for nm, seq in per.items():
        for i in range(1, len(seq)):
            dt = seq[i][0] - seq[i - 1][0]
            if dt <= 0:
                continue
            d = math.hypot(seq[i][1][0] - seq[i - 1][1][0],
                           seq[i][1][1] - seq[i - 1][1][1])
            if d / dt > max_speed:
                glitches.append(
                    f'  {nm} sim_t={seq[i][0]:.2f} {seq[i-1][1]} -> {seq[i][1]}  '
                    f'speed={d/dt:.1f} m/s (GROUND TRUTH)')
    return True, glitches


def parse_args():
    ap = argparse.ArgumentParser(description='Detect physics-explosion position jumps in a repeat')
    ap.add_argument('--bag', required=True, help='Path to the rosbag2 directory of one repeat')
    ap.add_argument('--robots', nargs='+', required=True, help='Robot names, space separated')
    ap.add_argument('--max-speed', type=float, default=MAX_PLAUSIBLE_SPEED_MPS, help='Speed above which a position jump counts as a glitch, in m/s')
    ap.add_argument('--ground-truth', default=None,
                    help='ground_truth_poses.jsonl; defaults to the file beside --bag')
    return ap.parse_args()


def main():
    args = parse_args()
    gt_path = args.ground_truth or os.path.join(
        os.path.dirname(os.path.abspath(args.bag)), 'ground_truth_poses.jsonl')
    if os.path.exists(gt_path):
        checked, glitches = check_ground_truth(gt_path, GROUND_TRUTH_MAX_SPEED_MPS)
        if checked:
            if glitches:
                print(f'PHYSICS GLITCH DETECTED: {len(glitches)} impossible-speed '
                      f'jump(s) in ground truth')
                for g in glitches:
                    print(g)
                return 1
            print('clean (ground truth)')
            return 0
    print(f'no usable ground truth at [{gt_path}] -- falling back to /fleet_states, '
          f'which cannot separate a physics explosion from an AMCL jump')

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id='mcap'), ConverterOptions('', ''))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/fleet_states' not in topic_types:
        print('no /fleet_states topic in bag -- cannot check, treating as clean')
        return 0
    msg_class = get_message(topic_types['/fleet_states'])

    last = {}
    glitches = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != '/fleet_states':
            continue
        msg = deserialize_message(data, msg_class)
        t = t_ns / 1e9
        for r in msg.robots:
            if r.name not in args.robots:
                continue
            xy = (r.location.x, r.location.y)
            if r.name in last:
                pt, pxy = last[r.name]
                dt = t - pt
                if dt > 0:
                    d = math.hypot(xy[0] - pxy[0], xy[1] - pxy[1])
                    speed = d / dt
                    if speed > args.max_speed:
                        glitches.append((r.name, t, pxy, xy, speed))
            last[r.name] = (t, xy)

    if glitches:
        print(f'PHYSICS GLITCH DETECTED: {len(glitches)} impossible-speed jump(s)')
        for name, t, pxy, xy, speed in glitches:
            print(f'  {name} t={t:.2f} {pxy} -> {xy}  speed={speed:.1f} m/s')
        return 1

    print('clean: no impossible-speed position jumps found')
    return 0


if __name__ == '__main__':
    sys.exit(main())
