#!/usr/bin/env python3

import argparse
import bisect
import glob
import json
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'congestion_handling'))
sys.path.insert(0, os.path.join(HERE, '..', 'fleet_coordination'))
import analyze_congestion_handling as pose_classifier  # noqa: E402
import analyze_fleet_coordination_4b as cmd_classifier  # noqa: E402

STEP_S = 0.05
GT_LINEAR_MPS = 0.005
GT_ANGULAR_RPS = 0.05


def command_moving(commands, t):
    times = [c[0] for c in commands]
    i = bisect.bisect_right(times, t) - 1
    return i >= 0 and t - commands[i][0] <= cmd_classifier.STALE_S and commands[i][1]


def read_fleet_states(bag, robots):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/fleet_states']))
    out = {r: [] for r in robots}
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        for state in deserialize_message(data, types['/fleet_states']).robots:
            if state.name in out:
                out[state.name].append((t_ns / 1e9, state.location.x, state.location.y, True))
    return out


def read_ground_truth(path, robots):
    out = {r: ([], []) for r in robots}
    if not os.path.exists(path):
        return None
    for line in open(path):
        sample = json.loads(line)
        for r in robots:
            p = sample['robots'].get(r)
            if p:
                out[r][0].append(sample['_recv_wall'])
                out[r][1].append((p['x'], p['y'], 2.0 * math.atan2(p['qz'], p['qw'])))
    return out


def gt_moving(track, t):
    times, poses = track
    i = bisect.bisect_left(times, t)
    if not 0 < i < len(times) or times[i] <= times[i - 1]:
        return None
    dt = times[i] - times[i - 1]
    a, b = poses[i - 1], poses[i]
    turn = abs((b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi) / dt
    return math.dist(a[:2], b[:2]) / dt >= GT_LINEAR_MPS or turn >= GT_ANGULAR_RPS


def tally(counts, name, source, reference):
    c = counts.setdefault(name, [0, 0, 0, 0])
    c[0 if source and reference else 1 if source else 2 if reference else 3] += 1


def summarise(c):
    both_moving, source_only, reference_only, both_still = c
    n = sum(c)
    return {'samples': n,
            'agreement_pct': round(100.0 * (both_moving + both_still) / n, 2),
            'reference_still_called_moving_pct': round(100.0 * source_only / max(source_only + both_still, 1), 2),
            'reference_moving_called_still_pct': round(100.0 * reference_only / max(both_moving + reference_only, 1), 2),
            'source_moving_pct': round(100.0 * (both_moving + source_only) / n, 2),
            'reference_moving_pct': round(100.0 * (both_moving + reference_only) / n, 2)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run-dir', required=True, action='append', help='repeat as many times as there are scenarios')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    counts = {}
    for run_dir in args.run_dir:
        for repeat in sorted(glob.glob(os.path.join(run_dir, 'repeat_*'))):
            tasks = json.load(open(os.path.join(repeat, 'task_planning_metrics.json')))['per_task']
            robots = sorted({t['robot'] for t in tasks})
            bag = os.path.join(repeat, 'bag')
            commands = cmd_classifier.read_commands(bag, robots)
            poses = read_fleet_states(bag, robots)
            gt = read_ground_truth(os.path.join(repeat, 'ground_truth_poses.jsonl'), robots)
            for task in tasks:
                r, a, b = task['robot'], task['t_award'], task['t_completed']
                if not commands[r]:
                    continue
                samples = [p for p in poses[r] if a <= p[0] <= b]
                if len(samples) < 5:
                    continue
                still = pose_classifier.blocked_episodes(samples)
                t = a
                while t < b:
                    cmd = command_moving(commands[r], t)
                    pose = not any(s0 <= t <= s1 for s0, s1 in still)
                    tally(counts, 'reported_pose_vs_cmd_vel', pose, cmd)
                    if gt is not None:
                        truth = gt_moving(gt[r], t)
                        if truth is not None:
                            tally(counts, 'cmd_vel_vs_ground_truth', cmd, truth)
                            tally(counts, 'reported_pose_vs_ground_truth', pose, truth)
                    t += STEP_S
    result = {'step_s': STEP_S, 'run_dirs': args.run_dir,
              'comparisons': {k: summarise(v) for k, v in counts.items()}}
    json.dump(result, open(args.output, 'w'), indent=2)
    print(json.dumps(result['comparisons'], indent=2))


if __name__ == '__main__':
    main()
