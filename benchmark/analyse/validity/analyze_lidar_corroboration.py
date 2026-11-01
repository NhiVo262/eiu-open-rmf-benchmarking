#!/usr/bin/env python3
"""Cross-checks the 7c lidar separation against the poses RMF held, for hardware, where
no ground truth exists.

Two directions, per pair of robots:

  7c -> RMF    at each instant 7c reads a separation below --d-safe-m, the separation
               RMF held at the same instant
  RMF -> 7c    at each instant RMF holds the robots closer than --near-m, whether a
               head-sized cluster appears in the expected bearing and range window

Neither is a verdict: the two sources are independent and this reports how far they
agree.
"""
import argparse
import bisect
import glob
import itertools
import json
import math
import os
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'safety'))
import analyze_safety_lidar as lidar  # noqa: E402

ROBOTS = ['tb3_robot1', 'tb3_robot2', 'tb3_robot3']
POSE_MATCH_TOLERANCE_S = 0.2
SCAN_PAIR_MAX_DT_S = 0.15         


def read_reported_lasers(bag):
    """{robot: ([bag times], [(laser x, laser y, yaw)])} from /fleet_states."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {r: ([], []) for r in ROBOTS}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != '/fleet_states':
            continue
        for state in deserialize_message(data, types[topic]).robots:
            if state.name not in out:
                continue
            x, y, yaw = state.location.x, state.location.y, state.location.yaw
            out[state.name][0].append(t_ns / 1e9)
            out[state.name][1].append((x - lidar.LASER_BEHIND_ORIGIN_M * math.cos(yaw),
                                       y - lidar.LASER_BEHIND_ORIGIN_M * math.sin(yaw), yaw))
    return out


def nearest(times, values, t):
    """The value nearest t, or None beyond POSE_MATCH_TOLERANCE_S."""
    i = bisect.bisect_left(times, t)
    best = min((j for j in (i - 1, i) if 0 <= j < len(times)),
               key=lambda j: abs(times[j] - t), default=None)
    if best is None or abs(times[best] - t) > POSE_MATCH_TOLERANCE_S:
        return None
    return values[best]


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def assess_pair(scans, poses, a, b, args):
    held_at_close = []
    near_instants = with_head = 0
    chords = []
    for t, scan_a, scan_b in lidar.pair_by_time(scans[a], scans[b], SCAN_PAIR_MAX_DT_S):
        pa, pb = nearest(*poses[a], t), nearest(*poses[b], t)
        if pa is None or pb is None:
            continue
        held = math.dist(pa[:2], pb[:2])
        match = lidar.mutual_range(scan_a, scan_b)
        if match is not None and match[0] + lidar.HEAD_NEAR_M < args.d_safe_m:
            held_at_close.append(held)
        if held >= args.near_m:
            continue
        near_instants += 1
        expected = wrap(math.atan2(pb[1] - pa[1], pb[0] - pa[0]) - pa[2])
        heads = [c for c in lidar.clusters(*scan_a[1:])
                 if abs(wrap(c['bearing_rad'] - expected)) <= math.radians(args.bearing_deg)
                 and abs(c['min_range_m'] + lidar.HEAD_NEAR_M - held) <= args.range_tol_m]
        if heads:
            with_head += 1
            chords.append(min(heads, key=lambda c: abs(wrap(c['bearing_rad'] - expected)))['chord_m'])
    return {'pair': [a, b],
            'instants_7c_below_d_safe': len(held_at_close),
            'rmf_held_median_m': round(statistics.median(held_at_close), 3) if held_at_close else None,
            'instants_rmf_held_below_near_m': sum(1 for h in held_at_close if h < args.near_m),
            'rmf_near_instants': near_instants,
            'rmf_near_with_head_cluster': with_head,
            'head_chords_m': [round(c, 4) for c in chords]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run-dir', required=True, action='append',
                    help='repeat as many times as there are scenarios')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    ap.add_argument('--d-safe-m', type=float, default=0.356,
                    help='separation, in metres, below which a 7c reading is checked')
    ap.add_argument('--near-m', type=float, default=1.0,
                    help='RMF-held separation, in metres, that counts as near')
    ap.add_argument('--bearing-deg', type=float, default=30.0,
                    help='bearing window, in degrees, around the bearing RMF predicts')
    ap.add_argument('--range-tol-m', type=float, default=0.5,
                    help='range window, in metres, around the range RMF predicts')
    args = ap.parse_args()

    runs = []
    for run_dir in args.run_dir:
        repeats = []
        for repeat_dir in sorted(glob.glob(os.path.join(run_dir, 'repeat_*'))):
            if not os.path.isdir(repeat_dir):
                continue
            bag = os.path.join(repeat_dir, 'bag')
            scans = lidar.read_scans(bag, ROBOTS)
            poses = read_reported_lasers(bag)
            repeats.append({'repeat_dir': repeat_dir,
                            'pairs': [assess_pair(scans, poses, a, b, args)
                                      for a, b in itertools.combinations(ROBOTS, 2)]})
        runs.append({'run_dir': run_dir, 'repeats': repeats})
    json.dump({'analysis_version': lidar.ANALYSIS_VERSION, 'd_safe_m': args.d_safe_m,
               'near_m': args.near_m, 'bearing_deg': args.bearing_deg,
               'range_tol_m': args.range_tol_m, 'runs': runs},
              open(args.output, 'w'), indent=1)

    pairs = [p for run in runs for rep in run['repeats'] for p in rep['pairs']]
    flagged = [p for p in pairs if p['instants_7c_below_d_safe']]
    corroborated = [p for p in flagged
                    if p['instants_rmf_held_below_near_m'] * 2 > p['instants_7c_below_d_safe']]
    near = sum(p['rmf_near_instants'] for p in pairs)
    heads = sum(p['rmf_near_with_head_cluster'] for p in pairs)
    chords = [c for p in pairs for c in p['head_chords_m']]
    print('pair-repeats with 7c below d_safe   %d, RMF-held near at most of them %d'
          % (len(flagged), len(corroborated)))
    print('instants RMF holds near             %d, head-sized cluster present %d (%.1f%%)'
          % (near, heads, 100.0 * heads / near if near else float('nan')))
    if chords:
        print('head cluster chord, median          %.3f m' % statistics.median(chords))


if __name__ == '__main__':
    main()
