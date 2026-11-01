#!/usr/bin/env python3
"""Validates the 7c lidar separation against Gazebo ground truth.

For each pair of robots, every mutual range analyze_safety_lidar.py accepts is checked
against the true laser-centre separation at that instant and classified:

  attributable        the range is consistent with the other robot's LDS head
  via_third_robot     it is consistent with the third robot instead
  unattributable      it matches no robot

Also counts the close approaches a pair reached, and scores the below-d_safe readings
against the truth as true positive, false positive and false negative. Simulation only;
a repeat without ground_truth_poses.jsonl is reported and skipped.
"""
import argparse
import bisect
import glob
import itertools
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'safety'))
import analyze_safety_lidar as lidar  

ROBOTS = ['tb3_robot1', 'tb3_robot2', 'tb3_robot3']
NOISE_MARGIN_M = 4 * lidar.LIDAR_NOISE_SD_M
DEFAULT_CLOSE_M = 0.628           # footprint + vicinity
DEFAULT_D_SAFE_M = 0.356          # twice the footprint radius, as 7a and 7b use
GT_MATCH_TOLERANCE_S = 0.2
SCAN_PAIR_MAX_DT_S = 0.15         # the pairing window analyze_safety_lidar.py uses


def load_ground_truth(path):
    """(wall times, [{robot: laser centre (x, y)}]) for samples holding every robot."""
    times, lasers = [], []
    with open(path) as f:
        for line in f:
            sample = json.loads(line)
            robots = sample.get('robots') or {}
            if not all(r in robots for r in ROBOTS):
                continue
            centres = {}
            for r in ROBOTS:
                p = robots[r]
                yaw = 2.0 * math.atan2(p['qz'], p['qw'])
                centres[r] = (p['x'] - lidar.LASER_BEHIND_ORIGIN_M * math.cos(yaw),
                              p['y'] - lidar.LASER_BEHIND_ORIGIN_M * math.sin(yaw))
            times.append(sample['_recv_wall'])
            lasers.append(centres)
    return times, lasers


def nearest_sample(times, samples, t):
    """The ground-truth sample nearest t, or None beyond GT_MATCH_TOLERANCE_S."""
    i = bisect.bisect_left(times, t)
    best = min((j for j in (i - 1, i) if 0 <= j < len(times)),
               key=lambda j: abs(times[j] - t), default=None)
    if best is None or abs(times[best] - t) > GT_MATCH_TOLERANCE_S:
        return None
    return samples[best]


def is_head_range(r, d):
    """Whether a laser range r can be the LDS head of a robot whose laser is d away."""
    return (d - lidar.HEAD_FAR_M - NOISE_MARGIN_M
            <= r <= d - lidar.HEAD_NEAR_M + NOISE_MARGIN_M)


def assess_pair(scans, times, lasers, a, b, close_m, d_safe_m):
    """Attribution and detection counts for one pair in one repeat."""
    c = next(r for r in ROBOTS if r not in (a, b))
    out = {'pair': [a, b], 'matched': 0, 'attributable': 0, 'via_third_robot': 0,
           'unattributable': 0, 'close_instants': 0, 'close_with_attributable_match': 0,
           'below_d_safe_tp': 0, 'below_d_safe_fp': 0, 'below_d_safe_fn': 0,
           'true_min_separation_m': None, 'min_separation_lower_m': None,
           'min_range_attributable': None}
    for t, scan_a, scan_b in lidar.pair_by_time(scans[a], scans[b], SCAN_PAIR_MAX_DT_S):
        pos = nearest_sample(times, lasers, t)
        if pos is None:
            continue
        d_ab = math.dist(pos[a], pos[b])
        if out['true_min_separation_m'] is None or d_ab < out['true_min_separation_m']:
            out['true_min_separation_m'] = round(d_ab, 4)
        match = lidar.mutual_range(scan_a, scan_b)
        r = match[0] if match is not None else None
        attributable = r is not None and is_head_range(r, d_ab)
        if d_ab < close_m:
            out['close_instants'] += 1
            out['close_with_attributable_match'] += attributable
        truly_below = d_ab < d_safe_m
        read_below = r is not None and r + lidar.HEAD_NEAR_M < d_safe_m
        out['below_d_safe_tp'] += truly_below and read_below
        out['below_d_safe_fp'] += read_below and not truly_below
        out['below_d_safe_fn'] += truly_below and not read_below
        if r is None:
            continue
        out['matched'] += 1
        if attributable:
            out['attributable'] += 1
        elif is_head_range(r, math.dist(pos[a], pos[c])) and is_head_range(r, math.dist(pos[b], pos[c])):
            out['via_third_robot'] += 1
        else:
            out['unattributable'] += 1
        lower = r + lidar.HEAD_NEAR_M
        if out['min_separation_lower_m'] is None or lower < out['min_separation_lower_m']:
            out['min_separation_lower_m'] = round(lower, 4)
            out['min_range_attributable'] = attributable
    return out


def assess_repeat(repeat_dir, close_m, d_safe_m):
    gt_path = os.path.join(repeat_dir, 'ground_truth_poses.jsonl')
    if not os.path.exists(gt_path):
        return {'repeat_dir': repeat_dir, 'pairs': None, 'reason': 'no ground_truth_poses.jsonl'}
    times, lasers = load_ground_truth(gt_path)
    scans = lidar.read_scans(os.path.join(repeat_dir, 'bag'), ROBOTS)
    return {'repeat_dir': repeat_dir,
            'pairs': [assess_pair(scans, times, lasers, a, b, close_m, d_safe_m)
                      for a, b in itertools.combinations(ROBOTS, 2)]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run-dir', required=True, action='append',
                    help='repeat as many times as there are scenarios')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    ap.add_argument('--close-m', type=float, default=DEFAULT_CLOSE_M,
                    help='true laser-centre separation, in metres, below which an instant is a close approach')
    ap.add_argument('--d-safe-m', type=float, default=DEFAULT_D_SAFE_M,
                    help='separation, in metres, for the below-d_safe detection counts')
    args = ap.parse_args()

    runs = []
    for run_dir in args.run_dir:
        repeats = [assess_repeat(r, args.close_m, args.d_safe_m)
                   for r in sorted(glob.glob(os.path.join(run_dir, 'repeat_*'))) if os.path.isdir(r)]
        runs.append({'run_dir': run_dir, 'repeats': repeats})
    json.dump({'analysis_version': lidar.ANALYSIS_VERSION, 'close_m': args.close_m,
               'd_safe_m': args.d_safe_m, 'runs': runs},
              open(args.output, 'w'), indent=1)

    pairs = [p for run in runs for rep in run['repeats'] for p in (rep['pairs'] or [])]
    total = lambda key: sum(p[key] for p in pairs)
    print('matched instants      %d' % total('matched'))
    print('  attributable        %d' % total('attributable'))
    print('  via third robot     %d' % total('via_third_robot'))
    print('  unattributable      %d' % total('unattributable'))
    print('close instants        %d, with an attributable match %d'
          % (total('close_instants'), total('close_with_attributable_match')))
    print('below d_safe          TP %d  FP %d  FN %d'
          % (total('below_d_safe_tp'), total('below_d_safe_fp'), total('below_d_safe_fn')))
    print('pair minima           %d of %d attributable'
          % (sum(1 for p in pairs if p['min_range_attributable']),
             sum(1 for p in pairs if p['min_separation_lower_m'] is not None)))


if __name__ == '__main__':
    main()
