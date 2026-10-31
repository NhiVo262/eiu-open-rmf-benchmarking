#!/usr/bin/env python3

import argparse
import glob
import json
import math
import os
import sys


SPIN_ANGULAR_MIN = 0.15      # rad/s
SPIN_LINEAR_MAX = 0.02       # m/s
NOMINAL_SCAN_HZ = 10.0


def read_topics(bag, wanted):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in wanted}
    out = {name: [] for name in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in types:
            out[topic].append((stamp / 1e9, deserialize_message(data, types[topic])))
    return out


def covariance_stats(poses):
    if not poses:
        return None, None
    radii = []
    for _, msg in poses:
        cov = msg.pose.covariance
        radii.append(math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0)))
    return max(radii), sum(radii) / len(radii)


def spin_time(cmds):
    total = 0.0
    for (t0, m0), (t1, _) in zip(cmds, cmds[1:]):
        if abs(m0.angular.z) >= SPIN_ANGULAR_MIN and abs(m0.linear.x) <= SPIN_LINEAR_MAX:
            total += min(t1 - t0, 1.0)
    return total


def stalled_time(cmds):
    total = 0.0
    for (t0, m0), (t1, _) in zip(cmds, cmds[1:]):
        if abs(m0.linear.x) <= SPIN_LINEAR_MAX and abs(m0.angular.z) < SPIN_ANGULAR_MIN:
            total += min(t1 - t0, 1.0)
    return total


def path_length(cmds):
    """Return the distance driven, integrated from the commanded forward speed."""
    total = 0.0
    for (t0, m0), (t1, _) in zip(cmds, cmds[1:]):
        total += abs(m0.linear.x) * min(t1 - t0, 1.0)
    return total


def clip(series, window):
    if window is None:
        return series
    start, end = window
    return [(t, m) for t, m in series if start <= t <= end]


def analyse(bag, robot, window=None):
    topics = [f'/{robot}/amcl_pose', f'/{robot}/plan', f'/{robot}/cmd_vel',
              f'/{robot}/scan']
    data = read_topics(bag, topics)
    cmds = clip(data.get(f'/{robot}/cmd_vel', []), window)
    scans = clip(data.get(f'/{robot}/scan', []), window)
    poses = clip(data.get(f'/{robot}/amcl_pose', []), window)
    plans = clip(data.get(f'/{robot}/plan', []), window)

    duration = 0.0
    for series in (cmds, scans, poses, plans):
        if len(series) > 1:
            duration = max(duration, series[-1][0] - series[0][0])

    cov_max, cov_mean = covariance_stats(poses)
    return {
        'bag': bag,
        'duration_s': duration,
        'n_replans': len(plans),
        'replans_per_min': len(plans) / duration * 60.0 if duration else None,
        'spin_s': spin_time(cmds),
        'stalled_s': stalled_time(cmds),
        'commanded_m': path_length(cmds),
        'amcl_cov_radius_max_m': cov_max,
        'amcl_cov_radius_mean_m': cov_mean,
        'scan_hz': len(scans) / duration if duration else None,
        'scan_delivered_pct': (len(scans) / duration / NOMINAL_SCAN_HZ * 100.0
                               if duration else None),
    }


def task_window(repeat_dir):
    path = os.path.join(repeat_dir, 'task_planning_metrics.json')
    if not os.path.exists(path):
        return None, None
    try:
        task = json.load(open(path))['per_task'][0]
        return task['makespan_s'], (task['t_arrival_at_route_start'],
                                    task['t_end_physical'])
    except (KeyError, IndexError, ValueError):
        return None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run_dir', help='A run directory holding repeat_N subdirectories')
    ap.add_argument('--robot', default='tb3_robot1',
                    help='Robot whose indicators are reported')
    ap.add_argument('--whole-bag', action='store_true',
                    help='Measure over the whole recording instead of the task window')
    ap.add_argument('--output', default=None, help='JSON file to write the table to')
    args = ap.parse_args()

    repeats = sorted(glob.glob(os.path.join(args.run_dir, 'repeat_*')),
                     key=lambda p: int(''.join(c for c in os.path.basename(p) if c.isdigit())))
    if not repeats:
        print(f'no repeat_N directory under {args.run_dir}')
        return 1

    rows = []
    header = (f'{"repeat":10}{"makespan":>10}{"spin_s":>9}{"stalled_s":>11}'
              f'{"commanded_m":>13}{"replan/min":>12}{"cov_max_m":>11}{"scan_%":>8}')
    print(header)
    for d in repeats:
        bag = os.path.join(d, 'bag')
        if not os.path.isdir(bag):
            continue
        makespan, window = task_window(d)
        row = analyse(bag, args.robot, window if not args.whole_bag else None)
        row['repeat'] = os.path.basename(d)
        row['makespan_s'] = makespan
        rows.append(row)
        def cell(value, width, digits):
            return (f'{value:>{width}.{digits}f}' if value is not None
                    else f'{"-":>{width}}')
        print(f'{row["repeat"]:10}'
              f'{cell(row["makespan_s"], 10, 1)}'
              f'{row["spin_s"]:>9.1f}{row["stalled_s"]:>11.1f}{row["commanded_m"]:>13.2f}'
              f'{cell(row["replans_per_min"], 12, 1)}'
              f'{cell(row["amcl_cov_radius_max_m"], 11, 3)}'
              f'{cell(row["scan_delivered_pct"], 8, 1)}')

    if args.output:
        with open(args.output, 'w') as f:
            json.dump(rows, f, indent=2)
        print(f'\nwritten to {args.output}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
