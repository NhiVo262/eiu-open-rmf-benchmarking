#!/usr/bin/env python3

import argparse
import bisect
import glob
import json
import math
import os

from rclpy.serialization import deserialize_message
import rmf_adapter.easy_full_control as rmf_easy
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

# footprint + vicinity, halved; see the module docstring
DEFAULT_MAX_MEDIAN_ERROR_M = 0.314
GT_MATCH_TOLERANCE_S = 0.2
# A ground-truth step below this counts as the robot standing still.
STATIONARY_STEP_M = 0.005


def read_profile_separation(fleet_config_path, nav_graph_path):
    """Conflict separation from the fleet's own participant profile, if readable."""
    try:
        cfg = rmf_easy.FleetConfiguration.from_config_files(
            fleet_config_path, nav_graph_path)
        robots = sorted(cfg.known_robots)
        rc = cfg.get_known_robot_configuration(robots[0])
        profile = rc.profile
        return (profile.footprint.characteristic_length
                + profile.vicinity.characteristic_length)
    except Exception:
        return None


def recorded_capture(run_dir, repeat_dir):
    metrics = os.path.join(repeat_dir, 'task_planning_metrics.json')
    if not os.path.exists(metrics):
        return None, set()
    with open(metrics) as f:
        record = json.load(f)
    name = os.path.basename(record.get('api_state_path') or '')
    ids = {t.get('task_id') for t in record.get('per_task') or [] if t.get('task_id')}
    if not name:
        return None, ids
    path = os.path.join(run_dir, name)
    return (path if os.path.exists(path) else None), ids


def robots_given_work(run_dir, repeat_dir):
    path, ids = recorded_capture(run_dir, repeat_dir)
    out = set()
    if path is None:
        return out
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        m = o.get('msg')
        if not isinstance(m, dict) or m.get('type') != 'task_state_update':
            continue
        data = m.get('data') or {}
        if ids and (data.get('booking') or {}).get('id') not in ids:
            continue
        who = (data.get('assigned_to') or {}).get('name')
        if who:
            out.add(who)
    return out


def load_ground_truth(repeat_dir):
    """Return {robot: [(timestamp_s, x, y)]} from a repeat's ground_truth_poses.jsonl,
    or an empty mapping when the file is absent."""
    frames = {}
    path = os.path.join(repeat_dir, 'ground_truth_poses.jsonl')
    if not os.path.exists(path):
        return frames
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        frames[o['sim_t']] = {k: (v['x'], v['y']) for k, v in o['robots'].items()}
    return frames


def per_robot_error(repeat_dir):
    gt = load_ground_truth(repeat_dir)
    gts = sorted(gt)
    bag = os.path.join(repeat_dir, 'bag')
    if not gts or not os.path.isdir(bag):
        return {}, {}, {}

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    errors = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/fleet_states':
            continue
        msg = deserialize_message(data, types[topic])
        for robot in msg.robots:
            stamp = robot.location.t.sec + robot.location.t.nanosec * 1e-9
            if stamp <= 0:
                continue
            i = bisect.bisect_left(gts, stamp)
            candidates = [k for k in (i - 1, i) if 0 <= k < len(gts)]
            if not candidates:
                continue
            k = min(candidates, key=lambda k: abs(gts[k] - stamp))
            if abs(gts[k] - stamp) > GT_MATCH_TOLERANCE_S:
                continue
            truth = gt[gts[k]].get(robot.name)
            if truth is None:
                continue
            errors.setdefault(robot.name, []).append(
                (gts[k], math.hypot(robot.location.x - truth[0],
                                    robot.location.y - truth[1])))

    # distance each robot really covered, so a robot that never drove is visible,
    # and the last instant it actually moved, which bounds the verdict window
    travelled, previous, last_moved = {}, {}, {}
    for t in gts:
        for name, p in gt[t].items():
            if name in previous:
                step = math.hypot(p[0] - previous[name][0],
                                  p[1] - previous[name][1])
                travelled[name] = travelled.get(name, 0.0) + step
                if step > STATIONARY_STEP_M:
                    last_moved[name] = t
            previous[name] = p
    return errors, travelled, last_moved


def assess_repeat(run_dir, repeat_dir, threshold):
    dispatched = robots_given_work(run_dir, repeat_dir)
    errors, travelled, last_moved = per_robot_error(repeat_dir)
    robots = {}
    for name in sorted(set(errors) | dispatched):
        stamped = errors.get(name, [])
        if not stamped:
            robots[name] = {'samples': 0, 'dispatched': name in dispatched,
                            'valid_for_rmf_decisions': False,
                            'reason': 'no matched samples'}
            continue
        series = sorted(e for _, e in stamped)
        stop = last_moved.get(name)
        moving = sorted(e for t, e in stamped
                        if stop is None or t <= stop)
        median = series[len(series) // 2]
        # The verdict uses the moving window; see the header. Falls back to the
        # whole repeat when the robot never moved, where the two agree anyway.
        median_moving = moving[len(moving) // 2] if moving else median
        dispatched_here = name in dispatched
        valid = dispatched_here and median_moving < threshold
        robots[name] = {
            'samples': len(series),
            'samples_moving': len(moving),
            'median_error_m': round(median_moving, 4),
            'median_error_whole_repeat_m': round(median, 4),
            'p95_error_m': round(series[int(len(series) * 0.95)], 4),
            'max_error_m': round(series[-1], 4),
            'travelled_m': round(travelled.get(name, 0.0), 2),
            'dispatched': dispatched_here,
            'valid_for_rmf_decisions': valid,
            'reason': ('' if valid
                       else ('not dispatched' if not dispatched_here
                             else 'median error over threshold while moving')),
        }
    return {'repeat_dir': repeat_dir, 'robots': robots}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir', required=True, action='append',
                    help='repeat as many times as there are scenarios')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    ap.add_argument('--threshold', type=float, default=DEFAULT_MAX_MEDIAN_ERROR_M, help='Largest median error, in metres, at which a robot still counts as measurable')
    ap.add_argument('--fleet-config', help='Fleet config YAML the participant profile is read from')
    ap.add_argument('--nav-graph', help='Navigation graph YAML the waypoints are read from')
    args = ap.parse_args()

    separation = None
    if args.fleet_config and args.nav_graph:
        separation = read_profile_separation(args.fleet_config, args.nav_graph)

    runs = []
    for run_dir in args.run_dir:
        repeats = []
        for repeat_dir in sorted(glob.glob(os.path.join(run_dir, 'repeat_*'))):
            if not os.path.isdir(repeat_dir):
                continue
            if not os.path.exists(os.path.join(repeat_dir, 'ground_truth_poses.jsonl')):
                raise SystemExit(
                    'no ground_truth_poses.jsonl in %s. This criterion compares the '
                    'reported pose against a position reference, which this deployment '
                    'records in simulation only, so there is nothing to compare a '
                    'hardware repeat against.' % repeat_dir)
            repeats.append(assess_repeat(run_dir, repeat_dir, args.threshold))
        runs.append({'run_dir': run_dir, 'repeats': repeats})

    json.dump({'threshold_m': args.threshold,
               'conflict_separation_m': separation,
               'runs': runs},
              open(args.output, 'w'), indent=1)

    header = '%-12s %-10s %-12s %10s %10s %9s %s' % (
        'scenario', 'repeat', 'robot', 'med moving', 'med whole', 'travelled',
        'valid')
    print(header)
    print('-' * len(header))
    valid = total = flipped = 0
    for run in runs:
        scenario = os.path.basename(os.path.dirname(run['run_dir'])).split('_FLEET')[0]
        for entry in run['repeats']:
            name = os.path.basename(entry['repeat_dir'])
            for robot, value in sorted(entry['robots'].items()):
                if not value.get('dispatched'):
                    continue
                total += 1
                valid += bool(value['valid_for_rmf_decisions'])
                whole = value.get('median_error_whole_repeat_m',
                                  value.get('median_error_m', 0))
                differs = (whole >= args.threshold) != (
                    value.get('median_error_m', 0) >= args.threshold)
                flipped += differs
                print('%-12s %-10s %-12s %9.3fm %9.3fm %8.2fm %s%s' % (
                    scenario, name, robot, value.get('median_error_m', 0),
                    whole, value.get('travelled_m', 0),
                    'VALID' if value['valid_for_rmf_decisions'] else 'excluded',
                    '  (windows disagree)' if differs else ''))
    print()
    if separation is not None:
        print('conflict separation read from the live profile: %.3f m' % separation)
    print('valid for F2/F3/F5/F6: %d of %d dispatched robot-repeats (%.0f%%)'
          % (valid, total, 100.0 * valid / max(1, total)))
    if flipped:
        print('%d of %d would have been judged differently on the whole-repeat '
              'median; the verdict uses the moving window (see the header).'
              % (flipped, total))
    print('F7 reads ground truth and is not gated by this.')
    print('F4b is not gated either, but it reads /fleet_states: what exempts it is '
          'that it uses a first difference, which a constant offset cancels out of. '
          'It stays sensitive to AMCL jumps, which this threshold does not measure.')


if __name__ == '__main__':
    main()
