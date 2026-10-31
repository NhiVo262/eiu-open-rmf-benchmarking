#!/usr/bin/env python3

import argparse
import bisect
import json
import math
import os
import sys

def load_ground_truth(path):
    """(ordered sample times, per-robot presence, per-robot distance travelled)."""
    times, present, travelled, previous = [], {}, {}, {}
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        t = o.get('sim_t')
        if t is None:
            continue
        times.append(t)
        for name, pose in o.get('robots', {}).items():
            present.setdefault(name, []).append(t)
            if name in previous:
                travelled[name] = travelled.get(name, 0.0) + math.hypot(
                    pose['x'] - previous[name][0], pose['y'] - previous[name][1])
            previous[name] = (pose['x'], pose['y'])
    return times, present, travelled


def robots_given_work(run_dir, attempt):
    path = os.path.join(run_dir, 'api_state_attempt%s.jsonl' % attempt)
    out = set()
    if not os.path.exists(path):
        return out
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        m = o.get('msg')
        if not isinstance(m, dict) or m.get('type') != 'task_state_update':
            continue
        who = (m.get('data', {}).get('assigned_to') or {}).get('name')
        if who:
            out.add(who)
    return out


def battery_minima(bag, robots):
    """Lowest state of charge per robot during the repeat, or {} if unreadable."""
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return {}
    if not os.path.isdir(bag):
        return {}
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    types = {t.name: get_message(t.type)
             for t in reader.get_all_topics_and_types()}
    wanted = {'/%s/battery_state' % r: r for r in robots}
    if not (set(wanted) & set(types)):
        return {}
    lowest = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        name = wanted.get(topic)
        if name is None:
            continue
        msg = deserialize_message(data, types[topic])
        soc = msg.percentage
        # percentage arrives on either the 0..1 or the 0..100 scale; NaN is an unpublished field.
        if soc != soc:
            continue
        if soc > 1.0:
            soc = soc / 100.0
        lowest[name] = min(lowest.get(name, 1.0), soc)
    return lowest


def pose_sigma(covariance):
    if covariance is None or len(covariance) < 8:
        return None
    var = max(covariance[0], covariance[7])
    if var != var or var < 0.0:
        return None
    return math.sqrt(var)


def amcl_sigma_max(bag, robots):
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return {}
    if not os.path.isdir(bag):
        return {}
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    types = {t.name: get_message(t.type)
             for t in reader.get_all_topics_and_types()}
    wanted = {'/%s/amcl_pose' % r: r for r in robots}
    if not (set(wanted) & set(types)):
        return {}
    worst = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        name = wanted.get(topic)
        if name is None:
            continue
        sigma = pose_sigma(deserialize_message(data, types[topic]).pose.covariance)
        if sigma is None:
            continue
        worst[name] = max(worst.get(name, 0.0), sigma)
    return worst


ROBOT_STREAMS = ('scan', 'tf')

NOMINAL_HZ = {
    'scan': 10.0,
    'tf': 42.0,          # the sum over every publisher on the transform tree
    'fleet_states': 10.0,
}


def sim_clock_map(wall, sim):
    if len(wall) < 2:
        return None

    def to_sim(t):
        i = bisect.bisect_left(wall, t)
        if i <= 0:
            return sim[0] + (t - wall[0])
        if i >= len(wall):
            return sim[-1] + (t - wall[-1])
        w0, w1, s0, s1 = wall[i - 1], wall[i], sim[i - 1], sim[i]
        if w1 == w0:
            return s0
        return s0 + (s1 - s0) * (t - w0) / (w1 - w0)

    return to_sim


def stream_coverage(bag, robots, window=None):
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosgraph_msgs.msg import Clock
    except ImportError:
        return None
    if not os.path.isdir(bag):
        return None
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    names = {t.name for t in reader.get_all_topics_and_types()}
    wanted = {}
    for base in NOMINAL_HZ:
        for candidate in ['/%s' % base] + ['/%s/%s' % (r, base) for r in robots]:
            if candidate in names:
                wanted[candidate] = base
    if not wanted:
        return None
    stamps = {t: [] for t in wanted}
    clock = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in stamps:
            stamps[topic].append(t_ns / 1e9)
        elif topic == '/clock':
            msg = deserialize_message(data, Clock)
            clock.append((t_ns / 1e9, msg.clock.sec + msg.clock.nanosec / 1e9))

    # Measure a simulated run on its own clock. Every rate, duration and gap below is then
    # the one the same stream would show on hardware, where the two clocks coincide.
    clock.sort()
    to_sim = sim_clock_map([w for w, _ in clock], [t for _, t in clock])
    if to_sim is not None:
        stamps = {t: [to_sim(x) for x in v] for t, v in stamps.items()}
        if window:
            window = (to_sim(window[0]), to_sim(window[1]))

    out = {}
    for topic, base in wanted.items():
        ts = sorted(stamps[topic])
        if window:
            start, end = window
            ts = [t for t in ts if start <= t <= end]
            duration = end - start
        else:
            start = ts[0] if ts else 0.0
            end = ts[-1] if ts else 0.0
            duration = end - start
        if not ts or duration <= 0:
            out[topic] = {'messages': len(ts), 'duration_s': round(max(duration, 0.0), 2),
                          'rate_hz': 0.0, 'delivered_pct': 0.0,
                          'head_gap_s': round(max(duration, 0.0), 3),
                          'max_gap_s': round(max(duration, 0.0), 3),
                          'tail_gap_s': round(max(duration, 0.0), 3)}
            continue
        rate = len(ts) / duration
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        out[topic] = {
            'messages': len(ts),
            'duration_s': round(duration, 2),
            'rate_hz': round(rate, 2),
            'delivered_pct': round(100.0 * rate / NOMINAL_HZ[base], 1),
            'head_gap_s': round(ts[0] - start, 3),
            'max_gap_s': round(max(gaps), 3) if gaps else 0.0,
            'tail_gap_s': round(end - ts[-1], 3),
        }
    return out


def check_repeat(args):
    repeat_dir = args.repeat_dir
    gt_path = args.ground_truth or os.path.join(repeat_dir,
                                                'ground_truth_poses.jsonl')
    if args.no_ground_truth:
        times, present, travelled = [], {}, {}
    elif not os.path.exists(gt_path):
        print('CANNOT CHECK: no ground truth at %s' % gt_path)
        print('  The ground-truth metrics (F7 and the localisation gate) are '
              'unavailable for this repeat. F4b is NOT among them -- it reads '
              '/fleet_states. Fix the tracker before '
              'collecting more; this is not a repeat failure, so it exits 2.')
        return 2
    else:
        times, present, travelled = load_ground_truth(gt_path)
        if len(times) < 2:
            print('CANNOT CHECK: ground truth has %d sample(s)' % len(times))
            return 2

    run_dir = os.path.dirname(os.path.abspath(repeat_dir))
    if not args.attempt:
        print('CANNOT CHECK: --attempt is required; it names the '
              'api_state_attempt<N>.jsonl that holds this attempt\'s dispatches, which '
              'the repeat directory name does not give')
        return 2
    dispatched = robots_given_work(run_dir, args.attempt)
    robots = args.robots or sorted(set(present) | dispatched)
    battery = battery_minima(os.path.join(repeat_dir, 'bag'), robots)

    if times:
        duration = times[-1] - times[0]
        print('%s: %d samples over %.1f s (%.1f Hz)'
              % (repeat_dir, len(times), duration,
                 len(times) / duration if duration > 0 else 0.0))
    else:
        duration = 0.0
        print('%s: no external tracker, so coverage, gap and travelled distance '
              'are not measured' % repeat_dir)
    print('dispatched by RMF: %s'
          % (', '.join(sorted(dispatched)) if dispatched
             else 'unknown (no api_state capture found)'))
    header = ('%-12s %9s %9s %10s %8s %s'
              % ('robot', 'coverage', 'max gap', 'travelled', 'min soc', 'verdict'))
    print(header)
    print('-' * len(header))

    failures = []        # the repeat is not usable
    unavailable = []     # the check could not be made, which is not the same thing
    for name in robots:
        soc = battery.get(name)
        was_dispatched = name in dispatched or not dispatched

        reasons = []
        if times:
            stamps = present.get(name, [])
            coverage = len(stamps) / len(times)
            gap = max((stamps[i] - stamps[i - 1] for i in range(1, len(stamps))),
                      default=duration)
            if not stamps:
                gap = duration
            moved = travelled.get(name, 0.0)
            if coverage < args.min_coverage:
                reasons.append('coverage %.2f < %.2f' % (coverage, args.min_coverage))
            if gap > args.max_gap:
                reasons.append('gap %.1fs > %.1fs' % (gap, args.max_gap))
            if was_dispatched and moved < args.min_travel:
                reasons.append('travelled %.2fm < %.2fm' % (moved, args.min_travel))
            tracked = '%8.2f%% %8.1fs %9.2fm' % (100.0 * coverage, gap, moved)
        else:
            tracked = '%9s %9s %10s' % ('-', '-', '-')
        if (was_dispatched and soc is not None
                and soc < args.recharge_threshold):
            reasons.append('soc %.2f crossed recharge_threshold %.2f'
                           % (soc, args.recharge_threshold))
        # A robot RMF never dispatched is reported only and cannot fail the repeat.
        if reasons and was_dispatched:
            failures.append((name, reasons))
        print('%-12s %s %8s %s'
              % (name, tracked,
                 '%.2f' % soc if soc is not None else '-',
                 'ok' if not reasons else '; '.join(reasons)
                 + ('' if was_dispatched else ' (not dispatched, not fatal)')))

    sigma = amcl_sigma_max(os.path.join(repeat_dir, 'bag'), robots)
    if sigma:
        print()
        header = '%-12s %14s  %s' % ('robot', 'max amcl sd', 'against reference')
        print(header)
        print('-' * len(header))
        for name in sorted(sigma):
            mark = 'above' if sigma[name] > args.amcl_sigma_reference else 'within'
            print('%-12s %13.2fm  %s' % (name, sigma[name], mark))
        print('A robot above the reference reported an unusually wide pose spread. '
              'This is a property of the run, not of the recording, so it is reported '
              'and never discards the repeat.')

    window = None
    if args.window_start is not None and args.window_end is not None:
        window = (args.window_start, args.window_end)
    streams = stream_coverage(os.path.join(repeat_dir, 'bag'), robots, window)
    if streams is None:
        print()
        print('recorded streams: the bag could not be read, so no stream was measured.')
        unavailable.append(('bag', ['unreadable or absent, so the streams are unmeasured']))
    elif not streams:
        print()
        print('recorded streams: the bag holds none of the streams in NOMINAL_HZ.')
        unavailable.append(('bag', ['holds none of the expected robot streams']))
    else:
        print()
        if window is None:
            print('No workload window was given, so each stream is measured between its '
                  'own first and last message and an absent head or tail cannot show.')
            unavailable.append(('workload window',
                                ['not given, so stream coverage is measured against the '
                                 'data itself']))
        expected = sorted('/%s/%s' % (r, base) for r in robots for base in ROBOT_STREAMS)
        absent = [t for t in expected if t not in streams]
        if absent:
            unavailable.append(('recorded streams',
                                ['%s was never recorded' % ', '.join(absent)]))
        header = '%-26s %9s %9s %11s %9s %9s %9s' % (
            'recorded stream', 'messages', 'rate', 'delivered', 'head gap', 'max gap',
            'tail gap')
        print(header)
        print('-' * len(header))
        for topic in sorted(streams):
            v = streams[topic]
            print('%-26s %9d %8.2fHz %10.1f%% %8.3fs %8.3fs %8.3fs' % (
                topic, v['messages'], v['rate_hz'], v['delivered_pct'],
                v['head_gap_s'], v['max_gap_s'], v['tail_gap_s']))
            if v['delivered_pct'] < 100.0 * args.min_stream_delivered:
                failures.append((topic, ['delivered %.1f%% of its nominal rate'
                                         % v['delivered_pct']]))
            worst = max(v['head_gap_s'], v['max_gap_s'], v['tail_gap_s'])
            if worst > args.max_stream_gap_s:
                failures.append((topic, ['absent for %.1fs (head %.1f, middle %.1f, '
                                         'tail %.1f), over the %.1fs allowed'
                                         % (worst, v['head_gap_s'], v['max_gap_s'],
                                            v['tail_gap_s'], args.max_stream_gap_s)]))

    print()
    for name, reasons in unavailable:
        print('UNMEASURED: %s: %s' % (name, ', '.join(reasons)))
    if args.report_only and (failures or unavailable):
        print('REPORT ONLY: %d failure(s) and %d unmeasured check(s); keeping the repeat '
              'because --report-only was given' % (len(failures), len(unavailable)))
        return 0
    if unavailable and not failures:
        print('DISCARD THIS REPEAT: the checks above could not be made, so the recording '
              'cannot be shown to be complete.')
        return 1
    if failures:
        print('DISCARD THIS REPEAT: %s'
              % '; '.join('%s (%s)' % (n, ', '.join(r)) for n, r in failures))
        return 1
    if times:
        print('KEEP: every dispatched robot is tracked, moved, and stayed above the '
              'recharge threshold')
    else:
        print('KEEP: every dispatched robot stayed above the recharge threshold and '
              'every recorded stream arrived; it was not tracked')
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repeat-dir', help='Repeat directory holding the bag and the ground truth')
    ap.add_argument('--ground-truth',
                    help='defaults to ground_truth_poses.jsonl in --repeat-dir')
    ap.add_argument('--robots', nargs='*', help='Robot names, space separated')
    ap.add_argument('--attempt',
                    help='which api_state_attempt<N>.jsonl holds this attempt\'s '
                         'dispatches. Required with --repeat-dir: the repeat directory '
                         'is renamed to its position in the run, so its name does not '
                         'give the attempt')
    ap.add_argument('--min-coverage', type=float, default=0.80,
                    help='Fraction of the repeat the tracker must have seen a robot for')
    ap.add_argument('--max-gap', type=float, default=2.0,
                    help='Longest stretch, in seconds, the tracker may lose a robot for')
    ap.add_argument('--min-travel', type=float, default=0.5,
                    help='Metres a dispatched robot must have moved')
    ap.add_argument('--min-stream-delivered', type=float, default=0.70,
                    help='fraction of a stream\'s nominal rate that must reach the '
                         'bag. Below this the metrics computed from it describe the '
                         'network rather than the robot (default %(default)s)')
    ap.add_argument('--window-start', type=float,
                    help='epoch seconds when the workload was submitted, measured by the '
                         'caller. With --window-end it becomes the denominator for every '
                         'stream rate and the span the gaps are measured against')
    ap.add_argument('--window-end', type=float,
                    help='epoch seconds when recording stopped being expected')
    ap.add_argument('--max-stream-gap-s', type=float, default=5.0,
                    help='seconds a recorded stream may be absent, at the start, in the '
                         'middle or at the end of the window. Separates the gaps observed '
                         'in the accepted repeats to 2026-10-15 from everything else and '
                         'is not a verified failure threshold (default %(default)s)')
    ap.add_argument('--recharge-threshold', type=float, default=0.10,
                    help='must match the fleet config; a robot below this is '
                         'diverted to a charger by RMF')
    ap.add_argument('--amcl-sigma-reference', type=float, default=1.6,
                    help='metres of positional standard deviation the report compares '
                         'each robot against. Measured over the 60 accepted N=1 '
                         'baseline repeats, where the per-repeat maximum spans 0.556 to '
                         '1.581 m; the default sits just above that range. No failing '
                         'run has been measured yet, so the value separates the observed '
                         'range from everything else and is not a verified failure '
                         'threshold (default %(default)s)')
    ap.add_argument('--no-ground-truth', action='store_true',
                    help='collect without an external tracker. The pose comes from '
                         'AMCL, so F7 and the localisation gate are unavailable; the '
                         'battery and stream-delivery checks still gate the repeat.')
    ap.add_argument('--report-only', action='store_true',
                    help='print the verdict but always exit 0. Use while the '
                         'tracker is being commissioned, never for a '
                         'collection whose numbers will be reported.')
    args = ap.parse_args()

    if not args.repeat_dir:
        ap.error('give --repeat-dir')

    return check_repeat(args)


if __name__ == '__main__':
    sys.exit(main())
