#!/usr/bin/env python3
"""Measures 7c, the separation between two robots' laser centres, from /scan alone.

7c does not pass through AMCL, the map or an external tracker, so it is computable on
both platforms.

The scan plane lies above the Burger's body, so the part of the other robot a laser
returns is that robot's LDS head, and the separation is measured between laser centres.
A mutual range at the lidar's 0.120 m floor is reported as censored rather than as a
number. Coverage is the fraction of instants at which both lasers returned each other.

Validated against Gazebo ground truth by analyse/validity/analyze_lidar_attribution.py.
METRIC_DESIGN.md section 7c defines the measurand.

Usage:
    python3 analyze_safety_lidar.py --bag <repeat>/bag --scenario <label> --output <json>
    python3 analyze_safety_lidar.py --self-test
"""
import argparse
import json
import math
import os

ANALYSIS_VERSION = 2

LIDAR_MIN_RANGE_M = 0.120
LIDAR_MAX_RANGE_M = 3.5
LIDAR_NOISE_SD_M = 0.01
LASER_FRAME = 'base_scan'      # frame_id of the LDS scan, tb3_robot1.sdf and the hardware driver
LASER_BEHIND_ORIGIN_M = 0.032  # laser centre behind the model origin, tb3_robot1.sdf
HEAD_NEAR_M = 0.029            # laser centre to the LDS head's outline at the scan plane, lds.stl
HEAD_FAR_M = 0.034
HEAD_DIAMETER_M = 0.064
MUTUAL_TOL_M = HEAD_FAR_M - HEAD_NEAR_M + 4 * LIDAR_NOISE_SD_M   # 0.045


MIN_CHORD_M = 0.02
MAX_CHORD_M = 0.12
EPISODE_GAP_S = 1.0            # matched instants closer than this belong to one episode


def read_scans(bag, robots):
    """{robot: [(t, angle_min, angle_increment, [ranges]), ...]} from the bag.

    Keeps only messages in LASER_FRAME, the frame the robot's own LDS publishes in, so
    another publisher on the same topic is not paired as that robot's laser.
    """
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    types = {t.name: get_message(t.type)
             for t in reader.get_all_topics_and_types()}
    wanted = {}
    for robot in robots:
        for candidate in ('/%s/scan' % robot, '/scan'):
            if candidate in types:
                wanted[candidate] = robot
                break
    out = {r: [] for r in robots}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        robot = wanted.get(topic)
        if robot is None:
            continue
        msg = deserialize_message(data, types[topic])
        if msg.header.frame_id != LASER_FRAME:
            continue
        out[robot].append((t_ns / 1e9, msg.angle_min, msg.angle_increment,
                           list(msg.ranges)))
    return out


def clusters(angle_min, angle_increment, ranges):
    """Isolated, robot-sized clusters in one scan."""
    valid = [(i, r) for i, r in enumerate(ranges)
             if r is not None and not math.isnan(r)
             and LIDAR_MIN_RANGE_M <= r <= LIDAR_MAX_RANGE_M]
    if not valid:
        return []
    out = []
    run = [valid[0]]
    for (i, r), (pi, pr) in zip(valid[1:], valid[:-1]):
        step = max(0.05, 0.08 * pr)
        if i == pi + 1 and abs(r - pr) <= step:
            run.append((i, r))
        else:
            out.append(run)
            run = [(i, r)]
    out.append(run)

    if len(out) > 1:
        first, last = out[0], out[-1]
        n = len(ranges)
        if (first[0][0] == 0 and last[-1][0] == n - 1
                and abs(first[0][1] - last[-1][1]) <= max(0.05, 0.08 * last[-1][1])):
            out[0] = last + first
            out.pop()

    kept = []
    for run in out:
        if len(run) < 2:
            continue
        rs = [r for _, r in run]
        mean_r = sum(rs) / len(rs)
        width = (len(run) - 1) * abs(angle_increment)
        chord = 2.0 * mean_r * math.sin(width / 2.0) if width > 0 else 0.0
        if not (MIN_CHORD_M <= chord <= MAX_CHORD_M):
            continue
        sin_sum = sum(math.sin(angle_min + i * angle_increment) for i, _ in run)
        cos_sum = sum(math.cos(angle_min + i * angle_increment) for i, _ in run)
        kept.append({'mean_range_m': mean_r,
                     'min_range_m': min(rs),
                     'beams': len(run),
                     'chord_m': chord,
                     'bearing_rad': math.atan2(sin_sum, cos_sum)})
    return kept


def pair_by_time(a_scans, b_scans, max_dt):
    """(t, scan_a, scan_b) for scans close enough in time to be one instant."""
    out = []
    j = 0
    for ta, *a in a_scans:
        while j + 1 < len(b_scans) and abs(b_scans[j + 1][0] - ta) < abs(b_scans[j][0] - ta):
            j += 1
        if not b_scans:
            break
        tb = b_scans[j][0]
        if abs(tb - ta) <= max_dt:
            out.append((ta, (ta, *a), b_scans[j]))
    return out


def mutual_range(scan_a, scan_b):
    """Smallest range on which the two scans agree, or None."""
    ca = clusters(*scan_a[1:])
    cb = clusters(*scan_b[1:])
    best = None
    for x in ca:
        for y in cb:
            if abs(x['mean_range_m'] - y['mean_range_m']) <= MUTUAL_TOL_M:
                r = min(x['min_range_m'], y['min_range_m'])
                if best is None or r < best[0]:
                    best = (r, x, y)
    return best


def count_episodes(times, gap_s=EPISODE_GAP_S):
    """Number of runs in a sorted list of instants, split where two lie gap_s apart."""
    return sum(1 for i, t in enumerate(times) if i == 0 or t - times[i - 1] > gap_s)


def analyse_pair(name_a, a_scans, name_b, b_scans, d_safe_m, max_dt):
    """7c for one robot pair: coverage, minimum separation interval and below-d_safe counts."""
    paired = pair_by_time(a_scans, b_scans, max_dt)
    matched = []
    for t, sa, sb in paired:
        m = mutual_range(sa, sb)
        if m is not None:
            matched.append((t, m[0]))

    at_floor = [t for t, r in matched if r <= LIDAR_MIN_RANGE_M + LIDAR_NOISE_SD_M]
    result = {
        'pair': [name_a, name_b],
        'scan_instants_compared': len(paired),
        'instants_with_a_mutual_match': len(matched),
        'coverage_pct': round(100.0 * len(matched) / len(paired), 1) if paired else None,
        'n_at_sensor_floor': len(at_floor),
        'contact_suspected': bool(at_floor),
    }
    if matched:
        r_min = min(r for _, r in matched)
        result['min_mutual_range_m'] = round(r_min, 4)
        # D in [r + near, r + far]; report the interval, not a false point value
        result['min_separation_lower_m'] = round(r_min + HEAD_NEAR_M, 4)
        result['min_separation_upper_m'] = round(r_min + HEAD_FAR_M, 4)
        below = sorted(t for t, r in matched if r + HEAD_NEAR_M < d_safe_m)
        result['instants_below_d_safe'] = len(below)
        result['episodes_below_d_safe'] = count_episodes(below)
    return result


def self_test():
    """Recover known separations from synthetic scans; returns a shell exit code."""
    print('%10s %14s %18s %16s %s' % ('D set', 'range seen', 'D recovered',
                                      'contains truth', 'contact flag'))
    print('-' * 78)
    ok = True
    for d in (1.50, 0.80, 0.40, 0.25, 0.18, 0.14):
        n = 360
        inc = 2.0 * math.pi / n
        r = d - 0.031
        half = max(1, int(math.atan2(HEAD_DIAMETER_M / 2, max(r, 0.12)) / inc))
        wall_half = int(math.atan2(0.5, r) / inc)
        def make(centre):
            """A synthetic scan with a head-sized cluster at one index and a wall opposite."""
            ranges = [float('inf')] * n
            for k in range(centre - half, centre + half + 1):
                ranges[k % n] = r if r >= LIDAR_MIN_RANGE_M else float('inf')
            for k in range(centre + n // 2 - wall_half, centre + n // 2 + wall_half + 1):
                ranges[k % n] = r - 0.02
            return (0.0, -math.pi, inc, ranges)
        # A sees B mid-array, B sees A across the seam: both paths are exercised.
        sa, sb = make(n // 2), make(0)
        res = analyse_pair('A', [sa], 'B', [sb], d_safe_m=0.356, max_dt=0.1)
        seen = res.get('min_mutual_range_m')
        if seen is None:
            below_floor = r < LIDAR_MIN_RANGE_M
            print('%9.3fm %14s %18s %16s %s' % (
                d, 'below floor' if below_floor else 'NOT SEEN', '-', '-',
                'n/a' if below_floor else 'MISS'))
            ok = ok and below_floor
            continue
        lo, hi = res['min_separation_lower_m'], res['min_separation_upper_m']
        contains = lo - 1e-9 <= d <= hi + 1e-9
        ok = ok and contains
        print('%9.3fm %13.3fm %9.3f-%.3fm %16s %s' % (
            d, seen, lo, hi, 'yes' if contains else 'NO', res['contact_suspected']))
    episodes = count_episodes([0.0, 0.2, 0.4, 3.0, 3.2, 9.0])
    print('episode split of 3 runs: %d' % episodes)
    ok = ok and episodes == 3
    print()
    print('self-test %s' % ('PASSED' if ok else 'FAILED'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', help='Path to the rosbag2 directory of one repeat')
    ap.add_argument('--robots', nargs='+',
                    help='The two robot names whose scans are compared',
                    default=['tb3_robot1', 'tb3_robot2', 'tb3_robot3'])
    ap.add_argument('--scenario', default='', help='Scenario label recorded in the output')
    ap.add_argument('--d-safe-m', type=float, default=0.356,
                    help='near-collision separation, twice the 0.178 m footprint radius '
                         '(default %(default)s)')
    ap.add_argument('--max-dt', type=float, default=0.15,
                    help='two scans closer than this count as one instant; the '
                         'lasers run at 5 Hz and are not synchronised')
    ap.add_argument('--output', help='Where to write the metrics JSON')
    ap.add_argument('--self-test', action='store_true',
                    help='run the synthetic check and exit')
    args = ap.parse_args()

    if args.self_test:
        raise SystemExit(self_test())
    if not args.bag:
        ap.error('--bag is required unless --self-test is given')

    scans = read_scans(args.bag, args.robots)
    present = [r for r in args.robots if scans.get(r)]
    if len(present) < 2:
        raise SystemExit(
            '7c needs /scan from at least two robots; found %s in %s. With one '
            'robot there is no pair and 7c is undefined, not zero.'
            % (present or 'none', args.bag))

    pairs = []
    for i, a in enumerate(present):
        for b in present[i + 1:]:
            pairs.append(analyse_pair(a, scans[a], b, scans[b],
                                      args.d_safe_m, args.max_dt))

    contact = [p for p in pairs if p['contact_suspected']]
    out = {
        'scenario': args.scenario,
        'bag': args.bag,
        'analysis_version': ANALYSIS_VERSION,
        'method': ('mutual laser range to the other robot\'s LDS head: a %.2f-%.2f m '
                   'cluster in each scan, the two agreeing in range within %.3f m; '
                   'laser-centre separation = range + [%.3f, %.3f]. No pose, no map, '
                   'no tracker.' % (MIN_CHORD_M, MAX_CHORD_M, MUTUAL_TOL_M,
                                    HEAD_NEAR_M, HEAD_FAR_M)),
        'd_safe_m': args.d_safe_m,
        'sensor_floor_m': LIDAR_MIN_RANGE_M,
        'smallest_resolvable_separation_m': round(LIDAR_MIN_RANGE_M + HEAD_NEAR_M, 4),
        '7c_contact_suspected': bool(contact),
        '7c_pairs_with_contact': [p['pair'] for p in contact],
        'per_pair': pairs,
    }
    if args.output:
        json.dump(out, open(args.output, 'w'), indent=1)

    header = '%-26s %9s %9s %11s %16s %s' % (
        'pair', 'instants', 'matched', 'min range', 'separation', 'contact')
    print(header)
    print('-' * len(header))
    for p in pairs:
        print('%-26s %9d %9d %10s %16s %s' % (
            '+'.join(p['pair']), p['scan_instants_compared'],
            p['instants_with_a_mutual_match'],
            '%.3fm' % p['min_mutual_range_m'] if 'min_mutual_range_m' in p else '-',
            '%.3f-%.3fm' % (p['min_separation_lower_m'], p['min_separation_upper_m'])
            if 'min_mutual_range_m' in p else 'never in view',
            'SUSPECTED' if p['contact_suspected'] else 'no'))
    print()
    if contact:
        print('CONTACT SUSPECTED on %d pair(s): a mutual range sat at the %.3f m '
              'sensor floor, so the laser centres were at most %.3f m apart.'
              % (len(contact), LIDAR_MIN_RANGE_M,
                 LIDAR_MIN_RANGE_M + HEAD_FAR_M))
    else:
        print('No contact suspected: no mutual range reached the sensor floor.')
    print('Coverage is how often both lasers returned each other; a '
          'pair never in view contributes nothing and is not counted as safe.')


if __name__ == '__main__':
    main()
