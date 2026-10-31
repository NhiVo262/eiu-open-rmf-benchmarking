#!/usr/bin/env python3
import argparse
import bisect
import glob
import json
import math
import os

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

REAL_MOTION_MPS = 1.0    
LOC_BAD_M = 0.30          

def load_gt(path):
    """Return {robot: [(timestamp_s, x, y)]} from a repeat's ground_truth_poses.jsonl."""
    frames = {}
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        frames[o['sim_t']] = {k: (v['x'], v['y']) for k, v in o['robots'].items()}
    return frames


def read_bag(bag):
    """Return {topic: [(timestamp_s, message)]} for the topics this record needs."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    tm = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in tm.items()}
    fs, clock = [], []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic == '/fleet_states':
            m = deserialize_message(data, cls[topic])
            robots, stamps = {}, []
            for rb in m.robots:
                s = rb.location.t.sec + rb.location.t.nanosec / 1e9
                if s > 0:
                    robots[rb.name] = (rb.location.x, rb.location.y)
                    stamps.append(s)
            if robots:
                fs.append((sum(stamps) / len(stamps), robots))
        elif topic == '/clock':
            m = deserialize_message(data, cls[topic])
            clock.append((t_ns / 1e9, m.clock.sec + m.clock.nanosec * 1e-9))
    fs.sort(key=lambda x: x[0])
    clock.sort(key=lambda x: x[0])
    return set(tm), fs, clock


def assess(repeat_dir, run_dir):
    bag = os.path.join(repeat_dir, 'bag')
    gt_path = os.path.join(repeat_dir, 'ground_truth_poses.jsonl')
    rec = {'repeat_dir': repeat_dir}

    topics, fs, clock = read_bag(bag)
    gt = load_gt(gt_path) if os.path.exists(gt_path) else {}
    gts = sorted(gt)

    api = sorted(glob.glob(os.path.join(run_dir, 'api_state*.jsonl'))) if run_dir else []
    rec['streams_present'] = {
        'ground_truth': bool(gt),
        'fleet_states': '/fleet_states' in topics,
        'bid_response': '/rmf_task/bid_response' in topics,
        'clock': bool(clock),
        'itinerary_delay': '/rmf_traffic/itinerary_delay' in topics,
        'mutex_group_states': '/mutex_group_states' in topics,
        'api_state': bool(api),
        'negotiation_statuses': '/rmf_traffic/negotiation_statuses' in topics,
    }

    # --- physics: did the body actually move impossibly fast? ---
    per_gt = {}
    for t in gts:
        for nm, p in gt[t].items():
            per_gt.setdefault(nm, []).append((t, p))
    worst, path_len = 0.0, {}
    for nm, seq in per_gt.items():
        d_tot = 0.0
        for i in range(1, len(seq)):
            dt = seq[i][0] - seq[i - 1][0]
            d = math.hypot(seq[i][1][0] - seq[i - 1][1][0], seq[i][1][1] - seq[i - 1][1][1])
            d_tot += d
            if dt > 0:
                worst = max(worst, d / dt)
        path_len[nm] = round(d_tot, 2)
    rec['physics_anomaly'] = bool(gt) and worst >= REAL_MOTION_MPS
    rec['max_ground_truth_speed_mps'] = round(worst, 3) if gt else None
    rec['route_completion_path_m'] = path_len

    # --- localisation quality: AMCL vs ground truth ---
    if gt and fs:
        errs, bad_time = [], 0.0
        prev_t = None
        for t, robots in fs:
            i = bisect.bisect_left(gts, t)
            cand = [k for k in (i - 1, i) if 0 <= k < len(gts)]
            if not cand:
                continue
            k = min(cand, key=lambda k: abs(gts[k] - t))
            if abs(gts[k] - t) > 0.2:
                continue
            frame_worst = 0.0
            for nm, p in robots.items():
                g = gt[gts[k]].get(nm)
                if g:
                    e = math.hypot(p[0] - g[0], p[1] - g[1])
                    errs.append(e)
                    frame_worst = max(frame_worst, e)
            if prev_t is not None and frame_worst > LOC_BAD_M:
                bad_time += t - prev_t
            prev_t = t
        if errs:
            s = sorted(errs)
            rec['localization_error'] = {
                'median_m': round(s[len(s) // 2], 3),
                'p95_m': round(s[int(len(s) * 0.95)], 3),
                'max_m': round(s[-1], 3),
                'seconds_above_%.2fm' % LOC_BAD_M: round(bad_time, 1),
            }
    rec.setdefault('localization_error', None)

    # --- per-metric validity ---
    sp = rec['streams_present']
    rec['valid_for'] = {
        'F7_safety': sp['ground_truth'] and not rec['physics_anomaly'],

        'F4b_utilization': sp['fleet_states'] and not rec['physics_anomaly'],
        'F1_makespan': not rec['physics_anomaly'],
        'F1_t_arrival': not rec['physics_anomaly'],
        'F2_schedule': sp['itinerary_delay'],
        'F3_negotiation': True,
        'F4a_task_state': sp['api_state'],

        'F5_fidelity': sp['bid_response'] and sp['api_state'],
        'F6_congestion': sp['mutex_group_states'],
    }
    rec['note'] = ('valid_for reflects ONLY physics and stream availability. '
                   'localization_error is a covariate to report alongside any '
                   'metric that measures an RMF decision, not a rejection test.')
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir', required=True, help='Run directory holding the repeat_* directories')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    ap.add_argument('--include-discarded', action='store_true', help='Also assess the attempts that were discarded')
    args = ap.parse_args()

    dirs = sorted(glob.glob(os.path.join(args.run_dir, 'repeat_*')))
    if args.include_discarded:
        dirs += sorted(glob.glob(os.path.join(args.run_dir, 'discarded_physics_glitch_attempt*')))

    out = []
    for d in dirs:
        if not os.path.isdir(os.path.join(d, 'bag')):
            continue
        try:
            out.append(assess(d, args.run_dir))
        except Exception as e:
            out.append({'repeat_dir': d, 'error': str(e)})

    json.dump({'run_dir': args.run_dir, 'repeats': out}, open(args.output, 'w'),
              indent=2, default=str)

    print('%-52s %-8s %-12s %s' % ('repeat', 'physics', 'p95/max', 'valid_for'))
    for r in out:
        if 'error' in r:
            print('%-52s ERROR %s' % (os.path.basename(r['repeat_dir']), r['error'][:40]))
            continue
        loc = r['localization_error']
        ok = [k.split('_')[0] for k, v in r['valid_for'].items() if v]
        print('%-52s %-8s %-12s %s' % (
            os.path.basename(r['repeat_dir']),
            'ANOMALY' if r['physics_anomaly'] else 'ok',
            ('%.2f/%.2f' % (loc['p95_m'], loc['max_m'])) if loc else 'n/a',
            ','.join(sorted(set(ok))) or 'NONE'))


if __name__ == '__main__':
    main()
