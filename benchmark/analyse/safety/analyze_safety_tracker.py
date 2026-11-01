#!/usr/bin/env python3
"""Computes 7a and 7b as the F7 requirement defines them, and checks them where a
position reference exists.

  as specified   /fleet_states RobotState.location, the requirement's data source: the
                 pose RMF held, after AMCL. Available on both platforms.
  ground truth   /world/<world>/dynamic_pose/info, the simulator's physics state,
                 recorded by collect/simulation/record_ground_truth_poses.py.
                 Simulation only; absent on hardware.

7a is the minimum over time and over robot pairs of the Euclidean distance between the
robots' reference points, with the minimum per pair beside it. 7b counts events: a
contiguous run below a threshold for one pair is one event, with the number of samples
below it reported beside the count. Both are computed at D_SAFE_M and
D_RMF_CONFLICT_M, derived from the fleet profile.

Where ground truth exists, the same figures are computed from it, and the as-specified
events at D_SAFE_M are matched against the true ones by overlap in time: detected,
missed, false alarm. Ground truth is downsampled to 10 Hz by the recorder, the rate
/fleet_states arrives at. METRIC_DESIGN.md defines the measurands.

Usage:
    python3 analyze_safety_tracker.py --repeat-dir <repeat> --scenario <label> --output <json>
"""
import argparse
import itertools
import json
import math
import os

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

FOOTPRINT_M = 0.178
D_SAFE_M = 2 * FOOTPRINT_M          # 0.356 m -- bodies at risk of touching
D_RMF_CONFLICT_M = 0.178 + 0.45     # 0.628 m -- RMF's own footprint-vs-vicinity
THRESHOLDS_M = {'d_safe_body_contact': D_SAFE_M, 'rmf_conflict_separation': round(D_RMF_CONFLICT_M, 3)}
ANALYSIS_VERSION = 2


def load_ground_truth(path):
    """Return {timestamp_s: {robot: (x, y)}} from a ground_truth_poses.jsonl file."""
    frames = {}
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        frames[o['sim_t']] = {k: (v['x'], v['y']) for k, v in o['robots'].items()}
    return frames


def load_rmf_observed(bag):
    """{timestamp_s: {robot: (x, y)}} from /fleet_states, for frames holding two robots or more."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    tm = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in tm.items()}
    frames = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != '/fleet_states':
            continue
        m = deserialize_message(data, cls[topic])
        robots, stamps = {}, []
        for rb in m.robots:
            sim = rb.location.t.sec + rb.location.t.nanosec / 1e9
            if sim <= 0:
                continue
            robots[rb.name] = (rb.location.x, rb.location.y)
            stamps.append(sim)
        if len(robots) >= 2:
            frames[sum(stamps) / len(stamps)] = robots
    return frames


def min_distance(frames):
    """Return the smallest inter-robot distance across the frames, with the instant and
    the pair it occurred at."""
    best = (float('inf'), None, None)
    for t, robots in frames.items():
        for a, b in itertools.combinations(sorted(robots), 2):
            d = math.hypot(robots[a][0] - robots[b][0], robots[a][1] - robots[b][1])
            if d < best[0]:
                best = (d, t, (a, b))
    return {'min_distance_m': None if best[1] is None else round(best[0], 4),
            'at_sim_t': best[1], 'pair': list(best[2]) if best[2] else None}


def episodes(frames, thresh):
    """Contiguous runs below thresh, per pair. Returns [(pair, t_start, t_end, d_min)]."""
    out = []
    open_ep = {}
    for t in sorted(frames):
        robots = frames[t]
        pairs_now = set()
        for a, b in itertools.combinations(sorted(robots), 2):
            d = math.hypot(robots[a][0] - robots[b][0], robots[a][1] - robots[b][1])
            key = (a, b)
            if d < thresh:
                pairs_now.add(key)
                if key not in open_ep:
                    open_ep[key] = [t, t, d]
                else:
                    open_ep[key][1] = t
                    open_ep[key][2] = min(open_ep[key][2], d)
        for key in list(open_ep):
            if key not in pairs_now:
                s, e, dmin = open_ep.pop(key)
                out.append((key, s, e, dmin))
    for key, (s, e, dmin) in open_ep.items():
        out.append((key, s, e, dmin))
    return sorted(out, key=lambda x: x[1])


def min_distance_per_pair(frames):
    """{'a+b': smallest distance} over the frames."""
    out = {}
    for robots in frames.values():
        for a, b in itertools.combinations(sorted(robots), 2):
            d = math.hypot(robots[a][0] - robots[b][0], robots[a][1] - robots[b][1])
            key = '%s+%s' % (a, b)
            if key not in out or d < out[key]:
                out[key] = d
    return {k: round(v, 4) for k, v in sorted(out.items())}


def below_threshold(frames, thresh):
    """Events, samples below thresh, and the rate the sample count depends on."""
    samples = 0
    for robots in frames.values():
        for a, b in itertools.combinations(sorted(robots), 2):
            if math.hypot(robots[a][0] - robots[b][0], robots[a][1] - robots[b][1]) < thresh:
                samples += 1
    times = sorted(frames)
    rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else None
    return {'events': len(episodes(frames, thresh)), 'samples_below': samples,
            'frame_rate_hz': round(rate, 2) if rate else None}


def summarise(frames):
    """7a and 7b from one position source."""
    return {'n_frames': len(frames),
            '7a_min_distance': min_distance(frames),
            '7a_min_distance_per_pair_m': min_distance_per_pair(frames),
            '7b_below_threshold': {name: below_threshold(frames, t) for name, t in THRESHOLDS_M.items()}}


def overlaps(e1, e2):
    """Return whether two episodes concern the same robot pair and overlap in time."""
    return e1[0] == e2[0] and e1[1] <= e2[2] and e2[1] <= e1[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repeat-dir', required=True, help='Repeat directory holding the bag and the ground truth')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    gt_path = os.path.join(args.repeat_dir, 'ground_truth_poses.jsonl')
    bag = os.path.join(args.repeat_dir, 'bag')
    obs = load_rmf_observed(bag)
    gt = load_ground_truth(gt_path) if os.path.exists(gt_path) else None

    def fmt(eps):
        """Episodes as JSON records."""
        return [{'pair': list(p), 't_start': round(s, 3), 't_end': round(e, 3),
                 'min_distance_m': round(d, 4)} for p, s, e, d in eps]

    result = {
        'scenario': args.scenario,
        'repeat_dir': args.repeat_dir,
        'analysis_version': ANALYSIS_VERSION,
        'thresholds_m': THRESHOLDS_M,
        'threshold_status': 'derived from the fleet profile; the requirement leaves X and d_safe unset',
        'sources': {
            'as_specified': '/fleet_states RobotState.location (the pose RMF held, post-AMCL)',
            'ground_truth': ('Gazebo /world/*/dynamic_pose/info (physical state)' if gt is not None
                             else 'unavailable: no ground_truth_poses.jsonl in this repeat'),
        },
        'as_specified': summarise(obs),
        'ground_truth': summarise(gt) if gt is not None else None,
        # Read by show_collection_summary.py and show_metric_trace.py.
        '7a_min_distance': {
            'rmf_observed': min_distance(obs),
            'ground_truth': min_distance(gt) if gt is not None else None,
        },
        '7b_episodes': None,
    }
    if gt is not None:
        gt_eps = episodes(gt, D_SAFE_M)
        obs_eps = episodes(obs, D_SAFE_M)
        detected, missed = [], []
        for e in gt_eps:
            (detected if any(overlaps(e, o) for o in obs_eps) else missed).append(e)
        false_alarm = [o for o in obs_eps if not any(overlaps(o, e) for e in gt_eps)]
        result['7b_episodes'] = {
            'threshold_m': D_SAFE_M,
            'really_happened_and_rmf_saw_it': len(detected),
            'really_happened_but_rmf_MISSED_it': len(missed),
            'rmf_saw_it_but_did_not_really_happen': len(false_alarm),
            'detail': {'detected': fmt(detected), 'missed': fmt(missed),
                       'false_alarm': fmt(false_alarm)},
        }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    for source in ('as_specified', 'ground_truth'):
        block = result[source]
        if block is None:
            print('%-13s unavailable' % source)
            continue
        a7 = block['7a_min_distance']
        print('%-13s 7a %.3f m %s   7b events/samples: %s' % (
            source, a7['min_distance_m'], '+'.join(a7['pair'] or []),
            '  '.join('%s %d/%d' % (n, v['events'], v['samples_below'])
                      for n, v in block['7b_below_threshold'].items())))
    if result['7b_episodes']:
        ep = result['7b_episodes']
        print('at %.3f m: detected %d, missed %d, false alarm %d' % (
            D_SAFE_M, ep['really_happened_and_rmf_saw_it'],
            ep['really_happened_but_rmf_MISSED_it'], ep['rmf_saw_it_but_did_not_really_happen']))


if __name__ == '__main__':
    main()
