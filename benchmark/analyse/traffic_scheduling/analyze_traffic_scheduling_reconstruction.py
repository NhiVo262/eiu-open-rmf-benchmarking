#!/usr/bin/env python3
"""Reports whether a repeat's schedule can be reconstructed faithfully enough to
judge Feature 2a, in two stages.

  Stage 1, fidelity    the reconstructed scheduled position against the Gazebo
                       ground-truth position, per participant, at several horizons
  Stage 2, agreement   conflicts detected in the reconstructed schedule using
                       RMF's own separation rule, footprint 0.178 + vicinity 0.45
                       = 0.628 m, against the negotiation_notice events RMF
                       published: matched, missed, extra

Stage 1 is the prerequisite: a reconstruction that does not track reality cannot
judge conflicts, whatever its conflict counts are.

Delay model: an itinerary_set establishes a base itinerary, and each subsequent
itinerary_delay for that participant adds its signed delay to the itinerary in
force; querying a position at time t applies the delays received up to t. This
approximates rmf_traffic's semantics, which is what stage 1 measures.
"""
import argparse
import bisect
import itertools
import json
import math
import os

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

D_CONFLICT_M = 0.178 + 0.45
SAMPLE_DT = 0.2


def hermite(wps, t):
    """Return the position on a cubic Hermite spline through the waypoints at time t, or
    None when t falls outside them."""
    if not wps or t < wps[0][0] or t > wps[-1][0]:
        return None
    lo, hi = 0, len(wps) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if wps[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, x0, y0, vx0, vy0 = wps[lo]
    t1, x1, y1, vx1, vy1 = wps[hi]
    dt = t1 - t0
    if dt <= 0:
        return (x0, y0)
    s = (t - t0) / dt
    h00 = 2 * s ** 3 - 3 * s ** 2 + 1
    h10 = s ** 3 - 2 * s ** 2 + s
    h01 = -2 * s ** 3 + 3 * s ** 2
    h11 = s ** 3 - s ** 2
    return (h00 * x0 + h10 * dt * vx0 + h01 * x1 + h11 * dt * vx1,
            h00 * y0 + h10 * dt * vy0 + h01 * y1 + h11 * dt * vy1)


def read_bag(bag):
    """Return {topic: [(timestamp_s, message)]} for the topics this analysis reads."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    tm = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in tm.items()}
    out = {'sets': [], 'delays': [], 'notices': [], 'names': {}, 'clock': []}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        t = t_ns / 1e9
        if topic not in cls:
            continue
        m = deserialize_message(data, cls[topic])
        if topic == '/rmf_traffic/participants':
            for p in m.participants:
                out['names'][p.id] = p.description.name
        elif topic == '/rmf_traffic/itinerary_set':
            wps = sorted((w.time / 1e9, w.position[0], w.position[1],
                          w.velocity[0], w.velocity[1])
                         for rt in m.itinerary for w in rt.trajectory.waypoints)
            if wps:
                out['sets'].append((t, m.participant, wps))
        elif topic == '/rmf_traffic/itinerary_delay':
            out['delays'].append((t, m.participant, m.delay / 1e9))
        elif topic == '/rmf_traffic/negotiation_notice':
            out['notices'].append((t, m.conflict_version, list(m.participants)))
        elif topic == '/clock':
            out['clock'].append((t, m.clock.sec + m.clock.nanosec * 1e-9))
    for k in ('sets', 'delays', 'notices', 'clock'):
        out[k].sort()
    return out


class Schedule:
    """Reconstructed itinerary per participant, with delays applied."""

    def __init__(self, sets, delays):
        self.sets = {}
        for t, pid, wps in sets:
            self.sets.setdefault(pid, []).append((t, wps))
        self.delays = {}
        for t, pid, d in delays:
            self.delays.setdefault(pid, []).append((t, d))

    def itinerary_at(self, pid, t_wall):
        cand = [(t, w) for t, w in self.sets.get(pid, []) if t <= t_wall]
        if not cand:
            return None, 0.0
        t_set, wps = cand[-1]
        shift = sum(d for t, d in self.delays.get(pid, []) if t_set <= t <= t_wall)
        return wps, shift

    def position_at(self, pid, t_wall, t_sim):
        wps, shift = self.itinerary_at(pid, t_wall)
        if wps is None:
            return None
        return hermite([(w[0] + shift,) + w[1:] for w in wps], t_sim)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repeat-dir', required=True, help='Repeat directory holding the bag and the ground truth')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    bag = os.path.join(args.repeat_dir, 'bag')
    gt_path = os.path.join(args.repeat_dir, 'ground_truth_poses.jsonl')
    d = read_bag(bag)
    names = d['names']
    rev = {v: k for k, v in names.items()}

    gt = {}
    if os.path.exists(gt_path):
        for line in open(gt_path):
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            gt[o['sim_t']] = {k: (v['x'], v['y']) for k, v in o['robots'].items()}
    gts = sorted(gt)

    # wall -> sim, via the nearest /clock sample; the offset drifts by about 1%.
    # A bag without /clock was recorded against the system clock, where the
    # conversion is the identity.
    cw = [c[0] for c in d['clock']]
    cs = [c[1] for c in d['clock']]

    def wall_to_sim(tw):
        if not cw:
            return tw
        i = bisect.bisect_left(cw, tw)
        j = min([k for k in (i - 1, i) if 0 <= k < len(cw)],
                key=lambda k: abs(cw[k] - tw))
        return tw - (cw[j] - cs[j])

    sched = Schedule(d['sets'], d['delays'])

    # ---- stage 1: does the schedule describe where the robot really went ----
    horizons = [0, 5, 10, 20, 40]
    fidelity = {}
    for t_set, pid, wps in d['sets']:
        nm = names.get(pid, str(pid))
        sim_set = wall_to_sim(t_set)
        if sim_set is None:
            continue
        for h in horizons:
            t_sim = sim_set + h
            sp = sched.position_at(pid, t_set + h, t_sim)
            if sp is None or not gts:
                continue
            i = bisect.bisect_left(gts, t_sim)
            cand = [k for k in (i - 1, i) if 0 <= k < len(gts)]
            if not cand:
                continue
            k = min(cand, key=lambda k: abs(gts[k] - t_sim))
            if abs(gts[k] - t_sim) > 0.2:
                continue
            g = gt[gts[k]].get(nm)
            if g is None:
                continue
            fidelity.setdefault(h, []).append(math.hypot(sp[0] - g[0], sp[1] - g[1]))

    stage1 = {}
    for h in horizons:
        v = fidelity.get(h, [])
        if v:
            v = sorted(v)
            stage1[f'+{h}s'] = {'n': len(v), 'median_m': round(v[len(v) // 2], 3),
                                'p90_m': round(v[int(len(v) * 0.9)], 3),
                                'max_m': round(v[-1], 3)}

    # ---- stage 2: do our conflicts match RMF's ----
    ours = []
    pids = sorted(sched.sets)
    t_lo = min(t for t, _, _ in d['sets'])
    t_hi = max(t for t, _, _ in d['sets']) + 120
    t = t_lo
    active = {}
    while t <= t_hi:
        ts = wall_to_sim(t)
        if ts is not None:
            pos = {}
            for pid in pids:
                p = sched.position_at(pid, t, ts)
                if p:
                    pos[pid] = p
            for a, b in itertools.combinations(sorted(pos), 2):
                dd = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
                key = (a, b)
                if dd < D_CONFLICT_M:
                    if key not in active:
                        active[key] = [t, t, dd]
                    else:
                        active[key][1] = t
                        active[key][2] = min(active[key][2], dd)
                elif key in active:
                    s, e, dm = active.pop(key)
                    ours.append({'pair': [names.get(a, a), names.get(b, b)],
                                 'wall_start': round(s, 3), 'wall_end': round(e, 3),
                                 'min_m': round(dm, 3)})
        t += SAMPLE_DT
    for key, (s, e, dm) in active.items():
        ours.append({'pair': [names.get(key[0], key[0]), names.get(key[1], key[1])],
                     'wall_start': round(s, 3), 'wall_end': round(e, 3),
                     'min_m': round(dm, 3)})

    rmf = [{'conflict_version': cv, 'wall': round(t, 3),
            'participants': [names.get(p, p) for p in parts]}
           for t, cv, parts in d['notices']]

    matched, missed = [], []
    for n in rmf:
        hit = [o for o in ours
               if set(o['pair']) == set(n['participants'])
               and o['wall_start'] - 5 <= n['wall'] <= o['wall_end'] + 5]
        (matched if hit else missed).append(n)
    extra = [o for o in ours
             if not any(set(o['pair']) == set(n['participants'])
                        and o['wall_start'] - 5 <= n['wall'] <= o['wall_end'] + 5
                        for n in rmf)]

    result = {
        'repeat_dir': args.repeat_dir,
        'inputs': {'itinerary_set': len(d['sets']), 'itinerary_delay': len(d['delays']),
                   'negotiation_notice': len(d['notices']),
                   'ground_truth_frames': len(gt), 'clock_samples': len(d['clock'])},
        'stage1_schedule_vs_ground_truth_error': stage1,
        'stage1_verdict': 'see median/p90 against the 0.628m conflict threshold',
        'stage2': {
            'rmf_notices': len(rmf),
            'ours': len(ours),
            'matched': len(matched),
            'rmf_found_we_missed': len(missed),
            'we_found_rmf_did_not': len(extra),
            'detail': {'missed': missed, 'extra': extra},
        },
    }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    print(json.dumps({k: v for k, v in result.items() if k != 'stage2'}, indent=2))
    print(json.dumps({k: v for k, v in result['stage2'].items() if k != 'detail'}, indent=2))


if __name__ == '__main__':
    main()
