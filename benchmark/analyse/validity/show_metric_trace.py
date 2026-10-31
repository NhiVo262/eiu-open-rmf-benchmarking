#!/usr/bin/env python3

import argparse
import bisect
import json
import math
import os
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

WANTED = ('/rmf_task/bid_notice', '/rmf_task/bid_response',
          '/rmf_traffic/itinerary_set', '/rmf_traffic/query_update_1',
          '/rmf_traffic/negotiation_notice', '/rmf_traffic/negotiation_conclusion',
          '/mutex_group_request', '/mutex_group_states',
          '/clock', '/fleet_states', '/rmf_traffic/participants')


def read(bag):
    """Return {topic: [(timestamp_s, message)]} for every topic a trace reads."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    classes = {t.name: get_message(t.type)
               for t in reader.get_all_topics_and_types()}
    out = {name: [] for name in classes}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in WANTED:
            continue
        out[topic].append((t_ns / 1e9, deserialize_message(data, classes[topic])))
    return out


def head(title):
    """Print a section heading for one metric's trace."""
    print()
    print('=' * 78)
    print(title)
    print('-' * 78)


def trace_planner(rec):
    """Print bid_notice to bid_response per task, the interval F2b (i) measures."""
    head('F2b (i)  PLANNER COMPUTE   bid_response - bid_notice')
    notices = {}
    for t, m in rec.get('/rmf_task/bid_notice', []):
        notices.setdefault(m.task_id, t)
    for t, m in rec.get('/rmf_task/bid_response', []):
        t0 = notices.get(m.task_id)
        if t0 is None:
            continue
        print('  bid_notice     t = %.6f   task %s' % (t0, m.task_id))
        print('  bid_response   t = %.6f   robot %s' % (
            t, getattr(m.proposal, 'expected_robot_name', '?')))
        print('  planner compute        = %.6f s  (%.2f ms)' % (t - t0, (t - t0) * 1000))
        return
    print('  no matched bid_notice/bid_response pair in this repeat')


def trace_confirm(rec):
    """Print itinerary_set to the first query_update_1 carrying that
    (participant, itinerary_version), the interval F2b (ii) measures."""
    head('F2b (ii) SCHEDULE CONFIRM  query_update_1 - itinerary_set')
    for t_set, m in rec.get('/rmf_traffic/itinerary_set', []):
        pid, iv = m.participant, m.itinerary_version
        for t_qu, q in rec.get('/rmf_traffic/query_update_1', []):
            if t_qu < t_set:
                continue
            hit = any(getattr(p, 'participant_id', None) == pid
                      and getattr(p, 'itinerary_version', None) == iv
                      for p in getattr(q.patch, 'participants', []))
            if hit:
                print('  itinerary_set    t = %.6f   participant %s, itinerary_version %s'
                      % (t_set, pid, iv))
                print('  query_update_1   t = %.6f   patch carries that exact pair' % t_qu)
                print('  schedule confirm         = %.6f s  (%.2f ms)'
                      % (t_qu - t_set, (t_qu - t_set) * 1000))
                print()
                print('  The planner finished before itinerary_set was published,')
                print('  so this interval contains no planning.')
                return
    print('  no itinerary_set was reflected in query_update_1 in this repeat')


def trace_negotiation(rec):
    """Print negotiation_notice to negotiation_conclusion per episode, with the
    resolved flag F3 reads."""
    head('F3  NEGOTIATION   conclusion - notice, with the resolved flag')
    notices = {}
    for t, m in rec.get('/rmf_traffic/negotiation_notice', []):
        notices.setdefault(m.conflict_version, t)
    concluded = rec.get('/rmf_traffic/negotiation_conclusion', [])
    if not concluded:
        print('  no negotiation in this repeat (0 conflicts)')
        return
    for t, m in concluded:
        t0 = notices.get(m.conflict_version)
        if t0 is None:
            continue
        print('  negotiation_notice      t = %.6f   conflict_version %s'
              % (t0, m.conflict_version))
        print('  negotiation_conclusion  t = %.6f   resolved = %s' % (t, m.resolved))
        print('  latency                         = %.6f s  (%.2f ms)'
              % (t - t0, (t - t0) * 1000))
        print()
        print('  resolved distinguishes a settled conflict from an abandoned')
        print('  negotiation. A fast conclusion is not by itself a good outcome.')
        return


def trace_mutex(rec, names):
    """Print mutex_group_request to the mutex_group_states that grants it; an episode
    never granted is printed as a lower bound."""
    head('F6  CONGESTION   mutex_group_request -> the state that grants it')
    states = [(t, {a.group: a.claimant for a in m.assignments})
              for t, m in rec.get('/mutex_group_states', [])]
    seen = set()
    t_last = max((t for t, _ in rec.get('/mutex_group_states', [])), default=None)
    for t_req, m in rec.get('/mutex_group_request', []):
        key = (m.group, m.claimant, round(
            m.claim_time.sec + m.claim_time.nanosec / 1e9, 3))
        if key in seen:
            continue
        seen.add(key)
        holder = None
        for ts, assign in states:
            if ts > t_req:
                break
            holder = assign.get(m.group)
        got = next((ts for ts, assign in states
                    if ts >= t_req and assign.get(m.group) == m.claimant), None)
        who = names.get(m.claimant, 'participant_%s' % m.claimant)
        held = names.get(holder, holder)
        print('  request   t = %.6f   %s asks for [%s]' % (t_req, who, m.group))
        print('            holder at that instant: %s' % (
            'unheld' if holder in (None, 65535) else held))
        if got is not None:
            print('  granted   t = %.6f   wait = %.3f s' % (got, got - t_req))
        else:
            print('  granted   never, within the recording')
            if t_last is not None:
                print('            wait >= %.1f s  -- right-censored, a lower bound,'
                      % (t_last - t_req))
                print('            not a missing value and not zero')
        print()


def trace_fidelity(rec, repeat_dir):
    """Print the F5 delta between the predicted and the actual finish instant, beside
    the two durations, from execution_fidelity_metrics.json."""
    head('F5  EXECUTION FIDELITY   delta between instants, and the duration error')
    ef = os.path.join(repeat_dir, 'execution_fidelity_metrics.json')
    if not os.path.exists(ef):
        print('  no execution_fidelity_metrics.json beside this repeat')
        return
    tasks = [t for t in json.load(open(ef)).get('per_task', [])
             if t.get('5a_delta_actual_minus_predicted_s') is not None]
    if not tasks:
        print('  no task with a delta in this repeat')
        return
    t = tasks[0]
    clock = sorted((s, m.clock.sec + m.clock.nanosec / 1e9)
                   for s, m in rec.get('/clock', []))
    sims = [c[1] for c in clock]
    i = bisect.bisect_left(sims, t['finish_time_sim'])
    print('  bid_response    wall = %.6f   proposal.finish_time = %.6f (SIM clock)'
          % (t['t_bid_response_wall'], t['finish_time_sim']))
    if not clock:
        print('  no /clock in this bag: the stack ran on the system clock, so '
              'finish_time is already wall clock and the conversion is the identity')
    elif 0 < i < len(clock):
        print('  /clock brackets sim %.6f between (wall %.6f, sim %.6f)'
              % (t['finish_time_sim'], clock[i - 1][0], clock[i - 1][1]))
        print('                                and (wall %.6f, sim %.6f)'
              % (clock[i][0], clock[i][1]))
    print('  predicted wall finish  = %.6f' % t['predicted_wall_finish'])
    print('  actual   wall finish   = %.6f   (t_end_physical)'
          % t['t_actual_finish_wall'])
    print('  5a delta               = %+.3f s   <- two INSTANTS subtracted'
          % t['5a_delta_actual_minus_predicted_s'])
    print()
    if t.get('predicted_duration_s') is not None and t.get('actual_duration_s') is not None:
        print('  predicted duration     = %.3f s   (predicted finish - bid_response)'
              % t['predicted_duration_s'])
        print('  actual    duration     = %.3f s   (t_end_physical - t_award)'
              % t['actual_duration_s'])
        print('  duration error         = %+.1f %%  <- a DURATION compared, not instants'
              % (t['5a_duration_error_pct'] or 0.0))
        print()
        print('  The dispatcher queueing (%.1f s here) sits outside both durations,'
              % (t.get('queueing_s') or 0.0))
        print('  which is why the two figures can disagree in sign.')


def trace_safety(rec, repeat_dir):
    """Print the closest approach F7 reports, from ground truth and from
    /fleet_states, from safety_tracker_metrics.json."""
    head('F7  SAFETY   the closest approach, from both sources')
    sf = os.path.join(repeat_dir, 'safety_tracker_metrics.json')
    if not os.path.exists(sf):
        print('  no safety_tracker_metrics.json beside this repeat')
        return
    d = json.load(open(sf)).get('7a_min_distance') or {}
    for src, label in (('ground_truth', 'Gazebo ground truth'),
                       ('rmf_observed', 'what RMF observed (/fleet_states)')):
        v = d.get(src) or {}
        if v.get('min_distance_m') is None:
            continue
        print('  %-34s %.4f m  at sim_t %.2f  pair %s'
              % (label, v['min_distance_m'], v.get('at_sim_t', 0.0),
                 ' / '.join(v.get('pair', []))))
    gt = (d.get('ground_truth') or {}).get('min_distance_m')
    rmf = (d.get('rmf_observed') or {}).get('min_distance_m')
    if gt is not None and rmf is not None:
        print()
        print('  The two differ by %.4f m. By the triangle inequality, the error in a'
              % abs(gt - rmf))
        print('  reported separation is bounded by the two robots\' own position')
        print('  errors: | ||p^i-p^j|| - ||pi-pj|| | <= ||ei|| + ||ej||. That is why')
        print('  the validity threshold is half the 0.628 m conflict separation.')
        print()
        print('  Ground truth here is the simulator\'s own state. It is the reference')
        print('  for this deployment, not evidence about physical accuracy on hardware.')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repeat-dir', required=True,
                    help='A repeat directory holding bag/ and its metrics files')
    ap.add_argument('--only', action='append',
                    help='Limit to these traces: planner confirm negotiation '
                         'mutex fidelity safety')
    args = ap.parse_args()

    bag = os.path.join(args.repeat_dir, 'bag')
    if not os.path.isdir(bag):
        print('no bag at %s' % bag, file=sys.stderr)
        return 2
    rec = read(bag)
    names = {}
    for _, m in rec.get('/rmf_traffic/participants', []):
        for p in m.participants:
            names[p.id] = p.description.name

    print('Repeat: %s' % args.repeat_dir)
    print('Every time below is the message\'s bag arrival time, in seconds.')

    want = set(args.only or [])
    order = [('planner', lambda: trace_planner(rec)),
             ('confirm', lambda: trace_confirm(rec)),
             ('negotiation', lambda: trace_negotiation(rec)),
             ('mutex', lambda: trace_mutex(rec, names)),
             ('fidelity', lambda: trace_fidelity(rec, args.repeat_dir)),
             ('safety', lambda: trace_safety(rec, args.repeat_dir))]
    for name, fn in order:
        if want and name not in want:
            continue
        fn()
    return 0


if __name__ == '__main__':
    sys.exit(main())
