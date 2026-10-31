#!/usr/bin/env python3
"""Computes the Feature 4a (task completion) metrics from the RMF websocket task
state.

RMF pushes task state over rmf_websocket rather than over a ROS 2 topic, with an
explicit lifecycle:

    queued -> standby -> underway -> completed      (normal)
    queued -> standby -> underway -> error -> ...   (failed)

Emits 4a_n_submitted, 4a_n_completed, 4a_completed_pct, 4a_state_counts and
4a_per_task. Tasks whose last status is "error", and tasks that reached no
terminal state before the recording stopped, are counted separately and folded
into neither bucket. METRIC_DESIGN.md defines the measurands.

Input is api_state_attempt<N>.jsonl, which sits at run level because the
websocket listener starts before the fleet adapter and therefore before the
attempt directory exists. It is matched to a repeat by time overlap with that
repeat's bag, not by name.
"""
import argparse
import glob
import json
import os

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

TERMINAL = ('completed', 'error', 'canceled', 'killed', 'failed')


def bag_time_range(bag):
    """Return the bag's first and last message timestamps, in seconds."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    lo = hi = None
    while r.has_next():
        _, _, t_ns = r.read_next()
        t = t_ns / 1e9
        if lo is None or t < lo:
            lo = t
        if hi is None or t > hi:
            hi = t
    return lo, hi


def load_states(path):
    """task_id -> ordered list of (wall_time, status)."""
    per = {}
    for line in open(path):
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        m = o.get('msg') or {}
        if m.get('type') != 'task_state_update':
            continue
        d = m.get('data') or {}
        tid = (d.get('booking') or {}).get('id')
        status = d.get('status')
        if not tid or not status:
            continue
        per.setdefault(tid, []).append((o.get('_recv_wall', 0.0), status))
    for tid in per:
        per[tid].sort()
    return per


def recorded_api_file(repeat_dir):
    """The api_state file the Task Planning analyser recorded for this repeat, or None.

    A resumed run restarts its attempt numbering, so two passes over one run directory
    write the same `api_state_attempt<N>.jsonl` name and the window a file covers no
    longer identifies the repeat it belongs to. The recorded path does.
    """
    tp = os.path.join(repeat_dir, 'task_planning_metrics.json')
    try:
        path = json.load(open(tp)).get('api_state_path')
    except Exception:
        return None
    if not path:
        return None
    # Resolved by name against the run directory, so a renamed collection still finds it.
    local = os.path.join(os.path.dirname(os.path.normpath(repeat_dir)),
                         os.path.basename(path))
    for candidate in (path, local):
        if os.path.exists(candidate):
            return candidate
    return None


def pick_api_file(run_dir, lo, hi):
    """The api_state file overlapping this repeat's bag window."""
    best, best_overlap = None, 0.0
    # api_state_attempt<N>.jsonl is the current layout; api_state.jsonl is the
    # run-level file the first pilots wrote before the split, kept readable so
    # those runs stay analysable.
    candidates = (sorted(glob.glob(os.path.join(run_dir, 'api_state_attempt*.jsonl')))
                  + sorted(glob.glob(os.path.join(run_dir, 'api_state.jsonl'))))
    for f in candidates:
        times = []
        for line in open(f):
            try:
                times.append(json.loads(line).get('_recv_wall', 0.0))
            except json.JSONDecodeError:
                continue
        if not times:
            continue
        overlap = min(hi, max(times)) - max(lo, min(times))
        if overlap > best_overlap:
            best, best_overlap = f, overlap
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repeat-dir', required=True, help='Repeat directory holding the bag and the ground truth')
    ap.add_argument('--run-dir', required=True, help='Run directory holding the repeat_* directories')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    bag = os.path.join(args.repeat_dir, 'bag')
    lo, hi = bag_time_range(bag)
    api = recorded_api_file(args.repeat_dir) or pick_api_file(args.run_dir, lo, hi)
    if api is None:
        raise SystemExit(f'no api_state_attempt*.jsonl overlaps {bag} -- '
                         f'was the websocket listener running for this repeat?')

    per = load_states(api)
    # only tasks that actually belong to this repeat's window
    per = {tid: seq for tid, seq in per.items()
           if any(lo <= t <= hi for t, _ in seq)}

    rows = []
    counts = {}
    for tid, seq in sorted(per.items()):
        last = seq[-1][1]
        transitions = []
        for _, s in seq:
            if not transitions or transitions[-1] != s:
                transitions.append(s)
        state = last if last in TERMINAL else 'NO_TERMINAL_STATE'
        counts[state] = counts.get(state, 0) + 1
        rows.append({
            'task_id': tid,
            'final_status': last,
            'classified': state,
            'transitions': transitions,
            'n_updates': len(seq),
        })

    n = len(rows)
    n_completed = counts.get('completed', 0)
    result = {
        'scenario': args.scenario,
        'repeat_dir': args.repeat_dir,
        'api_state_file': api,
        'method': 'explicit task state from rmf_websocket (task_state_update); '
                  'no inference, no W_RELEASE, no INDETERMINATE bucket',
        '4a_n_submitted': n,
        '4a_n_completed': n_completed,
        '4a_completed_pct': (100.0 * n_completed / n) if n else None,
        '4a_state_counts': counts,
        '4a_per_task': rows,
    }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    print(json.dumps({k: v for k, v in result.items() if k != '4a_per_task'},
                     indent=2, default=str))


if __name__ == '__main__':
    main()
