#!/usr/bin/env python3
"""Computes the Feature 5 (Execution fidelity) metrics: how far execution departed
from what RMF predicted, per task.

    5a_duration_error_pct = (D_actual - D_planned) / D_planned x 100
        D_planned = t_estimate_finish - t(bid_response), the job as the planner priced it
        D_actual  = t_actual_finish - t_award, the job as executed
    5a_delta_s            = t_actual_finish - t_estimate_finish

t_estimate_finish comes from bid_response.proposal.finish_time, a sim-clock instant,
converted to wall time through the /clock samples bracketing it. An instant outside the
recorded /clock range is extrapolated at the local rate and flagged with
clock_extrapolated.

t_actual_finish comes from the task's terminal transition in the websocket task
state, reported as actual_finish_source and terminal_status. A task that reached
no terminal state yields no delta and is counted in n_tasks_no_terminal_state.

Also emitted for audit: 5a_delta_v1_bid_offset_s, the delta a single bid-time
offset would give; drift_correction_s, the difference; and t_end_physical_wall.

Only the winning robot's bid_response is used per task, matched on task_id and on
expected_robot_name against task['robot']; losing bids appear on the same topic.
METRIC_DESIGN.md defines the measurands.
"""
import argparse
import bisect
import json
import os
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'common'))
from metric_statistics import summarize  # noqa: E402

ANALYSIS_VERSION = 2


def read_bag(path):
    """Return {topic: [(timestamp_s, message)]} for the topics this analysis reads."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='mcap'), ConverterOptions('', ''))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes = {name: get_message(t) for name, t in topic_types.items()}
    records = {name: [] for name in topic_types}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        records[topic].append((t_ns / 1e9, deserialize_message(data, msg_classes[topic])))
    return records


TERMINAL = ('completed', 'error', 'canceled', 'killed', 'failed')


def terminal_times(path):
    """task_id -> wall time of its first transition into a terminal status.

    Same parse as analyse/fleet_coordination/analyze_fleet_coordination_4a.py.
    """
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
        if not tid or status not in TERMINAL:
            continue
        t = o.get('_recv_wall', 0.0)
        if tid not in per or t < per[tid][0]:
            per[tid] = (t, status)
    return per


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--task-planning-metrics', required=True, help='task_planning_metrics.json for the same repeat')
    ap.add_argument('--bag', required=True, help='Path to the rosbag2 directory of one repeat')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--api-state',
                    help='api_state_attempt<N>.jsonl for this repeat; without it no '
                         'delta is computed')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    tp = json.load(open(args.task_planning_metrics))
    records = read_bag(args.bag)
    clock_samples = sorted(
        (t, msg.clock.sec + msg.clock.nanosec * 1e-9) for t, msg in records.get('/clock', [])
    )
    bid_responses = records.get('/rmf_task/bid_response', [])

    if not clock_samples:
        # No /clock: the run used the system clock, so finish_time is already a wall
        # instant and the conversion is the identity.
        wall_clock_run = True
    else:
        wall_clock_run = False

    if wall_clock_run:
        def sim_to_wall(sim_t):
            return sim_t, False

        def nearest_offset(bag_wall_t):
            return 0.0
    else:

        def nearest_offset(bag_wall_t):
            # v1 conversion, reported only as the size of the drift correction
            best = min(clock_samples, key=lambda c: abs(c[0] - bag_wall_t))
            wall, sim = best
            return wall - sim  # wall = sim + offset

        # v2: look the instant up directly rather than assume a constant offset
        by_sim = sorted(clock_samples, key=lambda c: c[1])
        sims = [c[1] for c in by_sim]

        def _rate(i0, i1):
            dw = by_sim[i1][0] - by_sim[i0][0]
            ds = by_sim[i1][1] - by_sim[i0][1]
            return dw / ds if ds > 0 else 1.0

        def sim_to_wall(sim_t):
            """Wall time at which the sim clock reads sim_t. Returns
            (wall, extrapolated)."""
            i = bisect.bisect_left(sims, sim_t)
            if i <= 0:
                w0, s0 = by_sim[0]
                j = min(len(by_sim) - 1, 100)
                return w0 - (s0 - sim_t) * _rate(0, j), True
            if i >= len(by_sim):
                w1, s1 = by_sim[-1]
                j = max(0, len(by_sim) - 101)
                return w1 + (sim_t - s1) * _rate(j, len(by_sim) - 1), True
            w0, s0 = by_sim[i - 1]
            w1, s1 = by_sim[i]
            if s1 == s0:
                return w0, False
            return w0 + (w1 - w0) * (sim_t - s0) / (s1 - s0), False

    per_task = []
    terminal = terminal_times(args.api_state) if args.api_state else {}
    deltas = []
    duration_error_pcts = []
    for task in tp['per_task']:
        task_id = task['task_id']
        robot = task.get('robot')
        t_end_physical = task.get('t_end_physical')
        term = terminal.get(task['task_id'])
        t_end = term[0] if term else None
        if robot is None or t_end is None:
            continue
        # winning bid: matches task_id AND expected_robot_name == owning robot
        win = None
        for t, msg in bid_responses:
            if msg.task_id == task_id and msg.has_proposal and msg.proposal.expected_robot_name == robot:
                win = (t, msg)
                break
        if win is None:
            per_task.append({'task_id': task_id, 'robot': robot,
                             'status': 'no_matching_bid_response'})
            continue
        t_bid_wall, msg = win
        finish_sim = msg.proposal.finish_time.sec + msg.proposal.finish_time.nanosec * 1e-9
        predicted_wall_finish, extrapolated = sim_to_wall(finish_sim)
        delta = t_end - predicted_wall_finish

        offset_at_bid = nearest_offset(t_bid_wall)
        delta_v1 = t_end - (finish_sim + offset_at_bid)

        # The two durations start from different instants, each from the moment its
        # own clock starts, so queueing falls outside both:
        #   predicted = predicted_wall_finish - t_bid_response
        #   actual    = t(terminal state) - t_award
        t_award = task.get('t_award')
        predicted_duration = predicted_wall_finish - t_bid_wall
        actual_duration = duration_error_pct = None
        if t_award is not None:
            actual_duration = t_end - t_award
        if predicted_duration > 0 and actual_duration is not None:
            duration_error_pct = 100.0 * (
                actual_duration - predicted_duration) / predicted_duration
            duration_error_pcts.append(duration_error_pct)

        deltas.append(delta)
        per_task.append({
            'task_id': task_id, 'robot': robot,
            't_bid_response_wall': t_bid_wall,
            'finish_time_sim': finish_sim,
            'predicted_wall_finish': predicted_wall_finish,
            'clock_extrapolated': extrapolated,
            't_actual_finish_wall': t_end,
            'terminal_status': term[1] if term else None,
            't_end_physical_wall': t_end_physical,
            '5a_delta_actual_minus_predicted_s': round(delta, 3),
            '5a_delta_v1_bid_offset_s': round(delta_v1, 3),
            'drift_correction_s': round(delta - delta_v1, 3),
            # carried through for the merge's outlier rule
            'queueing_s': task.get('queueing_s'),
            't_award': t_award,
            'predicted_duration_s': round(predicted_duration, 3),
            'actual_duration_s': (round(actual_duration, 3)
                                  if actual_duration is not None else None),
            '5a_duration_error_pct': (round(duration_error_pct, 1)
                                      if duration_error_pct is not None else None),
        })

    result = {
        'scenario': args.scenario,
        'bag': args.bag,
        'analysis_version': ANALYSIS_VERSION,
        'method': 'finish_time from the winning bid_response, converted to wall time through the '
                  '/clock samples bracketing it in simulation; actual finish from the terminal task '
                  'state. duration_error_pct = (D_actual - D_planned) / D_planned, D_planned from '
                  'bid_response to the predicted finish, D_actual from award to the terminal state; '
                  'delta = actual finish - predicted finish, a difference of two instants',
        'n_tasks_considered': len(tp['per_task']),
        'n_tasks_with_delta': len(deltas),
        'n_tasks_no_terminal_state': sum(
            1 for t in tp['per_task']
            if t.get('task_id') not in terminal),
        'actual_finish_source': ('task terminal transition (websocket task state)'
                                 if args.api_state else 'UNAVAILABLE -- no --api-state'),
        '5a_delta_s': summarize(deltas),
        '5a_duration_error_pct': summarize(duration_error_pcts),
        'per_task': per_task,
    }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
