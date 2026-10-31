#!/usr/bin/env python3
"""Computes the Feature 1 (Task Planning) metrics from a concurrent-submission bag.

Emits 1a_task_dispatch_success_rate_pct, 1b_planning_latency_s, 1c_makespan_s,
1c_makespan_incl_transit_s, 1c_queueing_s and 1c_batch_makespan_s, plus a
per_task[] breakdown.

Makespan runs from the assigned robot's arrival at the route's first waypoint,
located on the nav graph within --arrival-threshold-m, to the task's completion
as reported by the RMF API. A task that has reached no terminal state by the
end of the capture is censored: its makespan is null and a lower bound is
recorded instead. A task that terminated without completing carries no
makespan and is not censored. METRIC_DESIGN.md §F1 defines the measurands.

ANALYSIS_VERSION 1 ended makespan at the robot's last observed movement, which
assigned a makespan to tasks that never completed. Outputs carry the version
they were produced with.
"""
import argparse
import json
import math
import os
import statistics as st
import sys

import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'common'))
from metric_statistics import summarize  # noqa: E402

POS_EPS = 0.01  # meters, movement threshold between consecutive fleet_states samples
# Metres per second above which a step between two fleet_states samples is the reported
# pose moving rather than the robot. The TurtleBot3 Burger's maximum is 0.22 m/s; the
# margin covers sampling jitter without admitting a relocalisation.
MAX_PLAUSIBLE_SPEED = 0.30
ANALYSIS_VERSION = 2


def load_waypoints(nav_graph_path):
    """name -> (x, y), from a building_map_generator nav_graph YAML's vertices list."""
    with open(nav_graph_path) as f:
        data = yaml.safe_load(f)
    coords = {}
    for level in data.get('levels', {}).values():
        for v in level.get('vertices', []):
            x, y = v[0], v[1]
            params = v[2] if len(v) > 2 else {}
            name = params.get('name')
            if name:
                coords[name] = (x, y)
    return coords

TERMINAL = ('completed', 'error', 'canceled', 'killed', 'failed')


def completion_fields(state, t_submit, t_award, has_api_state):
    """Return the task fields that do not depend on a robot being observed holding the
    task: its terminal status, whether it completed, whether it is censored, the
    dispatcher's queueing, and the submit-to-completion duration.

    state is this task's entry from load_task_terminal_state, or {} when it has none.
    unresolved carries why no makespan is defined, or None when completion is in hand.
    """
    status = state.get('status')
    completed = status == 'completed'
    t_completed = state.get('t_terminal') if completed else None
    # Censored means the task reached no terminal state before the capture stopped, so its
    # duration is only known to exceed what was observed. A task that terminated without
    # completing is finished, not censored, and carries no makespan either.
    censored = has_api_state and status not in TERMINAL

    if not has_api_state:
        unresolved = 'no task state capture'
    elif completed:
        unresolved = None
    elif censored:
        unresolved = 'no terminal state within the capture (last status %s)' % status
    else:
        unresolved = 'task terminated without completing (status %s)' % status

    return {
        'task_status': status,
        't_completed': t_completed,
        'completed': completed,
        'censored': censored,
        't_last_observed_api': state.get('t_last_observed'),
        # Queueing is award minus submit, a dispatcher quantity independent of both the
        # holder and the arrival position test.
        'queueing_s': t_award - t_submit if t_award is not None and t_submit is not None else None,
        'makespan_incl_transit_s': (t_completed - t_submit
                                    if t_completed is not None and t_submit is not None else None),
        'unresolved': unresolved,
    }


def batch_makespan(tasks, has_api_state):
    """Return the batch makespan: the last completion in the batch minus the first
    submission, with the status that qualifies it.

    The batch is the set of tasks submitted together in one repeat, so the figure answers
    when the fleet finished all of them. It is defined only when every task completed; a
    batch still holding an unfinished task has no last completion, and taking the maximum
    over the completed ones would report the batch as finished early. Statuses:

        complete            every task completed, value given
        censored            a task reached no terminal state, value null, lower bound
                            given from the capture horizon when one was observed
        not_completed       a task terminated without completing, value null
        incomplete_data     no API state capture, or a timestamp is missing
    """
    if not tasks:
        return {'status': 'incomplete_data', 'batch_makespan_s': None,
                'batch_censored_lower_bound_s': None,
                'detail': 'no tasks'}
    if not has_api_state:
        return {'status': 'incomplete_data', 'batch_makespan_s': None,
                'batch_censored_lower_bound_s': None,
                'detail': 'no task state capture'}

    submits = [t['t_submit'] for t in tasks if t['t_submit'] is not None]
    if len(submits) != len(tasks):
        return {'status': 'incomplete_data', 'batch_makespan_s': None,
                'batch_censored_lower_bound_s': None,
                'detail': 'a task has no submission time'}
    t_first_submit = min(submits)

    failed = [t['task_id'] for t in tasks
              if t['task_status'] in TERMINAL and t['task_status'] != 'completed']
    if failed:
        return {'status': 'not_completed', 'batch_makespan_s': None,
                'batch_censored_lower_bound_s': None,
                'detail': 'terminated without completing: %s' % ', '.join(sorted(failed))}

    censored = [t for t in tasks if t['censored']]
    if censored:
        horizons = [t['t_last_observed_api'] for t in censored
                    if t['t_last_observed_api'] is not None]
        return {'status': 'censored', 'batch_makespan_s': None,
                'batch_censored_lower_bound_s': (max(horizons) - t_first_submit
                                                 if horizons else None),
                'detail': 'no terminal state: %s'
                          % ', '.join(sorted(t['task_id'] for t in censored))}

    completions = [t['t_completed'] for t in tasks if t['t_completed'] is not None]
    if len(completions) != len(tasks):
        return {'status': 'incomplete_data', 'batch_makespan_s': None,
                'batch_censored_lower_bound_s': None,
                'detail': 'a completed task has no completion time'}
    return {'status': 'complete', 'batch_makespan_s': max(completions) - t_first_submit,
            'batch_censored_lower_bound_s': None, 'detail': None}


def load_task_terminal_state(path):
    """task_id -> {'status', 't_terminal', 't_last_observed'} from an API state capture
    written by collect/rmf/record_api_state.py.

    status is the one the task first transitioned into a terminal state with, or its last
    reported status when it reached none. t_terminal is the wall time of that transition.

    Same parse as analyse/execution_fidelity/analyze_execution_fidelity.py and
    analyse/fleet_coordination/analyze_fleet_coordination_4a.py, so 1c's makespan, 4a's
    completion verdict and 5a's finish instant cannot disagree about what a task did.
    """
    per = {}
    with open(path) as f:
        lines = f.readlines()
    for line in lines:
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
        if not tid:
            continue
        t = o.get('_recv_wall', 0.0)
        entry = per.setdefault(tid, {'status': None, 't_terminal': None, 't_last_observed': None})
        entry['t_last_observed'] = t
        if status in TERMINAL:
            if entry['t_terminal'] is None or t < entry['t_terminal']:
                entry['t_terminal'] = t
                entry['status'] = status
        elif entry['t_terminal'] is None:
            entry['status'] = status
    return per


NEGOTIATION_TOPICS = [
    '/rmf_traffic/negotiation_notice',
    '/rmf_traffic/negotiation_proposal',
    '/rmf_traffic/negotiation_conclusion',
    '/rmf_traffic/negotiation_rejection',
    '/rmf_traffic/negotiation_forfeit',
]
BLOCKADE_TOPICS = [
    '/rmf_traffic/blockade_set',
    '/rmf_traffic/blockade_ready',
    '/rmf_traffic/blockade_reached',
    '/rmf_traffic/blockade_release',
]


def read_bag(path):
    """Return {topic: [(timestamp_s, message)]} for the topics this analysis reads."""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=path, storage_id='mcap')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes = {name: get_message(t) for name, t in topic_types.items()}

    records = {name: [] for name in topic_types}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, msg_classes[topic])
        records[topic].append((t_ns / 1e9, msg))
    return records


def parse_args():
    """Parse the command line."""
    ap = argparse.ArgumentParser(
        description='Compute Task Planning metrics from a concurrent-submission (N>=2) rosbag')
    ap.add_argument('--bag', required=True, help='Path to the rosbag directory')
    ap.add_argument('--robots', nargs='+', required=True,
                     help='Robot names expected to appear in /fleet_states')
    ap.add_argument('--scenario', required=True, help='Human-readable scenario label')
    ap.add_argument('--output', required=True, help='Path to write the metrics JSON to')
    ap.add_argument('--nav-graph', required=True,
                     help='Path to the nav_graph YAML (e.g. .../maps/world_tb3/nav_graphs/0.yaml), '
                          'used to look up each route\'s first waypoint (x, y) for 1c makespan\'s '
                          'transit-exclusion fix.')
    ap.add_argument('--api-state',
                     help='Path to the api_state_attempt<N>.jsonl capture for this repeat. It '
                          'carries each task\'s terminal state, which the bag does not. Without '
                          'it no task can be shown to have completed, so every makespan is '
                          'reported as unresolved.')
    ap.add_argument('--arrival-threshold-m', type=float, default=1.0,
                     help='A robot counts as having arrived at a route\'s first waypoint once '
                          'within this many meters of it (default 1.0, a loose match for '
                          '/fleet_states sampling granularity, not a precision goal tolerance).')
    return ap.parse_args()


def robot_state(msg, name):
    """The named robot's entry in one /fleet_states message, or None."""
    for r in msg.robots:
        if r.name == name:
            return r
    return None


def path_length_per_robot(fleet_states, robots):
    """robot -> metres travelled over the capture, summed between consecutive
    /fleet_states samples more than POS_EPS apart.

    Steps that imply more than MAX_PLAUSIBLE_SPEED are dropped. A TurtleBot3 Burger tops
    out at 0.22 m/s, so a larger step is the localisation moving, not the robot: on the
    hardware collection of 2026-10-15 one robot reported 55.5 m over a 7.08 m route, of
    which 43.3 m arrived in 22 jumps. Counting those would let a robot that never moved
    clear the distance the N=3 gate asks for, which is the one thing that gate exists to
    catch.

    This is still the distance RMF perceived, not ground truth: a robot whose
    localisation froze reports no path however far it drove. Both cases make the repeat
    unusable, so the figure serves as a gate either way.
    """
    out = {}
    for name in robots:
        total = 0.0
        prev = None
        for stamp, msg in fleet_states:
            r = robot_state(msg, name)
            if r is None:
                continue
            xy = (r.location.x, r.location.y)
            if prev is not None:
                dt = stamp - prev[0]
                d = math.hypot(xy[0] - prev[1][0], xy[1] - prev[1][1])
                if d > POS_EPS and (dt <= 0 or d / dt <= MAX_PLAUSIBLE_SPEED):
                    total += d
            prev = (stamp, xy)
        out[name] = round(total, 3)
    return out


def main():
    args = parse_args()
    records = read_bag(args.bag)
    waypoints = load_waypoints(args.nav_graph)
    terminal_state = load_task_terminal_state(args.api_state) if args.api_state else {}

    bid_notice = records.get('/rmf_task/bid_notice', [])
    bid_response = records.get('/rmf_task/bid_response', [])
    dispatch_request = records.get('/rmf_task/dispatch_request', [])
    dispatch_ack = records.get('/rmf_task/dispatch_ack', [])
    task_api_requests = records.get('/task_api_requests', [])
    fleet_states = records.get('/fleet_states', [])

    # Order tasks by bid_notice bag time (== submission order)
    bid_notice_sorted = sorted(bid_notice, key=lambda x: x[0])
    tasks = [{'task_id': msg.task_id, 't_bid_notice': t} for t, msg in bid_notice_sorted]

    # 1b: match bid_response by task_id
    bid_response_by_id = {}
    for t, msg in bid_response:
        bid_response_by_id.setdefault(msg.task_id, t)
    for task in tasks:
        task['t_bid_response'] = bid_response_by_id.get(task['task_id'])
        task['planning_latency_s'] = (
            task['t_bid_response'] - task['t_bid_notice']
            if task['t_bid_response'] is not None else None
        )

    dispatch_id_by_task = {}
    for t, msg in dispatch_request:
        dispatch_id_by_task[msg.task_id] = msg.dispatch_id
    success_by_dispatch_id = {}
    for t, msg in sorted(dispatch_ack, key=lambda x: x[0]):
        if msg.dispatch_id not in success_by_dispatch_id:
            success_by_dispatch_id[msg.dispatch_id] = msg.success
    for task in tasks:
        dispatch_id = dispatch_id_by_task.get(task['task_id'])
        task['dispatch_id'] = dispatch_id
        task['dispatch_success'] = success_by_dispatch_id.get(dispatch_id, False)

    t_award_by_task = {msg.task_id: t for t, msg in dispatch_request}
    for task in tasks:
        task['t_award'] = t_award_by_task.get(task['task_id'])

    api_req_sorted = sorted(task_api_requests, key=lambda x: x[0])
    for i, task in enumerate(tasks):
        task['t_submit'] = api_req_sorted[i][0] if i < len(api_req_sorted) else task['t_bid_notice']
        task['places'] = None
        if i < len(api_req_sorted):
            try:
                payload = json.loads(api_req_sorted[i][1].json_msg)
                task['places'] = payload['request']['description']['places']
            except (json.JSONDecodeError, KeyError, TypeError):
                pass
        task['route_start_xy'] = (
            waypoints.get(task['places'][0]) if task['places'] else None
        )
    fs_sorted = sorted(fleet_states, key=lambda x: x[0])

    for task in tasks:
        task_id = task['task_id']
        holder_segment_start = {}   # robot_name -> t of the first sample (in this scan) holding it
        last_seen_with_task_t = task['t_submit']
        owning_robot = None
        for t, msg in fs_sorted:
            for name in args.robots:
                r = robot_state(msg, name)
                if r is not None and r.task_id == task_id:
                    holder_segment_start.setdefault(name, t)
                    owning_robot = name
                    last_seen_with_task_t = t
                    break
        task['robot'] = owning_robot
        task['reassigned'] = len(holder_segment_start) > 1

        # Completion is a task property reported by the API, not a property of motion: a
        # robot that stops short of its destination also stops moving. Completion and the
        # dispatcher's timings are resolved before the holder is established, because
        # neither depends on a robot having been observed holding the task.
        state = terminal_state.get(task_id, {})
        fields = completion_fields(state, task['t_submit'], task['t_award'],
                                   bool(args.api_state))
        unresolved = fields.pop('unresolved')
        task.update(fields)

        if owning_robot is None:
            task['t_end_physical'] = None
            task['t_arrival_at_route_start'] = None
            task['makespan_s'] = None
            task['censored_lower_bound_s'] = None
            task['makespan_unresolved_reason'] = (
                unresolved or 'arrival at route start not observed: assigned robot not '
                              'observed in /fleet_states')
            continue

        window_start = holder_segment_start[owning_robot]
        window_end = last_seen_with_task_t
        window = [(t, m) for t, m in fs_sorted if window_start <= t <= window_end]

        # t_arrival is an ABSOLUTE position test (robot vs a nav-graph waypoint),
        # unlike everything else here, which compares consecutive positions. Two
        # consequences, both measured rather than assumed:
        #
        # 1. How much of a task it excludes depends entirely on the scenario.
        #    Measured over all 60 official tasks, t_arrival - t_award is
        #    Bottleneck 40.99s (max 64.7) but Crossing 0.057s and Head-on 0.062s,
        #    because those routes start where the robot already stands. So
        #    makespan excludes ~41s of transit in Bottleneck and ~0s in Crossing:
        #    the four scenarios' makespans are NOT directly comparable.
        # 2. Being absolute, it is the one timing here that a localisation offset
        #    can break outright - a delocalised robot never registers as having
        #    reached the waypoint. Ground-truth poses are recorded alongside so
        #    this can be recomputed.
        #
        # Positions come from /fleet_states, that is what RMF perceived through
        # AMCL, not physical ground truth.
        t_arrival = None
        if task['route_start_xy'] is not None:
            sx, sy = task['route_start_xy']
            for t, msg in window:
                r = robot_state(msg, owning_robot)
                if r is None:
                    continue
                if math.hypot(r.location.x - sx, r.location.y - sy) <= args.arrival_threshold_m:
                    t_arrival = t
                    break
        task['t_arrival_at_route_start'] = t_arrival

        last_moving_t = window_start
        prev_xy = None
        for t, msg in window:
            r = robot_state(msg, owning_robot)
            if r is None:
                continue
            xy = (r.location.x, r.location.y)
            if prev_xy is not None:
                d = math.hypot(xy[0] - prev_xy[0], xy[1] - prev_xy[1])
                if d > POS_EPS:
                    last_moving_t = t
            prev_xy = xy

        task['t_end_physical'] = last_moving_t

        if task['completed'] and t_arrival is not None:
            task['makespan_s'] = task['t_completed'] - t_arrival
            task['censored_lower_bound_s'] = None
            task['makespan_unresolved_reason'] = None
        else:
            task['makespan_s'] = None
            horizon = state.get('t_last_observed') or last_moving_t
            task['censored_lower_bound_s'] = (horizon - t_arrival
                                              if task['censored'] and t_arrival is not None
                                              else None)
            task['makespan_unresolved_reason'] = (
                unresolved or 'arrival at route start not observed')

    n = len(tasks)
    dispatch_success_rate = 100.0 * sum(1 for t in tasks if t['dispatch_success']) / n if n else None
    latencies = [t['planning_latency_s'] for t in tasks if t['planning_latency_s'] is not None]
    makespans = [t['makespan_s'] for t in tasks if t['makespan_s'] is not None]
    makespans_incl_transit = [t['makespan_incl_transit_s'] for t in tasks if t['makespan_incl_transit_s'] is not None]
    queueings = [t['queueing_s'] for t in tasks if t['queueing_s'] is not None]
    n_makespan_unresolved = sum(1 for t in tasks if t['makespan_s'] is None)
    n_completed = sum(1 for t in tasks if t['completed'])
    n_censored = sum(1 for t in tasks if t['censored'])
    batch = batch_makespan(tasks, bool(args.api_state))

    makespan_resolution_rate_pct = 100.0 * (n - n_makespan_unresolved) / n if n else None

    # ---- per-robot breakdown ----
    observed_path = path_length_per_robot(fs_sorted, args.robots)
    per_robot = {}
    for name in args.robots:
        robot_tasks = [t for t in tasks if t['robot'] == name]
        per_robot[name] = {
            'n_tasks': len(robot_tasks),
            'makespan_s': summarize([t['makespan_s'] for t in robot_tasks if t['makespan_s'] is not None]),
            'observed_path_m': observed_path.get(name),
        }

    negotiation_counts = {topic: len(records.get(topic, [])) for topic in NEGOTIATION_TOPICS}
    blockade_counts = {topic: len(records.get(topic, [])) for topic in BLOCKADE_TOPICS}

    result = {
        'scenario': args.scenario,
        'bag_path': args.bag,
        'robots': args.robots,
        'analysis_version': ANALYSIS_VERSION,
        'api_state_path': args.api_state,
        'n_tasks': n,
        'n_tasks_completed': n_completed,
        'n_tasks_censored': n_censored,
        'n_makespan_unresolved': n_makespan_unresolved,
        'makespan_resolution_rate_pct': makespan_resolution_rate_pct,
        'per_task': tasks,
        'per_robot': per_robot,
        '1a_task_dispatch_success_rate_pct': dispatch_success_rate,
        '1b_planning_latency_s': summarize(latencies),
        '1c_makespan_s': summarize(makespans),
        '1c_makespan_incl_transit_s': summarize(makespans_incl_transit),
        '1c_queueing_s': summarize(queueings),
        '1c_batch_makespan_s': batch['batch_makespan_s'],
        '1c_batch_makespan_status': batch['status'],
        '1c_batch_censored_lower_bound_s': batch['batch_censored_lower_bound_s'],
        '1c_batch_makespan_detail': batch['detail'],
        'negotiation_message_counts': negotiation_counts,
        'blockade_message_counts': blockade_counts,
    }

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2, default=str)

    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
