#!/usr/bin/env python3
import argparse
import json
import math
import statistics as st

import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

POS_EPS = 0.01  # meters, movement threshold between consecutive fleet_states samples


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


def summarize(vals):
    if not vals:
        return None
    return {
        'n': len(vals),
        'mean': st.mean(vals),
        'stdev': st.stdev(vals) if len(vals) > 1 else 0.0,
        'min': min(vals),
        'max': max(vals),
    }


def parse_args():
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
    ap.add_argument('--arrival-threshold-m', type=float, default=1.0,
                     help='A robot counts as having arrived at a route\'s first waypoint once '
                          'within this many meters of it (default 1.0, a loose match for '
                          '/fleet_states sampling granularity, not a precision goal tolerance).')
    return ap.parse_args()


def main():
    args = parse_args()
    records = read_bag(args.bag)
    waypoints = load_waypoints(args.nav_graph)

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

    def robot_state(msg, name):
        for r in msg.robots:
            if r.name == name:
                return r
        return None

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

        if owning_robot is None:

            task['t_end_physical'] = None
            task['t_arrival_at_route_start'] = None
            task['makespan_s'] = None
            task['makespan_incl_transit_s'] = None
            task['queueing_s'] = None
            task['execution_s'] = None
            continue

        window_start = holder_segment_start[owning_robot]
        window_end = last_seen_with_task_t
        window = [(t, m) for t, m in fs_sorted if window_start <= t <= window_end]

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

        task['makespan_s'] = last_moving_t - t_arrival if t_arrival is not None else None
        task['makespan_incl_transit_s'] = last_moving_t - task['t_submit']

        if task['t_award'] is not None and t_arrival is not None:
            task['queueing_s'] = task['t_award'] - task['t_submit']
            task['execution_s'] = last_moving_t - max(task['t_award'], t_arrival)
        else:
            task['queueing_s'] = None
            task['execution_s'] = None

    n = len(tasks)
    dispatch_success_rate = 100.0 * sum(1 for t in tasks if t['dispatch_success']) / n if n else None
    latencies = [t['planning_latency_s'] for t in tasks if t['planning_latency_s'] is not None]
    makespans = [t['makespan_s'] for t in tasks if t['makespan_s'] is not None]
    makespans_incl_transit = [t['makespan_incl_transit_s'] for t in tasks if t['makespan_incl_transit_s'] is not None]
    queueings = [t['queueing_s'] for t in tasks if t['queueing_s'] is not None]
    executions = [t['execution_s'] for t in tasks if t['execution_s'] is not None]
    n_makespan_unresolved = sum(1 for t in tasks if t['makespan_s'] is None)

    makespan_resolution_rate_pct = 100.0 * (n - n_makespan_unresolved) / n if n else None

    # ---- per-robot breakdown ----
    per_robot = {}
    for name in args.robots:
        robot_tasks = [t for t in tasks if t['robot'] == name]
        per_robot[name] = {
            'n_tasks': len(robot_tasks),
            'makespan_s': summarize([t['makespan_s'] for t in robot_tasks if t['makespan_s'] is not None]),
        }

    negotiation_counts = {topic: len(records.get(topic, [])) for topic in NEGOTIATION_TOPICS}
    blockade_counts = {topic: len(records.get(topic, [])) for topic in BLOCKADE_TOPICS}

    result = {
        'scenario': args.scenario,
        'bag_path': args.bag,
        'robots': args.robots,
        'n_tasks': n,
        'n_makespan_unresolved': n_makespan_unresolved,
        'makespan_resolution_rate_pct': makespan_resolution_rate_pct,
        'per_task': tasks,
        'per_robot': per_robot,
        '1a_task_dispatch_success_rate_pct': dispatch_success_rate,
        '1b_planning_latency_s': summarize(latencies),
        '1c_makespan_s': summarize(makespans),
        '1c_makespan_incl_transit_s': summarize(makespans_incl_transit),
        '1c_queueing_s': summarize(queueings),
        '1c_execution_s': summarize(executions),
        'negotiation_message_counts': negotiation_counts,
        'blockade_message_counts': blockade_counts,
    }

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2, default=str)

    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
