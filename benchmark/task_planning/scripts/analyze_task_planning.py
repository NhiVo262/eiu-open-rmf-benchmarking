#!/usr/bin/env python3
"""

  1a. Task dispatch success rate
      - /rmf_task/bid_notice       (BidNotice)        -> denominator: bid notices submitted
      - /rmf_task/dispatch_request (DispatchCommand)   -> maps task_id -> dispatch_id
      - /rmf_task/dispatch_ack     (DispatchAck)        -> maps dispatch_id -> success (bool)


  1b. Planning latency
      - /rmf_task/bid_notice   (BidNotice)
      - /rmf_task/bid_response (BidResponse)  -- its `.proposal` field is what the
        report's "bid_proposal" data source refers to; there is no separate
        `bid_proposal` topic in this deployment, so /rmf_task/bid_response is used.
      L_plan = bag_time(bid_response) - bag_time(bid_notice), matched by task_id.
      (Neither message carries its own header.stamp, so the rosbag *recording*
      timestamp is used as the closest available proxy.)

  1c. Makespan
      Report specifies /task_state (via RMF API Server, booking.unix_millis_request_time
      / unix_millis_finish_time). This deployment does not run rmf_api_server, and
      /task_summaries (the ROS analogue) is not populated by this fleet adapter for
      patrol tasks (0 messages recorded) -- confirmed empirically. As a documented
      substitute:
      - /task_api_requests (ApiRequest) -> t_start (~ booking.unix_millis_request_time)
      - /fleet_states       (FleetState) -> t_end, taken as the last moment the robot's
        position was still changing before it settled (physical completion)

Assumes tasks were submitted sequentially, one at a time (single robot,
single active task at any moment) -- matches run_benchmark.py's protocol.

Example:
    python3 analyze_task_planning.py \\
        --bag .../run_20260811_1306/bag --robot tb3_robot1 \\
        --scenario "Clear path (baseline), N=1, TRAF_00, CONFIG_01" \\
        --output .../run_20260811_1306/task_planning_metrics.json
"""
import argparse
import json
import math
import statistics as st

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

POS_EPS = 0.01  # meters, movement threshold between consecutive fleet_states samples


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
    ap = argparse.ArgumentParser(description='Compute Task Planning metrics from a rosbag')
    ap.add_argument('--bag', required=True, help='Path to the rosbag directory')
    ap.add_argument('--robot', default='tb3_robot1', help='Robot name to track in /fleet_states')
    ap.add_argument('--scenario', required=True, help='Human-readable scenario label')
    ap.add_argument('--output', required=True, help='Path to write the metrics JSON to')
    return ap.parse_args()


def main():
    args = parse_args()
    records = read_bag(args.bag)

    bid_notice = records.get('/rmf_task/bid_notice', [])
    bid_response = records.get('/rmf_task/bid_response', [])
    dispatch_request = records.get('/rmf_task/dispatch_request', [])
    dispatch_ack = records.get('/rmf_task/dispatch_ack', [])
    task_api_requests = records.get('/task_api_requests', [])
    fleet_states = records.get('/fleet_states', [])

    # Order tasks by bid_notice bag time (== submission order, one per repeat)
    bid_notice_sorted = sorted(bid_notice, key=lambda x: x[0])
    tasks = [{'task_id': msg.task_id, 't_bid_notice': t} for t, msg in bid_notice_sorted]

    # 1b: match bid_response by task_id (BidResponse.proposal == report's "bid_proposal")
    bid_response_by_id = {}
    for t, msg in bid_response:
        bid_response_by_id.setdefault(msg.task_id, t)
    for task in tasks:
        task['t_bid_response'] = bid_response_by_id.get(task['task_id'])
        task['planning_latency_s'] = (
            task['t_bid_response'] - task['t_bid_notice']
            if task['t_bid_response'] is not None else None
        )

    # 1a: dispatch success -- task_id -> dispatch_id (dispatch_request) -> success (dispatch_ack)
    dispatch_id_by_task = {}
    for t, msg in dispatch_request:
        dispatch_id_by_task[msg.task_id] = msg.dispatch_id
    success_by_dispatch_id = {}
    for t, msg in dispatch_ack:
        success_by_dispatch_id[msg.dispatch_id] = msg.success
    for task in tasks:
        dispatch_id = dispatch_id_by_task.get(task['task_id'])
        task['dispatch_id'] = dispatch_id
        task['dispatch_success'] = success_by_dispatch_id.get(dispatch_id, False)

    # t_submit: pair task_api_requests to tasks by submission order (one ApiRequest
    # precedes each bid_notice under the sequential single-task-at-a-time protocol)
    api_req_sorted = sorted(task_api_requests, key=lambda x: x[0])
    for i, task in enumerate(tasks):
        task['t_submit'] = api_req_sorted[i][0] if i < len(api_req_sorted) else task['t_bid_notice']

    # 1c: makespan via fleet_states position settling, within window
    # [t_submit, next task's t_bid_notice)
    fs_sorted = sorted(fleet_states, key=lambda x: x[0])

    def robot_xy(msg):
        for r in msg.robots:
            if r.name == args.robot:
                return (r.location.x, r.location.y)
        return None

    for i, task in enumerate(tasks):
        window_start = task['t_submit']
        window_end = tasks[i + 1]['t_bid_notice'] if i + 1 < len(tasks) else fs_sorted[-1][0] + 1
        window = [(t, m) for t, m in fs_sorted if window_start <= t <= window_end]

        last_moving_t = window_start
        prev_xy = None
        for t, msg in window:
            xy = robot_xy(msg)
            if xy is None:
                continue
            if prev_xy is not None:
                d = math.hypot(xy[0] - prev_xy[0], xy[1] - prev_xy[1])
                if d > POS_EPS:
                    last_moving_t = t
            prev_xy = xy

        task['t_end_physical'] = last_moving_t
        task['makespan_s'] = last_moving_t - task['t_submit']

    # ---- aggregate ----
    n = len(tasks)
    dispatch_success_rate = 100.0 * sum(1 for t in tasks if t['dispatch_success']) / n if n else None
    latencies = [t['planning_latency_s'] for t in tasks if t['planning_latency_s'] is not None]
    makespans = [t['makespan_s'] for t in tasks]

    result = {
        'scenario': args.scenario,
        'bag_path': args.bag,
        'n_repeats': n,
        'per_task': tasks,
        '1a_task_dispatch_success_rate_pct': dispatch_success_rate,
        '1b_planning_latency_s': summarize(latencies),
        '1c_makespan_s': summarize(makespans),
    }

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2, default=str)

    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
