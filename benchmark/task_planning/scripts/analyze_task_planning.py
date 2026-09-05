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
      - /fleet_states       (FleetState) -> t_end, taken as the last moment the robot
        actually held this task_id and its position was still changing (physical
        completion). A task the robot was never seen holding in /fleet_states at
        all (e.g. dispatch accepted but Nav2 never got a moving goal) is left out
        of the 1c mean (n_makespan_unresolved) rather than collapsing to a fake
        0.0s just because the scan window never advanced past t_submit -- see
        makespan_resolution_rate_pct for how much of n_repeats the mean covers.

  1c, decomposed: queueing_s and execution_s split makespan into the part the
      dispatcher controls and the part traffic/execution costs, using the
      award time already in /rmf_task/dispatch_request -- no extra recording
      needed. queueing_s = t_award - t_submit, execution_s = t_end - t_award.

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
    #
    # Use the FIRST dispatch_ack seen per dispatch_id, not the last. If the
    # fleet adapter process is restarted mid-run, the dispatcher re-broadcasts
    # success=False for every dispatch_id it still remembers as ever having
    # been active, once per adapter reconnection -- this is bookkeeping churn
    # about the OLD connection dropping, not a real failure signal for tasks
    # that already completed. The genuine real-time ack always arrives within
    # milliseconds of the matching dispatch_request, so taking the first
    # occurrence per dispatch_id recovers the true result.
    dispatch_id_by_task = {}
    t_award_by_task = {}
    for t, msg in dispatch_request:
        dispatch_id_by_task[msg.task_id] = msg.dispatch_id
        t_award_by_task[msg.task_id] = t
    success_by_dispatch_id = {}
    for t, msg in sorted(dispatch_ack, key=lambda x: x[0]):
        if msg.dispatch_id not in success_by_dispatch_id:
            success_by_dispatch_id[msg.dispatch_id] = msg.success
    for task in tasks:
        dispatch_id = dispatch_id_by_task.get(task['task_id'])
        task['dispatch_id'] = dispatch_id
        task['dispatch_success'] = success_by_dispatch_id.get(dispatch_id, False)
        task['t_award'] = t_award_by_task.get(task['task_id'])

    # t_submit: pair task_api_requests to tasks by submission order (one ApiRequest
    # precedes each bid_notice under the sequential single-task-at-a-time protocol)
    api_req_sorted = sorted(task_api_requests, key=lambda x: x[0])
    for i, task in enumerate(tasks):
        task['t_submit'] = api_req_sorted[i][0] if i < len(api_req_sorted) else task['t_bid_notice']

    # 1c: makespan via fleet_states position settling, within window
    # [t_submit, next task's t_bid_notice). Movement itself is still scanned
    # from EVERY position sample in the window, unconditionally, exactly as
    # before -- fleet_states.task_id can clear for this robot slightly before
    # its final leg physically finishes decelerating, and gating each sample
    # on task_id (as the concurrent, multi-robot analyzer does, where it's
    # needed to tell robots' positions apart) was tried and found to clip
    # that tail, silently shortening every makespan by up to ~13s against
    # the previously-verified published numbers. task_id is used only
    # separately below, to tell a genuine "never assigned" failure apart
    # from a task that really did run -- not to filter which samples count.
    fs_sorted = sorted(fleet_states, key=lambda x: x[0])

    def robot_state(msg):
        for r in msg.robots:
            if r.name == args.robot:
                return r
        return None

    for i, task in enumerate(tasks):
        task_id = task['task_id']
        window_start = task['t_submit']
        window_end = tasks[i + 1]['t_bid_notice'] if i + 1 < len(tasks) else fs_sorted[-1][0] + 1
        window = [(t, m) for t, m in fs_sorted if window_start <= t <= window_end]

        task_ever_held = False
        last_moving_t = window_start
        prev_xy = None
        for t, msg in window:
            r = robot_state(msg)
            if r is None:
                continue
            if r.task_id == task_id:
                task_ever_held = True
            xy = (r.location.x, r.location.y)
            if prev_xy is not None:
                d = math.hypot(xy[0] - prev_xy[0], xy[1] - prev_xy[1])
                if d > POS_EPS:
                    last_moving_t = t
            prev_xy = xy

        if not task_ever_held:
            # fleet_states never showed this robot holding this task_id in its
            # own window -- a real task failure (dispatch accepted but
            # execution never actually started), not missing data. Leave it
            # unset rather than reporting a fake 0.0, which would corrupt the
            # 1c aggregate.
            task['t_end_physical'] = None
            task['makespan_s'] = None
            task['queueing_s'] = None
            task['execution_s'] = None
            continue

        task['t_end_physical'] = last_moving_t
        task['makespan_s'] = last_moving_t - task['t_submit']

        # Decompose: queueing is the dispatcher's own contribution (tunable
        # via bidding_time_window), execution is what the drive/traffic cost
        # on top of it. Only computable when t_award was actually observed.
        if task['t_award'] is not None:
            task['queueing_s'] = task['t_award'] - task['t_submit']
            task['execution_s'] = last_moving_t - task['t_award']
        else:
            task['queueing_s'] = None
            task['execution_s'] = None

    # ---- aggregate ----
    n = len(tasks)
    dispatch_success_rate = 100.0 * sum(1 for t in tasks if t['dispatch_success']) / n if n else None
    latencies = [t['planning_latency_s'] for t in tasks if t['planning_latency_s'] is not None]
    makespans = [t['makespan_s'] for t in tasks if t['makespan_s'] is not None]
    queueings = [t['queueing_s'] for t in tasks if t['queueing_s'] is not None]
    executions = [t['execution_s'] for t in tasks if t['execution_s'] is not None]
    n_makespan_unresolved = sum(1 for t in tasks if t['makespan_s'] is None)
    # "Unresolved" means fleet_states never showed this robot holding this
    # task_id long enough to bound a window -- a real task failure, not
    # missing data. The mean above is computed over n - n_makespan_unresolved
    # tasks, not n -- resolution_rate_pct makes that gap visible instead of
    # implicit.
    makespan_resolution_rate_pct = 100.0 * (n - n_makespan_unresolved) / n if n else None

    result = {
        'scenario': args.scenario,
        'bag_path': args.bag,
        'n_repeats': n,
        'n_makespan_unresolved': n_makespan_unresolved,
        'makespan_resolution_rate_pct': makespan_resolution_rate_pct,
        'per_task': tasks,
        '1a_task_dispatch_success_rate_pct': dispatch_success_rate,
        '1b_planning_latency_s': summarize(latencies),
        '1c_makespan_s': summarize(makespans),
        '1c_queueing_s': summarize(queueings),
        '1c_execution_s': summarize(executions),
    }

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2, default=str)

    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
