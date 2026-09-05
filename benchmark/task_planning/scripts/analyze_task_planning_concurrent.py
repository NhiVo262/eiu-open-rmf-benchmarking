#!/usr/bin/env python3
"""
Concurrent-submission counterpart of analyze_task_planning.py, for any
scenario with N >= 2 robots dispatched together to produce real traffic
interaction (Crossing, Bottleneck, Shared lane, Head-on...). Pairs with
run_benchmark_concurrent.py the same way analyze_task_planning.py pairs
with run_benchmark.py.

Metrics (same definitions/formulas as analyze_task_planning.py):
  1a. Task dispatch success rate
      - /rmf_task/bid_notice       (BidNotice)        -> denominator: bid notices submitted
      - /rmf_task/dispatch_request (DispatchCommand)   -> maps task_id -> dispatch_id
      - /rmf_task/dispatch_ack     (DispatchAck)        -> maps dispatch_id -> success (bool)

  1b. Planning latency
      - /rmf_task/bid_notice   (BidNotice)
      - /rmf_task/bid_response (BidResponse)  -- its `.proposal` field is what the
        report's "bid_proposal" data source refers to.
      L_plan = bag_time(bid_response) - bag_time(bid_notice), matched by task_id.

  1c. Makespan
      Same substitute as the single-robot script (/task_state not available):
      - t_start = bag time of the ApiRequest that produced this task (matched by
        chronological submission order across ALL tasks in the bag, since
        run_benchmark_concurrent.py submits 2+ ApiRequests back-to-back per repeat).
      - t_end   = last moment the OWNING robot's position was still changing
        (> 1cm between consecutive /fleet_states samples), bounded to the window
        where that robot's fleet_states.task_id equals this task_id. The owning
        robot is discovered directly from /fleet_states (whichever robot carries
        this task_id), which is what makes this safe for N robots running
        different concurrent tasks -- unlike the single-robot script's "next
        bid_notice" window bound, which breaks when 2+ tasks are submitted at
        nearly the same time.
      A task where /fleet_states never showed an owning robot is left out of
      the 1c mean (n_makespan_unresolved), but is a real failure, not missing
      data -- every case checked corresponds to a robot the runner's own
      repeats_summary.json records as having travelled 0.0m. See
      makespan_resolution_rate_pct for how much of n_tasks the 1c mean above
      actually covers.

  1c, decomposed: queueing_s and execution_s split makespan into the part the
      dispatcher controls and the part traffic/execution costs, using the
      award time already in /rmf_task/dispatch_request -- no extra recording
      needed. queueing_s = t_award - t_submit, execution_s = t_end - t_award.
      Worth checking directly for scenarios run with a bidding_time_window
      much larger than its 2.0s default (e.g. 60s): with the dispatcher
      holding only one auction open at a time, a task submitted late in a
      concurrent batch waits proportionally longer for its own bid window to
      open, and that wait is queueing_s, not execution_s -- makespan alone
      does not distinguish the two.

  Bonus for concurrent scenarios: counts of /rmf_traffic/negotiation_* and
  /rmf_traffic/blockade_* messages across the whole bag, as evidence real
  traffic negotiation occurred between the robots (not part of the report's
  slide-24 formulas, purely diagnostic).

Assumes tasks are submitted in short concurrent bursts per repeat (one burst
per repeat, tasks within a burst going to different robots) -- matches
run_benchmark_concurrent.py's protocol.

Example:
    python3 analyze_task_planning_concurrent.py \\
        --bag .../crossing_FLEET02_TRAF01_CONFIG01/run_.../bag \\
        --robots tb3_robot1 tb3_robot3 \\
        --scenario "Crossing, N=2, TRAF_01, CONFIG_01" \\
        --output .../run_.../task_planning_metrics.json
"""
import argparse
import json
import math
import statistics as st

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

POS_EPS = 0.01  # meters, movement threshold between consecutive fleet_states samples

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

    # 1a: dispatch success -- task_id -> dispatch_id (dispatch_request) -> success (dispatch_ack)
    #
    # Use the FIRST dispatch_ack seen per dispatch_id, not the last. If the
    # fleet adapter process is restarted mid-run (e.g. as a mitigation for a
    # memory leak, see headon_FLEET0N_TRAF04_CONFIG01/PROCEDURE.md), the
    # dispatcher re-broadcasts success=False for every dispatch_id it still
    # remembers as ever having been active, once per adapter reconnection --
    # this is bookkeeping churn about the OLD connection dropping, not a real
    # failure signal for tasks that already completed. The genuine real-time
    # ack always arrives within milliseconds of the matching dispatch_request,
    # so taking the first occurrence per dispatch_id recovers the true result.
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

    # t_award: bag time of the dispatch_request that awarded this task_id --
    # already in the bag, no extra recording needed. Used below to split
    # makespan into queueing (dispatcher's own contribution, tunable via
    # bidding_time_window) vs. execution (what the drive/traffic cost).
    t_award_by_task = {msg.task_id: t for t, msg in dispatch_request}
    for task in tasks:
        task['t_award'] = t_award_by_task.get(task['task_id'])

    # t_submit: pair task_api_requests to tasks by submission order. Concurrent
    # bursts (2+ ApiRequests per repeat, back-to-back) still preserve relative
    # chronological order against the equally-ordered bid_notice_sorted list.
    api_req_sorted = sorted(task_api_requests, key=lambda x: x[0])
    for i, task in enumerate(tasks):
        task['t_submit'] = api_req_sorted[i][0] if i < len(api_req_sorted) else task['t_bid_notice']

    # 1c: makespan. For each task, find its owning robot from /fleet_states
    # (whichever robot carries this task_id), then bound the position-settling
    # window to [t_submit, last time that robot still had this task_id] --
    # this is what makes it safe when 2+ different tasks run concurrently for
    # different robots, unlike bounding by "next bid_notice" chronologically.
    fs_sorted = sorted(fleet_states, key=lambda x: x[0])

    def robot_state(msg, name):
        for r in msg.robots:
            if r.name == name:
                return r
        return None

    for task in tasks:
        task_id = task['task_id']
        owning_robot = None
        last_seen_with_task_t = task['t_submit']
        for t, msg in fs_sorted:
            for name in args.robots:
                r = robot_state(msg, name)
                if r is not None and r.task_id == task_id:
                    owning_robot = name
                    last_seen_with_task_t = t
                    break
        task['robot'] = owning_robot

        if owning_robot is None:
            # fleet_states.task_id was observed to sometimes stay "stuck" on the
            # previous task_id and never update to this one (seen for the final
            # repeat of a run, where there is no subsequent repeat to trigger a
            # state refresh) -- makespan can't be reliably derived from the bag
            # in that case. Leave it unset rather than reporting a fake 0.0,
            # which would corrupt the 1c aggregate.
            task['t_end_physical'] = None
            task['makespan_s'] = None
            task['queueing_s'] = None
            task['execution_s'] = None
            continue

        window_start = task['t_submit']
        window_end = last_seen_with_task_t
        window = [(t, m) for t, m in fs_sorted if window_start <= t <= window_end]

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

    # ---- aggregate (pooled across all robots/tasks) ----
    n = len(tasks)
    dispatch_success_rate = 100.0 * sum(1 for t in tasks if t['dispatch_success']) / n if n else None
    latencies = [t['planning_latency_s'] for t in tasks if t['planning_latency_s'] is not None]
    makespans = [t['makespan_s'] for t in tasks if t['makespan_s'] is not None]
    queueings = [t['queueing_s'] for t in tasks if t['queueing_s'] is not None]
    executions = [t['execution_s'] for t in tasks if t['execution_s'] is not None]
    n_makespan_unresolved = sum(1 for t in tasks if t['makespan_s'] is None)
    # "Unresolved" means fleet_states never showed a robot holding this
    # task_id long enough to bound a window -- in every case checked against
    # the runner's own repeats_summary.json, that robot's total_distance_m
    # was 0.0. These are real task failures, not missing data, and the mean
    # above is computed over n - n_makespan_unresolved tasks, not n --
    # resolution_rate_pct makes that gap visible instead of implicit.
    makespan_resolution_rate_pct = 100.0 * (n - n_makespan_unresolved) / n if n else None

    # ---- per-robot breakdown ----
    per_robot = {}
    for name in args.robots:
        robot_tasks = [t for t in tasks if t['robot'] == name]
        per_robot[name] = {
            'n_tasks': len(robot_tasks),
            'makespan_s': summarize([t['makespan_s'] for t in robot_tasks if t['makespan_s'] is not None]),
        }

    # ---- negotiation/blockade diagnostics (whole-bag counts) ----
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
