#!/usr/bin/env python3
"""Computes the Feature 2 (Traffic Scheduling) metrics from a concurrent-submission
(N>=2) bag.

Emits two rates:

  2a_schedule_feasibility_pct      did every detected conflict end with a valid
                                   schedule, over N_submit from /task_api_requests
  2a_conflict_free_scheduling_pct  did an itinerary enter the schedule without
                                   triggering a negotiation, over N_itineraries
                                   from /rmf_traffic/itinerary_set

and two latencies, which have different denominators:

  planner compute    bid_notice -> bid_response for one task, bounding the fleet
                     adapter's run of rmf_task::TaskPlanner
  schedule confirm   itinerary_set -> the first query_update_1 whose patch carries
                     that (participant, itinerary_version); contains no planning

A conflict makes a task infeasible only when an abandoned negotiation left no
subsequent valid itinerary within RECOVERY_WINDOW_S. per_conflict[] carries each
episode's outcome tag, which the feasibility verdict derives from.

METRIC_DESIGN.md defines the measurands.
"""
import argparse
import json
import os
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'common'))
from metric_statistics import summarize  # noqa: E402

RECOVERY_WINDOW_S = 5.0


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
        description='Compute Traffic Scheduling metrics (2a, 2b) from a rosbag')
    ap.add_argument('--bag', required=True, help='Path to the rosbag directory')
    ap.add_argument('--robots', nargs='+', required=True,
                     help='Robot names expected to appear in /fleet_states')
    ap.add_argument('--scenario', required=True, help='Human-readable scenario label')
    ap.add_argument('--output', required=True, help='Path to write the metrics JSON to')
    return ap.parse_args()


def main():
    args = parse_args()
    records = read_bag(args.bag)

    task_api_requests = records.get('/task_api_requests', [])
    fleet_states = sorted(records.get('/fleet_states', []), key=lambda x: x[0])
    participants_msgs = records.get('/rmf_traffic/participants', [])
    negotiation_notice = records.get('/rmf_traffic/negotiation_notice', [])
    negotiation_conclusion = records.get('/rmf_traffic/negotiation_conclusion', [])
    itinerary_set = sorted(records.get('/rmf_traffic/itinerary_set', []), key=lambda x: x[0])
    query_update = sorted(records.get('/rmf_traffic/query_update_1', []), key=lambda x: x[0])

    # ---- participant_id -> robot name, from /rmf_traffic/participants ----
    participant_name = {}
    for _, msg in participants_msgs:
        for p in msg.participants:
            participant_name[p.id] = p.description.name

    # ---- conflict_version -> (participants, first-notice bag time) ----
    conflict_participants = {}
    conflict_notice_time = {}
    for t, msg in sorted(negotiation_notice, key=lambda x: x[0]):
        conflict_participants.setdefault(msg.conflict_version, list(msg.participants))
        conflict_notice_time.setdefault(msg.conflict_version, t)

    # ---- conclusion per conflict_version (time + resolved flag) ----
    conclusion = {}
    for t, msg in sorted(negotiation_conclusion, key=lambda x: x[0]):
        conclusion[msg.conflict_version] = (t, bool(msg.resolved))

    def robot_state(msg, name):
        """The named robot's entry in one /fleet_states message, or None."""
        for r in msg.robots:
            if r.name == name:
                return r
        return None

    def task_active_on_robot_at(robot_name, t_query):
        """The task_id the robot held at t_query, or None."""
        # The /fleet_states sample at or before t_query gives the task_id the robot
        # was carrying then.
        active = None
        for t, msg in fleet_states:
            if t > t_query:
                break
            r = robot_state(msg, robot_name)
            if r is not None:
                active = r.task_id
        return active

    def recovered_after(t_concl, participants):
        """Whether a participant established a new itinerary within RECOVERY_WINDOW_S
        of an abandoned negotiation: the replan recovery path."""
        for t_set, msg in itinerary_set:
            if t_set < t_concl:
                continue
            if t_set > t_concl + RECOVERY_WINDOW_S:
                break
            if msg.participant in participants:
                return {
                    'participant': participant_name.get(msg.participant, msg.participant),
                    'itinerary_version': msg.itinerary_version,
                    'delay_s': round(t_set - t_concl, 4),
                }
        return None

    # ---- 2a: schedule feasibility ----
    n_submit = len(task_api_requests)
    infeasible_task_ids = set()
    per_conflict = []

    for cv, (t_concl, resolved) in conclusion.items():
        parts = conflict_participants.get(cv, [])
        entry = {
            'conflict_version': cv,
            'resolved': resolved,
            'participants': [participant_name.get(p, p) for p in parts],
        }
        if resolved:
            entry['outcome'] = 'resolved_by_negotiation'
        else:
            recovery = recovered_after(t_concl, parts)
            if recovery is not None:
                entry['outcome'] = 'recovered_by_replan'
                entry['recovery'] = recovery
            else:
                entry['outcome'] = 'unrecovered'
                t_conflict = conflict_notice_time.get(cv)
                affected = []
                if t_conflict is not None:
                    for pid in parts:
                        robot_name = participant_name.get(pid)
                        if robot_name is None or robot_name not in args.robots:
                            continue
                        tid = task_active_on_robot_at(robot_name, t_conflict)
                        if tid:
                            infeasible_task_ids.add(tid)
                            affected.append(tid)
                entry['infeasible_task_ids'] = affected
        per_conflict.append(entry)

    n_task_infeasible = len(infeasible_task_ids)
    schedule_feasibility_pct = (
        100.0 * (n_submit - n_task_infeasible) / n_submit if n_submit else None
    )

    outcome_counts = {}
    for c in per_conflict:
        outcome_counts[c['outcome']] = outcome_counts.get(c['outcome'], 0) + 1

    # ---- 2a (ii): conflict-free scheduling rate ----
    # Each notice is charged to the participant's most recent itinerary at or before it.
    itinerary_by_participant = {}
    for t_set, msg in itinerary_set:
        itinerary_by_participant.setdefault(msg.participant, []).append(
            (t_set, msg.itinerary_version))
    triggered = set()
    for t_notice, msg in negotiation_notice:
        for pid in msg.participants:
            prior = [(t, v) for t, v in itinerary_by_participant.get(pid, [])
                     if t <= t_notice]
            if prior:
                triggered.add((pid, max(prior)[1]))
    n_itineraries = len(itinerary_set)
    conflict_free_pct = (
        100.0 * (n_itineraries - len(triggered)) / n_itineraries
        if n_itineraries else None)

    # ---- 2b (i): planner compute, bid_notice -> bid_response ----
    # Matched on task_id; a notice with no response is absent from the sample.
    bid_notice = sorted(records.get('/rmf_task/bid_notice', []), key=lambda x: x[0])
    bid_response = sorted(records.get('/rmf_task/bid_response', []), key=lambda x: x[0])
    notice_at = {}
    for t_n, msg in bid_notice:
        notice_at.setdefault(msg.task_id, t_n)
    planner_compute_s = []
    for t_r, msg in bid_response:
        t_n = notice_at.pop(msg.task_id, None)
        if t_n is not None and t_r >= t_n:
            planner_compute_s.append(t_r - t_n)

    # ---- 2b (ii): schedule confirmation, itinerary_set -> mirror ----
    # The first query_update_1 at or after the set whose patch carries the same
    # (participant_id, itinerary_version) pair.
    scheduling_times_s = []
    for t_set, msg in itinerary_set:
        pid = msg.participant
        iv = msg.itinerary_version
        for t_qu, qmsg in query_update:
            if t_qu < t_set:
                continue
            matched = any(
                pp.participant_id == pid and pp.itinerary_version == iv
                for pp in qmsg.patch.participants
            )
            if matched:
                scheduling_times_s.append(t_qu - t_set)
                break

    result = {
        'scenario': args.scenario,
        'bag_path': args.bag,
        'robots': args.robots,
        'n_submit': n_submit,
        'n_conflicts_detected': len(conclusion),
        '2a_method': (
            'schedule feasibility: a conflict makes its tasks infeasible only if the '
            f'negotiation ended unresolved AND no participant established a new '
            f'itinerary within {RECOVERY_WINDOW_S}s (replan recovery)'
        ),
        '2a_method_conflict_free': (
            'conflict-free scheduling rate = itineraries that did not trigger a '
            'negotiation / all itineraries; an itinerary triggered one when its '
            'participant is in a negotiation_notice and it is that participant\'s '
            'most recent itinerary at or before the notice'
        ),
        '2a_conflict_free_scheduling_pct': conflict_free_pct,
        '2a_n_itineraries': n_itineraries,
        '2a_n_itineraries_triggering_negotiation': len(triggered),
        '2a_n_task_infeasible': n_task_infeasible,
        '2a_schedule_feasibility_pct': schedule_feasibility_pct,
        '2a_conflict_outcome_counts': outcome_counts,
        '2b_method': (
            'planner compute = bid_response - bid_notice (one per task, bounds '
            'rmf_task::TaskPlanner); schedule confirm = query_update_1 - '
            'itinerary_set (one per itinerary, contains no planning). '
            '2b_scheduling_time_s is the schedule-confirm figure under its '
            'original key, kept so metrics files already written stay readable'
        ),
        '2b_planner_compute_s': summarize(planner_compute_s),
        'raw_planner_compute_s': planner_compute_s,
        '2b_schedule_confirm_s': summarize(scheduling_times_s),
        '2b_scheduling_time_s': summarize(scheduling_times_s),
        'raw_scheduling_times_s': scheduling_times_s,
        'per_conflict': per_conflict,
    }

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2, default=str)

    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
