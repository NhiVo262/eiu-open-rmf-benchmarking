#!/usr/bin/env python3
"""Computes the Feature 3 (Negotiation) metrics from one bag.

Emits 3a_n_negotiation, 3a_n_conclusions,
3a_n_distinct_conflict_versions_noticed, 3b_latency_resolved_s,
3b_latency_abandoned_s, 3b_latency_refused_s, 3b_latency_search_failed_s and their
counts.

Latency runs from a negotiation_notice to the negotiation_conclusion for the same
conflict_version, reported separately by the conclusion's resolved flag. An abandoned
negotiation is split by whether a negotiation_refusal names its conflict_version: the
schedule node answers a refusal with a conclusion whose resolved flag is false
(rmf_traffic_ros2/schedule/Node.cpp, ScheduleNode::refuse).
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

ANALYSIS_VERSION = 2
TOPICS = ('/rmf_traffic/negotiation_notice', '/rmf_traffic/negotiation_conclusion',
          '/rmf_traffic/negotiation_refusal')


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
        if topic not in TOPICS:
            continue
        msg = deserialize_message(data, msg_classes[topic])
        records[topic].append((t_ns / 1e9, msg))
    return records


def parse_args():
    """Parse the command line."""
    ap = argparse.ArgumentParser(
        description='Compute Negotiation metrics (3a, 3b) from a Traffic Scheduling rosbag')
    ap.add_argument('--bag', required=True, help='Path to the rosbag directory')
    ap.add_argument('--scenario', required=True, help='Human-readable scenario label')
    ap.add_argument('--output', required=True, help='Path to write the metrics JSON to')
    return ap.parse_args()


def main():
    args = parse_args()
    records = read_bag(args.bag)

    negotiation_notice = sorted(records.get('/rmf_traffic/negotiation_notice', []), key=lambda x: x[0])
    negotiation_conclusion = sorted(records.get('/rmf_traffic/negotiation_conclusion', []), key=lambda x: x[0])
    refused_versions = {msg.conflict_version
                        for _, msg in records.get('/rmf_traffic/negotiation_refusal', [])}

    # ---- 3a: raw negotiation_notice message count ----
    n_negotiation = len(negotiation_notice)

    # ---- first notice time per conflict_version ----
    first_notice_time = {}
    for t, msg in negotiation_notice:
        first_notice_time.setdefault(msg.conflict_version, t)

    # ---- conclusion time per conflict_version (last one, defensive only) ----
    conclusion_time = {}
    for t, msg in negotiation_conclusion:
        conclusion_time[msg.conflict_version] = (t, bool(msg.resolved))

    # ---- 3b: per-conflict latency, notice -> conclusion, split by outcome ----
    # NegotiationConclusion.msg states it in the field's own comment: "True if
    # the conflict was resolved. False if the negotiation was abandoned." The
    # capability claim says RMF *resolves* negotiations, so a latency that pools
    # both lets an instant abandonment read as fast coordination. They are
    # summarised apart; the pooled figure is kept because it is what the metric
    # table's formula (t_conclusion - t_notice) literally asks for.
    latencies_s = []
    latencies_resolved_s = []
    latencies_abandoned_s = []
    latencies_refused_s = []
    latencies_search_failed_s = []
    per_negotiation = []
    for cv, t_notice in first_notice_time.items():
        entry = conclusion_time.get(cv)
        if entry is None:
            continue
        t_conclusion, resolved = (entry if isinstance(entry, tuple)
                                  else (entry, None))
        latency = t_conclusion - t_notice
        latencies_s.append(latency)
        if resolved is True:
            latencies_resolved_s.append(latency)
        elif resolved is False:
            latencies_abandoned_s.append(latency)
            if cv in refused_versions:
                latencies_refused_s.append(latency)
            else:
                latencies_search_failed_s.append(latency)
        per_negotiation.append({
            'conflict_version': cv,
            't_notice': t_notice,
            't_conclusion': t_conclusion,
            'resolved': resolved,
            'refused': cv in refused_versions,
            'latency_s': latency,
        })

    result = {
        'scenario': args.scenario,
        'bag_path': args.bag,
        'analysis_version': ANALYSIS_VERSION,
        '3a_n_negotiation': n_negotiation,
        '3a_n_distinct_conflict_versions_noticed': len(first_notice_time),
        '3a_n_conclusions': len(negotiation_conclusion),
        '3b_negotiation_latency_s': summarize(latencies_s),
        '3b_latency_resolved_s': summarize(latencies_resolved_s),
        '3b_latency_abandoned_s': summarize(latencies_abandoned_s),
        '3b_n_resolved': len(latencies_resolved_s),
        '3b_n_abandoned': len(latencies_abandoned_s),
        '3b_latency_refused_s': summarize(latencies_refused_s),
        '3b_latency_search_failed_s': summarize(latencies_search_failed_s),
        '3b_n_refused': len(latencies_refused_s),
        '3b_n_search_failed': len(latencies_search_failed_s),
        '3b_n_refusal_messages': len(records.get('/rmf_traffic/negotiation_refusal', [])),
        '3b_target': ('below RMF\'s negotiation timeout, 15 s, fixed at '
                      'rmf_traffic_ros2/schedule/Negotiation.cpp:216 and not '
                      'overridden in this tree'),
        'per_negotiation': sorted(per_negotiation, key=lambda x: x['t_notice']),
    }

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2, default=str)

    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
