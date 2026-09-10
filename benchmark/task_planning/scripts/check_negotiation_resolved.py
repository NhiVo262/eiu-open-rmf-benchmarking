#!/usr/bin/env python3
"""
Reads /rmf_traffic/negotiation_conclusion directly from a bag and reports how
many of its messages have .resolved == True vs False.

Why this exists separately from analyze_task_planning_concurrent.py:
that script only counts *how many* messages arrived on each /rmf_traffic/
negotiation_* topic (negotiation_message_counts), as a purely diagnostic
signal that real traffic negotiation happened. It never opens the message
body, so it can't tell a negotiation that concluded resolved from one that
concluded abandoned -- both are just one more NegotiationConclusion message
in that count. `.resolved` is the one field that actually says which.

Forfeits (/rmf_traffic/negotiation_forfeit) are deliberately not counted as
an outcome here -- a forfeit is a step inside an ongoing negotiation, not a
conclusion of one; only negotiation_conclusion.resolved answers "did this
negotiation end resolved or abandoned."

Usage:
  python3 check_negotiation_resolved.py --bag <path-to-bag-dir>
"""
import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_resolved_counts(bag_path: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic = "/rmf_traffic/negotiation_conclusion"
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return 0, 0
    msg_type = get_message(type_map[topic])

    total = 0
    resolved = 0
    while reader.has_next():
        t, data, _ts = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(data, msg_type)
        total += 1
        if bool(msg.resolved):
            resolved += 1
    return resolved, total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to the rosbag2 directory")
    args = parser.parse_args()

    resolved, total = read_resolved_counts(args.bag)
    if total == 0:
        print("No /rmf_traffic/negotiation_conclusion messages in this bag.")
        return
    pct = 100.0 * resolved / total
    print(f"resolved: {resolved}/{total} ({pct:.1f}%)")
    print(f"abandoned: {total - resolved}/{total} ({100.0 - pct:.1f}%)")


if __name__ == "__main__":
    main()
