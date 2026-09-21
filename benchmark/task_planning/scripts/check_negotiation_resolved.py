#!/usr/bin/env python3

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
