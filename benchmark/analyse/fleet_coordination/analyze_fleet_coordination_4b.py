#!/usr/bin/env python3
"""Computes 4b, average robot utilisation, and the repeat's throughput, from one bag.

The window is the repeat's execution window, the same for every robot:

  T_window   [min(t_award), max(t_completed)] over the repeat's tasks, read from
             task_planning_metrics.json

A robot is moving while the last command on its /<robot>/cmd_vel is no older than
STALE_S and has a linear speed of at least CMD_LINEAR_MPS or a turn rate of at least
CMD_ANGULAR_RPS. The robots scored are the robots that carried a task; one whose
cmd_vel holds no message is reported as null and left out of U_avg, and
4b_n_robots_scored says how many robots the mean covers.

  U_r        T_moving(r) / T_window
  U_avg      mean of U_r over those robots
  throughput completed tasks / T_window, in tasks per minute

A repeat with a task that never completed has no window, and every figure is null.
Writes analysis_version 2. METRIC_DESIGN.md §F4 defines the measurands.

Usage, inside the open-rmf container:
    python3 analyze_fleet_coordination_4b.py --task-planning-metrics <json> --bag <bag> \\
        --scenario <label> --output <json>
"""
import argparse
import json

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

ANALYSIS_VERSION = 2
CMD_LINEAR_MPS = 0.005
CMD_ANGULAR_RPS = 0.05
STALE_S = 0.25        # a command older than this no longer drives the robot


def read_commands(bag, robots):
    """{robot: [(t, moving)]} from each robot's cmd_vel, in bag receive time."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    topics = {'/%s/cmd_vel' % r: r for r in robots if '/%s/cmd_vel' % r in types}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    out = {r: [] for r in robots}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        moving = abs(msg.linear.x) >= CMD_LINEAR_MPS or abs(msg.angular.z) >= CMD_ANGULAR_RPS
        out[topics[topic]].append((t_ns / 1e9, moving))
    return out


def moving_time(commands, t_start, t_end):
    """Seconds within [t_start, t_end] in which the held command moves the robot."""
    total = 0.0
    for i, (t, moving) in enumerate(commands):
        if not moving:
            continue
        hold_end = t + STALE_S
        if i + 1 < len(commands):
            hold_end = min(hold_end, commands[i + 1][0])
        total += max(0.0, min(hold_end, t_end) - max(t, t_start))
    return total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--task-planning-metrics', required=True, help='task_planning_metrics.json for the same repeat')
    ap.add_argument('--bag', required=True, help='Path to the rosbag2 directory of one repeat')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    tasks = json.load(open(args.task_planning_metrics))['per_task']
    robots = sorted({t['robot'] for t in tasks if t.get('robot')})
    complete = tasks and all(t.get('t_completed') and t.get('t_award') for t in tasks)

    result = {
        'scenario': args.scenario,
        'bag': args.bag,
        'analysis_version': ANALYSIS_VERSION,
        '4b_method': ('cmd_vel moving if |v| >= %.3f m/s or |w| >= %.2f rad/s, held at most %.2f s; '
                      'window [min award, max completed] over the repeat'
                      % (CMD_LINEAR_MPS, CMD_ANGULAR_RPS, STALE_S)),
        '4b_window_s': None,
        '4b_per_robot': None,
        '4b_u_avg_pct': None,
        'n_tasks_completed': sum(1 for t in tasks if t.get('t_completed')),
        'throughput_tasks_per_min': None,
    }
    if complete:
        t_start = min(t['t_award'] for t in tasks)
        t_end = max(t['t_completed'] for t in tasks)
        window = t_end - t_start
        commands = read_commands(args.bag, robots)
        per_robot = {}
        for r in robots:
            if not commands[r]:
                per_robot[r] = {'t_moving_s': None, 'u_pct': None, 'n_cmd_vel': 0}
                continue
            t_mov = moving_time(commands[r], t_start, t_end)
            per_robot[r] = {'t_moving_s': round(t_mov, 3), 'u_pct': round(100.0 * t_mov / window, 2),
                            'n_cmd_vel': len(commands[r])}
        scored = [v['u_pct'] for v in per_robot.values() if v['u_pct'] is not None]
        result.update({
            '4b_window_s': round(window, 3),
            '4b_per_robot': per_robot,
            '4b_n_robots_scored': len(scored),
            '4b_u_avg_pct': round(sum(scored) / len(scored), 2) if scored else None,
            'throughput_tasks_per_min': round(60.0 * len(tasks) / window, 4),
        })

    json.dump(result, open(args.output, 'w'), indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != '4b_per_robot'}, indent=2))


if __name__ == '__main__':
    main()
