#!/usr/bin/env python3
"""Computes the Feature 6 (Congestion handling) metrics from one repeat's bag.

The measurand is the requirement's description: a robot that is stationary while it holds
an assignment. Writes analysis_version 2.

  window       each task's [t_award, t_completed], from task_planning_metrics.json
  moving       the robot's /<robot>/cmd_vel commands motion, as 4b reads it
               (analyse/fleet_coordination/analyze_fleet_coordination_4b.py)
  episode      a stretch of the window with no motion command, at least MIN_EPISODE_S long;
               shorter stretches are the hand-over between consecutive commands

  6a           the sum of a robot's episodes
  6b           the mean duration of an episode, also at SENSITIVITY_EPISODE_S

A robot whose cmd_vel holds no message has no figure. Nothing in the recording says, per
interval, whether traffic caused a stop, and the requirement's /mode_requests is not
published by this adapter, so attribution to traffic is made by comparing a route against
the same route run alone; this script emits one repeat's figures.

blocked_episodes() is the earlier reported-pose classifier, kept for
analyse/validity/analyze_motion_reference.py. METRIC_DESIGN.md section F6 defines the
measurands.
"""
import argparse
import json
import math
import os
import statistics as st
import sys
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'fleet_coordination'))
import analyze_fleet_coordination_4b as motion  # noqa: E402

ANALYSIS_VERSION = 2
MIN_EPISODE_S = 0.3           # three 10 Hz samples, the earlier classifier's hysteresis
SENSITIVITY_EPISODE_S = 1.0

# The earlier reported-pose classifier: below this speed is stationary, and HYSTERESIS_N
# consecutive stationary samples are required before an episode opens, which survives one
# tick of AMCL publish lag without fragmenting a continuous stop.
THRESHOLD_MPS = 0.001
HYSTERESIS_N = 3
# Two fleet-state samples further apart than this are not one interval: the recording
# stopped, or the stream dropped, and the gap is not evidence of standing still.
MAX_SAMPLE_GAP_S = 2.0


def read_fleet_states(bag, robots):
    """{robot: [(t, x, y, task_id), ...]} in bag time, sorted."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = defaultdict(list)
    wanted = set(robots)
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != '/fleet_states' or topic not in types:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        for rb in msg.robots:
            if rb.name in wanted:
                out[rb.name].append((t / 1e9, rb.location.x, rb.location.y, rb.task_id))
    return {k: sorted(v) for k, v in out.items()}


def blocked_episodes(samples, t_limit=None):
    """Contiguous intervals where the robot holds a task and is not moving.

    Returns [(t_start, t_end), ...]. t_limit, when given, ends the search at the task's
    last observed motion, so the tail between arriving and the task_id being released is
    not counted.
    """
    episodes, low, start = [], 0, None
    for a, b in zip(samples, samples[1:]):
        if t_limit is not None and b[0] > t_limit:
            break
        dt = b[0] - a[0]
        if dt <= 0 or dt > MAX_SAMPLE_GAP_S:
            if start is not None:
                episodes.append((start, a[0]))
            low, start = 0, None
            continue
        moving = math.hypot(b[1] - a[1], b[2] - a[2]) / dt >= THRESHOLD_MPS
        if not moving and b[3]:
            low += 1
            if low == HYSTERESIS_N:
                start = a[0]
        else:
            if start is not None:
                episodes.append((start, a[0]))
            low, start = 0, None
    if start is not None:
        episodes.append((start, samples[-1][0]))
    return episodes


def stationary_episodes(commands, t_start, t_end, min_s):
    """[(start, end)] within [t_start, t_end] holding no motion command, at least min_s long."""
    moving = []
    for i, (t, is_moving) in enumerate(commands):
        if not is_moving:
            continue
        end = t + motion.STALE_S
        if i + 1 < len(commands):
            end = min(end, commands[i + 1][0])
        a, b = max(t, t_start), min(end, t_end)
        if b > a:
            if moving and a <= moving[-1][1]:
                moving[-1] = (moving[-1][0], max(moving[-1][1], b))
            else:
                moving.append((a, b))
    gaps, cursor = [], t_start
    for a, b in moving:
        if a - cursor >= min_s:
            gaps.append((cursor, a))
        cursor = max(cursor, b)
    if t_end - cursor >= min_s:
        gaps.append((cursor, t_end))
    return gaps


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True, help='Path to the rosbag2 directory of one repeat')
    ap.add_argument('--task-planning-metrics', required=True,
                    help='task_planning_metrics.json for the same repeat: each task\'s robot, '
                         'route, award and completion')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    tasks = [t for t in json.load(open(args.task_planning_metrics))['per_task'] if t.get('robot')]
    robots = sorted({t['robot'] for t in tasks})
    commands = motion.read_commands(args.bag, robots)

    per_robot, episodes, episodes_sens = {}, [], []
    for task in tasks:
        rb = task['robot']
        entry = {'route': '->'.join(task.get('places') or []), 'task_id': task['task_id']}
        if not commands[rb] or not task.get('t_award') or not task.get('t_completed'):
            entry.update({'time_blocked_s': None, 'n_episodes': None, 'longest_episode_s': None,
                          'reason': 'no cmd_vel' if not commands[rb] else 'no completed window'})
            per_robot[rb] = entry
            continue
        eps = stationary_episodes(commands[rb], task['t_award'], task['t_completed'], MIN_EPISODE_S)
        eps_s = stationary_episodes(commands[rb], task['t_award'], task['t_completed'], SENSITIVITY_EPISODE_S)
        d = [b - a for a, b in eps]
        episodes.extend(d)
        episodes_sens.extend(b - a for a, b in eps_s)
        entry.update({'window_s': round(task['t_completed'] - task['t_award'], 3),
                      'time_blocked_s': round(sum(d), 3), 'n_episodes': len(d),
                      'longest_episode_s': round(max(d), 3) if d else 0.0})
        per_robot[rb] = entry

    scored = [v['time_blocked_s'] for v in per_robot.values() if v['time_blocked_s'] is not None]
    result = {
        'scenario': args.scenario,
        'bag': args.bag,
        'analysis_version': ANALYSIS_VERSION,
        '6_method': ('stationary while assigned: no motion command on cmd_vel within each task\'s '
                     '[award, completed]; episodes of at least %.1f s; attribution to traffic by '
                     'comparing a route against the same route run alone' % MIN_EPISODE_S),
        '6a_time_blocked_per_robot_s': {k: v['time_blocked_s'] for k, v in per_robot.items()},
        '6a_per_robot': per_robot,
        'n_robots_scored': len(scored),
        '6b_mean_per_episode_s': round(st.mean(episodes), 3) if episodes else None,
        '6b_mean_per_episode_s_min_%gs' % SENSITIVITY_EPISODE_S:
            round(st.mean(episodes_sens), 3) if episodes_sens else None,
        'n_episodes_total': len(episodes),
        'n_episodes_total_min_%gs' % SENSITIVITY_EPISODE_S: len(episodes_sens),
    }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    print(json.dumps({k: v for k, v in result.items() if k != '6a_per_robot'}, indent=2, default=str))


if __name__ == '__main__':
    main()
