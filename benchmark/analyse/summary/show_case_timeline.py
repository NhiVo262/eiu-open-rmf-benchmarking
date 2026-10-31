#!/usr/bin/env python3
"""Prints the timeline of one robot's task in one repeat, for a case study.

The events, in bag receive time relative to the task's award:

  task          submit, award and completion, from task_planning_metrics.json
  itinerary     every /rmf_traffic/itinerary_set of the robot's participant, with the
                named waypoints its trajectory passes within VERTEX_RADIUS_M
  negotiation   every notice naming the participant and its conclusion, marked refused
                when a /rmf_traffic/negotiation_refusal names the conflict version
  adapter       the fleet adapter's log for the repeat's attempt: commands, Nav2 goals
                accepted, reached, aborted and cancelled
  stationary    each stretch of at least 0.3 s with no motion command on the robot's
                cmd_vel, as F6 defines it, with the distance to the nearest other robot
                RMF held at its start and whether a Nav2 goal was open throughout

A stationary stretch is attributed, in order, to RMF when the adapter logs the robot as
waiting for traffic over most of it; to Nav2 when a goal it accepted was open throughout;
and otherwise to neither. Goal outcomes in the log carry a goal id rather than a robot
name, so each id is attributed through the command it was accepted for. Positions are what RMF
held, not ground truth. Writes nothing.

With --attribute <data/platform>, it prints instead, for every scenario, how the
stationary time of all N=3 robot-tasks divides between the three attributions, with the
Nav2 aborts logged during the tasks. The division is exploratory: it rests on the
adapter's log lines, and its third class is a remainder, stationary time between goals
or spanning a mix of states.

Usage, inside the open-rmf container:
    python3 show_case_timeline.py --repeat-dir <repeat> --route wp_5,wp_2
    python3 show_case_timeline.py --attribute <data/platform>
"""
import argparse
import bisect
import glob
import json
import math
import os
import re
import sys

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'congestion_handling'))
sys.path.insert(0, os.path.join(HERE, '..', 'fleet_coordination'))
import analyze_congestion_handling as congestion  # noqa: E402
import analyze_fleet_coordination_4b as motion  # noqa: E402

VERTEX_RADIUS_M = 0.5
LOG_LINE = re.compile(r'\[(\d+\.\d+)\] \[tb3_fleet_command_handle\]: (.*)$')
RMF_LINE = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*\]: (.*)$')


def default_nav_graph():
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        path = os.path.join(prefix, 'share', 'tb3_fleet', 'maps', 'world_tb3', 'nav_graphs', '0.yaml')
        if prefix and os.path.exists(path):
            return path
    return None


def vertices(nav_graph):
    level = list(yaml.safe_load(open(nav_graph))['levels'].values())[0]
    return [(v[2].get('name'), v[0], v[1]) for v in level['vertices'] if v[2].get('name')]


def waypoint_sequence(points, named):
    seq = []
    for x, y in points:
        name, d = min(((n, math.hypot(x - vx, y - vy)) for n, vx, vy in named), key=lambda a: a[1])
        if d <= VERTEX_RADIUS_M and (not seq or seq[-1] != name):
            seq.append(name)
    return seq


def read_bag(bag):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    wanted = ['/rmf_traffic/participants', '/rmf_traffic/itinerary_set', '/rmf_traffic/negotiation_notice',
              '/rmf_traffic/negotiation_conclusion', '/rmf_traffic/negotiation_refusal', '/fleet_states']
    reader.set_filter(rosbag2_py.StorageFilter(topics=[w for w in wanted if w in types]))
    out = {w: [] for w in wanted}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns / 1e9, deserialize_message(data, types[topic])))
    return out


GOAL_TERMINAL = ('goal aborted', 'goal cancelled', 'goal reached')


def goal_name(gid):
    """A Nav2 goal id shortened to its first and last byte."""
    parts = [p.strip() for p in gid.split(',')]
    return 'goal %s..%s' % (parts[0], parts[-1])


def adapter_events(log_path, robot):
    """The adapter's events for one robot. Goal outcomes name the goal, not the robot, so
    each goal id is attributed to the robot whose command it was accepted for."""
    events, my_goals = [], set()
    if not os.path.exists(log_path):
        return events
    for line in open(log_path, errors='replace'):
        m = LOG_LINE.search(line) or RMF_LINE.search(line)
        if not m:
            continue
        t, text = float(m.group(1)), m.group(2)
        goal = re.search(r'goal \[([0-9, ]+)\]', text)
        gid = goal.group(1) if goal else None
        if robot in text and 'accepted by Nav2' in text and gid:
            my_goals.add(gid)
            events.append((t, 'goal accepted', goal_name(gid)))
        elif gid and gid in my_goals:
            if 'instead of succeeding' in text or 'aborted' in text:
                events.append((t, 'goal aborted', goal_name(gid)))
            elif 'ancel' in text:
                events.append((t, 'goal cancelled', goal_name(gid)))
            elif text.startswith('Navigation goal') and 'reached' in text:
                events.append((t, 'goal reached', goal_name(gid)))
        elif robot in text:
            dest = re.search(r'destination=\[([^\]]*)\]', text)
            if dest and text.startswith('command'):
                events.append((t, 'command', dest.group(1)))
            elif 'done waiting for traffic' in text:
                events.append((t, 'traffic wait ends', ''))
            elif 'waiting for traffic' in text:
                events.append((t, 'traffic wait starts', ''))
    return events


def traffic_wait_through(events, a, b):
    """Whether RMF reported the robot waiting for traffic over most of [a, b]."""
    spans, start = [], None
    for t, label, _ in events:
        if label == 'traffic wait starts':
            start = t
        elif label == 'traffic wait ends' and start is not None:
            spans.append((start, t)); start = None
    if start is not None:
        spans.append((start, float('inf')))
    covered = sum(max(0.0, min(b, e) - max(a, s0)) for s0, e in spans)
    return covered >= 0.5 * (b - a)


def goal_active_spans(events):
    """[(start, end)] per goal id, from its acceptance to its own outcome, or to infinity
    when no outcome follows."""
    opened, spans = {}, []
    for t, label, name in events:
        if label == 'goal accepted':
            if name in opened:
                spans.append((opened.pop(name), t))
            opened[name] = t
        elif label in GOAL_TERMINAL and name in opened:
            spans.append((opened.pop(name), t))
    spans.extend((start, float('inf')) for start in opened.values())
    return sorted(spans)


def goal_open_through(events, a, b):
    """Whether the union of the goal spans covers the whole of [a, b]."""
    cursor = a
    for start, end in goal_active_spans(events):
        if start > cursor:
            return False
        cursor = max(cursor, end)
        if cursor >= b:
            return True
    return cursor >= b


def adapter_log(repeat_dir, tp):
    attempt = re.search(r'attempt(\d+)', os.path.basename(tp['api_state_path'])).group(1)
    return os.path.join(os.path.dirname(repeat_dir), 'fleet_adapter_attempt%s.log' % attempt)


def attribute(platform_dir):
    """Print the division of N=3 stationary time by holder, per scenario."""
    labels = ('RMF waiting for traffic', 'Nav2 goal open', 'remainder')
    total = dict.fromkeys(labels, 0.0)
    for scenario_dir in sorted(glob.glob(os.path.join(platform_dir, 'scenarios', '*'))):
        if not os.path.isdir(scenario_dir):
            continue
        split, aborts = dict.fromkeys(labels, 0.0), 0
        for repeat in sorted(glob.glob(os.path.join(scenario_dir, 'run_*', 'repeat_*'))):
            tp = json.load(open(os.path.join(repeat, 'task_planning_metrics.json')))
            log = adapter_log(repeat, tp)
            commands = motion.read_commands(os.path.join(repeat, 'bag'), [t['robot'] for t in tp['per_task']])
            for task in tp['per_task']:
                robot = task['robot']
                events = adapter_events(log, robot)
                aborts += sum(1 for t, label, _ in events
                              if label == 'goal aborted' and task['t_award'] <= t <= task['t_completed'])
                if not commands[robot]:
                    continue
                for a, b in congestion.stationary_episodes(commands[robot], task['t_award'],
                                                           task['t_completed'], congestion.MIN_EPISODE_S):
                    if traffic_wait_through(events, a, b):
                        key = labels[0]
                    elif goal_open_through(events, a, b):
                        key = labels[1]
                    else:
                        key = labels[2]
                    split[key] += b - a
        n = sum(split.values())
        for k in labels:
            total[k] += split[k]
        print('%-11s %7.0f s  %s  Nav2 aborts %d' % (
            os.path.basename(scenario_dir), n,
            '  '.join('%s %.0f%%' % (k, 100 * split[k] / n if n else 0) for k in labels), aborts))
    n = sum(total.values())
    print('%-11s %7.0f s  %s' % ('all', n, '  '.join('%s %.0f%%' % (k, 100 * total[k] / n) for k in labels)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repeat-dir')
    ap.add_argument('--route', help='the task places, comma separated, e.g. wp_5,wp_2')
    ap.add_argument('--attribute', metavar='PLATFORM_DIR', help='data/<platform>: print the division instead')
    ap.add_argument('--nav-graph', default=None)
    args = ap.parse_args()
    if args.attribute:
        attribute(args.attribute)
        return
    if not (args.repeat_dir and args.route):
        ap.error('--repeat-dir and --route are required unless --attribute is given')

    named = vertices(args.nav_graph or default_nav_graph())
    tp = json.load(open(os.path.join(args.repeat_dir, 'task_planning_metrics.json')))
    task = next(t for t in tp['per_task'] if t['places'] == args.route.split(','))
    robot, t0 = task['robot'], task['t_award']
    log = adapter_log(args.repeat_dir, tp)
    bag = read_bag(os.path.join(args.repeat_dir, 'bag'))

    pid = {p.id: p.description.name for _, m in bag['/rmf_traffic/participants'] for p in m.participants}
    mine = {k for k, v in pid.items() if v == robot}
    refused = {m.conflict_version for _, m in bag['/rmf_traffic/negotiation_refusal']}
    conclusion = {m.conflict_version: (t, m.resolved) for t, m in bag['/rmf_traffic/negotiation_conclusion']}
    t_end = task['t_completed']
    events = [(task['t_submit'], 'task', 'submitted'), (t0, 'task', 'awarded to %s' % robot),
              (t_end, 'task', 'completed')]

    for t, m in bag['/rmf_traffic/itinerary_set']:
        if m.participant in mine and t0 - 5 <= t <= t_end:
            pts = [(w.position[0], w.position[1]) for r in m.itinerary for w in r.trajectory.waypoints]
            events.append((t, 'itinerary', 'v%d %s' % (m.itinerary_version, '->'.join(waypoint_sequence(pts, named)))))
    seen = set()
    for t, m in bag['/rmf_traffic/negotiation_notice']:
        if mine & set(m.participants) and m.conflict_version not in seen and t0 - 5 <= t <= t_end:
            seen.add(m.conflict_version)
            others = [pid.get(p, p) for p in m.participants if p not in mine]
            tc, resolved = conclusion.get(m.conflict_version, (None, None))
            outcome = 'resolved' if resolved else ('refused' if m.conflict_version in refused else 'search failed')
            events.append((t, 'negotiation', 'conflict %d with %s, %s after %.3f s' % (
                m.conflict_version, ','.join(others), outcome, (tc - t) if tc else float('nan'))))
    adapter = adapter_events(log, robot)
    events += [(t, 'adapter', '%s %s' % (label, dest)) for t, label, dest in adapter if t0 - 5 <= t <= t_end]

    fs = {}
    for t, m in bag['/fleet_states']:
        for rs in m.robots:
            fs.setdefault(rs.name, []).append((t, rs.location.x, rs.location.y))
    commands = motion.read_commands(os.path.join(args.repeat_dir, 'bag'), [robot])[robot]
    for a, b in congestion.stationary_episodes(commands, t0, t_end, congestion.MIN_EPISODE_S):
        here = fs.get(robot, [])
        i = min(bisect.bisect_left([p[0] for p in here], a), len(here) - 1)
        x, y = here[i][1], here[i][2]
        near = []
        for other, pts in fs.items():
            if other == robot or not pts:
                continue
            j = min(bisect.bisect_left([p[0] for p in pts], a), len(pts) - 1)
            near.append((math.hypot(pts[j][1] - x, pts[j][2] - y), other))
        d, who = min(near) if near else (float('nan'), '-')
        if traffic_wait_through(adapter, a, b):
            held = 'RMF waiting for traffic'
        elif goal_open_through(adapter, a, b):
            held = 'Nav2 goal open'
        else:
            held = 'remainder: between goals or mixed'
        events.append((a, 'stationary', '%.1f s, nearest %s at %.2f m, %s' % (b - a, who, d, held)))

    print('%s  route %s  robot %s  award at bag time %.3f' % (args.repeat_dir, args.route, robot, t0))
    for t, kind, text in sorted(events):
        print('%+8.2f s  %-12s %s' % (t - t0, kind, text))


if __name__ == '__main__':
    main()
