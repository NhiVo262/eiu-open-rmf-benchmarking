#!/usr/bin/env python3

import argparse
import bisect
import datetime
import glob
import json
import os

import numpy as np
from rclpy.serialization import deserialize_message
import rmf_adapter.easy_full_control as rmf_easy
import rmf_adapter.plan as plan
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

# RobotContext::NavParams::multipliers, applied to max_merge_lane_distance until
# a merge succeeds. Not exposed through the bindings, so duplicated here.
GRAPH_MERGE_MULTIPLIERS = (1.0, 2.0, 3.0, 5.0, 10.0)


def read_fleet_states(bag):
    """Poses RMF held, plus the /clock samples needed to reach sim time."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='mcap'),
                ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    clock, poses = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        wall = t_ns / 1e9
        if topic == '/clock':
            msg = deserialize_message(data, types[topic])
            clock.append((wall, msg.clock.sec + msg.clock.nanosec * 1e-9))
        elif topic == '/fleet_states':
            msg = deserialize_message(data, types[topic])
            for robot in msg.robots:
                poses.append((wall, robot.name, robot.location.x,
                              robot.location.y, robot.location.yaw))
    clock.sort()
    return clock, poses


def to_sim_time(clock_wall, clock_sim, wall):
    i = bisect.bisect_left(clock_wall, wall)
    candidates = [k for k in (i - 1, i) if 0 <= k < len(clock_wall)]
    if not candidates:
        return None
    k = min(candidates, key=lambda k: abs(clock_wall[k] - wall))
    return wall - (clock_wall[k] - clock_sim[k])


def is_pose_on_navigation_graph(graph, map_name, x, y, yaw, params):
    position = np.array([x, y, yaw])
    now = datetime.datetime.now()
    for multiplier in GRAPH_MERGE_MULTIPLIERS:
        starts = plan.compute_plan_starts(
            graph,
            map_name,
            position,
            now,
            max_merge_waypoint_distance=params['max_merge_waypoint_distance'],
            max_merge_lane_distance=multiplier * params['max_merge_lane_distance'],
            min_lane_length=params['min_lane_length'],
        )
        if len(starts) > 0:
            return True
    return False


def find_episodes(repeat_dir, graph, map_name, params):
    """Off-graph episodes per robot, in sim seconds."""
    bag = os.path.join(repeat_dir, 'bag')
    if not os.path.isdir(bag):
        return {'error': 'no bag'}
    clock, poses = read_fleet_states(bag)
    if not poses:
        return {'error': 'no /fleet_states in bag'}
    wall_clock_run = not clock
    clock_wall = [c[0] for c in clock]
    clock_sim = [c[1] for c in clock]

    per_robot = {}
    for wall, name, x, y, yaw in poses:
        sim_t = (wall if wall_clock_run
                 else to_sim_time(clock_wall, clock_sim, wall))
        if sim_t is None:
            continue
        record = per_robot.setdefault(name, {
            'samples': 0, 'samples_off_graph': 0, 'episodes': [],
            '_open_at': None, '_last_t': None,
        })
        record['samples'] += 1
        record['_last_t'] = sim_t
        if is_pose_on_navigation_graph(graph, map_name, x, y, yaw, params):
            if record['_open_at'] is not None:
                record['episodes'].append(
                    {'start_sim_t': record['_open_at'],
                     'duration_s': sim_t - record['_open_at']})
                record['_open_at'] = None
        else:
            record['samples_off_graph'] += 1
            if record['_open_at'] is None:
                record['_open_at'] = sim_t

    out = {}
    for name, record in per_robot.items():
        if record['_open_at'] is not None:
            # still off graph when the recording ended
            record['episodes'].append(
                {'start_sim_t': record['_open_at'],
                 'duration_s': record['_last_t'] - record['_open_at'],
                 'unresolved': True})
        episodes = record['episodes']
        out[name] = {
            'samples': record['samples'],
            'samples_off_graph': record['samples_off_graph'],
            'fraction_off_graph': (record['samples_off_graph'] / record['samples']
                                   if record['samples'] else 0.0),
            'episode_count': len(episodes),
            'longest_episode_s': max((e['duration_s'] for e in episodes), default=0.0),
            'total_off_graph_s': sum(e['duration_s'] for e in episodes),
            'unresolved_at_end': any(e.get('unresolved') for e in episodes),
            'episodes': episodes,
        }
    return out


def read_nav_params(fleet_config, robot_name):
    """Merge distances for a robot, falling back to RMF's own defaults."""
    defaults = {'max_merge_waypoint_distance': 1e-3,
                'max_merge_lane_distance': 0.3,
                'min_lane_length': 1e-8}
    try:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
    except Exception:
        return defaults
    params = dict(defaults)
    for key in params:
        value = getattr(robot_config, key, None)
        if callable(value):
            value = value()
        if isinstance(value, float):
            params[key] = value
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir', required=True, help='Run directory holding the repeat_* directories')
    ap.add_argument('--fleet-config', required=True, help='Fleet config YAML the participant profile is read from')
    ap.add_argument('--nav-graph', required=True, help='Navigation graph YAML the waypoints are read from')
    ap.add_argument('--map-name', default='world_tb3', help='Map name used when merging a pose onto the navigation graph')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        args.fleet_config, args.nav_graph)
    if not fleet_config:
        raise SystemExit(f'could not parse fleet config [{args.fleet_config}]')
    graph = fleet_config.graph

    repeats = []
    candidates = sorted(glob.glob(os.path.join(args.run_dir, 'repeat_*')))
    for repeat_dir in [d for d in candidates if os.path.isdir(d)]:
        robots = sorted(fleet_config.known_robots)
        params = read_nav_params(fleet_config, robots[0]) if robots else {}
        result = find_episodes(repeat_dir, graph, args.map_name, params)
        repeats.append({'repeat_dir': repeat_dir, 'robots': result})

    json.dump({'run_dir': args.run_dir,
               'map_name': args.map_name,
               'merge_multipliers': list(GRAPH_MERGE_MULTIPLIERS),
               'repeats': repeats},
              open(args.output, 'w'), indent=1)

    header = '%-12s %-12s %9s %8s %10s %s' % (
        'repeat', 'robot', 'lost', 'episodes', 'longest_s', 'unresolved')
    print(header)
    print('-' * len(header))
    for entry in repeats:
        name = os.path.basename(entry['repeat_dir'])
        robots = entry['robots']
        if 'error' in robots:
            print('%-12s %s' % (name, robots['error']))
            continue
        for robot in sorted(robots):
            value = robots[robot]
            if not value['episode_count']:
                continue
            print('%-12s %-12s %8.1f%% %8d %10.0f %s' % (
                name, robot, 100.0 * value['fraction_off_graph'],
                value['episode_count'], value['longest_episode_s'],
                'yes' if value['unresolved_at_end'] else ''))


if __name__ == '__main__':
    main()
