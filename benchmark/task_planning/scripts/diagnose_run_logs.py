#!/usr/bin/env python3
"""Scan tb3_fleet operator logs (fleet adapter, Nav2/Gazebo launch, Zenoh router)
for the failure patterns already diagnosed by hand during N=5 trials, and print
a per-robot summary instead of requiring manual log scrolling.

Usage:
    python3 diagnose_run_logs.py --fleet-adapter-log fleet_adapter.log \
        [--nav2-log nav2.log] [--zenoh-log zenoh_router.log] \
        --robots tb3_robot1 tb3_robot2 tb3_robot3 tb3_robot4 tb3_robot5

Each log argument is optional so this also works with whichever logs happen to
have been captured for a given run.
"""

import argparse
import re
from collections import defaultdict


def read_lines(path):
    if path is None:
        return []
    with open(path, 'r', errors='replace') as f:
        return f.readlines()


def scan_fleet_adapter(lines, robots):
    per_robot = {r: {
        'registered': False,
        'goals_accepted': 0,
        'goals_cancelled': 0,
        'goals_reached': 0,
        'goals_rejected': 0,
        'no_valid_reply': 0,
        'replan_requested': 0,
        'cant_get_location': 0,
        'unable_to_find_path': 0,
        'last_go_to_place': None,
    } for r in robots}

    re_added = re.compile(r'Successfully added robot\s*[\[\'"]?([\w]+)')
    # Real fleet-adapter log lines for "accepted"/"cancelled"/"reached" carry a
    # goal_id, not a robot name — attribution relies on `current_robot`, set
    # from the preceding "Commanding [robotN] to navigate" line.
    re_accepted = re.compile(r'Navigation goal .* accepted')
    re_cancelled = re.compile(r'Navigation goal .* was cancelled')
    re_reached = re.compile(r'Navigation goal .* reached')
    re_rejected = re.compile(r'send_goal for \[([\w]+)\] was rejected')
    re_no_reply = re.compile(r'send_goal for \[([\w]+)\] got no valid reply')
    re_replan = re.compile(r'Replanning requested for \[tb3_fleet/([\w]+)\]')
    re_cant_location = re.compile(r"Robot \[tb3_fleet/([\w]+)\] can't get location")
    re_no_path = re.compile(r'Unable to find a path to any of the goal options .* for \[tb3_fleet/([\w]+)\]')
    re_go_to = re.compile(r'Executing go_to_place \[([\w_]+)\] for robot \[tb3_fleet/([\w]+)\]')

    # commands like "Commanding [tb3_robotN] to navigate" name the robot directly;
    # accept/cancel/reach lines from RMF core often don't, so track "current robot"
    # per navigate command as a best-effort attribution.
    re_commanding = re.compile(r'Commanding \[([\w]+)\] to navigate')
    current_robot = None

    for line in lines:
        m = re_added.search(line)
        if m and m.group(1) in per_robot:
            per_robot[m.group(1)]['registered'] = True

        m = re_commanding.search(line)
        if m and m.group(1) in per_robot:
            current_robot = m.group(1)

        if re_accepted.search(line) and current_robot in per_robot:
            per_robot[current_robot]['goals_accepted'] += 1

        if re_cancelled.search(line) and current_robot in per_robot:
            per_robot[current_robot]['goals_cancelled'] += 1

        if re_reached.search(line) and current_robot in per_robot:
            per_robot[current_robot]['goals_reached'] += 1

        m = re_rejected.search(line)
        if m and m.group(1) in per_robot:
            per_robot[m.group(1)]['goals_rejected'] += 1

        m = re_no_reply.search(line)
        if m and m.group(1) in per_robot:
            per_robot[m.group(1)]['no_valid_reply'] += 1

        m = re_replan.search(line)
        if m and m.group(1) in per_robot:
            per_robot[m.group(1)]['replan_requested'] += 1

        m = re_cant_location.search(line)
        if m and m.group(1) in per_robot:
            per_robot[m.group(1)]['cant_get_location'] += 1

        m = re_no_path.search(line)
        if m and m.group(1) in per_robot:
            per_robot[m.group(1)]['unable_to_find_path'] += 1

        m = re_go_to.search(line)
        if m and m.group(2) in per_robot:
            per_robot[m.group(2)]['last_go_to_place'] = m.group(1)

    return per_robot


def scan_nav2_log(lines):
    findings = {
        'polygon_shape_warnings': 0,
        'lifecycle_manage_nodes_failed': 0,
        'nodes_stuck_inactive': set(),
    }
    re_polygon = re.compile(r'Polygon shape is not set yet')
    re_lifecycle_fail = re.compile(r'Failed to bring up|manage_nodes.*fail', re.IGNORECASE)
    re_inactive = re.compile(r'\[(tb3_robot\d+)/(\w+)\].*inactive')

    for line in lines:
        if re_polygon.search(line):
            findings['polygon_shape_warnings'] += 1
        if re_lifecycle_fail.search(line):
            findings['lifecycle_manage_nodes_failed'] += 1
        m = re_inactive.search(line)
        if m:
            findings['nodes_stuck_inactive'].add(f'{m.group(1)}/{m.group(2)}')

    return findings


def scan_zenoh_log(lines):
    findings = {'timeouts_10s': 0, 'route_query_not_found': 0}
    re_timeout = re.compile(r'Timeout\(10s\)!')
    re_not_found = re.compile(r'Query not found')

    for line in lines:
        if re_timeout.search(line):
            findings['timeouts_10s'] += 1
        if re_not_found.search(line):
            findings['route_query_not_found'] += 1

    return findings


def classify(robot_stats):
    """Best-effort verdict per robot, matching the failure patterns already
    root-caused by hand this session."""
    s = robot_stats
    if not s['registered']:
        return 'NOT REGISTERED — check bt_navigator lifecycle / TF for this robot'
    if s['goals_reached'] > 0 and s['replan_requested'] < 3:
        return 'OK — completed at least one leg cleanly'
    if s['replan_requested'] >= 5 and s['goals_reached'] == 0:
        return 'STUCK — repeated replan loop, no leg completed (Nav2 aborting goals under load, or zenoh get_result timing out)'
    if s['cant_get_location'] > 0:
        return "STUCK — RMF lost the robot's location (TF/zenoh position query issue)"
    if s['unable_to_find_path'] > 0:
        return 'STUCK — planner found no path to the assigned goal (check route validity / graph)'
    if s['goals_accepted'] == 0 and s['goals_reached'] == 0:
        return 'IDLE — never received a navigate command (never won a bid, or no task assigned)'
    return 'UNCLEAR — check raw log around this robot manually'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fleet-adapter-log', required=True)
    ap.add_argument('--nav2-log')
    ap.add_argument('--zenoh-log')
    ap.add_argument('--robots', nargs='+', required=True)
    args = ap.parse_args()

    fa_lines = read_lines(args.fleet_adapter_log)
    nav2_lines = read_lines(args.nav2_log)
    zenoh_lines = read_lines(args.zenoh_log)

    per_robot = scan_fleet_adapter(fa_lines, args.robots)
    nav2_findings = scan_nav2_log(nav2_lines) if nav2_lines else None
    zenoh_findings = scan_zenoh_log(zenoh_lines) if zenoh_lines else None

    print('=' * 78)
    print('PER-ROBOT SUMMARY')
    print('=' * 78)
    for r in args.robots:
        s = per_robot[r]
        print(f'\n{r}: {classify(s)}')
        print(f'  registered={s["registered"]}  goals_accepted={s["goals_accepted"]}  '
              f'goals_reached={s["goals_reached"]}  goals_cancelled={s["goals_cancelled"]}')
        print(f'  goals_rejected={s["goals_rejected"]}  no_valid_reply={s["no_valid_reply"]}  '
              f'replan_requested={s["replan_requested"]}')
        print(f"  cant_get_location={s['cant_get_location']}  "
              f'unable_to_find_path={s["unable_to_find_path"]}  '
              f'last_go_to_place={s["last_go_to_place"]}')

    if nav2_findings is not None:
        print('\n' + '=' * 78)
        print('NAV2 / GAZEBO LOG FINDINGS')
        print('=' * 78)
        print(f"  Polygon shape warnings (collision_monitor footprint not configured): "
              f"{nav2_findings['polygon_shape_warnings']}")
        print(f"  Lifecycle manage_nodes failures: {nav2_findings['lifecycle_manage_nodes_failed']}")
        if nav2_findings['nodes_stuck_inactive']:
            print(f"  Nodes seen reporting inactive: {sorted(nav2_findings['nodes_stuck_inactive'])}")

    if zenoh_findings is not None:
        print('\n' + '=' * 78)
        print('ZENOH ROUTER LOG FINDINGS')
        print('=' * 78)
        print(f"  10s query timeouts (get_result/send_goal congestion): {zenoh_findings['timeouts_10s']}")
        print(f"  'Query not found' route errors: {zenoh_findings['route_query_not_found']}")
        if zenoh_findings['timeouts_10s'] > 5:
            print('  -> High timeout count: this run was likely CPU/zenoh congested, not a routing/scenario bug.')

    print()


if __name__ == '__main__':
    main()
