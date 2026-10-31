#!/usr/bin/env python3
import argparse
import json
import math
import os

import yaml


def robots_with_tasks(metrics_path):
    with open(metrics_path) as f:
        metrics = json.load(f)
    return {name: entry.get('n_tasks', 0)
            for name, entry in (metrics.get('per_robot') or {}).items()}


def observed_path_per_robot(metrics_path):
    with open(metrics_path) as f:
        metrics = json.load(f)
    return {name: entry.get('observed_path_m')
            for name, entry in (metrics.get('per_robot') or {}).items()}


def task_counts(metrics_path):
    with open(metrics_path) as f:
        metrics = json.load(f)
    return metrics.get('n_tasks', 0), metrics.get('n_tasks_completed', 0)


def resolved_conflicts(metrics_path):
    """Number of negotiations that concluded with the conflict settled."""
    if not os.path.exists(metrics_path):
        return None
    with open(metrics_path) as f:
        metrics = json.load(f)
    return sum(1 for c in (metrics.get('per_conflict') or []) if c.get('resolved'))


def graph_lanes(nav_graph_path):
    """[((x1, y1), (x2, y2))] for the first level's lanes, or [] when unreadable."""
    if not nav_graph_path or not os.path.exists(nav_graph_path):
        return []
    with open(nav_graph_path) as f:
        graph = yaml.safe_load(f)
    for level in ((graph or {}).get('levels') or {}).values():
        vertices = [(v[0], v[1]) for v in level.get('vertices', [])]
        return [(vertices[lane[0]], vertices[lane[1]])
                for lane in level.get('lanes', [])
                if lane[0] < len(vertices) and lane[1] < len(vertices)]
    return []


def point_to_segment_m(point, start, end):
    """Shortest distance from a point to a line segment, in metres."""
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    span = dx * dx + dy * dy
    t = 0.0 if span == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / span))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def max_off_lane_m(api_state_path, lanes):
    """robot -> the furthest it was ever seen from the nearest lane, in metres."""
    out = {}
    if not lanes or not os.path.exists(api_state_path):
        return out
    with open(api_state_path) as f:
        for line in f:
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            message = entry.get('msg')
            if not isinstance(message, dict) or message.get('type') != 'fleet_state_update':
                continue
            robots = (message.get('data') or {}).get('robots') or {}
            for name, state in robots.items():
                location = state.get('location') or {}
                if 'x' not in location or 'y' not in location:
                    continue
                point = (location['x'], location['y'])
                away = min(point_to_segment_m(point, a, b) for a, b in lanes)
                if away > out.get(name, -1.0):
                    out[name] = away
    return out


def state_coverage(api_state_path, robots, window):
    if not os.path.exists(api_state_path):
        return None
    start, end = window
    stamps = {robot: [] for robot in robots}
    with open(api_state_path) as f:
        for line in f:
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            message = entry.get('msg')
            received = entry.get('_recv_wall')
            if not isinstance(message, dict) or message.get('type') != 'fleet_state_update':
                continue
            if not isinstance(received, (int, float)) or not math.isfinite(received):
                continue
            if received < start or received > end:
                continue
            for name, state in ((message.get('data') or {}).get('robots') or {}).items():
                if name not in stamps:
                    continue
                state_time = state.get('unix_millis_time')
                if not isinstance(state_time, (int, float)) or isinstance(state_time, bool):
                    continue
                if not math.isfinite(state_time) or state_time <= 0:
                    continue
                stamps[name].append(float(received))
    out = {}
    for robot, series in stamps.items():
        series.sort()
        if not series:
            out[robot] = {'samples': 0, 'head_gap_s': end - start,
                          'max_gap_s': end - start, 'tail_gap_s': end - start}
            continue
        gaps = [b - a for a, b in zip(series, series[1:])]
        out[robot] = {'samples': len(series),
                      'head_gap_s': series[0] - start,
                      'max_gap_s': max(gaps) if gaps else 0.0,
                      'tail_gap_s': end - series[-1]}
    return out


def robot_issues(api_state_path, window=None):
    out = {}
    if not os.path.exists(api_state_path):
        return None
    with open(api_state_path) as f:
        for line in f:
            try:
                entry = json.loads(line)
            except ValueError:
                continue
            message = entry.get('msg')
            if not isinstance(message, dict):
                continue
            if message.get('type') != 'fleet_state_update':
                continue
            if window:
                received = entry.get('_recv_wall')
                if not isinstance(received, (int, float)) or not math.isfinite(received):
                    continue
                if received < window[0] or received > window[1]:
                    continue
            robots = (message.get('data') or {}).get('robots') or {}
            for name, state in robots.items():
                for issue in (state.get('issues') or []):
                    category = str(issue.get('category', '')).lower() or 'unnamed'
                    counts = out.setdefault(name, {})
                    counts[category] = counts.get(category, 0) + 1
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--task-planning-metrics', required=True)
    ap.add_argument('--traffic-scheduling-metrics', required=True)
    ap.add_argument('--api-state', required=True)
    ap.add_argument('--window-start', type=float,
                    help='epoch seconds when the workload was submitted, measured by the '
                         'caller. With --window-end it turns the lost check from a read '
                         'of the capture into a read of a declared window')
    ap.add_argument('--window-end', type=float,
                    help='epoch seconds when the workload stopped being observed')
    ap.add_argument('--max-state-gap-s', type=float, default=5.0,
                    help='seconds of fleet state a robot may be missing from, at the '
                         'start, in the middle or at the end of the window. The fleet '
                         'publishes at 1 Hz and the largest gap in the 57 captured series '
                         'to 2026-10-15 is 1.42 s, so this separates the observed range '
                         'from everything else and is not a verified failure threshold. '
                         'It is also the staleness limit the fleet adapter applies to a '
                         'transform (default %(default)s)')
    ap.add_argument('--nav-graph',
                    help='the navigation graph the fleet ran on. Given, the gate reports '
                         'how far each robot strayed from it; the figure decides nothing')
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--min-path-m', type=float, default=1.0,
                    help='metres a robot must be seen to travel for the repeat to count '
                         '(default 1.0)')
    ap.add_argument('--report-only', action='store_true',
                    help='print the verdict and exit 0, so a repeat is kept for audit')
    args = ap.parse_args()

    failures = []      # the system under test did something that voids the repeat
    notes = []         # reported, never blocking: see the note above main()

    tasks = robots_with_tasks(args.task_planning_metrics)
    working = [r for r in args.robots if tasks.get(r, 0) > 0]
    print('tasks per robot: %s' % ', '.join('%s %d' % (r, tasks.get(r, 0))
                                            for r in args.robots))
    print('robots the fleet used: %d of %d' % (len(working), len(args.robots)))
    if not working:
        failures.append('no robot carried a task, so nothing was allocated')

    # Only the robots that were given work: one left parked carried no task to execute.
    path = observed_path_per_robot(args.task_planning_metrics)
    print('path per robot: %s' % ', '.join(
        '%s %s' % (r, 'unknown' if path.get(r) is None else '%.2f m' % path[r])
        for r in args.robots))
    still = [r for r in working
             if path.get(r) is not None and path[r] < args.min_path_m]
    if still:
        failures.append('%s carried a task and moved less than %.2f m, so the repeat '
                        'holds a robot that was given work and never drove'
                        % (', '.join(still), args.min_path_m))
    unknown = [r for r in working if path.get(r) is None]
    if unknown:
        failures.append('no observed path for %s, which carried a task'
                        % ', '.join(unknown))

    dispatched, completed = task_counts(args.task_planning_metrics)
    print('tasks completed: %d of %d' % (completed, dispatched))
    if dispatched < 1:
        failures.append('the metrics report no dispatched task, so there is nothing to '
                        'judge')
    elif completed < dispatched:
        failures.append('%d of %d tasks completed, so the repeat has a makespan for some '
                        'routes and none for others' % (completed, dispatched))

    resolved = resolved_conflicts(args.traffic_scheduling_metrics)
    if resolved is None:
        print('resolved conflicts: no traffic-scheduling metrics')
        notes.append('traffic-scheduling metrics are missing, so no conflict can be shown')
    else:
        print('resolved conflicts: %d' % resolved)
        if resolved < 1:
            notes.append('no negotiation concluded with the conflict resolved. The count '
                         'ranged 1..97 across repeats of one scenario, so it reports when '
                         'the capture saw a negotiation, not whether the fleet coordinated')

    off_lane = max_off_lane_m(args.api_state, graph_lanes(args.nav_graph))
    if off_lane:
        print('furthest from any lane: %s' % ', '.join(
            '%s %.2f m' % (r, off_lane[r]) for r in args.robots if r in off_lane))

    # Whether a robot was lost is read from the capture, so the capture has to be shown to
    # cover the window first. Without that, silence and health are the same reading.
    window = None
    if args.window_start is None or args.window_end is None:
        print('fleet state coverage: not checked, no window given')
        notes.append('the workload window was not given, so the capture cannot be '
                          'shown to cover it')
    elif not (math.isfinite(args.window_start) and math.isfinite(args.window_end)
              and args.window_start < args.window_end):
        print('fleet state coverage: not checked, the window [%s, %s] is not usable'
              % (args.window_start, args.window_end))
        notes.append('the workload window [%s, %s] is not two finite seconds in '
                          'order, so nothing can be measured against it'
                          % (args.window_start, args.window_end))
    elif not (math.isfinite(args.max_state_gap_s) and args.max_state_gap_s > 0):
        print('fleet state coverage: not checked, --max-state-gap-s [%s] is not a '
              'positive number of seconds' % args.max_state_gap_s)
        notes.append('--max-state-gap-s [%s] is not a positive number of seconds, '
                          'so no gap can be judged' % args.max_state_gap_s)
    else:
        window = (args.window_start, args.window_end)
        coverage = state_coverage(args.api_state, args.robots, window)
        if coverage is None:
            print('fleet state coverage: no API capture at [%s]' % args.api_state)
            notes.append('there is no API capture, so nothing about the robots can '
                              'be read from it')
        else:
            for robot in args.robots:
                entry = coverage[robot]
                print('fleet state for %s: %d sample(s), gaps head %.1f s, max %.1f s, '
                      'tail %.1f s' % (robot, entry['samples'], entry['head_gap_s'],
                                       entry['max_gap_s'], entry['tail_gap_s']))
            thin = [r for r in args.robots
                    if coverage[r]['samples'] < 1
                    or max(coverage[r]['head_gap_s'], coverage[r]['max_gap_s'],
                           coverage[r]['tail_gap_s']) > args.max_state_gap_s]
            if thin:
                notes.append('the capture is missing more than %.1f s of fleet '
                                  'state for %s, so it does not cover the window'
                                  % (args.max_state_gap_s, ', '.join(thin)))

    issues = robot_issues(args.api_state, window)
    if issues is None:
        print('robot issues: no API capture')
    else:
        print('robot issues: %s' % (', '.join(
            '%s %s' % (r, dict(sorted(issues[r].items())))
            for r in args.robots if issues.get(r)) or 'none'))
        lost = [r for r in args.robots if issues.get(r, {}).get('lost')]
        if lost:
            notes.append('RMF lost %s during the repeat. The cause lies in localisation '
                         'or lane closure, which this gate cannot repair by discarding '
                         'the repeat' % ', '.join(lost))
        stale = [r for r in args.robots if issues.get(r, {}).get('localization')]
        if stale:
            notes.append('the adapter had no pose for %s during the repeat, so RMF '
                         'acted on a position the robot was no longer confirming'
                         % ', '.join(stale))

    print()
    for reason in notes:
        print('NOTE: %s' % reason)
    for reason in failures:
        print('NOT A VALID N=3 SAMPLE: %s' % reason)
    if not failures:
        print('VALID N=3 SAMPLE: %d robot(s) carried a task, each of them moved, and '
              'every dispatched task completed.' % len(working))
        return 0
    return 0 if args.report_only else 1


if __name__ == '__main__':
    raise SystemExit(main())
