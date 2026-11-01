#!/usr/bin/env python3
"""Writes F4's throughput scaling and its utilisation against the single-robot baseline,
per scenario, from the per-repeat fleet_coordination_metrics.json files.

For one scenario run at N robots and the baseline runs of its routes at N=1:

  Th(N)       tasks completed / execution window, pooled over the scenario's repeats
  Th(1)       tasks completed / execution time, pooled over the baseline repeats of the
              scenario's routes; a single robot is taken to run the routes one after
              another with no travel between them
  scaling     Th(N) / (N x Th(1)), the requirement's ideal linear scaling
  ceiling     sum(d) / (N x max(d)), d each route's mean execution time alone: the
              scaling three interference-free robots would reach on routes of unequal
              length
  U(N)        the fleet's moving time over the robots' window time, pooled
  U(1)        the same for each baseline route, and their mean

Th(1) omits the travel between routes, so scaling is a lower bound. Writes one JSON for
the collection. METRIC_DESIGN.md §F4 defines the measurands.

Usage:
    python3 analyze_fleet_scaling.py --platform-dir data/<platform> --output <json>
"""
import argparse
import glob
import json
import os

METRICS = 'fleet_coordination_metrics.json'


def repeats(run_dir):
    for repeat in sorted(glob.glob(os.path.join(run_dir, 'repeat_*'))):
        f = os.path.join(repeat, METRICS)
        if os.path.exists(f):
            yield repeat, json.load(open(f))


def places(repeat):
    return tuple(json.load(open(os.path.join(repeat, 'task_planning_metrics.json')))['per_task'][0]['places'])


def pooled_utilisation(entries):
    moving = window = 0.0
    for _, m in entries:
        for robot in (m['4b_per_robot'] or {}).values():
            if robot['t_moving_s'] is not None:
                moving += robot['t_moving_s']
                window += m['4b_window_s']
    return 100.0 * moving / window if window else None


def assess(scenario_run, baseline_runs):
    scen = list(repeats(scenario_run))
    n_robots = max(len(m['4b_per_robot'] or {}) for _, m in scen)
    th_n = 60.0 * sum(m['n_tasks_completed'] for _, m in scen) / sum(m['4b_window_s'] for _, m in scen)
    routes = {}
    for run in baseline_runs:
        entries = list(repeats(run))
        exec_s = [m['4b_window_s'] for _, m in entries]
        routes[os.path.basename(os.path.dirname(run))] = {
            'places': list(places(entries[0][0])),
            'n_repeats': len(entries),
            'mean_execution_s': round(sum(exec_s) / len(exec_s), 3),
            'u_pct': round(pooled_utilisation(entries), 2),
            'tasks': sum(m['n_tasks_completed'] for _, m in entries),
            'time_s': sum(exec_s),
        }
    th_1 = 60.0 * sum(r['tasks'] for r in routes.values()) / sum(r['time_s'] for r in routes.values())
    d = [r['mean_execution_s'] for r in routes.values()]
    u1 = [r['u_pct'] for r in routes.values()]
    scored = [m.get('4b_n_robots_scored', n_robots) for _, m in scen]
    return {
        'scenario_run': scenario_run,
        'n_robots': n_robots,
        'n_repeats': len(scen),
        'th_n_tasks_per_min': round(th_n, 4),
        'th_1_tasks_per_min': round(th_1, 4),
        'scaling_pct_of_ideal': round(100.0 * th_n / (n_robots * th_1), 2),
        'ceiling_pct_of_ideal': round(100.0 * sum(d) / (n_robots * max(d)), 2),
        'u_n_pct': round(pooled_utilisation(scen), 2),
        'u_n_robots_scored_per_repeat': scored,
        'u_1_route_mean_pct': round(sum(u1) / len(u1), 2),
        'baseline_routes': {k: {kk: vv for kk, vv in v.items() if kk not in ('tasks', 'time_s')}
                            for k, v in routes.items()},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--platform-dir', required=True, help='data/<platform>, holding scenarios/ and baselines/')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    out = {}
    for scenario_dir in sorted(glob.glob(os.path.join(args.platform_dir, 'scenarios', '*'))):
        if not os.path.isdir(scenario_dir):
            continue
        name = os.path.basename(scenario_dir)
        run = sorted(glob.glob(os.path.join(scenario_dir, 'run_*')))[-1]
        baselines = [sorted(glob.glob(os.path.join(b, 'run_*')))[-1]
                     for b in sorted(glob.glob(os.path.join(args.platform_dir, 'baselines', name + '_r*')))]
        out[name] = assess(run, baselines)
    json.dump(out, open(args.output, 'w'), indent=2)
    for name, r in out.items():
        print('%-11s Th(N) %.3f  Th(1) %.3f  scaling %.1f%%  ceiling %.1f%%  U(N) %.1f%%  U(1) %.1f%%'
              % (name, r['th_n_tasks_per_min'], r['th_1_tasks_per_min'], r['scaling_pct_of_ideal'],
                 r['ceiling_pct_of_ideal'], r['u_n_pct'], r['u_1_route_mean_pct']))


if __name__ == '__main__':
    main()
