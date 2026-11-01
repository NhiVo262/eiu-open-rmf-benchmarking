#!/usr/bin/env python3
"""Writes F8 for one platform: planning-latency growth and throughput between fleet sizes.

8a reads each task's planning latency, 1b, from task_planning_metrics.json:

  P(N)       mean 1b over the tasks at fleet size N; the median is written beside it
  G          (P(3) - P(1)) / P(1) x 100
  as specified   every N=3 task against every N=1 task
  first task     only the first task of each N=3 batch, priced with no other task in
                 the fleet's plan, as an N=1 task is

The scenario-level figures compare a scenario's tasks with the N=1 tasks of its own
routes. 8b reads Th(1), Th(3) and the scaling from fleet_scaling_metrics.json, written by
analyse/fleet_coordination/analyze_fleet_scaling.py. METRIC_DESIGN.md §F8 defines the
measurands.

Usage:
    python3 analyze_scalability.py --platform-dir data/<platform> --output <json>
"""
import argparse
import glob
import json
import os
import statistics as st


def latencies_ms(paths, first_only=False):
    out = []
    for f in paths:
        tasks = sorted(json.load(open(f))['per_task'], key=lambda t: t['t_bid_notice'])
        for t in tasks[:1] if first_only else tasks:
            if t.get('planning_latency_s') is not None:
                out.append(1000.0 * t['planning_latency_s'])
    return out


def growth(p3, p1):
    def stats(v):
        return {'n': len(v), 'mean_ms': round(st.mean(v), 3), 'median_ms': round(st.median(v), 3),
                'max_ms': round(max(v), 3)}
    a, b = stats(p3), stats(p1)
    return {'n3': a, 'n1': b,
            'g_mean_pct': round(100.0 * (a['mean_ms'] - b['mean_ms']) / b['mean_ms'], 1),
            'g_median_pct': round(100.0 * (a['median_ms'] - b['median_ms']) / b['median_ms'], 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--platform-dir', required=True, help='data/<platform>, holding scenarios/ and baselines/')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    root = args.platform_dir
    n3 = sorted(glob.glob(os.path.join(root, 'scenarios', '*', 'run_*', 'repeat_*', 'task_planning_metrics.json')))
    n1 = sorted(glob.glob(os.path.join(root, 'baselines', '*', 'run_*', 'repeat_*', 'task_planning_metrics.json')))
    result = {
        '8a_as_specified': growth(latencies_ms(n3), latencies_ms(n1)),
        '8a_first_task': growth(latencies_ms(n3, first_only=True), latencies_ms(n1)),
        '8a_per_scenario': {},
        '8b': {},
    }
    for scenario_dir in sorted(glob.glob(os.path.join(root, 'scenarios', '*'))):
        if not os.path.isdir(scenario_dir):
            continue
        name = os.path.basename(scenario_dir)
        s3 = glob.glob(os.path.join(scenario_dir, 'run_*', 'repeat_*', 'task_planning_metrics.json'))
        s1 = glob.glob(os.path.join(root, 'baselines', name + '_r*', 'run_*', 'repeat_*', 'task_planning_metrics.json'))
        result['8a_per_scenario'][name] = growth(latencies_ms(s3), latencies_ms(s1))
    scaling = os.path.join(root, 'fleet_scaling_metrics.json')
    if os.path.exists(scaling):
        for name, r in json.load(open(scaling)).items():
            result['8b'][name] = {k: r[k] for k in ('th_1_tasks_per_min', 'th_n_tasks_per_min',
                                                    'scaling_pct_of_ideal', 'ceiling_pct_of_ideal')}
    json.dump(result, open(args.output, 'w'), indent=2)
    for key in ('8a_as_specified', '8a_first_task'):
        g = result[key]
        print('%-16s P(3) %.2f / %.2f ms  P(1) %.2f / %.2f ms  G mean %+.1f%%  median %+.1f%%' % (
            key, g['n3']['mean_ms'], g['n3']['median_ms'], g['n1']['mean_ms'], g['n1']['median_ms'],
            g['g_mean_pct'], g['g_median_pct']))
    for name, g in result['8a_per_scenario'].items():
        print('  %-11s P(3) %.2f  P(1) %.2f  G %+.1f%% (median %+.1f%%)' % (
            name, g['n3']['mean_ms'], g['n1']['mean_ms'], g['g_mean_pct'], g['g_median_pct']))


if __name__ == '__main__':
    main()
