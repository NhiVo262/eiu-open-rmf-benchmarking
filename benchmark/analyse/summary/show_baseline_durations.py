#!/usr/bin/env python3
"""Prints the hardware baseline duration of each repeat beside the simulation median
for the same route.

One line per repeat: the route leg, the total including transit and the auction, then
the simulation median for the same route.
"""

import argparse
import glob
import json
import os
import re
import statistics

# Route tags, in collection order.
ROUTE_TAGS = (
    'bottleneck_r1', 'bottleneck_r2', 'bottleneck_r3',
    'crossing_r1', 'crossing_r2', 'crossing_r3',
    'headon_r1', 'headon_r2', 'headon_r3',
    'sharedlane_r1', 'sharedlane_r2', 'sharedlane_r3',
)

# benchmark/ -- two levels above analyse/summary/show_baseline_durations.py
BENCHMARK_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
DEFAULT_SIM_ROOT = os.path.join(BENCHMARK_ROOT, 'data', 'simulation', 'baselines')
DEFAULT_HW_ROOT = os.path.join(BENCHMARK_ROOT, 'data', 'hardware', 'baselines')



def sim_medians_from(root):
    """Median route-leg and total duration per route, from a baseline tree."""
    out = {}
    for tag in ROUTE_TAGS:
        legs, totals = [], []
        for f in glob.glob(os.path.join(root, tag, 'run_*', 'repeat_*',
                                        'task_planning_metrics.json')):
            for task in json.load(open(f)).get('per_task', []):
                if task.get('makespan_s') is not None:
                    legs.append(task['makespan_s'])
                if task.get('makespan_incl_transit_s') is not None:
                    totals.append(task['makespan_incl_transit_s'])
        if legs:
            out[tag] = (statistics.median(legs), statistics.median(totals))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--baselines-root',
                    default=DEFAULT_HW_ROOT,
                    help='the hardware baselines tree (default %(default)s)')
    ap.add_argument('--tag', action='append', dest='tags',
                    help='route tag, repeatable; default every tag present')
    ap.add_argument('--sim-root', default=DEFAULT_SIM_ROOT,
                    help='simulation baseline tree the reference is computed from '
                         '(default %(default)s)')
    args = ap.parse_args()

    reference = sim_medians_from(args.sim_root)
    if not reference:
        print('no simulation reference found under %s; the sim columns will be '
              'blank' % args.sim_root)
    tags = args.tags or [t for t in ROUTE_TAGS
                         if os.path.isdir(os.path.join(args.baselines_root, t))]

    row = lambda *a: ('%-26s %12s %25s' % a).rstrip()
    header = row('', 'makespan_s', 'makespan_incl_transit_s')
    print(header)
    print('-' * len(header))
    for tag in tags:
        sim = reference.get(tag)
        files = sorted(
            glob.glob(os.path.join(args.baselines_root, tag, 'run_*',
                                   'repeat_*', 'task_planning_metrics.json')),
            key=lambda p: int(re.search(r'repeat_(\d+)', p).group(1)))
        if not files:
            print('%-26s no repeats found' % tag)
            continue
        print(row(tag + '  sim median',
                     '%.1f' % sim[0] if sim else '-',
                  '%.1f' % sim[1] if sim else '-'))
        values = []
        for f in files:
            name = re.search(r'(repeat_\d+)', f).group(1)
            data = json.load(open(f))
            for task in data.get('per_task', []):
                measured = task.get('makespan_s')
                with_transit = task.get('makespan_incl_transit_s')
                if measured is not None:
                    values.append(measured)
                print(row('  ' + name,
                             '%.1f' % measured if measured is not None else '-',
                          '%.1f' % with_transit if with_transit is not None else '-'))
        if len(values) > 1:
            print(row('  median', '%.1f' % statistics.median(values), ''))
        print()


if __name__ == '__main__':
    main()
