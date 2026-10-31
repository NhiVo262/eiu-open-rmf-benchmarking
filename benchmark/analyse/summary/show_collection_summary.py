#!/usr/bin/env python3
"""Prints one table per feature from the per-repeat metrics of several scenario runs.

A cell reads `value (low-high)`. The observation unit differs by metric and the header of
each column names it:

    pooled over n       a metric carrying its own sample count: the mean weighted by that
                        count, with the lowest and highest repeat mean in brackets
    mean of repeats     one figure per repeat: their mean, with their range in brackets

RESULTS.md reports pooled figures, so a `mean of repeats` cell can differ from it.
METRIC_DESIGN.md defines the metrics; the codes in the tables are its section numbers.
"""
import argparse
import glob
import json
import os
import statistics
import sys

FEATURE_FILES = {
    'F2': 'traffic_scheduling_metrics.json',
    'F3': 'negotiation_metrics.json',
    'F4a': 'fleet_coordination_4a_metrics.json',
    'F4b': 'fleet_coordination_metrics.json',
    'F5': 'execution_fidelity_metrics.json',
    'F6': 'congestion_handling_metrics.json',
    'F7': 'safety_tracker_metrics.json',
}


def load(run_dir, filename):
    """Return [(repeat_dir, metrics)] for every repeat of the run that holds that file."""
    out = []
    for repeat in sorted(glob.glob(os.path.join(run_dir, 'repeat_*'))):
        path = os.path.join(repeat, filename)
        if not os.path.exists(path):
            continue
        try:
            out.append((os.path.basename(repeat), json.load(open(path))))
        except (json.JSONDecodeError, OSError):
            continue
    return out


def spread(values, unit='', dp=2, mean=None):
    """Return `mean (low-high)` over the values, or a dash when none are present. `mean`
    overrides the arithmetic mean of the values."""
    values = [v for v in values if v is not None]
    if not values:
        return '-'
    fmt = '%.' + str(dp) + 'f'
    if len(values) == 1:
        return (fmt + '%s') % (values[0], unit)
    return (fmt + '%s (' + fmt + '-' + fmt + ')') % (
        statistics.mean(values) if mean is None else mean,
        unit, min(values), max(values))


def nested_pooled(entries, key, scale=1.0):
    """The mean of a nested metric weighted by each entry's sample count, or None when no
    entry carries one."""
    total = count = 0.0
    for _, m in entries:
        v = m.get(key)
        if isinstance(v, dict) and v.get('n') and v.get('mean') is not None:
            total += v['n'] * v['mean'] * scale
            count += v['n']
    return total / count if count else None


def nested_mean(entries, key, field='mean', scale=1.0):
    """Return the mean of one field of a nested metric across the entries."""
    out = []
    for _, m in entries:
        v = m.get(key)
        if isinstance(v, dict) and v.get('n') and v.get(field) is not None:
            out.append(v[field] * scale)
    return out


def f2(scenarios):
    """Print the Feature 2 table: submissions, conflicts, feasibility and 2b latency."""
    print('%-13s %8s %10s %12s %s' % (
        'scenario', 'submit', 'conflicts', 'feasible %', '2b scheduling time, pooled'))
    print('-' * 72)
    for name, run in scenarios.items():
        ms = load(run, FEATURE_FILES['F2'])
        submit = sum(m.get('n_submit', 0) for _, m in ms)
        conflicts = sum(m.get('n_conflicts_detected', 0) for _, m in ms)
        feas = [m['2a_schedule_feasibility_pct'] for _, m in ms
                if m.get('2a_schedule_feasibility_pct') is not None]
        print('%-13s %8d %10d %12s %s' % (
            name, submit, conflicts,
            '%.1f' % statistics.mean(feas) if feas else '-',
            spread(nested_mean(ms, '2b_scheduling_time_s', scale=1000), ' ms',
                   mean=nested_pooled(ms, '2b_scheduling_time_s', scale=1000))))


def f3(scenarios):
    """Print the Feature 3 table: negotiations, conclusions and 3b latency."""
    print('%-13s %14s %12s %s' % (
        'scenario', 'negotiations', 'conclusions', '3b latency, pooled'))
    print('-' * 62)
    for name, run in scenarios.items():
        ms = load(run, FEATURE_FILES['F3'])
        print('%-13s %14d %12d %s' % (
            name,
            sum(m.get('3a_n_negotiation', 0) for _, m in ms),
            sum(m.get('3a_n_conclusions', 0) for _, m in ms),
            spread(nested_mean(ms, '3b_negotiation_latency_s', scale=1000), ' ms',
                   mean=nested_pooled(ms, '3b_negotiation_latency_s', scale=1000))))


def f4(scenarios):
    """Print the Feature 4 table: tasks submitted and completed, and 4b utilisation."""
    print('%-13s %9s %11s %12s %s' % (
        'scenario', 'submitted', 'completed', 'completed %', '4b U_avg, mean of repeats'))
    print('-' * 68)
    for name, run in scenarios.items():
        a = load(run, FEATURE_FILES['F4a'])
        b = load(run, FEATURE_FILES['F4b'])
        sub = sum(m.get('4a_n_submitted', 0) for _, m in a)
        comp = sum(m.get('4a_n_completed', 0) for _, m in a)
        u = [m['4b_u_avg_pct'] for _, m in b if m.get('4b_u_avg_pct') is not None]
        print('%-13s %9d %11d %11.1f%% %s' % (
            name, sub, comp, 100.0 * comp / sub if sub else 0.0,
            spread(u, ' %', 1)))


def f5(scenarios):
    """Print the Feature 5 table: the schedule-against-execution delta per scenario."""
    print('%-13s %10s %s' % ('scenario', 'n tasks',
                         '5a schedule-vs-execution delta, pooled'))
    print('-' * 62)
    for name, run in scenarios.items():
        ms = load(run, FEATURE_FILES['F5'])
        n = sum(m.get('n_tasks_with_delta', 0) for _, m in ms)
        print('%-13s %10d %s' % (name, n,
                                 spread(nested_mean(ms, '5a_delta_s'), ' s',
                   mean=nested_pooled(ms, '5a_delta_s'))))


def f6(scenarios):
    """Print the Feature 6 table: time blocked per robot-task, the episode count and the
    mean episode, pooled over the run's repeats. The comparison against the N=1 baseline
    is made per route when reporting."""
    print('%-13s %8s %20s %9s %20s' % ('scenario', 'scored',
                                   '6a blocked/task, mean of robots', 'episodes',
                                   '6b per episode, mean of repeats'))
    print('-' * 70)
    for name, run in scenarios.items():
        ms = load(run, FEATURE_FILES['F6'])
        blocked = [v['time_blocked_s'] for _, m in ms for v in (m.get('6a_per_robot') or {}).values()
                   if v.get('time_blocked_s') is not None]
        total = sum(m.get('n_episodes_total', 0) for _, m in ms)
        tasks = sum(len(m.get('6a_per_robot') or {}) for _, m in ms)
        mean_ep = (sum((m.get('6b_mean_per_episode_s') or 0) * m.get('n_episodes_total', 0) for _, m in ms)
                   / total) if total else None
        print('%-13s %8s %16s %9d %16s' % (
            name, '%d/%d' % (len(blocked), tasks), spread(blocked, ' s'), total,
            '%.2f s' % mean_ep if mean_ep is not None else '-'))


def f7(scenarios):
    """Print the Feature 7 table: minimum separation from both sources, and the episodes
    RMF saw against those it missed."""
    print('%-13s %16s %16s %10s %10s' % (
        'scenario', '7a true min', '7a RMF believed', 'RMF saw', 'RMF MISSED'))
    print('-' * 72)
    for name, run in scenarios.items():
        ms = load(run, FEATURE_FILES['F7'])
        true_min, rmf_min, saw, missed = [], [], 0, 0
        for _, m in ms:
            d = m.get('7a_min_distance') or {}
            for src, bucket in (('ground_truth', true_min), ('rmf_observed', rmf_min)):
                v = (d.get(src) or {}).get('min_distance_m')
                if v is not None:
                    bucket.append(v)
            ep = m.get('7b_episodes')
            if isinstance(ep, dict):
                saw += ep.get('really_happened_and_rmf_saw_it', 0)
                missed += ep.get('really_happened_but_rmf_MISSED_it', 0)
        print('%-13s %16s %16s %10d %10d' % (
            name, spread(true_min, ' m', 3), spread(rmf_min, ' m', 3), saw, missed))


SECTIONS = [
    ('F2 - Traffic Scheduling', f2),
    ('F3 - Negotiation', f3),
    ('F4a - Task State  /  F4b - Utilization', f4),
    ('F5 - Execution Fidelity', f5),
    ('F6 - Congestion Handling', f6),
    ('F7 - Safety', f7),
]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', action='append', required=True, dest='scenarios',
                    metavar='NAME=RUN_DIR',
                    help='Repeat per scenario, e.g. --scenario "Head-on=data/'
                         'hardware/scenarios/headon/run_20261019_0203"')
    ap.add_argument('--only', action='append', dest='only',
                    help='Limit to these features, e.g. --only F6 --only F7')
    args = ap.parse_args()

    scenarios = {}
    for spec in args.scenarios:
        if '=' not in spec:
            ap.error('--scenario needs NAME=RUN_DIR, got %r' % spec)
        name, run = spec.split('=', 1)
        if not os.path.isdir(run):
            ap.error('not a directory: %s' % run)
        scenarios[name] = run

    wanted = set(args.only or [])
    for title, fn in SECTIONS:
        tag = title.split()[0]
        if wanted and not any(w in title for w in wanted):
            continue
        print('=' * 80)
        print(title)
        fn(scenarios)
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
