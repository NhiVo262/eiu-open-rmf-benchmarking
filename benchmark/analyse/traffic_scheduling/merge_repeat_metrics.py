#!/usr/bin/env python3
"""Merges a run's per-repeat traffic_scheduling_metrics.json files into one
scenario-level metrics file.

  counts        n_submit, n_conflicts_detected and 2a_n_task_infeasible are summed
  percentages   recomputed from the summed counts, not averaged over repeats
  outcomes      resolved_by_negotiation, recovered_by_replan and unrecovered are
                summed per key
  2b            pooled from the concatenated raw_scheduling_times_s
"""
import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'common'))
from metric_statistics import summarize  # noqa: E402




def parse_args():
    """Parse the command line."""
    ap = argparse.ArgumentParser(description='Merge per-repeat traffic_scheduling_metrics.json files')
    ap.add_argument('--run-dir', required=True, help='Scenario run directory (contains repeat_N/traffic_scheduling_metrics.json)')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--robots', nargs='+', required=True, help='Robot names, space separated')
    ap.add_argument('--output', default=None, help='Where to write the metrics JSON')
    return ap.parse_args()


def main():
    args = parse_args()
    run_dir = args.run_dir
    output = args.output or os.path.join(run_dir, 'traffic_scheduling_metrics.json')

    files = sorted(glob.glob(os.path.join(run_dir, 'repeat_*', 'traffic_scheduling_metrics.json')))
    if not files:
        raise SystemExit(f'no per-repeat traffic_scheduling_metrics.json found under {run_dir}')

    per_repeat = [json.load(open(f)) for f in files]

    n_submit = sum(r['n_submit'] for r in per_repeat)
    n_conflicts_detected = sum(r['n_conflicts_detected'] for r in per_repeat)
    n_task_infeasible = sum(r['2a_n_task_infeasible'] for r in per_repeat)

    schedule_feasibility_pct = (
        100.0 * (n_submit - n_task_infeasible) / n_submit if n_submit else None
    )

    outcome_counts = {}
    for r in per_repeat:
        for k, v in (r.get('2a_conflict_outcome_counts') or {}).items():
            outcome_counts[k] = outcome_counts.get(k, 0) + v

    raw_scheduling_times_s = []
    for r in per_repeat:
        raw_scheduling_times_s.extend(r.get('raw_scheduling_times_s', []))

    per_conflict = []
    for i, r in enumerate(per_repeat, start=1):
        for c in r.get('per_conflict', []):
            c = dict(c)
            c['repeat'] = i
            per_conflict.append(c)

    result = {
        'scenario': args.scenario,
        'run_dir': run_dir,
        'robots': args.robots,
        'n_repeats': len(per_repeat),
        'n_submit': n_submit,
        'n_conflicts_detected': n_conflicts_detected,
        '2a_n_task_infeasible': n_task_infeasible,
        '2a_schedule_feasibility_pct': schedule_feasibility_pct,
        '2a_conflict_outcome_counts': outcome_counts,
        '2b_scheduling_time_s': summarize(raw_scheduling_times_s),
        'raw_scheduling_times_s': raw_scheduling_times_s,
        'per_conflict': per_conflict,
        'per_repeat_source_files': files,
    }

    with open(output, 'w') as f:
        json.dump(result, f, indent=2, default=str)
    print(json.dumps(result, indent=2, default=str))


if __name__ == '__main__':
    main()
