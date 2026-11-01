#!/usr/bin/env python3
"""Merges a run's per-repeat execution_fidelity_metrics.json into one scenario-level
file.

Deltas are pooled from the concatenated per-task values, never averaged over
per-repeat means.

A task whose queueing_s exceeds QUEUEING_OUTLIER_S is excluded and listed in the
output.
"""
import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'common'))
from metric_statistics import summarize 

QUEUEING_OUTLIER_S = 400.0




def main():
    ap = argparse.ArgumentParser(description='Merge per-repeat execution fidelity metrics')
    ap.add_argument('--run-dir', required=True, help='Run directory holding the repeat_* directories')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.run_dir, 'repeat_*', 'execution_fidelity_metrics.json')))
    if not files:
        raise SystemExit(f'no per-repeat execution_fidelity_metrics.json under {args.run_dir}')

    kept, excluded, v1_kept, corrections, extrapolated = [], [], [], [], []
    for i, f in enumerate(sorted(files), start=1):
        for t in json.load(open(f)).get('per_task', []):
            if '5a_delta_actual_minus_predicted_s' not in t:
                continue
            q = t.get('queueing_s')
            if q is not None and q > QUEUEING_OUTLIER_S:
                excluded.append({'repeat': i, 'task_id': t.get('task_id'),
                                 'robot': t.get('robot'), 'queueing_s': round(q, 1),
                                 'delta_s': t['5a_delta_actual_minus_predicted_s']})
                continue
            kept.append(t['5a_delta_actual_minus_predicted_s'])
            v1_kept.append(t['5a_delta_v1_bid_offset_s'])
            corrections.append(t['drift_correction_s'])
            if t.get('clock_extrapolated'):
                extrapolated.append({'repeat': i, 'task_id': t.get('task_id')})

    result = {
        'scenario': args.scenario,
        'run_dir': args.run_dir,
        'n_repeats': len(files),
        'clock_conversion': 'v2 -- finish_time looked up directly against /clock '
                            '(exact under sim-clock drift); see analyze_execution_fidelity.py',
        '5a_delta_actual_minus_predicted_s': summarize(kept),
        '5a_delta_v1_bid_offset_s_for_comparison': summarize(v1_kept),
        'drift_correction_s': summarize(corrections),
        'n_excluded': len(excluded),
        'excluded_tasks': excluded,
        'excluded_rule': f'queueing_s > {QUEUEING_OUTLIER_S}s (host CPU overload)',
        'n_clock_extrapolated': len(extrapolated),
        'clock_extrapolated_tasks': extrapolated,
        'per_repeat_source_files': files,
    }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ('excluded_tasks', 'per_repeat_source_files',
                                   'clock_extrapolated_tasks')}, indent=2, default=str))


if __name__ == '__main__':
    main()
