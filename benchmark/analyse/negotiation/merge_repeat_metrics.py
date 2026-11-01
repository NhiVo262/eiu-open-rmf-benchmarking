#!/usr/bin/env python3
"""Merges a run's per-repeat negotiation metrics into one scenario-level file.

All three 3a counts are carried through -- notices, distinct conflict_versions and
conclusions -- so a conflict that was re-noticed shows up as a mismatch between them.
conflict_version restarts at 0 in every repeat, so counts are summed and never
de-duplicated by version.

Latency is pooled from the per-negotiation records, split by outcome: resolved,
abandoned, and abandoned split again by whether a refusal names the conflict.

`3b_n_latency_samples` is the number of negotiations a latency was observed for,
`3a_n_negotiation` the number that occurred, and `3b_unconcluded` their difference.
"""
import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'common'))
from metric_statistics import summarize  # noqa: E402




def main():
    ap = argparse.ArgumentParser(description='Merge per-repeat negotiation metrics')
    ap.add_argument('--run-dir', required=True, help='Run directory holding the repeat_* directories')
    ap.add_argument('--scenario', required=True, help='Scenario label recorded in the output')
    ap.add_argument('--output', required=True, help='Where to write the metrics JSON')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.run_dir, 'repeat_*', 'negotiation_metrics.json')))
    if not files:
        raise SystemExit(f'no per-repeat negotiation_metrics.json under {args.run_dir}')
    per_repeat = [json.load(open(f)) for f in files]

    n_notices = sum(r['3a_n_negotiation'] for r in per_repeat)
    n_versions = sum(r['3a_n_distinct_conflict_versions_noticed'] for r in per_repeat)
    n_conclusions = sum(r['3a_n_conclusions'] for r in per_repeat)

    per_negotiation = []
    for i, r in enumerate(per_repeat, start=1):
        for n in r.get('per_negotiation', []):
            n = dict(n)
            n['repeat'] = i
            per_negotiation.append(n)

    def latencies(keep):
        """The latencies of the negotiations the predicate keeps, in seconds."""
        return [n['latency_s'] for n in per_negotiation
                if n.get('latency_s') is not None and keep(n)]

    raw = latencies(lambda n: True)
    resolved = latencies(lambda n: n.get('resolved') is True)
    abandoned = latencies(lambda n: n.get('resolved') is False)
    refused = latencies(lambda n: n.get('resolved') is False and n.get('refused'))
    search_failed = latencies(lambda n: n.get('resolved') is False and not n.get('refused'))

    consistent = (n_notices == n_versions == n_conclusions)
    result = {
        'scenario': args.scenario,
        'run_dir': args.run_dir,
        'n_repeats': len(per_repeat),
        '3a_n_negotiation': n_notices,
        '3a_n_distinct_conflict_versions_noticed': n_versions,
        '3a_n_conclusions': n_conclusions,
        '3a_counts_consistent': consistent,
        '3a_note': (
            'one negotiation event = one conflict episode (negotiation_notice). '
            'proposals and forfeits are steps within an episode and are deliberately '
            'not counted here.'
            + ('' if consistent else
               ' WARNING: notices / distinct conflict_versions / conclusions disagree -- '
               'a conflict was re-noticed or left unconcluded; do not report 3a until resolved.')
        ),
        '3b_negotiation_latency_s': summarize(raw),
        '3b_latency_resolved_s': summarize(resolved),
        '3b_latency_abandoned_s': summarize(abandoned),
        '3b_latency_refused_s': summarize(refused),
        '3b_latency_search_failed_s': summarize(search_failed),
        '3b_n_latency_samples': len(raw),
        '3b_n_resolved': sum(r.get('3b_n_resolved', 0) for r in per_repeat),
        '3b_n_abandoned': sum(r.get('3b_n_abandoned', 0) for r in per_repeat),
        '3b_n_refused': sum(r.get('3b_n_refused', 0) for r in per_repeat),
        '3b_n_search_failed': sum(r.get('3b_n_search_failed', 0) for r in per_repeat),
        '3b_n_refusal_messages': sum(r.get('3b_n_refusal_messages', 0) for r in per_repeat),
        '3b_unconcluded': n_notices - len(raw),
        'raw_latencies_s': raw,
        'per_negotiation': per_negotiation,
        'per_repeat_source_files': files,
    }
    json.dump(result, open(args.output, 'w'), indent=2, default=str)
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ('raw_latencies_s', 'per_negotiation')}, indent=2, default=str))


if __name__ == '__main__':
    main()
