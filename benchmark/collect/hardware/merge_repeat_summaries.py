#!/usr/bin/env python3
"""Concatenate the per-repeat repeats_summary.json files of one run directory.

Each repeat_N/repeats_summary.json holds a list of task entries. This writes their
concatenation, ordered by repeat number, to repeats_summary.json in the run directory.
"""

import argparse
import glob
import json
import os
import re
import sys


def repeat_number(path):
    """The N in the repeat_N component of a path, or -1 when absent."""
    match = re.search(r'repeat_(\d+)', path)
    return int(match.group(1)) if match else -1


def merge(run_dir):
    """Return the concatenated task entries and the files they came from."""
    files = sorted(glob.glob(os.path.join(run_dir, 'repeat_[0-9]*',
                                          'repeats_summary.json')),
                   key=repeat_number)
    merged = []
    for path in files:
        merged.extend(json.load(open(path)))
    return merged, files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True,
                        help='Run directory holding repeat_N subdirectories')
    parser.add_argument('--output', default=None,
                        help='defaults to repeats_summary.json in --run-dir')
    args = parser.parse_args()

    merged, files = merge(args.run_dir)
    if not files:
        print(f'no repeat_N/repeats_summary.json under {args.run_dir}')
        return 1

    out = args.output or os.path.join(args.run_dir, 'repeats_summary.json')
    with open(out, 'w') as handle:
        json.dump(merged, handle, indent=2)
    print(f'merged {len(files)} repeat dir(s) into {len(merged)} task entries -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
