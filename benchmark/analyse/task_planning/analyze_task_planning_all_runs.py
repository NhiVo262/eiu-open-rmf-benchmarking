#!/usr/bin/env python3
"""Re-runs the Task Planning analyser over every repeat of one or more collections.

A collection is a directory such as data/<platform>/scenarios or data/<platform>/baselines;
the default is data/simulation/scenarios, and --collection-dir can be given more than
once. Scenario and route directories are discovered rather than listed, so a new one needs
no change here.

Each repeat is re-analysed against the API state capture and the robot list its existing
task_planning_metrics.json names: the attempt number of api_state_attempt<N>.jsonl is not
a stable identifier, so the capture is taken from the repeat's own record. A repeat with no
existing metrics, or whose capture is missing, is skipped and reported rather than
overwritten with a figure that has no task state. The metrics are written beside the bag,
and the analyser's output is kept in analyze_task_planning.log next to them.
"""
import argparse
import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCHMARK_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
ANALYSER = os.path.join(HERE, 'analyze_task_planning.py')
DEFAULT_COLLECTION = os.path.join(BENCHMARK_ROOT, 'data', 'simulation', 'scenarios')
METRICS = 'task_planning_metrics.json'


def default_nav_graph():
    """Return the installed tb3_fleet nav graph, or None when the workspace is not sourced."""
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        path = os.path.join(prefix, 'share', 'tb3_fleet', 'maps',
                            'world_tb3', 'nav_graphs', '0.yaml')
        if os.path.exists(path):
            return path
    return None


def repeats_to_analyse(collections, name_filter):
    """Return the repeat directories holding a bag, filtered by directory name substring."""
    out = []
    for collection in collections:
        for repeat in sorted(glob.glob(os.path.join(collection, '*', 'run_*', 'repeat_*'))):
            if not os.path.isdir(os.path.join(repeat, 'bag')):
                continue
            if name_filter and name_filter not in repeat:
                continue
            out.append(repeat)
    return out


def recorded_inputs(repeat):
    """(api_state path, robots) from the repeat's existing metrics, or (None, reason)."""
    metrics = os.path.join(repeat, METRICS)
    if not os.path.exists(metrics):
        return None, 'no existing %s to take the API capture from' % METRICS
    record = json.load(open(metrics))
    name = os.path.basename(record.get('api_state_path') or '')
    capture = os.path.join(os.path.dirname(repeat), name)
    if not name or not os.path.exists(capture):
        return None, 'the API capture %r is not in the run directory' % name
    return (capture, record.get('robots') or []), None


def parse_args():
    """Parse the command line."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--collection-dir', action='append', default=None,
                    help='Collection directory, data/<platform>/<scenarios|baselines>; '
                         'repeat for several (default: data/simulation/scenarios)')
    ap.add_argument('--scenario', default=None,
                    help='Only the directories whose path contains this text')
    ap.add_argument('--nav-graph', default=None,
                    help='Navigation graph YAML (default: the installed tb3_fleet graph)')
    return ap.parse_args()


def main():
    args = parse_args()
    nav_graph = args.nav_graph or default_nav_graph()
    if not nav_graph:
        sys.exit('no nav graph given and tb3_fleet is not on AMENT_PREFIX_PATH; '
                 'pass --nav-graph')

    collections = args.collection_dir or [DEFAULT_COLLECTION]
    repeats = repeats_to_analyse(collections, args.scenario)
    if not repeats:
        sys.exit('no repeat with a bag under %s' % ', '.join(collections))

    failed = skipped = 0
    for repeat in repeats:
        run_dir = os.path.dirname(repeat)
        label = '%s, %s, %s' % (os.path.basename(os.path.dirname(run_dir)),
                                os.path.basename(run_dir),
                                os.path.basename(repeat))
        inputs, reason = recorded_inputs(repeat)
        if inputs is None:
            skipped += 1
            print('%s: skipped, %s' % (label, reason), flush=True)
            continue
        capture, robots = inputs
        log = os.path.join(repeat, 'analyze_task_planning.log')
        print('%s: analysing' % label, flush=True)
        with open(log, 'w') as f:
            rc = subprocess.call(
                ['python3', ANALYSER,
                 '--bag', os.path.join(repeat, 'bag'),
                 '--robots', *robots,
                 '--scenario', label,
                 '--nav-graph', nav_graph,
                 '--api-state', capture,
                 '--output', os.path.join(repeat, METRICS)],
                stdout=f, stderr=subprocess.STDOUT)
        if rc != 0:
            failed += 1
            print('%s: exit code %d, see %s' % (label, rc, log), flush=True)

    print('\n%d repeat(s) analysed, %d failed, %d skipped.'
          % (len(repeats) - skipped, failed, skipped))
    return 1 if failed or skipped else 0


if __name__ == '__main__':
    sys.exit(main())
