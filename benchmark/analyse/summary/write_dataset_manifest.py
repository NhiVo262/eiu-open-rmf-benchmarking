#!/usr/bin/env python3
"""Writes the manifest of a collected dataset: what each repeat holds, and the
configuration the collection ran under.

Per repeat it records the bag's size, message count, duration and topic count as the
recorder itself reported them, the metrics files present, the capture the repeat was
analysed against, and that repeat's task ids. Per collection it records the repeat count
and the analysis versions found. The configuration block carries the SHA-256 of the files
that define the system under test.

The manifest holds no timestamp, so two runs over an unchanged dataset produce the same
file and a difference between them is a difference in the data.

Usage:
    python3 write_dataset_manifest.py --data-dir <data> --output <json>
"""

import argparse
import hashlib
import json
import os
import re

import yaml

# The files that define the system under test. A path is resolved against the workspace
# root; one that is absent is recorded as null rather than omitted.
CONFIGURATION = {
    'nav_graph': 'install/tb3_fleet/share/tb3_fleet/maps/world_tb3/nav_graphs/0.yaml',
    'world': 'install/tb3_fleet/share/tb3_fleet/maps/world_tb3/world_tb3.world',
    'building_map': 'src/tb3_fleet/maps/turtlebot3_world/world_tb3.building.yaml',
    'fleet_config_simulation': 'src/tb3_fleet/config/simulation/fleet/multi_robot_config_sim.yaml',
    'fleet_config_hardware': 'src/tb3_fleet/config/hardware/fleet/multi_robot_config_hw.yaml',
    'nav2_params': 'src/tb3_fleet/config/simulation/nav2_params.yaml',
}

METRICS = re.compile(r'_metrics\.json$')


def sha256(path):
    """The file's SHA-256, or None when it is absent."""
    if not os.path.exists(path):
        return None
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def bag_record(bag_dir):
    """Size, message count, duration and topic count from the bag's own metadata."""
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.isdir(bag_dir) or not os.path.exists(meta):
        return None
    with open(meta) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    files = info.get('files') or []
    return {
        'bytes': sum(os.path.getsize(os.path.join(bag_dir, f['path']))
                     for f in files if os.path.exists(os.path.join(bag_dir, f['path']))),
        'messages': info.get('message_count'),
        'duration_s': round((info.get('duration') or {}).get('nanoseconds', 0) / 1e9, 3),
        'topics': len(info.get('topics_with_message_count') or []),
        'storage': info.get('storage_identifier'),
    }


def repeat_record(repeat_dir):
    """What one repeat holds."""
    names = sorted(os.listdir(repeat_dir))
    metrics = sorted(n for n in names if METRICS.search(n))
    record = {
        'bag': bag_record(os.path.join(repeat_dir, 'bag')),
        'metrics': metrics,
        'ground_truth': 'ground_truth_poses.jsonl' in names,
    }
    task_planning = os.path.join(repeat_dir, 'task_planning_metrics.json')
    if os.path.exists(task_planning):
        with open(task_planning) as f:
            tp = json.load(f)
        record['api_state'] = os.path.basename(tp.get('api_state_path') or '') or None
        record['task_ids'] = sorted(t.get('task_id') for t in (tp.get('per_task') or [])
                                    if t.get('task_id'))
        record['analysis_version'] = tp.get('analysis_version')
    return record


def collection_record(collection_dir):
    """Every run of one collection, and the analysis versions its metrics carry."""
    runs, versions = {}, {}
    for entry in sorted(os.listdir(collection_dir)):
        path = os.path.join(collection_dir, entry)
        if not os.path.isdir(path):
            continue
        for run in sorted(os.listdir(path)):
            run_dir = os.path.join(path, run)
            if not run.startswith('run_') or not os.path.isdir(run_dir):
                continue
            repeats = {}
            for repeat in sorted(os.listdir(run_dir)):
                repeat_dir = os.path.join(run_dir, repeat)
                if repeat.startswith('repeat_') and os.path.isdir(repeat_dir):
                    repeats[repeat] = repeat_record(repeat_dir)
                    for name in repeats[repeat]['metrics']:
                        try:
                            with open(os.path.join(repeat_dir, name)) as f:
                                v = json.load(f).get('analysis_version')
                        except (OSError, ValueError):
                            v = None
                        versions.setdefault(name, set()).add(v)
            runs['%s/%s' % (entry, run)] = {'repeats': len(repeats), 'repeat': repeats}
    return {
        'runs': len(runs),
        'repeats': sum(r['repeats'] for r in runs.values()),
        'analysis_versions': {k: sorted(x for x in v if x is not None)
                              for k, v in sorted(versions.items())},
        'run': runs,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data-dir', default='src/benchmark/data',
                    help='the data directory holding simulation/ and hardware/')
    ap.add_argument('--workspace', default='.',
                    help='workspace root the configuration paths resolve against')
    ap.add_argument('--output', required=True, help='where to write the manifest JSON')
    args = ap.parse_args()

    collections = {}
    for platform in sorted(os.listdir(args.data_dir)):
        platform_dir = os.path.join(args.data_dir, platform)
        if not os.path.isdir(platform_dir):
            continue
        for collection in sorted(os.listdir(platform_dir)):
            path = os.path.join(platform_dir, collection)
            if os.path.isdir(path):
                collections['%s/%s' % (platform, collection)] = collection_record(path)

    manifest = {
        'configuration': {name: sha256(os.path.join(args.workspace, path))
                          for name, path in sorted(CONFIGURATION.items())},
        'totals': {
            'collections': len(collections),
            'repeats': sum(c['repeats'] for c in collections.values()),
        },
        'collections': collections,
    }
    with open(args.output, 'w') as f:
        json.dump(manifest, f, indent=1, sort_keys=True)

    print('%-28s %7s %8s' % ('collection', 'runs', 'repeats'))
    for name, c in sorted(collections.items()):
        print('%-28s %7d %8d' % (name, c['runs'], c['repeats']))
    print('%-28s %7s %8d' % ('total', '', manifest['totals']['repeats']))
    missing = [k for k, v in manifest['configuration'].items() if v is None]
    if missing:
        print('configuration files not found: ' + ', '.join(missing))
    return 1 if missing else 0


if __name__ == '__main__':
    raise SystemExit(main())
