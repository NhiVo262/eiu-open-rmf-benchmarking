#!/usr/bin/env python3

import argparse
import json
import os


def task_states(metrics_path):
    """task_id -> {'task_status', 'completed', 'censored', 'robot'} from one repeat's
    task_planning_metrics.json."""
    with open(metrics_path) as f:
        metrics = json.load(f)
    return {t['task_id']: {k: t.get(k) for k in ('task_status', 'completed', 'censored', 'robot')}
            for t in metrics.get('per_task', []) if t.get('task_id')}


def outcome_from_task_state(entry, states):
    per_robot = {robot: None for robot in entry.get('total_distance_m', {})}
    for task in entry.get('tasks', []):
        state = states.get(task.get('task_id'))
        if state is None:
            return None
        robot = state.get('robot')
        if robot is None:
            continue
        held = per_robot.get(robot)
        per_robot[robot] = bool(state.get('completed')) if held is None else (
            held and bool(state.get('completed')))
    return {robot: 'no_task' if ok is None else ('completed' if ok else 'not_completed')
            for robot, ok in per_robot.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repeats-summary', required=True,
                    help='repeats_summary.json of one repeat, rewritten in place')
    ap.add_argument('--task-planning-metrics', required=True,
                    help='task_planning_metrics.json produced for the same repeat')
    args = ap.parse_args()

    for path in (args.repeats_summary, args.task_planning_metrics):
        if not os.path.exists(path):
            raise SystemExit('missing %s' % path)

    states = task_states(args.task_planning_metrics)
    with open(args.repeats_summary) as f:
        entries = json.load(f)

    for entry in entries:
        for task in entry.get('tasks', []):
            state = states.get(task.get('task_id'))
            if state is not None:
                task.update({k: state[k] for k in ('task_status', 'completed', 'censored')})
        outcome = outcome_from_task_state(entry, states)
        if outcome is None:
            entry['outcome_basis'] = 'release and distance (no task state for every task)'
        else:
            entry['outcome'] = outcome
            entry['outcome_basis'] = 'task state (RMF API terminal status)'

    with open(args.repeats_summary, 'w') as f:
        json.dump(entries, f, indent=2, default=str)
    print('annotated %d repeat entr(ies) in %s' % (len(entries), args.repeats_summary))


if __name__ == '__main__':
    main()
