#!/usr/bin/env python3

import argparse
import json
import os

TERMINAL_FAILURE_STATUS = ('error', 'failed', 'canceled', 'killed')

def task_states(metrics_path):
    """task_id -> {'task_status', 'completed', 'censored', 'robot'} from one repeat's
    task_planning_metrics.json."""
    with open(metrics_path) as f:
        metrics = json.load(f)
    return {t['task_id']: {k: t.get(k) for k in ('task_status', 'completed', 'censored', 'robot')}
            for t in metrics.get('per_task', []) if t.get('task_id')}



def outcome_from_task_state(entry, states):
    per_robot = {
        robot: []
        for robot in entry.get('total_distance_m', {})
    }

    unresolved = []

    for task in entry.get('tasks', []):
        task_id = task.get('task_id')

        if not task_id:
            unresolved.append({
                'request_id': task.get('request_id'),
                'reason': 'missing_task_id',
            })
            continue

        state = states.get(task_id)

        if state is None:
            unresolved.append({
                'task_id': task_id,
                'reason': 'missing_task_metrics',
            })
            continue

        robot = state.get('robot')

        if not robot or robot not in per_robot:
            unresolved.append({
                'task_id': task_id,
                'reason': 'unknown_robot_assignment',
            })
            continue

        status = state.get('task_status')
        if status == 'completed' and state.get('completed') is True:
            per_robot[robot].append('completed')
        elif status in TERMINAL_FAILURE_STATUS:
            per_robot[robot].append('not_completed')
        else:
            per_robot[robot].append('unknown')

    outcomes = {}

    for robot, task_results in per_robot.items():
        if not task_results:
            outcomes[robot] = 'unknown'
        elif all(result == 'completed' for result in task_results):
            outcomes[robot] = 'completed'
        elif any(result == 'not_completed' for result in task_results):
            outcomes[robot] = 'not_completed'
        else:
            outcomes[robot] = 'unknown'

    # Missing tasks or assignments invalidate repeat-level verification.
    verified = (
        len(unresolved) == 0
        and len(entry.get('tasks', [])) > 0
        and all(
            result == 'completed'
            for result in outcomes.values()
        )
    )

    return outcomes, unresolved, verified

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
        outcome, unresolved, verified = outcome_from_task_state(
            entry, states
        )

        entry['outcome'] = outcome
        entry['outcome_basis'] = 'task_planning_metrics'
        entry['outcome_verified'] = verified
        entry['unresolved_tasks'] = unresolved

    with open(args.repeats_summary, 'w') as f:
        json.dump(entries, f, indent=2, default=str)
    print('annotated %d repeat entr(ies) in %s' % (len(entries), args.repeats_summary))


if __name__ == '__main__':
    main()
