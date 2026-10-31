#!/usr/bin/env python3
"""Write the appendix tables A1 to A14 from the per-repeat metrics files.

Every figure is read from a metrics file written by that feature's analyser; no
value is transcribed from RESULTS.md. Pooled statistics are computed over the raw
per-observation values where an analyser recorded them, and over the per-repeat
summaries otherwise; each table names its unit.

Usage:
    python3 write_appendix_tables.py --data data --output APPENDIX.md
"""

import argparse
import glob
import json
import math
import os
import re
import statistics as st

PLATFORMS = ['simulation', 'hardware']
SCENARIOS = [('bottleneck', 'Bottleneck'), ('crossing', 'Crossing'),
             ('headon', 'Head-on'), ('sharedlane', 'Shared Lane')]
ROUTES = {
    'bottleneck': [('bottleneck_r1', 'wp_2->wp_5'), ('bottleneck_r2', 'wp_3->wp_4'),
                   ('bottleneck_r3', 'wp_4->wp_3')],
    'crossing': [('crossing_r1', 'wp_2->wp_6'), ('crossing_r2', 'wp_6->wp_9'),
                 ('crossing_r3', 'wp_7->wp_8')],
    'headon': [('headon_r1', 'wp_2->wp_5'), ('headon_r2', 'wp_5->wp_2'),
               ('headon_r3', 'wp_3->wp_7')],
    'sharedlane': [('sharedlane_r1', 'wp_2->wp_7'), ('sharedlane_r2', 'wp_4->wp_5'),
                   ('sharedlane_r3', 'wp_6->wp_3')],
}
BID_THRESHOLD_MS = 8.0          # the figure quoted in the milestone 2 report
SCHEDULE_BUDGET_S = 2.85        # from RMF's conflict separation and the robots' speed
D_SAFE_M = 0.356
FLOOR_M = 0.076                 # nose-to-nose reference-point separation
CONFLICT_SEP_M = 0.628
EPISODE_FLOOR_S = 0.3


CAPTIONS = {
  'a1':
    'One bid per task; 15 tasks per scenario–platform condition at N=3. Planning latency\n'
    'is t(bid_response) − t(bid_notice). The final column reports bids exceeding 8 ms.\n\n',
  'a2':
    'Execution duration is measured from task award to RMF-reported completion (s).\n'
    'Each route–platform condition includes five observations at N=1 and five at N=3.\n\n',
  'a3':
    'Counts are pooled over five repeats per condition. Conflict-free rate is\n'
    'itinerary-based; schedule feasibility is conflict-based.\n\n',
  'a4':
    'Scheduling latency is t(query_update_1) − t(itinerary_set) for matched\n'
    'itinerary updates, pooled across the four scenarios. The reference budget is 2.85 s.\n\n',
  'a5':
    'Negotiation notices are reported per repeat in collection order. Repeated notices\n'
    'can exceed the number of distinct conflicts in A3.\n\n',
  'a6':
    'Negotiation latency is measured from notice to conclusion (s), with one\n'
    'observation per concluded conflict and results stratified by outcome.\n\n',
  'a7':
    'Throughput is the batch completion rate over the execution window.\n'
    'Workload ceiling represents the theoretical scaling limit without inter-robot\n'
    'interference. Utilisation (U) is the fraction of the window with a motion command.\n\n',
  'a8':
    'F5a is relative duration error (%) against the prediction at bidding; F5b is\n'
    'finish-time offset (s), including dispatch queueing. Each condition includes 15 tasks.\n\n',
  'a9':
    'Stationary time (s) is measured without a motion command during an active task.\n'
    'Additional stationary time is the difference between N=3 and matched N=1 route means.\n\n',
  'a10':
    'An episode is a continuous interval without a motion command. Mean episode\n'
    'duration is aggregated across episodes; the reported SD represents variability\n'
    'among the five per-repeat means.\n\n',
  'a11':
    'F7a/F7b measure reference-point separation from RMF-reported poses\n'
    '(RobotState.location), not physical clearance. The geometric minimum\n'
    'reference-point separation is 0.076 m under the specified robot model.\n'
    'The table identifies repeats with reported separations below this limit.\n\n',
  'a12':
    'F7c estimates laser-centre separation using /scan, independent of reported\n'
    'poses and the navigation map. Coverage is the fraction of paired scan instants\n'
    'with a mutual match; estimates are subject to incomplete detection coverage.\n\n',
  'a12_validation':
    'F7c validation uses Gazebo ground truth in simulation only; no independent\n'
    'position reference is available for hardware.\n\n',
  'a13':
    'Results quantify the change from N=1 to N=3; scalability beyond this range was\n'
    'not evaluated. First-task statistics use the initial bid of each N=3 batch.\n\n',
  'a14':
    'Differences are calculated as N=3 minus the matched N=1 route means. Distance\n'
    'is RMF-reported; conflict counts are per-robot medians across repeats. Robot-level\n'
    'conflict counts within a repeat are not independent.\n\n',
}


def repeats(data, platform, kind, cell):
    """Per-repeat directories of one cell, ordered by repeat number."""
    pat = os.path.join(data, platform, kind, cell, 'run_*', 'repeat_*')
    return sorted(glob.glob(pat), key=lambda p: (os.path.dirname(p), int(p.rsplit('_', 1)[1])))


def load(repeat_dir, name):
    path = os.path.join(repeat_dir, name)
    if not os.path.exists(path):
        return None
    with open(path) as fh:
        return json.load(fh)


def route_of(task):
    return '->'.join(task['places'])


def stats(values):
    """mean, sd, min, max over the observations given; sd is 0 for a single value."""
    if not values:
        return None
    return {'n': len(values), 'mean': st.mean(values),
            'sd': st.stdev(values) if len(values) > 1 else 0.0,
            'min': min(values), 'max': max(values)}


def fmt(s, p=1, scale=1.0):
    if s is None:
        return '— | — | — | —'
    return ' | '.join('%.*f' % (p, s[k] * scale) for k in ('mean', 'sd', 'min', 'max'))


def execution_s(task):
    """Award to completion, the definition used throughout section A of RESULTS.md."""
    return task['makespan_incl_transit_s'] - task['queueing_s']


# ---------------------------------------------------------------- collection

def task_values(data, platform, kind, cell, pick, route=None):
    """One value per task, from every repeat of the cell."""
    out = []
    for rd in repeats(data, platform, kind, cell):
        tp = load(rd, 'task_planning_metrics.json')
        if not tp:
            continue
        for task in tp['per_task']:
            if route is None or route_of(task) == route:
                value = pick(task)
                if value is not None:
                    out.append(value)
    return out


def baseline_task_values(data, platform, route_id, route, pick):
    return task_values(data, platform, 'baselines', route_id, pick, route)


def fidelity_values(data, platform, kind, cell, key, route=None):
    """One value per task from execution_fidelity_metrics, keyed through task ids."""
    out = []
    for rd in repeats(data, platform, kind, cell):
        ef, tp = load(rd, 'execution_fidelity_metrics.json'), load(rd, 'task_planning_metrics.json')
        if not ef or not tp:
            continue
        routes = {t['task_id']: route_of(t) for t in tp['per_task']}
        for task in ef.get('per_task') or []:
            if route is not None and routes.get(task['task_id']) != route:
                continue
            if task.get(key) is not None:
                out.append(task[key])
    return out


def blocked_values(data, platform, kind, cell, route=None):
    """One value per robot-task from congestion_handling_metrics."""
    out = []
    for rd in repeats(data, platform, kind, cell):
        ch = load(rd, 'congestion_handling_metrics.json')
        if not ch:
            continue
        for entry in (ch.get('6a_per_robot') or {}).values():
            if route is None or entry['route'] == route:
                out.append(entry['time_blocked_s'])
    return out


# ------------------------------------------------------------------- tables

def a1(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            lat = [t['planning_latency_s'] * 1000 for t in
                   (t for rd in repeats(data, platform, 'scenarios', key)
                    for t in (load(rd, 'task_planning_metrics.json') or {'per_task': []})['per_task'])]
            won = task_values(data, platform, 'scenarios', key, lambda t: t['dispatch_success'])
            s = stats(lat)
            rows.append('| %s | %s | %d/%d | %.2f | %.2f | %.2f | %.2f | %d |' % (
                label, platform, sum(1 for x in won if x), len(won),
                s['mean'], s['sd'], s['min'], s['max'],
                sum(1 for x in lat if x > BID_THRESHOLD_MS)))
    return ('### A1 — F1: task dispatch and planning latency\n\n' + CAPTIONS['a1'] +
            '| Scenario | Platform | Dispatched/submitted | Mean (ms) | SD | Min | Max | Bids > %.0f ms |\n'
            '|---|---|---|---|---|---|---|---|\n%s\n' % (
                BID_THRESHOLD_MS, '\n'.join(rows)))


def a2(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            for route_id, route in ROUTES[key]:
                b = stats(baseline_task_values(data, platform, route_id, route, execution_s))
                n = stats(task_values(data, platform, 'scenarios', key, execution_s, route))
                delta = n['mean'] - b['mean']
                rows.append('| %s %s | %s | %s | %s | %+.1f | %+.1f%% |' % (
                    route_id, route.replace('->', '→'), platform, fmt(b), fmt(n),
                    delta, 100 * (n['mean'] / b['mean'] - 1)))
    return ('### A2 — F1: route-level execution time\n\n' + CAPTIONS['a2'] +
            '| Route | Platform | N=1 mean | SD | Min | Max | N=3 mean | SD | Min | Max | Δ (s) | Δ |\n'
            '|---|---|---|---|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a3(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            itin = trig = conf = res = replan = unrec = 0
            feas = []
            for rd in repeats(data, platform, 'scenarios', key):
                ts = load(rd, 'traffic_scheduling_metrics.json')
                if not ts:
                    continue
                itin += ts['2a_n_itineraries']
                trig += ts['2a_n_itineraries_triggering_negotiation']
                conf += ts['n_conflicts_detected']
                counts = ts.get('2a_conflict_outcome_counts') or {}
                res += counts.get('resolved_by_negotiation', 0)
                replan += counts.get('recovered_by_replan', 0)
                unrec += counts.get('unrecovered', 0)
                feas.append(ts['2a_schedule_feasibility_pct'])
            pct = '—' if not conf else '%.1f%% (%d/%d)' % (
                100 * (conf - unrec) / conf, conf - unrec, conf)
            rows.append('| %s | %s | %d | %d | %d | %d | %d | %d | %s |' % (
                label, platform, itin, trig, conf, res, replan, unrec, pct))
    return ('### A3 — F2: scheduling feasibility and event counts\n\n' + CAPTIONS['a3'] +
            '| Scenario | Platform | Itineraries | Triggering negotiation | Conflicts | '
            'Resolved | Recovered by replan | Unrecovered | Feasibility |\n'
            '|---|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a4(data):
    rows = []
    for platform in PLATFORMS:
        raw, total = [], 0
        for key, _ in SCENARIOS:
            for rd in repeats(data, platform, 'scenarios', key):
                ts = load(rd, 'traffic_scheduling_metrics.json')
                if not ts:
                    continue
                total += ts['2a_n_itineraries']
                raw += ts.get('raw_scheduling_times_s') or []
        s = stats(raw)
        rows.append('| %s | %d of %d | %.2f | %.2f | %.2f | %.2f | %d |' % (
            platform, s['n'], total, s['mean'] * 1000, s['sd'] * 1000,
            s['min'] * 1000, s['max'] * 1000,
            sum(1 for x in raw if x > SCHEDULE_BUDGET_S)))
    return ('### A4 — F2: scheduling latency\n\n' + CAPTIONS['a4'] +
            '| Platform | Itineraries with a matched update | Mean (ms) | SD | Min | Max | Above %.2f s |\n'
            '|---|---|---|---|---|---|---|\n%s\n' % (
                SCHEDULE_BUDGET_S, '\n'.join(rows)))


def a5(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            per = []
            for rd in repeats(data, platform, 'scenarios', key):
                ng = load(rd, 'negotiation_metrics.json')
                per.append(ng['3a_n_negotiation'] if ng else 0)
            rows.append('| %s | %s | %s | %d | %d |' % (
                label, platform, ' | '.join(str(x) for x in per),
                st.median(per), sum(per)))
    return ('### A5 — F3: negotiation notices by repeat\n\n' + CAPTIONS['a5'] +
            '| Scenario | Platform | R1 | R2 | R3 | R4 | R5 | Median | Total |\n'
            '|---|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a6(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            buckets = {'resolved': [], 'refused': [], 'search failed': []}
            for rd in repeats(data, platform, 'scenarios', key):
                ng = load(rd, 'negotiation_metrics.json')
                for item in (ng or {}).get('per_negotiation') or []:
                    name = ('resolved' if item['resolved']
                            else 'refused' if item['refused'] else 'search failed')
                    buckets[name].append(item['latency_s'])
            for name, values in buckets.items():
                if not values:
                    continue
                s = stats(values)
                rows.append('| %s | %s | %s | %d | %.3f | %.3f | %.3f | %.3f |' % (
                    label, platform, name, s['n'], s['mean'], s['sd'], s['min'], s['max']))
    return ('### A6 — F3: negotiation outcomes and latency\n\n' + CAPTIONS['a6'] +
            '| Scenario | Platform | Outcome | Count | Mean (s) | SD | Min | Max |\n'
            '|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a7(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            fs = json.load(open(os.path.join(data, platform, 'fleet_scaling_metrics.json')))[key]
            done = total = 0
            for rd in repeats(data, platform, 'scenarios', key):
                fc = load(rd, 'fleet_coordination_4a_metrics.json')
                if fc:
                    done += fc['4a_n_completed']
                    total += fc['4a_n_submitted']
            rows.append('| %s | %s | %d/%d | %.3f | %.3f | %.1f%% | %.1f%% | %.1f%% | %.1f%% |' % (
                label, platform, done, total, fs['th_1_tasks_per_min'],
                fs['th_n_tasks_per_min'], fs['scaling_pct_of_ideal'],
                fs['ceiling_pct_of_ideal'], fs['u_1_route_mean_pct'], fs['u_n_pct']))
    return ('### A7 — F4: throughput, scaling and utilisation\n\n' + CAPTIONS['a7'] +
            '| Scenario | Platform | 4a completed | Th(1) /min | Th(3) /min | Scaling, %% of ideal | '
            'Workload ceiling | U(1) routes alone | U(3) |\n'
            '|---|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a8(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            one = []
            for route_id, route in ROUTES[key]:
                one += fidelity_values(data, platform, 'baselines', route_id,
                                       '5a_duration_error_pct', route)
            three = fidelity_values(data, platform, 'scenarios', key, '5a_duration_error_pct')
            delta = fidelity_values(data, platform, 'scenarios', key,
                                    '5a_delta_actual_minus_predicted_s')
            rows.append('| %s | %s | %s | %s | %s |' % (
                label, platform, fmt(stats(one)), fmt(stats(three)), fmt(stats(delta))))
    return ('### A8 — F5: execution fidelity\n\n' + CAPTIONS['a8'] +
            '| Scenario | Platform | 5a N=1 mean %% | SD | Min | Max | 5a N=3 mean %% | SD | Min | Max | '
            '5b mean (s) | SD | Min | Max |\n'
            '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a9(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            for route_id, route in ROUTES[key]:
                b = stats(blocked_values(data, platform, 'baselines', route_id, route))
                n = stats(blocked_values(data, platform, 'scenarios', key, route))
                rows.append('| %s %s | %s | %s | %s | %+.1f |' % (
                    route_id, route.replace('->', '→'), platform, fmt(b), fmt(n),
                    n['mean'] - b['mean']))
    return ('### A9 — F6: route-level stationary time\n\n' + CAPTIONS['a9'] +
            '| Route | Platform | N=1 mean | SD | Min | Max | N=3 mean | SD | Min | Max | Excess (s) |\n'
            '|---|---|---|---|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a10(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            for threshold, nkey, mkey in ((EPISODE_FLOOR_S, 'n_episodes_total',
                                           '6b_mean_per_episode_s'),
                                          (1.0, 'n_episodes_total_min_1s',
                                           '6b_mean_per_episode_s_min_1s')):
                total, weighted, per_repeat = 0, 0.0, []
                for rd in repeats(data, platform, 'scenarios', key):
                    ch = load(rd, 'congestion_handling_metrics.json')
                    if not ch:
                        continue
                    count = ch.get(nkey) or 0
                    total += count
                    if ch.get(mkey) is not None and count:
                        weighted += ch[mkey] * count
                        per_repeat.append(ch[mkey])
                mean = '%.2f' % (weighted / total) if total else '—'
                spread = ('%.2f' % st.stdev(per_repeat)) if len(per_repeat) > 1 else '—'
                rows.append('| %s | %s | ≥ %.1f s | %d | %s | %s |' % (
                    label, platform, threshold, total, mean, spread))
    return ('### A10 — F6: waiting episode statistics\n\n' + CAPTIONS['a10'] +
            '| Scenario | Platform | Episode floor | Episodes | Mean duration (s) | SD across repeats |\n'
            '|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def a11(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            mins, below_floor = [], 0
            counts = {D_SAFE_M: [0, 0, 0], CONFLICT_SEP_M: [0, 0, 0]}
            names = {D_SAFE_M: 'd_safe_body_contact', CONFLICT_SEP_M: 'rmf_conflict_separation'}
            for rd in repeats(data, platform, 'scenarios', key):
                sm = load(rd, 'safety_tracker_metrics.json')
                if not sm:
                    continue
                spec = sm['as_specified']
                value = spec['7a_min_distance']['min_distance_m']
                mins.append(value)
                if value < FLOOR_M:
                    below_floor += 1
                for threshold, name in names.items():
                    block = spec['7b_below_threshold'][name]
                    counts[threshold][0] += block['events']
                    counts[threshold][1] += block['samples_below']
                    counts[threshold][2] += 1 if block['events'] else 0
            for n, threshold in enumerate((D_SAFE_M, CONFLICT_SEP_M)):
                events, samples, affected = counts[threshold]
                head = ('%s | %s | %.3f–%.3f | %.3f | %d' % (
                    label, platform, min(mins), max(mins), st.median(mins), below_floor)
                    if n == 0 else ' |  |  |  | ')
                rows.append('| %s | %.3f m | %d | %d | %d/5 |' % (
                    head, threshold, events, samples, affected))
    return ('### A11 — F7: pose-based separation and events\n\n' + CAPTIONS['a11'] +
            '| Scenario | Platform | 7a min, 5 repeats | Median | Repeats below %.3f m | '
            'Threshold | 7b events | Samples | Affected repeats |\n'
            '|---|---|---|---|---|---|---|---|---|\n%s\n' % (
                FLOOR_M, '\n'.join(rows)))


def a12(data):
    rows = []
    for key, label in SCENARIOS:
        for platform in PLATFORMS:
            mins, cover, instants, episodes, pair_repeats, contact = [], [], 0, 0, 0, 0
            for rd in repeats(data, platform, 'scenarios', key):
                sl = load(rd, 'safety_lidar_metrics.json')
                if not sl:
                    continue
                if sl.get('7c_contact_suspected'):
                    contact += 1
                per_repeat = []
                for pair in sl['per_pair']:
                    cover.append(pair['coverage_pct'])
                    instants += pair['instants_below_d_safe']
                    episodes += pair['episodes_below_d_safe']
                    pair_repeats += 1 if pair['instants_below_d_safe'] else 0
                    per_repeat.append(pair['min_separation_lower_m'])
                mins.append(min(per_repeat))
            c = stats(cover)
            rows.append('| %s | %s | %.3f–%.3f | %.3f | %.1f±%.1f%% | %d | %d | %d | %d |' % (
                label, platform, min(mins), max(mins), st.median(mins),
                c['mean'], c['sd'], instants, episodes, pair_repeats, contact))

    validation = []
    path = os.path.join(data, 'simulation', 'lidar_attribution_metrics.json')
    if os.path.exists(path):
        attribution = json.load(open(path))
        by_scenario = {}
        for run in attribution['runs']:
            name = next((k for k, _ in SCENARIOS if '/%s/' % k in run['run_dir']), None)
            agg = by_scenario.setdefault(name, [0, 0, 0, 0, 0])
            for repeat in run['repeats']:
                for pair in repeat['pairs']:
                    agg[0] += pair['close_instants']
                    agg[1] += pair['close_with_attributable_match']
                    agg[2] += pair['below_d_safe_tp']
                    agg[3] += pair['below_d_safe_fp']
                    agg[4] += pair['below_d_safe_fn']
        total = [0, 0, 0, 0, 0]
        for key, label in SCENARIOS:
            if key in by_scenario:
                close, matched, tp, fp, fn = by_scenario[key]
                for i, v in enumerate((close, matched, tp, fp, fn)):
                    total[i] += v
                validation.append('| %s | %d | %d | %d | %d | %d |' % (
                    label, close, matched, tp, fp, fn))
        share = '%d (%.1f%%)' % (total[1], 100 * total[1] / total[0]) if total[0] else '—'
        validation.append('| **All** | **%d** | **%s** | **%d** | **%d** | **%d** |' % (
            total[0], share, total[2], total[3], total[4]))

    text = ('### A12 — F7: laser separation and its validation\n\n' + CAPTIONS['a12'] +
            '| Scenario | Platform | 7c min, 5 repeats | Median | Coverage | '
            'Instants < %.3f m | Episodes < %.3f m | Pair-repeats | Contact suspected |\n'
            '|---|---|---|---|---|---|---|---|---|\n%s\n' % (
                D_SAFE_M, D_SAFE_M, '\n'.join(rows)))
    if validation:
        text += ('\n' + CAPTIONS['a12_validation'] +
                 '| Scenario | Close instants, true separation < %.3f m | With an '
                 'attributable match | TP | FP | FN |\n|---|---|---|---|---|---|\n%s\n' % (
                     CONFLICT_SEP_M, '\n'.join(validation)))
    return text


def a13(data):
    growth, scaling = [], []
    for platform in PLATFORMS:
        sc = json.load(open(os.path.join(data, platform, 'scalability_metrics.json')))
        for label, block in (('every task, as specified', sc['8a_as_specified']),
                             ("first task of each batch", sc['8a_first_task'])):
            growth.append('| %s | %s | %.2f | %.2f | %.2f | %.2f | %+.1f%% | %+.1f%% |' % (
                platform, label, block['n3']['mean_ms'], block['n3']['median_ms'],
                block['n1']['mean_ms'], block['n1']['median_ms'],
                block['g_mean_pct'], block['g_median_pct']))
        for key, name in SCENARIOS:
            per = sc['8a_per_scenario'][key]
            b = sc['8b'][key]
            scaling.append('| %s | %s | %.2f | %.2f | %+.1f%% | %.1f%% | %.1f%% |' % (
                name, platform, per['n3']['mean_ms'], per['n1']['mean_ms'],
                per['g_mean_pct'], b['scaling_pct_of_ideal'], b['ceiling_pct_of_ideal']))
    return ('### A13 — F8: planning growth and throughput scaling\n\n' + CAPTIONS['a13'] +
            '| Platform | Set | P(3) mean (ms) | Median | P(1) mean (ms) | Median | G mean | G median |\n'
            '|---|---|---|---|---|---|---|---|\n%s\n\n'
            '| Scenario | Platform | P(3) mean (ms) | P(1) mean | G mean | '
            'Throughput scaling | Workload ceiling |\n|---|---|---|---|---|---|---|\n%s\n' % (
                '\n'.join(growth), '\n'.join(scaling)))


def a14(data):
    rows = []
    for key, label in SCENARIOS:
        for route_id, route in ROUTES[key]:
            for platform in PLATFORMS:
                b = stats(baseline_task_values(data, platform, route_id, route, execution_s))
                n = stats(task_values(data, platform, 'scenarios', key, execution_s, route))
                blocked_one = stats(blocked_values(data, platform, 'baselines', route_id, route))
                blocked_three = stats(blocked_values(data, platform, 'scenarios', key, route))
                err_one = stats(fidelity_values(data, platform, 'baselines', route_id,
                                                '5a_duration_error_pct', route))
                err_three = stats(fidelity_values(data, platform, 'scenarios', key,
                                                  '5a_duration_error_pct', route))
                dist_one = stats(distance_values(data, platform, 'baselines', route_id, route))
                dist_three = stats(distance_values(data, platform, 'scenarios', key, route))
                conflicts = conflict_median(data, platform, key, route)
                rows.append('| %s | %s | %+.1f | %+.1f | %s | %+.1f | %s |' % (
                    route_id, platform, n['mean'] - b['mean'],
                    blocked_three['mean'] - blocked_one['mean'],
                    '%+.1f' % (dist_three['mean'] - dist_one['mean'])
                    if dist_one and dist_three else '—',
                    err_three['mean'] - err_one['mean'], conflicts))
    return ('### A14 — cross-feature route-level comparison\n\n' + CAPTIONS['a14'] +
            '| Route | Platform | Execution (s) | Stationary (s) | Distance (m) | '
            'Duration error (points) | Conflicts per repeat |\n'
            '|---|---|---|---|---|---|---|\n%s\n' % '\n'.join(rows))


def distance_values(data, platform, kind, cell, route):
    out = []
    for rd in repeats(data, platform, kind, cell):
        tp = load(rd, 'task_planning_metrics.json')
        if not tp:
            continue
        routes = {t['robot']: route_of(t) for t in tp['per_task']}
        for robot, entry in (tp.get('per_robot') or {}).items():
            if routes.get(robot) == route and entry.get('observed_path_m') is not None:
                out.append(entry['observed_path_m'])
    return out


def conflict_median(data, platform, scenario, route):
    """Median per repeat of the conflicts this route's robot took part in."""
    per = []
    for rd in repeats(data, platform, 'scenarios', scenario):
        ts, tp = load(rd, 'traffic_scheduling_metrics.json'), load(rd, 'task_planning_metrics.json')
        if not ts or not tp:
            continue
        robot = next((t['robot'] for t in tp['per_task'] if route_of(t) == route), None)
        per.append(sum(1 for c in (ts.get('per_conflict') or [])
                       if robot in c['participants']))
    return '%g' % st.median(per) if per else '—'


TABLES = [a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14]

HEADER = """# Appendix — Per-Feature Statistics

All statistics are generated from per-repeat metric outputs using
analyse/summary/write_appendix_tables.py. This appendix is the reference for
quantitative results in [RESULTS.md](RESULTS.md).

**Unit of analysis.** Each N=3 repeat comprises three dependent robot-tasks.
Robot-task statistics describe variation across tasks; scenario- and route-level
comparisons are based on five repeats per condition.

**Dataset.** 160 retained repeats across simulation and hardware: four scenarios ×
five repeats per platform at N=3, and 12 matched routes × five repeats per platform
at N=1.

**Thresholds.** Reference thresholds are derived from the fleet profile and
experimental geometry; they are not pass/fail criteria.

**Table numbering.** A1–A14 follow the appendix checklist.
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', default='data', help='the data directory')
    ap.add_argument('--output', default='APPENDIX.md', help='where to write the tables')
    args = ap.parse_args()

    parts = [HEADER]
    for table in TABLES:
        parts.append(table(args.data))
    text = re.sub(r'(?<=[|\s])-(?=\d)', '\u2212', '\n'.join(parts))
    with open(args.output, 'w') as fh:
        fh.write(text)
    print('wrote %s: %d tables' % (args.output, len(TABLES)))


if __name__ == '__main__':
    main()
