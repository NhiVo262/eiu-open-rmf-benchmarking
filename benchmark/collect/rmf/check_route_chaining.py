#!/usr/bin/env python3
import argparse
import math
import sys

SCENARIOS = {
    'Bottleneck': [('wp_2', 'wp_5'),
                   ('wp_3', 'wp_4'),
                   ('wp_4', 'wp_3')],
    'Crossing': [('wp_2', 'wp_6'),
                 ('wp_6', 'wp_9'),
                 ('wp_7', 'wp_8')],
    'Head-on': [('wp_2', 'wp_5'),
                ('wp_5', 'wp_2'),
                ('wp_3', 'wp_7')],
    'Shared Lane': [('wp_2', 'wp_7'),
                    ('wp_4', 'wp_5'),
                    ('wp_6', 'wp_3')],
}


def load_graph(path):
    import yaml
    doc = yaml.safe_load(open(path))
    level = list(doc['levels'].values())[0]
    coords, names = {}, {}
    for index, vertex in enumerate(level['vertices']):
        props = vertex[2] if len(vertex) > 2 else {}
        name = props.get('name', '')
        if name:
            names[index] = name
            coords[name] = (vertex[0], vertex[1])
    adjacency = {name: {} for name in coords}
    for lane in level['lanes']:
        a, b = names.get(lane[0]), names.get(lane[1])
        if a is None or b is None:
            continue
        d = math.dist(coords[a], coords[b])
        adjacency[a][b] = min(adjacency[a].get(b, d), d)
        adjacency[b][a] = min(adjacency[b].get(a, d), d)
    return coords, adjacency


def all_distances(adjacency):
    import heapq
    out = {}
    for source in adjacency:
        dist = {source: 0.0}
        queue = [(0.0, source)]
        while queue:
            d, here = heapq.heappop(queue)
            if d > dist.get(here, float('inf')):
                continue
            for nxt, step in adjacency[here].items():
                nd = d + step
                if nd < dist.get(nxt, float('inf')):
                    dist[nxt] = nd
                    heapq.heappush(queue, (nd, nxt))
        out[source] = dist
    return out


def chain_margins(routes, spawns, dist):
    rows = []
    for j, (start_j, _) in enumerate(routes):
        chain = None
        for i, (_, end_i) in enumerate(routes):
            if i == j:
                continue
            d = dist.get(end_i, {}).get(start_j)
            if d is not None and (chain is None or d < chain[0]):
                chain = (d, i)
        fresh = None
        for spawn in spawns:
            d = dist.get(spawn, {}).get(start_j)
            if d is not None and (fresh is None or d < fresh[0]):
                fresh = (d, spawn)
        rows.append({
            'route': j + 1,
            'start': start_j,
            'chain_from': ('R%d' % (chain[1] + 1)) if chain else None,
            'chain_m': chain[0] if chain else None,
            'fresh_from': fresh[1] if fresh else None,
            'fresh_m': fresh[0] if fresh else None,
            'margin_m': (fresh[0] - chain[0]) if (chain and fresh) else None,
        })
    return rows


def path_cover(routes):
    n = len(routes)
    succ, pred = {}, {}

    def would_cycle(a, b):
        cur = b
        while cur in succ:
            cur = succ[cur]
            if cur == a:
                return True
        return False

    for i, (_, end_i) in enumerate(routes):
        if i in succ:
            continue
        for j, (start_j, _) in enumerate(routes):
            if i != j and end_i == start_j and j not in pred \
                    and not would_cycle(i, j):
                succ[i], pred[j] = j, i
                break

    chains = []
    for head in (i for i in range(n) if i not in pred):
        chain, cur = [head], head
        while cur in succ:
            cur = succ[cur]
            chain.append(cur)
        chains.append(chain)

    covered = sum(len(c) for c in chains)
    if covered != n:
        raise AssertionError(
            'cover is incomplete: %d of %d routes' % (covered, n))
    return chains


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--route', action='append', dest='routes',
                    help='One route as "start,end". Repeat per concurrent task. '
                         'Omit to check the four built-in scenarios.')
    ap.add_argument('--robots', type=int, default=3,
                    help='Robots in the fleet (default 3)')
    ap.add_argument('--nav-graph',
                    help='nav_graphs/0.yaml. With it, chaining is judged on '
                         'shortest-path distance instead of endpoint equality.')
    ap.add_argument('--spawn', action='append', dest='spawns',
                    help='Waypoint a robot starts parked at. Repeat per robot. '
                         'Defaults to wp_1, wp_8, wp_9, the three parking spots that '
                         'start no route, which is where the robots stand when the '
                         'fleet is free to choose who serves what.')
    args = ap.parse_args()

    if args.routes:
        routes = []
        for spec in args.routes:
            places = [p.strip() for p in spec.split(',') if p.strip()]
            if len(places) < 2:
                ap.error('route %r needs at least a start and an end' % spec)
            routes.append((places[0], places[-1]))
        cases = {'(given)': routes}
    else:
        cases = SCENARIOS

    header = ['%-12s' % 'scenario', '%-28s' % 'chains one robot can serve',
              '%-11s' % 'robots used']
    print(' '.join(header))
    print('-' * (len(' '.join(header))))

    degenerate = []
    for name, routes in cases.items():
        chains = path_cover(routes)
        text = ' | '.join('->'.join('R%d' % (i + 1) for i in c) for c in chains)
        row = ['%-12s' % name, '%-28s' % text, '%-11s' % len(chains)]
        print(' '.join(row))
        if len(chains) < min(args.robots, len(routes)):
            degenerate.append((name, len(chains)))

    if args.nav_graph:
        coords, adjacency = load_graph(args.nav_graph)
        dist = all_distances(adjacency)
        spawns = args.spawns or ['wp_1', 'wp_8', 'wp_9']
        missing = [w for w in spawns if w not in coords]
        if missing:
            ap.error('spawn waypoint(s) not in the graph: %s' % ', '.join(missing))
        for name, routes in cases.items():
            unknown = [w for r in routes for w in r if w not in coords]
            if unknown:
                print()
                print('%s: waypoint(s) not in the graph: %s'
                      % (name, ', '.join(sorted(set(unknown)))))
                continue
            print()
            print('%s -- transit to each route start (metres along the graph):' % name)
            print('  %-6s %-15s %-18s %-16s %s' % (
                'route', 'start', 'from a route end', 'from a free robot', 'verdict'))
            worst = None
            for row in chain_margins(routes, spawns, dist):
                verdict = 'ok'
                if row['margin_m'] is not None and row['margin_m'] > 0:
                    verdict = 'CHAINED (saves %.1f m)' % row['margin_m']
                    degenerate.append((name, len(path_cover(routes))))
                print('  R%-5d %-15s %-18s %-16s %s' % (
                    row['route'], row['start'],
                    '%s: %.1f m' % (row['chain_from'], row['chain_m'])
                    if row['chain_m'] is not None else '-',
                    '%s: %.1f m' % (row['fresh_from'], row['fresh_m'])
                    if row['fresh_m'] is not None else '-',
                    verdict))
                if row['margin_m'] is not None:
                    worst = row['margin_m'] if worst is None else max(worst, row['margin_m'])
            if worst is not None and worst <= 0:
                print('  no route is cheaper to chain than to give a free robot.')

    if degenerate:
        print()
        print('Degenerate for a %d-robot fleet -- RMF will leave robots parked:'
              % args.robots)
        for name, k in dict.fromkeys(degenerate):
            print('  %-12s uses %d robot(s); change an endpoint so no route ends '
                  'where another starts' % (name, k))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
