#!/usr/bin/env python3

import argparse
import sys

from tb3_fleet.tb3_fleet_adapter import zenoh_config

import yaml
import zenoh

LIVELINESS_PREFIX = '@/*/@ros2_lv'
ACTION_SERVER = 'AS'
PUBLISHER = 'MP'


def announced_entities(session, timeout_s):
    """Set of (kind, ros_name) the connected bridges declare, e.g. ('AS', 'r1/nav')."""
    entities = set()
    for reply in session.liveliness().get('%s/**' % LIVELINESS_PREFIX, timeout=timeout_s):
        try:
            key = str(reply.ok.key_expr)
        except Exception:
            continue
        head, _, tail = key.partition('/@ros2_lv/')
        if not tail:
            continue
        fields = tail.split('/')
        if len(fields) < 2:
            continue
        entities.add((fields[0], fields[1].replace('§', '/')))
    return entities


def classify(entities, robot, action):
    """'ok', 'robot' or 'link' for one robot; see the module docstring."""
    if (ACTION_SERVER, '%s/%s' % (robot, action)) in entities:
        return 'ok'
    if any(kind == PUBLISHER and name.startswith('%s/' % robot)
           for kind, name in entities):
        return 'robot'
    return 'link'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--fleet-config', required=True,
                    help='the fleet config the adapter is launched with; its zenoh '
                         'section gives the session this queries through')
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--action', default='navigate_to_pose')
    ap.add_argument('--timeout-sec', type=float, default=5.0)
    args = ap.parse_args()

    with open(args.fleet_config) as f:
        config = yaml.safe_load(f)

    session = zenoh.open(zenoh_config(config.get('zenoh')))
    try:
        entities = announced_entities(session, args.timeout_sec)
    finally:
        session.close()

    verdicts = [(r, classify(entities, r, args.action)) for r in args.robots]
    for robot, verdict in verdicts:
        if verdict == 'ok':
            continue
        # stdout is the caller's list; one line per robot, "<robot> <verdict>".
        print('%s %s' % (robot, verdict))
        if verdict == 'robot':
            sys.stderr.write(
                '%s announces its publishers but no [%s] action server: its bridge and '
                'the link are up and Nav2 is not serving the action. Restart Nav2 on that '
                'robot; nothing on the workstation can supply it.\n' % (robot, args.action))
        else:
            sys.stderr.write(
                '%s announces no ROS entity at all: its bridge or the link to it is '
                'down.\n' % robot)
    return 1 if any(v != 'ok' for _, v in verdicts) else 0


if __name__ == '__main__':
    sys.exit(main())
