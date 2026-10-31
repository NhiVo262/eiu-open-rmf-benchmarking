#!/usr/bin/env python3

import argparse
import sys

from free_fleet.ros2_types import ActionMsgs_CancelGoal_Response

from tb3_fleet.tb3_fleet_adapter import zenoh_config
from tb3_fleet.tb3_robot_adapter import (
    CANCEL_RETURN_CODES,
    make_nav2_cancel_all_goals_request,
    namespacify,
)

import yaml
import zenoh


def cancel_all_goals(session, robot, timeout_s):
    replies = session.get(
        namespacify('navigate_to_pose/_action/cancel_goal', robot),
        payload=make_nav2_cancel_all_goals_request().serialize(),
        timeout=timeout_s,
    )
    for reply in replies:
        try:
            rep = ActionMsgs_CancelGoal_Response.deserialize(reply.ok.payload.to_bytes())
        except Exception:
            return 'UNDECODABLE', 0
        return CANCEL_RETURN_CODES.get(rep.return_code, str(rep.return_code)), \
            len(rep.goals_canceling)
    return 'NO_REPLY', 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--fleet-config', required=True,
                    help='the fleet config the adapter was launched with; its zenoh '
                         'section gives the session this reaches the robots through')
    ap.add_argument('--robots', nargs='+', required=True)
    ap.add_argument('--timeout-sec', type=float, default=5.0)
    args = ap.parse_args()

    with open(args.fleet_config) as f:
        config = yaml.safe_load(f)

    session = zenoh.open(zenoh_config(config.get('zenoh')))
    unconfirmed = []
    try:
        for robot in args.robots:
            code, canceling = cancel_all_goals(session, robot, args.timeout_sec)
            print('%s: %s, %d goal(s) canceling' % (robot, code, canceling))
            # ERROR_NONE with nothing canceling is the normal case for an idle robot.
            if code not in ('ERROR_NONE', 'ERROR_GOAL_TERMINATED'):
                unconfirmed.append(robot)
    finally:
        session.close()

    if unconfirmed:
        print('Navigation was not confirmed stopped on: %s' % ', '.join(unconfirmed))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
