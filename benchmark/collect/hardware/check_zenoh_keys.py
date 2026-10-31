#!/usr/bin/env python3

import argparse
import math
import sys
import time
from collections import Counter, defaultdict

import zenoh
from dataclasses import dataclass

from free_fleet.ros2_types import Header, TFMessage
import pycdr2
from pycdr2 import IdlStruct


def count_samples(session, keys, duration_s):
    counts = Counter()
    frames = defaultdict(Counter)

    def make_handler(key):
        def handler(sample):
            counts[key] += 1
            if not key.endswith('tf') and not key.endswith('tf_static'):
                return
            try:
                msg = TFMessage.deserialize(sample.payload.to_bytes())
            except Exception:
                frames[key]['<undeserializable>'] += 1
                return
            for t in msg.transforms:
                frames[key][f'{t.header.frame_id} -> {t.child_frame_id}'] += 1
        return handler

    subs = [session.declare_subscriber(k, make_handler(k)) for k in keys]
    time.sleep(duration_s)
    for s in subs:
        s.undeclare()
    return counts, frames


@dataclass
class BatteryState(IdlStruct):
    header: Header
    voltage: pycdr2.types.float32
    temperature: pycdr2.types.float32
    current: pycdr2.types.float32
    charge: pycdr2.types.float32
    capacity: pycdr2.types.float32
    design_capacity: pycdr2.types.float32
    percentage: pycdr2.types.float32


def read_battery(session, robot, timeout_s=5.0):
    latest = []

    def handler(sample):
        try:
            latest.append(BatteryState.deserialize(sample.payload.to_bytes()).percentage)
        except Exception:
            pass

    sub = session.declare_subscriber(f'{robot}/battery_state', handler)
    deadline = time.monotonic() + timeout_s
    while not latest and time.monotonic() < deadline:
        time.sleep(0.2)
    sub.undeclare()
    return latest[-1] if latest else None


def compose_chain(latest, chain):
    x = y = yaw = 0.0
    for parent, child in zip(chain, chain[1:]):
        t = latest.get((parent, child))
        if t is None:
            return None
        tx, ty, qz, qw = t
        c, s_ = math.cos(yaw), math.sin(yaw)
        x += c * tx - s_ * ty
        y += s_ * tx + c * ty
        yaw += 2.0 * math.atan2(qz, qw)
    return x, y, math.atan2(math.sin(yaw), math.cos(yaw))


def track_pose(session, robot, duration_s, map_frame='map', robot_frame='base_footprint'):
    latest = {}

    def handler(sample):
        try:
            msg = TFMessage.deserialize(sample.payload.to_bytes())
        except Exception:
            return
        for t in msg.transforms:
            tr, ro = t.transform.translation, t.transform.rotation
            latest[(t.header.frame_id, t.child_frame_id)] = (tr.x, tr.y, ro.z, ro.w)

    sub = session.declare_subscriber(f'{robot}/tf', handler)
    time.sleep(duration_s)
    sub.undeclare()
    return compose_chain(latest, [map_frame, 'odom', robot_frame])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='tb3_robot1',
                        help='Robot whose zenoh keys are checked')
    parser.add_argument('--router', default='tcp/127.0.0.1:7447',
                        help='Endpoint of the zenoh router to connect through')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Seconds to listen before deciding')
    parser.add_argument('--expect', default=None, metavar='X,Y',
                        help='Map coordinates the robot should be at. The composed pose is '
                             'checked against them, which catches a robot placed or seeded '
                             'somewhere other than the start waypoint.')
    parser.add_argument('--expect-tol', type=float, default=0.60,
                        help='Metres the composed pose may differ from --expect')
    parser.add_argument('--min-battery', type=float, default=None, metavar='PCT',
                        help='Charge below which the robot is reported as not ready.')
    args = parser.parse_args()

    sub_keys = [f'{args.robot}/tf', f'{args.robot}/battery_state',
                f'{args.robot}/tf_static']
    # tf and battery_state decide the result; tf_static is reported only.
    optional = {f'{args.robot}/tf_static'}

    config = zenoh.Config()
    config.insert_json5('mode', '"client"')
    config.insert_json5('connect/endpoints', f'["{args.router}"]')

    with zenoh.open(config) as session:
        print(f'connected to {args.router}; listening {args.duration:.0f}s')
        counts, frames = count_samples(session, sub_keys, args.duration)
        pose = track_pose(session, args.robot, min(args.duration, 3.0))
        battery = read_battery(session, args.robot)

    missing = []
    for key in sub_keys:
        n = counts[key]
        rate = n / args.duration
        print(f'{key:44s} {n:6d} msgs  {rate:6.1f} Hz')
        for pair, c in frames[key].most_common(8):
            print(f'{"":46s}{pair}  x{c}')
        if n == 0 and key not in optional:
            missing.append(key)

    expected = None
    misplaced = False
    if args.expect:
        try:
            ex, ey = (float(v) for v in args.expect.split(','))
            expected = (ex, ey)
        except ValueError:
            print(f'--expect wants X,Y, got [{args.expect}]')
            return 2

    if pose is None:
        print('\nmap -> odom is absent. Start Nav2 on the robot and set its initial pose.')
        missing.append('map -> odom')
    else:
        print(f'\nmap -> base_footprint composed from the tf key: '
              f'x={pose[0]:.3f} y={pose[1]:.3f} yaw={pose[2]:+.3f} rad '
              f'({math.degrees(pose[2]):+.1f} deg)')
        if expected is not None:
            offset = math.hypot(pose[0] - expected[0], pose[1] - expected[1])
            print(f'expected ({expected[0]:.3f}, {expected[1]:.3f}), off by {offset:.3f} m')
            if offset > args.expect_tol:
                print('Place the robot on the start waypoint and set its initial pose '
                      'again.')
                misplaced = True

    if missing:
        print('\nno data on:')
        for key in missing:
            print(f'  {key}')
        if counts[f'{args.robot}/tf'] == 0:
            print('Nothing is arriving. On the robot, start bringup, Nav2 and '
                  'zenoh-bridge-ros2dds, in that order, after the router is up.')
        elif any(k.startswith('map ->') for k in missing):
            print('AMCL is not publishing. Restart Nav2 on the robot and seed it there.')
        else:
            print('The bridge is not routing these. Set its namespace to the robot name '
                  'and add the topic to its allow rules.')
        return 1

    if battery is None:
        print('\nbattery: no reading')
    else:
        print(f'\nbattery: {battery:.1f}%')
    low_battery = (args.min_battery is not None and battery is not None
                   and battery < args.min_battery)

    if low_battery:
        print(f'Charge {battery:.1f}% is below the {args.min_battery:.0f}% floor.')
        return 1

    if misplaced:
        print('\nThe robot is localised, but not at the start waypoint.')
        return 1

    print('\nAll adapter keys carry data.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
