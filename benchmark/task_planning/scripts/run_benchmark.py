#!/usr/bin/env python3
"""
"""
import argparse
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import ApiRequest, ApiResponse, DispatchStates

POS_EPS = 0.01       # m, movement threshold between consecutive /fleet_states samples


class BenchRunner(Node):
    def __init__(self, robot_name):
        super().__init__('bench_task_planning_runner')
        self.robot_name = robot_name
        transient_qos = QoSProfile(
            history=History.KEEP_LAST, depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(ApiRequest, 'task_api_requests', transient_qos)
        self.create_subscription(ApiResponse, 'task_api_responses', self._on_response, transient_qos)
        self.create_subscription(DispatchStates, '/dispatch_states', self._on_dispatch, 10)
        self.create_subscription(FleetState, '/fleet_states', self._on_fleet_state, 10)
        self.pending_response = None
        self.pending_request_id = None
        self.dispatch_status = {}   # task_id -> DispatchState.status
        self.last_xy = None
        self.last_move_time = time.time()
        self.total_distance = 0.0   # cumulative distance since last reset_distance() call

    def _on_response(self, msg: ApiResponse):
        if msg.request_id == self.pending_request_id:
            self.pending_response = json.loads(msg.json_msg)

    def _on_dispatch(self, msg: DispatchStates):
        for entry in list(msg.active) + list(msg.finished):
            self.dispatch_status[entry.task_id] = entry.status

    def _on_fleet_state(self, msg: FleetState):
        for r in msg.robots:
            if r.name != self.robot_name:
                continue
            xy = (r.location.x, r.location.y)
            now = time.time()
            if self.last_xy is not None:
                d = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
                if d > POS_EPS:
                    self.last_move_time = now
                    self.total_distance += d
            self.last_xy = xy

    def submit_task(self, repeat_idx, places, rounds, requester):
        request_id = f'bench_patrol_{repeat_idx}_' + str(uuid.uuid4())
        msg = ApiRequest()
        msg.request_id = request_id
        now = self.get_clock().now().to_msg()
        start_ms = now.sec * 1000 + round(now.nanosec / 1e6)
        request = {
            'unix_millis_request_time': start_ms,
            'unix_millis_earliest_start_time': start_ms,
            'requester': requester,
            'category': 'patrol',
            'description': {'places': places, 'rounds': rounds},
        }
        payload = {'type': 'dispatch_task_request', 'request': request}
        msg.json_msg = json.dumps(payload)

        self.pending_response = None
        self.pending_request_id = request_id
        submit_wall_time = time.time()
        self.pub.publish(msg)

        deadline = time.time() + 5.0
        while self.pending_response is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

        return request_id, submit_wall_time, self.pending_response


def parse_args():
    ap = argparse.ArgumentParser(description='Open-RMF Task Planning benchmark orchestrator')
    ap.add_argument('--places', nargs='+', required=True, help='Waypoint sequence for the patrol task')
    ap.add_argument('--rounds', type=int, default=5, help='Rounds per submitted task')
    ap.add_argument('--repeats', type=int, default=5, help='Number of independent repeats')
    ap.add_argument('--robot', default='tb3_robot1', help='Robot name to track in /fleet_states')
    ap.add_argument('--requester', default='benchmark_task_planning')
    ap.add_argument('--inter-repeat-pause', type=float, default=5.0)
    ap.add_argument('--fixed-wait', type=float, default=220.0,
                     help='Fixed seconds to wait per task before moving to the next repeat')
    ap.add_argument('--min-expected-distance-m', type=float, default=None,
                     help='If set, flag a repeat as "short_distance" (likely a stalled/failed '
                          'task) when total_distance_m falls below this after the full wait')
    ap.add_argument('--output-dir', required=True, help='Directory to write repeats_summary.json into')
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = BenchRunner(args.robot)
    results = []

    for i in range(1, args.repeats + 1):
        print(f'=== Repeat {i}/{args.repeats}: submitting patrol task ===', flush=True)
        request_id, submit_wall_time, response = node.submit_task(
            i, args.places, args.rounds, args.requester)

        entry = {
            'repeat': i, 'request_id': request_id,
            'submit_wall_time': submit_wall_time,
            'response': response, 'task_id': None,
            'dispatch_status': None, 'completion_wall_time': None, 'outcome': None,
            'total_distance_m': None,
        }

        if response is None or not response.get('success', False):
            entry['outcome'] = 'no_response_or_rejected'
            print(f'Repeat {i}: dispatch request failed/no response', flush=True)
            results.append(entry)
            time.sleep(args.inter_repeat_pause)
            continue

        task_id = response.get('state', {}).get('booking', {}).get('id')
        entry['task_id'] = task_id
        print(f'Repeat {i}: task_id={task_id}', flush=True)

        node.last_move_time = time.time()
        node.total_distance = 0.0
        deadline = time.time() + args.fixed_wait
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.3)
            elapsed = time.time() - submit_wall_time
            if task_id not in node.dispatch_status and elapsed > 10:
                entry['outcome'] = 'dispatch_never_confirmed'
                break

        entry['completion_wall_time'] = time.time()
        entry['dispatch_status'] = node.dispatch_status.get(task_id)
        entry['total_distance_m'] = round(node.total_distance, 3)
        if entry['outcome'] is None:
            if (args.min_expected_distance_m is not None
                    and node.total_distance < args.min_expected_distance_m):
                entry['outcome'] = 'short_distance'
            else:
                entry['outcome'] = 'completed'

        print(f'Repeat {i}: outcome={entry["outcome"]} dispatch_status={entry["dispatch_status"]} '
              f'total_distance_m={entry["total_distance_m"]}', flush=True)
        results.append(entry)
        time.sleep(args.inter_repeat_pause)

    out_path = f'{args.output_dir}/repeats_summary.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)

    print('=== DONE ===', flush=True)
    print(json.dumps(results, indent=2, default=str))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
