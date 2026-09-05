#!/usr/bin/env python3
"""
Concurrent-submission variant of run_benchmark.py, for scenarios with N >= 2
robots that must be dispatched at (near) the same time to produce genuine
traffic interaction (Crossing, Bottleneck, Shared lane, Head-on...).

Differs from run_benchmark.py only in submitting multiple ApiRequests back
to back (not waiting for one task to finish before submitting the next) and
tracking total_distance_m per robot instead of a single robot.
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

POS_EPS = 0.01  # m, movement threshold between consecutive /fleet_states samples


class ConcurrentBenchRunner(Node):
    def __init__(self, robot_names):
        super().__init__('bench_task_planning_concurrent_runner')
        self.robot_names = robot_names
        transient_qos = QoSProfile(
            history=History.KEEP_LAST, depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(ApiRequest, 'task_api_requests', transient_qos)
        self.create_subscription(ApiResponse, 'task_api_responses', self._on_response, transient_qos)
        self.create_subscription(DispatchStates, '/dispatch_states', self._on_dispatch, 10)
        self.create_subscription(FleetState, '/fleet_states', self._on_fleet_state, 10)
        self.pending_responses = {}   # request_id -> response dict (once received)
        self.dispatch_status = {}     # task_id -> DispatchState.status (covers auction/assignment
                                       # only -- rmf_task_ros2's Dispatcher.cpp never publishes true
                                       # execution completion over a recordable ROS2 topic, it goes
                                       # out through a non-ROS2 broadcast client instead)
        self.last_xy = {name: None for name in robot_names}
        self.total_distance = {name: 0.0 for name in robot_names}

        # Real completion signal, reconstructed from /fleet_states instead: a
        # task counts as "released" once the robot that was holding its
        # task_id moves on to something else (finished, reassigned, or idle).
        # Until a task is released, it is still in progress as far as RMF is
        # concerned -- distance travelled says nothing about that.
        self.tracked_task_ids = set()
        self.task_seen_robot = {}     # task_id -> last robot name seen holding it
        self.task_released = {}       # task_id -> True once released
        self._last_robot_task = {name: None for name in robot_names}

    def _on_response(self, msg: ApiResponse):
        if msg.request_id in self.pending_responses and self.pending_responses[msg.request_id] is None:
            self.pending_responses[msg.request_id] = json.loads(msg.json_msg)

    def _on_dispatch(self, msg: DispatchStates):
        for entry in list(msg.active) + list(msg.finished):
            self.dispatch_status[entry.task_id] = entry.status

    def _on_fleet_state(self, msg: FleetState):
        for r in msg.robots:
            if r.name not in self.robot_names:
                continue
            xy = (r.location.x, r.location.y)
            prev = self.last_xy[r.name]
            if prev is not None:
                d = math.hypot(xy[0] - prev[0], xy[1] - prev[1])
                if d > POS_EPS:
                    self.total_distance[r.name] += d
            self.last_xy[r.name] = xy

            prev_task_id = self._last_robot_task.get(r.name)
            if (prev_task_id and prev_task_id in self.tracked_task_ids
                    and r.task_id != prev_task_id):
                self.task_released[prev_task_id] = True
            if r.task_id in self.tracked_task_ids:
                self.task_seen_robot[r.task_id] = r.name
            self._last_robot_task[r.name] = r.task_id

    def start_tracking_tasks(self, task_ids):
        self.tracked_task_ids = set(tid for tid in task_ids if tid)
        self.task_seen_robot = {tid: None for tid in self.tracked_task_ids}
        self.task_released = {tid: False for tid in self.tracked_task_ids}
        self._last_robot_task = {name: None for name in self.robot_names}

    def all_tracked_tasks_released(self) -> bool:
        return bool(self.tracked_task_ids) and all(self.task_released.values())

    def submit_task(self, request_id, places, rounds, requester):
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
        self.pending_responses[request_id] = None
        self.pub.publish(msg)

    def wait_for_responses(self, request_ids, timeout_s=5.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if all(self.pending_responses.get(rid) is not None for rid in request_ids):
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        return {rid: self.pending_responses.get(rid) for rid in request_ids}

    def reset_distances(self):
        for name in self.robot_names:
            self.total_distance[name] = 0.0


def parse_route(spec):
    # spec: "place1,place2[,place3...]" -> patrol places list for one concurrent task
    return spec.split(',')


def parse_args():
    ap = argparse.ArgumentParser(description='Open-RMF Task Planning benchmark orchestrator (concurrent, N>=2)')
    ap.add_argument('--route', action='append', required=True, dest='routes',
                     help='Comma-separated patrol places for one task, e.g. "bottleneck_1,charger_1". '
                          'Pass once per concurrent task (e.g. twice for a Crossing N=2 run).')
    ap.add_argument('--rounds', type=int, default=5, help='Rounds per submitted task')
    ap.add_argument('--repeats', type=int, default=5, help='Number of independent repeats')
    ap.add_argument('--robots', nargs='+', required=True, help='Robot names to track in /fleet_states')
    ap.add_argument('--requester', default='benchmark_task_planning')
    ap.add_argument('--inter-repeat-pause', type=float, default=5.0)
    ap.add_argument('--fixed-wait', type=float, default=220.0,
                     help='Fixed seconds to wait per repeat before moving to the next one')
    ap.add_argument('--min-expected-distance-m', type=float, default=None,
                     help='If set, flag a robot as "short_distance" for a repeat when its '
                          'total_distance_m falls below this after the full wait')
    ap.add_argument('--output-dir', required=True, help='Directory to write repeats_summary.json into')
    return ap.parse_args()


def main():
    args = parse_args()
    routes = [parse_route(r) for r in args.routes]
    rclpy.init()
    node = ConcurrentBenchRunner(args.robots)
    results = []

    for i in range(1, args.repeats + 1):
        print(f'=== Repeat {i}/{args.repeats}: submitting {len(routes)} concurrent patrol tasks ===', flush=True)
        request_ids = []
        submit_wall_times = {}
        for j, places in enumerate(routes):
            rid = f'bench_patrol_{i}_{j}_' + str(uuid.uuid4())
            submit_wall_times[rid] = time.time()
            node.submit_task(rid, places, args.rounds, args.requester)
            request_ids.append(rid)

        responses = node.wait_for_responses(request_ids, timeout_s=5.0)

        entry = {'repeat': i, 'tasks': []}
        task_ids = []
        for rid, places in zip(request_ids, routes):
            resp = responses.get(rid)
            task_entry = {
                'request_id': rid, 'places': places,
                'submit_wall_time': submit_wall_times[rid],
                'response': resp, 'task_id': None,
            }
            if resp is not None and resp.get('success', False):
                task_id = resp.get('state', {}).get('booking', {}).get('id')
                task_entry['task_id'] = task_id
                task_ids.append(task_id)
                print(f'  route {places}: task_id={task_id}', flush=True)
            else:
                print(f'  route {places}: dispatch request failed/no response', flush=True)
            entry['tasks'].append(task_entry)

        node.reset_distances()
        node.start_tracking_tasks(task_ids)
        deadline = time.time() + args.fixed_wait
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.3)
            if node.all_tracked_tasks_released():
                break
        entry['fixed_wait_exceeded'] = time.time() >= deadline

        entry['completion_wall_time'] = time.time()
        entry['dispatch_status'] = {tid: node.dispatch_status.get(tid) for tid in task_ids}
        entry['total_distance_m'] = {name: round(d, 3) for name, d in node.total_distance.items()}
        entry['task_released'] = dict(node.task_released)

        # A robot counts as timed out if RMF never released it from one of
        # this repeat's tasks before fixed_wait elapsed -- that is a real
        # failure signal (the task was still in progress when the clock ran
        # out), not an inference from how far the robot happened to travel.
        # short_distance is now reserved for tasks RMF DID release where the
        # robot still moved less than expected -- a genuinely short route,
        # not a truncated one.
        robot_released = {}
        for tid in task_ids:
            robot = node.task_seen_robot.get(tid)
            if robot:
                robot_released[robot] = node.task_released.get(tid, False)

        outcomes = {}
        for name, dist in node.total_distance.items():
            if name in robot_released and not robot_released[name]:
                outcomes[name] = 'timeout'
            elif args.min_expected_distance_m is not None and dist < args.min_expected_distance_m:
                outcomes[name] = 'short_distance'
            else:
                outcomes[name] = 'completed'
        entry['outcome'] = outcomes

        print(f'Repeat {i}: outcome={outcomes} total_distance_m={entry["total_distance_m"]}', flush=True)
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
