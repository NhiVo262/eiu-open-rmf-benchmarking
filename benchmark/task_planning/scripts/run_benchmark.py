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
    def __init__(self, robot_name, release_grace_period_s: float = 5.0):
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

        # Real completion signal (see run_benchmark_concurrent.py for the full
        # rationale): a task counts as released once no robot is holding its
        # task_id anymore, after having been held, sustained for at least
        # release_grace_period_s to avoid mistaking an in-flight replan
        # re-auction for genuine completion. Distance alone can't tell a
        # truncated task from a finished one, and Clear Path is the baseline
        # every other scenario's percentage is computed against.
        self.tracked_task_id = None
        self.task_ever_held = False
        self.task_released = False
        self.release_grace_period_s = release_grace_period_s
        self._unheld_since = None

    def _on_response(self, msg: ApiResponse):
        if msg.request_id == self.pending_request_id:
            self.pending_response = json.loads(msg.json_msg)

    def _on_dispatch(self, msg: DispatchStates):
        for entry in list(msg.active) + list(msg.finished):
            self.dispatch_status[entry.task_id] = entry.status

    def start_tracking_task(self, task_id):
        self.tracked_task_id = task_id
        self.task_ever_held = False
        self.task_released = False
        self._unheld_since = None

    def _on_fleet_state(self, msg: FleetState):
        held_now = False
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
            held_now = r.task_id == self.tracked_task_id and self.tracked_task_id is not None

        if held_now:
            self.task_ever_held = True
            self._unheld_since = None
        elif self.task_ever_held and not self.task_released:
            now = time.monotonic()
            if self._unheld_since is None:
                self._unheld_since = now
            elif now - self._unheld_since >= self.release_grace_period_s:
                self.task_released = True

    def submit_task(self, repeat_idx, places, rounds, requester):
        request_id = f'bench_patrol_{repeat_idx}_' + str(uuid.uuid4())
        msg = ApiRequest()
        msg.request_id = request_id
        now = self.get_clock().now().to_msg()
        start_ms = now.sec * 1000 + round(now.nanosec / 1e6)
        request = {
            'unix_millis_request_time': start_ms,
            # 0, not "now": this node runs on wall time (never declares
            # use_sim_time), but the fleet adapter's own node clock is
            # sim-time-aware (per the Fleet Adapter fix). RMF compares this
            # field against ITS OWN sim-time "now" to decide whether a
            # queued task's deployment time has arrived -- a wall-clock
            # epoch value (~1.7e12 ms) can never be <= a sim clock that
            # starts near 0 when Gazebo launches, so the task would sit
            # queued forever without 0 here.
            'unix_millis_earliest_start_time': 0,
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
    ap.add_argument('--release-grace-period', type=float, default=5.0,
                     help='Seconds the task must be held by no robot before it is trusted as genuinely '
                          'finished (protects against mistaking an in-flight replan re-auction for '
                          'completion).')
    ap.add_argument('--min-expected-distance-m', type=float, default=None,
                     help='If set, flag a repeat as "short_distance" (likely a stalled/failed '
                          'task) when total_distance_m falls below this after the full wait')
    ap.add_argument('--output-dir', required=True, help='Directory to write repeats_summary.json into')
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = BenchRunner(args.robot, release_grace_period_s=args.release_grace_period)
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
            'total_distance_m': None, 'task_released': None,
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
        node.start_tracking_task(task_id)
        deadline = time.time() + args.fixed_wait
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.3)
            elapsed = time.time() - submit_wall_time
            if task_id not in node.dispatch_status and elapsed > 10:
                entry['outcome'] = 'dispatch_never_confirmed'
                break
            if node.task_released:
                break
        entry['fixed_wait_exceeded'] = time.time() >= deadline

        entry['completion_wall_time'] = time.time()
        entry['dispatch_status'] = node.dispatch_status.get(task_id)
        entry['total_distance_m'] = round(node.total_distance, 3)
        entry['task_released'] = node.task_released
        # A task RMF never released before fixed_wait elapsed is a real
        # failure signal (still in progress when the clock ran out), not an
        # inference from distance travelled. short_distance is reserved for
        # tasks RMF DID release where the robot still moved less than
        # expected -- a genuinely short route, not a truncated one.
        if entry['outcome'] is None:
            if node.task_ever_held and not node.task_released:
                entry['outcome'] = 'timeout'
            elif (args.min_expected_distance_m is not None
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
