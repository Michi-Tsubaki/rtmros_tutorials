#!/usr/bin/env python3
"""Move via fullbody FollowJointTrajectory, then request static FT captures.

Offsets are relative to the initial measured fullbody configuration. Joint
limits and feedback are checked; this is NOT a collision-aware path planner.
Run in a cleared workspace without other nodes sending robot commands.
"""

from collections import deque
from pathlib import Path
import time
import signal
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


JOINTS = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1'] + [
    f'{arm}_JOINT{i}' for arm in ('LARM', 'RARM') for i in range(6)]


def build_targets(initial, arm, plan, urdf):
    allowed = {'CHEST_JOINT0'} | {f'{arm.upper()}_JOINT{i}' for i in range(6)}
    names = [name.replace('{ARM}', arm.upper()) for name in plan['offset_joints']]
    if len(names) != len(set(names)) or not set(names) <= allowed:
        raise ValueError('Offsets may change only the waist and selected arm')
    offsets = np.asarray(plan['offsets'], dtype=float)
    if offsets.ndim != 2 or offsets.shape[1] != len(names) or len(offsets) < 6:
        raise ValueError('Provide at least six pose offsets, matching offset_joints')
    if not np.isfinite(offsets).all() or np.max(np.abs(offsets)) > 0.6:
        raise ValueError('Offsets must be finite and within +/-0.6 rad of the start')
    initial = np.asarray(initial, dtype=float)
    if initial.shape != (15,) or not np.isfinite(initial).all():
        raise ValueError('Initial fullbody configuration must contain 15 finite positions')
    targets = np.tile(initial, (len(offsets), 1))
    targets[:, [JOINTS.index(name) for name in names]] += offsets
    limits = {joint.attrib['name']: joint.find('limit') for joint in ET.parse(urdf).findall('joint')}
    velocities = []
    for index, name in enumerate(JOINTS):
        limit = limits[name]
        low, high = float(limit.attrib['lower']), float(limit.attrib['upper'])
        values = np.r_[initial[index], targets[:, index]]
        if np.any(values < low) or np.any(values > high):
            raise ValueError(f'Pose plan exceeds URDF limits for {name}; reposition before retrying')
        velocities.append(float(limit.attrib['velocity']))
    return targets, np.asarray(velocities)


class CalibrationMotion(Node):
    def __init__(self):
        super().__init__('ft_calibration_motion')
        defaults = {
            'arm': 'rarm', 'action_name': '/fullbody_controller/follow_joint_trajectory_action',
            'joint_states_topic': '/joint_states', 'identification_node': '/rarm_ft_identification',
            'poses_file': '', 'urdf_file': '', 'move_duration': 5.0,
            'settle_time': 2.0, 'settle_timeout': 15.0, 'position_tolerance': 0.02,
            'stationary_tolerance': 0.003, 'freshness_timeout': 0.5,
            'startup_timeout': 30.0, 'capture_timeout': 20.0, 'max_joint_speed': 0.15,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['arm'] not in ('rarm', 'larm'):
            raise ValueError('arm must be rarm or larm')
        for key in ('move_duration', 'settle_time', 'settle_timeout', 'position_tolerance',
                    'stationary_tolerance', 'freshness_timeout', 'startup_timeout',
                    'capture_timeout', 'max_joint_speed'):
            if not np.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.states = {}
        self.goal_handle = None
        self.pending_goal = None
        self.create_subscription(JointState, self.p['joint_states_topic'], self.on_joints,
                                 qos_profile_sensor_data)
        self.action = ActionClient(self, FollowJointTrajectory, self.p['action_name'])
        prefix = self.p['identification_node'].rstrip('/')
        self.service_clients = {key: self.create_client(Trigger, f'{prefix}/{key}')
                         for key in ('capture', 'status', 'solve')}

    def on_joints(self, msg):
        age = (self.get_clock().now()-rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds*1e-9
        if age < -0.1 or age > self.p['freshness_timeout']:
            return
        now = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            if name in JOINTS and np.isfinite(value):
                self.states[name] = (value, now)

    def positions(self):
        now = time.monotonic()
        if any(name not in self.states or now-self.states[name][1] > self.p['freshness_timeout']
               for name in JOINTS):
            raise RuntimeError('Missing or stale fullbody joint feedback')
        return np.array([self.states[name][0] for name in JOINTS])

    def wait_future(self, future, timeout, monitor=False):
        deadline = time.monotonic()+timeout
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if monitor:
                self.positions()
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for action/service response')
        if not rclpy.ok() or future.cancelled():
            raise RuntimeError('ROS shutdown or future cancelled')
        return future.result()

    def call(self, name, timeout=5.0):
        return self.wait_future(self.service_clients[name].call_async(Trigger.Request()), timeout)

    def settle(self, target):
        history = deque()
        deadline = time.monotonic()+self.p['settle_timeout']
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now, positions = time.monotonic(), self.positions()
            if np.max(np.abs(positions-target)) > self.p['position_tolerance']:
                history.clear()
                continue
            history.append((now, positions))
            # Retain the endpoint preceding the requested window.
            while len(history) > 2 and history[1][0] < now-self.p['settle_time']:
                history.popleft()
            if now-history[0][0] >= self.p['settle_time']:
                if np.max(np.ptp([entry[1] for entry in history], axis=0)) <= self.p['stationary_tolerance']:
                    return
        raise RuntimeError('Robot did not reach a stationary target within settle_timeout')

    def move(self, target, velocities):
        current = self.positions()
        speed = np.minimum(velocities, self.p['max_joint_speed'])
        if np.any(speed <= 0):
            raise ValueError('URDF velocity limits must be positive')
        duration = max(self.p['move_duration'], float(np.max(2.0*np.abs(target-current)/speed)))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        point.velocities = [0.0]*len(JOINTS)
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]
        # Retain a late-accepted goal so a request timeout also cancels it.
        self.pending_goal = self.action.send_goal_async(goal)
        self.goal_handle = self.wait_future(self.pending_goal, 5.0, monitor=True)
        self.pending_goal = None
        if not self.goal_handle.accepted:
            self.goal_handle = None
            raise RuntimeError('Fullbody controller rejected the trajectory')
        result = self.wait_future(self.goal_handle.get_result_async(), duration+10.0, monitor=True)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError(f'Fullbody trajectory failed: {result.result.error_string}')
        self.goal_handle = None
        self.settle(target)

    def run(self):
        deadline = time.monotonic()+self.p['startup_timeout']
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.action.server_is_ready() or not all(c.service_is_ready() for c in self.service_clients.values()):
                continue
            try:
                initial = self.positions()
            except RuntimeError:
                continue
            status = self.call('status')
            if status.success and status.message == 'idle':
                break
        else:
            raise RuntimeError('Startup requires fullbody action, joint states, FT data and timestamped TF')
        plan = yaml.safe_load(Path(self.p['poses_file']).read_text())
        targets, velocities = build_targets(initial, self.p['arm'], plan, self.p['urdf_file'])
        self.settle(initial)
        self.get_logger().info(f'Executing {len(targets)} poses via {self.p["action_name"]}; '
                               'head and opposite arm joint positions remain at their initial values')
        for index, target in enumerate(targets, 1):
            self.get_logger().info(f'Pose {index}/{len(targets)}: move, settle, capture')
            self.move(target, velocities)
            response = self.call('capture')
            if not response.success:
                raise RuntimeError(response.message)
            capture_deadline = time.monotonic()+self.p['capture_timeout']
            while time.monotonic() < capture_deadline:
                status = self.call('status')
                if np.max(np.abs(self.positions()-target)) > self.p['position_tolerance']:
                    raise RuntimeError('Robot left its target during capture')
                if status.success and status.message == 'done':
                    break
                if status.message != 'collecting':
                    raise RuntimeError(status.message)
                until = time.monotonic()+0.1
                while time.monotonic() < until:
                    rclpy.spin_once(self, timeout_sec=0.05)
            else:
                raise RuntimeError('Capture timed out')
        self.get_logger().info('Returning to starting configuration before fitting')
        self.move(initial, velocities)
        response = self.call('solve', timeout=10.0)
        if not response.success:
            raise RuntimeError(response.message)
        self.get_logger().info(response.message)

    def cancel(self):
        if self.pending_goal is not None and rclpy.ok():
            self.goal_handle = self.wait_future(self.pending_goal, 5.0)
            self.pending_goal = None
        if self.goal_handle is not None and self.goal_handle.accepted and rclpy.ok():
            response = self.wait_future(self.goal_handle.cancel_goal_async(), 5.0)
            if not response.goals_canceling:
                self.get_logger().error('Controller did not confirm cancellation; use the robot stop control')


def main(args=None):
    # Keep ROS alive long enough to request action cancellation on Ctrl-C/SIGTERM.
    def interrupted(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    result = 0
    try:
        node = CalibrationMotion()
        node.run()
    except (Exception, KeyboardInterrupt) as exc:
        result = 1
        if node is None:
            print(f'Calibration initialization failed: {exc}', file=sys.stderr)
        if node is not None:
            node.get_logger().error(f'Calibration stopped: {exc}. No further poses will be sent.')
            try:
                node.cancel()
            except Exception as cancel_error:
                node.get_logger().error(f'Cancellation failed: {cancel_error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
