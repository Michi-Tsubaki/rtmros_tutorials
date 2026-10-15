#!/usr/bin/env python3
"""Identify six-axis bias, mass and CoG from requested static pose windows.

No robot commands. Services: ~/capture, ~/status, ~/solve. After a successful
fit, publish gravity/bias compensated wrenches. TF is evaluated at sensor time.
"""

from copy import deepcopy
from collections import deque
from pathlib import Path
import os
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from nextage_tutorials.ft_payload import gravity_from_quaternion, fit_payload, compensate
from nextage_tutorials.ft_zero import estimate_bias


class PayloadIdentification(Node):
    def __init__(self):
        super().__init__('ft_payload_identification')
        defaults = {
            'arm': 'rarm', 'input_topic': '/right_ft/force_torque',
            'output_topic': '/right_ft/force_torque_compensated',
            'sensor_frame': 'RARM_FTSENSOR_SENSOR_Link', 'base_frame': 'WAIST',
            'joint_states_topic': '/joint_states', 'output_file': '',
            'sample_count': 200, 'capture_timeout': 15.0,
            'force_std_limit': 0.2, 'torque_std_limit': 0.02,
            'max_force_rms': 0.3, 'max_torque_rms': 0.03,
            'max_condition': 200.0, 'wrench_sign': 1.0,
            'max_joint_drift': 0.005, 'freshness_timeout': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        p = self.p
        if p['arm'] not in ('rarm', 'larm') or p['wrench_sign'] not in (-1.0, 1.0):
            raise ValueError('Invalid arm or wrench_sign')
        if not p['output_file'] or p['sample_count'] < 2:
            raise ValueError('output_file is required; sample_count must be >= 2')
        for name in ('capture_timeout', 'force_std_limit', 'torque_std_limit',
                     'max_force_rms', 'max_torque_rms', 'max_condition',
                     'max_joint_drift', 'freshness_timeout'):
            if not np.isfinite(p[name]) or p[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.resolve_topic_name(p['input_topic']) == self.resolve_topic_name(p['output_topic']):
            raise ValueError('Input and output topics must differ')
        self.joint_names = ['CHEST_JOINT0'] + [f'{p["arm"].upper()}_JOINT{i}' for i in range(6)]
        self.joints = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.poses = []
        self.samples = []
        self.pending_wrenches = deque(maxlen=2000)
        self.collecting = False
        self.status = 'idle'
        self.calibration = None
        self.last_valid_wrench = 0.0
        self.publisher = self.create_publisher(WrenchStamped, p['output_topic'], 10)
        self.create_subscription(JointState, p['joint_states_topic'], self.on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, p['input_topic'], self.on_wrench,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '~/capture', self.capture)
        self.create_service(Trigger, '~/status', self.get_status)
        self.create_service(Trigger, '~/solve', self.solve)
        self.create_timer(0.1, self.timeout)
        self.create_timer(0.01, self.process_pending)

    def on_joints(self, msg):
        now = time.monotonic()
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names and np.isfinite(position):
                self.joints[name] = (position, now)

    def positions(self):
        now = time.monotonic()
        if any(name not in self.joints or now-self.joints[name][1] > self.p['freshness_timeout']
               for name in self.joint_names):
            raise ValueError('Missing or stale joint states')
        return np.array([self.joints[name][0] for name in self.joint_names])

    def capture(self, request, response):
        try:
            if self.collecting:
                raise ValueError('Capture already in progress')
            self.anchor = self.positions()
            if time.monotonic()-self.last_valid_wrench > self.p['freshness_timeout']:
                raise ValueError('No fresh, correctly framed wrench with timestamped TF')
            self.samples = []
            self.calibration = None
            self.collecting = True
            self.deadline = time.monotonic()+self.p['capture_timeout']
            self.capture_stamp = self.get_clock().now().nanoseconds
            self.status = 'collecting'
            response.success, response.message = True, 'collecting'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def get_status(self, request, response):
        response.success = self.status == 'done' or (
            self.status == 'idle' and time.monotonic()-self.last_valid_wrench < self.p['freshness_timeout'])
        response.message = self.status
        return response

    def fail(self, message):
        self.collecting = False
        self.status = 'error: ' + message
        self.samples = []
        self.get_logger().error(self.status)

    def timeout(self):
        if self.collecting and time.monotonic() > self.deadline:
            self.fail('Capture timed out')

    def on_wrench(self, msg):
        self.pending_wrenches.append(msg)

    def process_pending(self):
        # Joint TF often arrives after the corresponding FT sample. Wait for
        # that transform instead of pairing the wrench with the latest pose.
        while self.pending_wrenches:
            if self.process_wrench(self.pending_wrenches[0]) is False:
                break
            self.pending_wrenches.popleft()

    def process_wrench(self, msg):
        if msg.header.frame_id != self.p['sensor_frame']:
            return
        # Reject unstamped/stale samples rather than silently using latest TF.
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds * 1e-9
        if stamp.nanoseconds == 0 or age < -0.1 or age > self.p['freshness_timeout']:
            return
        w = msg.wrench
        values = self.p['wrench_sign'] * np.array([
            w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
        if not np.isfinite(values).all():
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.p['sensor_frame'], self.p['base_frame'], stamp)
            q = transform.transform.rotation
            gravity = gravity_from_quaternion([q.x, q.y, q.z, q.w])
        except TransformException:
            return False
        except ValueError:
            return
        self.last_valid_wrench = time.monotonic()
        if self.collecting and stamp.nanoseconds >= self.capture_stamp:
            try:
                if np.max(np.abs(self.positions()-self.anchor)) > self.p['max_joint_drift']:
                    raise ValueError('Robot moved during capture')
                self.samples.append((gravity, values))
                if len(self.samples) < self.p['sample_count']:
                    return
                g = np.array([sample[0] for sample in self.samples])
                readings = np.array([sample[1] for sample in self.samples])
                mean, std = estimate_bias(readings, self.p['force_std_limit'], self.p['torque_std_limit'])
                if np.max(g.std(axis=0)) > 0.02:
                    raise ValueError('Sensor orientation moved during capture')
                self.poses.append({'gravity': g.mean(axis=0).tolist(), 'wrench': mean,
                                   'std': std, 'joints': self.anchor.tolist()})
                self.collecting = False
                self.samples = []
                self.status = 'done'
                self.get_logger().info(f'Captured static pose {len(self.poses)}')
            except ValueError as exc:
                self.fail(str(exc))
        if self.calibration is not None:
            corrected = compensate(values, gravity, self.calibration).tolist()
            out = deepcopy(msg)
            out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = corrected[:3]
            out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = corrected[3:]
            self.publisher.publish(out)

    @staticmethod
    def write_yaml(path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
                temp = stream.name
                yaml.safe_dump(data, stream, sort_keys=False)
            os.replace(temp, path)
        finally:
            if temp and os.path.exists(temp):
                os.unlink(temp)

    def solve(self, request, response):
        try:
            if self.collecting:
                raise ValueError('Capture is still in progress')
            result = fit_payload([pose['gravity'] for pose in self.poses],
                                 [pose['wrench'] for pose in self.poses],
                                 self.p['max_condition'], self.p['max_force_rms'],
                                 self.p['max_torque_rms'])
            result.update(sensor_frame=self.p['sensor_frame'], base_frame=self.p['base_frame'],
                          wrench_sign=self.p['wrench_sign'], poses=self.poses,
                          joint_names=self.joint_names,
                          units={'mass': 'kg', 'center_of_mass': 'm',
                                 'force_bias': 'N', 'torque_bias': 'Nm'})
            path = Path(self.p['output_file']).expanduser()
            # This override is for bias-corrected, still gravity-bearing hardware
            # readings ONLY. Fully compensated readings must use CoG.force=0.
            override = {f'/**/{self.p["arm"]}_admittance_controller': {'ros__parameters': {
                'gravity_compensation': {'frame': {'id': self.p['sensor_frame']},
                    'CoG': {'pos': result['center_of_mass'], 'force': result['mass'] * 9.80665}}}}}
            self.write_yaml(path.with_suffix('.admittance.yaml'), override)
            self.write_yaml(path, result)
            self.calibration = result
            response.success, response.message = True, f'Saved {path}; mass={result["mass"]:.6f} kg'
            self.get_logger().info(response.message)
        except (ValueError, OSError) as exc:
            response.success, response.message = False, str(exc)
            self.get_logger().error(f'Identification failed: {exc}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadIdentification()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
