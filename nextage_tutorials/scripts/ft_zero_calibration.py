#!/usr/bin/env python3
"""Collect a static six-axis tare, save it, and publish zeroed wrenches.

Keep the sensor unloaded and stationary throughout calibration. A steady
external load cannot be distinguished from bias. No robot commands are sent.
"""

from copy import deepcopy
from pathlib import Path
import os
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
import yaml

from nextage_tutorials.ft_zero import estimate_bias


class FTZeroCalibration(Node):
    def __init__(self):
        super().__init__('ft_zero_calibration')
        defaults = {
            'input_topic': '/right_ft/force_torque',
            'output_topic': '/right_ft/force_torque_zeroed',
            'sensor_frame': 'RARM_FTSENSOR_SENSOR_Link',
            'calibration_file': '',
            'calibrate': True,
            'sample_count': 500,
            'timeout': 30.0,
            'force_std_limit': 0.2,
            'torque_std_limit': 0.02,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        s = self.settings
        if s['sample_count'] < 2 or not np.isfinite(s['timeout']) or s['timeout'] <= 0:
            raise ValueError('sample_count must be >= 2 and timeout finite and positive')
        if not s['calibration_file'] or not s['sensor_frame']:
            raise ValueError('calibration_file and sensor_frame are required')
        self.path = Path(s['calibration_file']).expanduser()
        self.publisher = self.create_publisher(WrenchStamped, s['output_topic'], 10)
        if self.resolve_topic_name(s['input_topic']) == self.resolve_topic_name(s['output_topic']):
            raise ValueError('Input and output topics must differ')
        self.bias = None
        self.samples = []
        self.collecting = False
        if s['calibrate']:
            self.start()
        else:
            data = yaml.safe_load(self.path.read_text())
            if data['sensor_frame'] != s['sensor_frame']:
                raise ValueError('Calibration sensor_frame does not match')
            self.bias = np.asarray(data['bias'], dtype=float)
            if self.bias.shape != (6,) or not np.isfinite(self.bias).all():
                raise ValueError('Calibration requires six finite bias values')
            self.get_logger().info(f'Loaded six-axis tare from {self.path}')
        self.create_subscription(WrenchStamped, s['input_topic'], self.receive,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '~/calibrate', self.recalibrate)
        self.create_timer(0.25, self.check_timeout)

    def start(self):
        self.samples = []
        self.bias = None
        self.collecting = True
        self.deadline = time.monotonic() + self.settings['timeout']
        self.get_logger().info('Collecting tare: keep the arm stationary and free of contact. '
                               'Corrected output is paused until calibration succeeds.')

    def recalibrate(self, request, response):
        if self.collecting:
            response.success = False
            response.message = 'Calibration already in progress'
        else:
            self.start()
            response.success = True
            response.message = 'Collection started; completion is reported in the node log'
        return response

    def check_timeout(self):
        if self.collecting and time.monotonic() >= self.deadline:
            self.collecting = False
            self.samples = []
            self.get_logger().error('Calibration timed out; no corrected output. '
                                    'Check topic/frame and call ~/calibrate to retry.')

    def save(self, bias, std):
        data = {'sensor_frame': self.settings['sensor_frame'],
                'bias': bias, 'standard_deviation': std,
                'sample_count': len(self.samples),
                'units': ['N', 'N', 'N', 'Nm', 'Nm', 'Nm']}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                                             delete=False) as stream:
                temporary = stream.name
                yaml.safe_dump(data, stream)
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def receive(self, msg):
        self.check_timeout()
        if msg.header.frame_id != self.settings['sensor_frame']:
            self.get_logger().warning('Ignoring wrench with unexpected frame_id',
                                      throttle_duration_sec=5.0)
            return
        w = msg.wrench
        values = np.array([w.force.x, w.force.y, w.force.z,
                           w.torque.x, w.torque.y, w.torque.z])
        if not np.isfinite(values).all():
            return
        if self.collecting:
            self.samples.append(values)
            if len(self.samples) < self.settings['sample_count']:
                return
            self.collecting = False
            try:
                bias, std = estimate_bias(self.samples, self.settings['force_std_limit'],
                                          self.settings['torque_std_limit'])
                self.save(bias, std)
            except (ValueError, OSError) as exc:
                self.get_logger().error(f'Calibration failed: {exc}; no corrected output. '
                                        'Call ~/calibrate to retry.')
                self.samples = []
                return
            self.bias = np.asarray(bias)
            self.samples = []
            self.get_logger().info(f'Saved tare to {self.path}: {bias}')
        if self.bias is None:
            return
        out = deepcopy(msg)
        corrected = (values - self.bias).tolist()
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = corrected[:3]
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = corrected[3:]
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FTZeroCalibration()
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
