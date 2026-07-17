#!/usr/bin/env python3
"""Estimate a force sensor's mass and bias from samples at multiple poses."""

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def quaternion_matrix(q):
    """Return the 3x3 rotation matrix for a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = x * x + y * y + z * z + w * w
    if norm < np.finfo(float).eps:
        return np.identity(3)
    scale = 2.0 / norm
    return np.array([
        [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
        [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
        [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
    ])


class FTGravityCalib(Node):
    def __init__(self):
        super().__init__("ft_gravity_calib")
        self.declare_parameter("ft_topic", "/left_ft/left_ft/force_torque")
        self.declare_parameter("sensor_frame", "LARM_JOINT5_Link")
        self.declare_parameter("base_frame", "WAIST")
        self.declare_parameter("yaml_path", "ft_gravity.yaml")
        self.ft_topic = self.get_parameter("ft_topic").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.output_yaml = Path(self.get_parameter("yaml_path").value).expanduser()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ft_data = []
        self.gravity = np.array([0.0, 0.0, -9.81])
        self.create_subscription(WrenchStamped, self.ft_topic, self.cb_ft, 10)
        self.get_logger().info(f"Collecting FT data; output: {self.output_yaml}")

    def cb_ft(self, msg):
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        try:
            transform = self.tf_buffer.lookup_transform(
                self.sensor_frame, self.base_frame, Time())
        except TransformException:
            return
        gravity_sensor = quaternion_matrix(transform.transform.rotation).dot(self.gravity)
        self.ft_data.append((force, gravity_sensor))
        if len(self.ft_data) % 50 == 0:
            self.get_logger().info(f"Collected samples: {len(self.ft_data)}")

    def solve(self):
        if not self.ft_data:
            self.get_logger().warning("No data collected; YAML was not written")
            return False
        matrix, measurements = [], []
        for force, gravity in self.ft_data:
            matrix.extend([[gravity[0], 1, 0, 0], [gravity[1], 0, 1, 0], [gravity[2], 0, 0, 1]])
            measurements.extend(force)
        solution, _, _, _ = np.linalg.lstsq(
            np.asarray(matrix, dtype=float), np.asarray(measurements, dtype=float), rcond=None)
        calibration = {"mass": float(solution[0]), "bias": solution[1:4].tolist()}
        self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
        with self.output_yaml.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(calibration, stream, default_flow_style=False)
        self.get_logger().info(f"Saved calibration to {self.output_yaml}: {calibration}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FTGravityCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.solve()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
