#!/usr/bin/env python3
"""Remove gravity and sensor bias from WrenchStamped force readings."""

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ft_gravity_calib import quaternion_matrix


class FTGravityCompensate(Node):
    def __init__(self):
        super().__init__("ft_gravity_compensate")
        self.declare_parameter("input_topic", "/force_torque")
        self.declare_parameter("output_topic", "/ft_compensated")
        self.declare_parameter("sensor_frame", "LARM_JOINT5_Link")
        self.declare_parameter("base_frame", "WAIST")
        self.declare_parameter("mass", 0.0)
        self.declare_parameter("bias", [0.0, 0.0, 0.0])
        self.ft_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.mass = float(self.get_parameter("mass").value)
        self.bias = np.asarray(self.get_parameter("bias").value, dtype=float)
        self.gravity = np.array([0.0, 0.0, -9.81])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(WrenchStamped, self.out_topic, 10)
        self.create_subscription(WrenchStamped, self.ft_topic, self.cb, 10)
        self.get_logger().info(
            f"Compensating {self.ft_topic} -> {self.out_topic}; mass={self.mass}, bias={self.bias.tolist()}")

    def cb(self, msg):
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        try:
            transform = self.tf_buffer.lookup_transform(
                self.sensor_frame, self.base_frame, Time())
        except TransformException:
            return
        gravity_sensor = quaternion_matrix(transform.transform.rotation).dot(self.gravity)
        corrected = force - self.mass * gravity_sensor - self.bias
        out = WrenchStamped()
        out.header = msg.header
        out.wrench = msg.wrench
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = corrected.tolist()
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FTGravityCompensate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
