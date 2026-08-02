#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("gripper_joint_state_publisher")
        self.declare_parameter("left_position", 0.05)
        self.declare_parameter("right_position", 0.05)
        self.declare_parameter("publish_rate", 10.0)

        rate = max(self._float_parameter("publish_rate"), 0.1)
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["lhand_joint", "rhand_joint"]
        msg.position = [
            self._float_parameter("left_position"),
            self._float_parameter("right_position"),
        ]
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = GripperJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
