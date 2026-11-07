#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class GripperJointStatePublisher(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("gripper_joint_state_publisher", **kwargs)
        self.declare_parameter("left_position", 0.05)
        self.declare_parameter("right_position", 0.05)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("source_topic", "")
        self.declare_parameter("output_topic", "joint_states")

        rate = max(self._float_parameter("publish_rate"), 0.1)
        self._publisher = self.create_publisher(
            JointState, self.get_parameter("output_topic").value, 10)
        source_topic = self.get_parameter("source_topic").value
        if source_topic:
            if self.resolve_topic_name(source_topic) == self._publisher.topic_name:
                raise ValueError("source_topic must differ from the output joint_states topic")
            self._subscription = self.create_subscription(
                JointState, source_topic, self._publish_merged, qos_profile_sensor_data)
        else:
            # Preserve standalone gripper publication for existing callers.
            self._timer = self.create_timer(1.0 / rate, self._publish)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _publish_merged(self, source: JointState) -> None:
        # Publish only on fresh robot samples; never refresh stale robot timestamps.
        if not source.name or len(source.position) != len(source.name):
            return
        if any(values and len(values) != len(source.name)
               for values in (source.velocity, source.effort)):
            return
        msg = deepcopy(source)
        for name, parameter in (("lhand_joint", "left_position"),
                                ("rhand_joint", "right_position")):
            if name in msg.name:
                continue  # Measured gripper positions take precedence over defaults.
            msg.name.append(name)
            msg.position.append(self._float_parameter(parameter))
            if msg.velocity:
                msg.velocity.append(0.0)
            if msg.effort:
                msg.effort.append(0.0)
        self._publisher.publish(msg)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
