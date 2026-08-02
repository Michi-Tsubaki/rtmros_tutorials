#!/usr/bin/env python3
"""Track RARM_JOINT5_Link with NEXTAGE head_controller only."""

from __future__ import annotations

import math

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class HeadTrackRarmJoint5(Node):
    def __init__(self) -> None:
        super().__init__("head_track_rarm_joint5")

        self.declare_parameter("reference_frame", "CHEST_JOINT0_Link")
        self.declare_parameter("head_frame", "HEAD_JOINT0_Link")
        self.declare_parameter("target_frame", "RARM_JOINT5_Link")
        self.declare_parameter(
            "action_name", "/head_controller/follow_joint_trajectory_action"
        )
        self.declare_parameter("update_rate", 5.0)
        self.declare_parameter("trajectory_duration", 0.35)
        self.declare_parameter("initial_trajectory_duration", 2.0)
        self.declare_parameter("start_delay", 0.05)
        self.declare_parameter("server_timeout", 5.0)
        self.declare_parameter("minimum_distance", 0.03)
        self.declare_parameter("minimum_angle_delta", 0.005)
        self.declare_parameter("resend_interval", 1.0)
        self.declare_parameter("log_period", 1.0)
        self.declare_parameter("yaw_offset", 0.0)
        self.declare_parameter("pitch_offset", 0.0)
        self.declare_parameter("yaw_limits", [-1.22172999, 1.22172999])
        self.declare_parameter("pitch_limits", [-0.349065989, 1.22172999])

        self.reference_frame = self.get_parameter("reference_frame").value
        self.head_frame = self.get_parameter("head_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.action_name = self.get_parameter("action_name").value
        self.update_rate = max(float(self.get_parameter("update_rate").value), 0.1)
        self.trajectory_duration = max(
            float(self.get_parameter("trajectory_duration").value), 0.01
        )
        self.initial_trajectory_duration = max(
            float(self.get_parameter("initial_trajectory_duration").value),
            self.trajectory_duration,
        )
        self.start_delay = max(float(self.get_parameter("start_delay").value), 0.0)
        self.server_timeout = float(self.get_parameter("server_timeout").value)
        self.minimum_distance = max(
            float(self.get_parameter("minimum_distance").value), 0.0
        )
        self.minimum_angle_delta = max(
            float(self.get_parameter("minimum_angle_delta").value), 0.0
        )
        self.resend_interval = max(
            float(self.get_parameter("resend_interval").value), 0.0
        )
        self.log_period = max(float(self.get_parameter("log_period").value), 0.1)
        self.yaw_offset = float(self.get_parameter("yaw_offset").value)
        self.pitch_offset = float(self.get_parameter("pitch_offset").value)
        self.yaw_limits = self._limits_parameter("yaw_limits")
        self.pitch_limits = self._limits_parameter("pitch_limits")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(
            self, FollowJointTrajectory, self.action_name
        )

        self._send_goal_future = None
        self._result_future = None
        self._goal_in_progress = False
        self._last_command: tuple[float, float] | None = None
        self._last_send_time = self.get_clock().now() - Duration(seconds=3600.0)
        self._last_log_time = self.get_clock().now() - Duration(seconds=3600.0)

        self.timer = self.create_timer(1.0 / self.update_rate, self.update_head)

    def _limits_parameter(self, name: str) -> tuple[float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 2:
            raise ValueError(f"{name} must contain [lower, upper]")
        lower = float(values[0])
        upper = float(values[1])
        if lower > upper:
            raise ValueError(f"{name} lower limit is greater than upper limit")
        return lower, upper

    def wait_for_head_controller(self) -> bool:
        self.get_logger().info(f"Waiting for head controller action: {self.action_name}")
        ok = self.action_client.wait_for_server(timeout_sec=self.server_timeout)
        if not ok:
            self.get_logger().error(
                f"Head controller action was unavailable after "
                f"{self.server_timeout:.1f}s: {self.action_name}"
            )
            return False
        self.get_logger().info("Head controller action is available")
        return True

    def update_head(self) -> None:
        try:
            target = self._lookup_translation(self.target_frame)
            head = self._lookup_translation(self.head_frame)
        except TransformException as exc:
            self._log_periodic(
                "warning",
                f"Waiting for TF in {self.reference_frame}: "
                f"{self.head_frame}, {self.target_frame}: {exc}",
            )
            return

        vector = (
            target[0] - head[0],
            target[1] - head[1],
            target[2] - head[2],
        )
        distance = math.sqrt(
            vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]
        )
        if distance < self.minimum_distance:
            self._log_periodic(
                "warning",
                f"Target is too close to head origin: distance={distance:.3f}m",
            )
            return

        yaw, pitch = self._vector_to_head_angles(vector)
        if not self._should_send(yaw, pitch):
            return

        is_initial_command = self._last_command is None
        duration = (
            self.initial_trajectory_duration
            if is_initial_command
            else self.trajectory_duration
        )

        self._send_head_goal(yaw, pitch, duration)
        self._last_command = (yaw, pitch)
        self._last_send_time = self.get_clock().now()
        self._log_periodic(
            "info",
            f"HEAD_JOINT0={math.degrees(yaw):.1f}deg "
            f"HEAD_JOINT1={math.degrees(pitch):.1f}deg "
            f"duration={duration:.2f}s "
            f"target={self.target_frame}",
        )

    def _lookup_translation(self, child_frame: str) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.reference_frame, child_frame, Time()
        )
        translation = transform.transform.translation
        return translation.x, translation.y, translation.z

    def _vector_to_head_angles(
        self, vector: tuple[float, float, float]
    ) -> tuple[float, float]:
        horizontal = math.hypot(vector[0], vector[1])
        if horizontal < 1.0e-9 and self._last_command is not None:
            yaw = self._last_command[0]
        else:
            yaw = math.atan2(vector[1], vector[0]) + self.yaw_offset

        # HEAD_JOINT1 rotates around +Y. Positive values pitch the head downward.
        pitch = math.atan2(-vector[2], horizontal) + self.pitch_offset

        yaw = clamp(yaw, self.yaw_limits[0], self.yaw_limits[1])
        pitch = clamp(pitch, self.pitch_limits[0], self.pitch_limits[1])
        return yaw, pitch

    def _should_send(self, yaw: float, pitch: float) -> bool:
        if self._goal_in_progress:
            return False

        if self._send_goal_future is not None and not self._send_goal_future.done():
            return False

        if self._result_future is not None and not self._result_future.done():
            return False

        if self._last_command is None:
            return True

        delta = max(
            abs(yaw - self._last_command[0]),
            abs(pitch - self._last_command[1]),
        )
        if delta >= self.minimum_angle_delta:
            return True

        if self.resend_interval <= 0.0:
            return False

        elapsed = self.get_clock().now() - self._last_send_time
        return elapsed >= Duration(seconds=self.resend_interval)

    def _send_head_goal(self, yaw: float, pitch: float, duration: float) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = (
            self.get_clock().now() + Duration(seconds=self.start_delay)
        ).to_msg()
        goal.trajectory.joint_names = ["HEAD_JOINT0", "HEAD_JOINT1"]

        point = JointTrajectoryPoint()
        point.positions = [yaw, pitch]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        self._goal_in_progress = True
        self._send_goal_future = self.action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_in_progress = False
            self._send_goal_future = None
            self._log_periodic("error", f"Failed to send head goal: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self._goal_in_progress = False
            self._send_goal_future = None
            self._log_periodic("warning", "Head controller rejected the goal")
            return

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._goal_in_progress = False
            self._send_goal_future = None
            self._result_future = None
            self._log_periodic("warning", f"Head goal result failed: {exc}")
            return

        self._goal_in_progress = False
        self._send_goal_future = None
        self._result_future = None
        result = response.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._log_periodic(
                "warning",
                f"Head controller result error_code={result.error_code} "
                f"error_string={result.error_string!r}",
            )

    def _log_periodic(self, level: str, message: str) -> None:
        now = self.get_clock().now()
        if now - self._last_log_time < Duration(seconds=self.log_period):
            return
        self._last_log_time = now
        logger = self.get_logger()
        if level == "debug":
            logger.debug(message)
        elif level == "info":
            logger.info(message)
        elif level == "warning":
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        elif level == "fatal":
            logger.fatal(message)
        else:
            raise ValueError(f"Unsupported log level: {level}")


def main() -> None:
    rclpy.init()
    node = HeadTrackRarmJoint5()
    try:
        if node.wait_for_head_controller():
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
