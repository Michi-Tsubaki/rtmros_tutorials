#!/usr/bin/env python3
"""Move the D405 link frame from an RViz2 interactive marker."""

from __future__ import annotations

import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm < 1.0e-12:
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0.0
        quaternion.w = 1.0
        return quaternion
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return quaternion


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    quaternion = Quaternion()
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    quaternion.w = cr * cp * cy + sr * sp * sy
    return normalize_quaternion(quaternion)


def rpy_from_quaternion(quaternion: Quaternion) -> tuple[float, float, float]:
    q = normalize_quaternion(deepcopy(quaternion))
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi * 0.5, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def pose_from_xyz_rpy(xyz: list[float], rpy: list[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation = quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return pose


def pose_from_transform(transform: TransformStamped) -> Pose:
    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation = normalize_quaternion(transform.transform.rotation)
    return pose


def fmt(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


class D405LinkInteractiveMarker(Node):
    def __init__(self) -> None:
        super().__init__("d405_link_interactive_marker")

        self.declare_parameter("parent_frame", "RARM_JOINT5_Link")
        self.declare_parameter("child_frame", "RARM_CAMERA_Link")
        self.declare_parameter("marker_namespace", "d405_link_marker")
        self.declare_parameter("marker_name", "rarm_camera_link")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("log_period", 0.5)
        self.declare_parameter("marker_scale", 0.15)
        self.declare_parameter("use_current_tf", True)
        self.declare_parameter("seed_timeout", 2.0)
        self.declare_parameter("initial_xyz", [0.02, -0.045, -0.03])
        self.declare_parameter("initial_rpy", [1.21300383, 0.0, 1.26012772])

        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.marker_namespace = self.get_parameter("marker_namespace").value
        self.marker_name = self.get_parameter("marker_name").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.use_current_tf = bool(self.get_parameter("use_current_tf").value)
        self.seed_timeout = float(self.get_parameter("seed_timeout").value)
        initial_xyz = list(self.get_parameter("initial_xyz").value)
        initial_rpy = list(self.get_parameter("initial_rpy").value)

        self.pose = pose_from_xyz_rpy(initial_xyz, initial_rpy)
        self.server: InteractiveMarkerServer | None = None
        self.publish_timer = None
        self.seed_timer = None
        self.last_log_time = self.get_clock().now() - Duration(seconds=3600.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(
            PoseStamped, f"{self.marker_namespace}/pose", 10
        )

        self.seed_start_time = self.get_clock().now()
        if self.use_current_tf:
            self.get_logger().info(
                "Waiting for initial TF "
                f"{self.parent_frame} -> {self.child_frame} "
                f"for up to {self.seed_timeout:.1f}s"
            )
            self.seed_timer = self.create_timer(0.1, self.try_seed_and_start)
        else:
            self.start_marker()

    def try_seed_and_start(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, Time()
            )
        except TransformException:
            elapsed = self.get_clock().now() - self.seed_start_time
            if elapsed < Duration(seconds=self.seed_timeout):
                return
            self.get_logger().warning(
                "Initial TF was not available; using initial_xyz/initial_rpy parameters"
            )
        else:
            self.pose = pose_from_transform(transform)
            self.get_logger().info(
                f"Seeded marker pose from TF {self.parent_frame} -> {self.child_frame}"
            )

        if self.seed_timer is not None:
            self.destroy_timer(self.seed_timer)
            self.seed_timer = None
        self.start_marker()

    def start_marker(self) -> None:
        if self.server is not None:
            return

        self.server = InteractiveMarkerServer(self, self.marker_namespace)
        marker = self.make_interactive_marker()
        self.server.insert(marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

        period = 1.0 / max(self.publish_rate, 1.0e-6)
        self.publish_timer = self.create_timer(period, self.publish_transform)
        self.publish_transform()
        self.log_urdf_origin("initial")
        self.get_logger().info(
            "Add an RViz2 InteractiveMarkers display and select namespace "
            f"'{self.marker_namespace}'. Moving the marker publishes "
            f"{self.parent_frame} -> {self.child_frame}."
        )

    def make_interactive_marker(self) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header.frame_id = self.parent_frame
        marker.name = self.marker_name
        marker.description = self.child_frame
        marker.scale = self.marker_scale
        marker.pose = deepcopy(self.pose)

        body_control = InteractiveMarkerControl()
        body_control.always_visible = True
        body_control.interaction_mode = InteractiveMarkerControl.NONE
        body_control.markers.append(self.make_camera_marker())
        marker.controls.append(body_control)

        self.append_axis_controls(marker)
        return marker

    def make_camera_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = self.marker_scale * 0.45
        marker.scale.y = self.marker_scale * 0.75
        marker.scale.z = self.marker_scale * 0.28
        marker.color.r = 0.05
        marker.color.g = 0.35
        marker.color.b = 0.95
        marker.color.a = 0.85
        return marker

    def append_axis_controls(self, marker: InteractiveMarker) -> None:
        axes = [
            ("x", (1.0, 1.0, 0.0, 0.0)),
            ("z", (1.0, 0.0, 1.0, 0.0)),
            ("y", (1.0, 0.0, 0.0, 1.0)),
        ]
        for axis_name, orientation in axes:
            rotate_control = self.make_axis_control(
                f"rotate_{axis_name}", orientation, InteractiveMarkerControl.ROTATE_AXIS
            )
            marker.controls.append(rotate_control)

            move_control = self.make_axis_control(
                f"move_{axis_name}", orientation, InteractiveMarkerControl.MOVE_AXIS
            )
            marker.controls.append(move_control)

    @staticmethod
    def make_axis_control(
        name: str, orientation: tuple[float, float, float, float], interaction_mode: int
    ) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = orientation[0]
        control.orientation.x = orientation[1]
        control.orientation.y = orientation[2]
        control.orientation.z = orientation[3]
        control.orientation = normalize_quaternion(control.orientation)
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = interaction_mode
        return control

    def process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose = deepcopy(feedback.pose)
            self.publish_transform()
            self.publish_pose()
            self.log_urdf_origin_throttled()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.pose = deepcopy(feedback.pose)
            self.publish_transform()
            self.publish_pose()
            self.log_urdf_origin("mouse_up")

    def publish_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.pose.position.x
        transform.transform.translation.y = self.pose.position.y
        transform.transform.translation.z = self.pose.position.z
        transform.transform.rotation = normalize_quaternion(deepcopy(self.pose.orientation))
        self.tf_broadcaster.sendTransform(transform)

    def publish_pose(self) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.parent_frame
        pose_msg.pose = deepcopy(self.pose)
        self.pose_pub.publish(pose_msg)

    def log_urdf_origin_throttled(self) -> None:
        now = self.get_clock().now()
        if now - self.last_log_time < Duration(seconds=self.log_period):
            return
        self.log_urdf_origin("pose")

    def log_urdf_origin(self, reason: str) -> None:
        xyz = (
            self.pose.position.x,
            self.pose.position.y,
            self.pose.position.z,
        )
        rpy = rpy_from_quaternion(self.pose.orientation)
        self.last_log_time = self.get_clock().now()
        self.get_logger().info(
            f"[{reason}] {self.parent_frame} -> {self.child_frame}: "
            f'origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"'
        )

    def destroy_node(self) -> bool:
        if self.server is not None:
            self.server.shutdown()
            self.server = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = D405LinkInteractiveMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
