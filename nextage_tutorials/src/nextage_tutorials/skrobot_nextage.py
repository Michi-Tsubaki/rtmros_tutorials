from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import threading
from typing import Iterable

from nextage_tutorials.uv_environment import add_uv_site_packages


add_uv_site_packages()

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from skrobot.interfaces.ros2 import NextageROS2RobotInterface
from trajectory_msgs.msg import JointTrajectoryPoint

from nextage_tutorials.jsk_nextage_model import JSKNextageOpenGripper


CONTROLLER_JOINT_GROUPS = {
    "fullbody_controller": (
        "CHEST_JOINT0",
        "HEAD_JOINT0",
        "HEAD_JOINT1",
        "LARM_JOINT0",
        "LARM_JOINT1",
        "LARM_JOINT2",
        "LARM_JOINT3",
        "LARM_JOINT4",
        "LARM_JOINT5",
        "RARM_JOINT0",
        "RARM_JOINT1",
        "RARM_JOINT2",
        "RARM_JOINT3",
        "RARM_JOINT4",
        "RARM_JOINT5",
    ),
    "larm_controller": (
        "LARM_JOINT0",
        "LARM_JOINT1",
        "LARM_JOINT2",
        "LARM_JOINT3",
        "LARM_JOINT4",
        "LARM_JOINT5",
    ),
    "rarm_controller": (
        "RARM_JOINT0",
        "RARM_JOINT1",
        "RARM_JOINT2",
        "RARM_JOINT3",
        "RARM_JOINT4",
        "RARM_JOINT5",
    ),
    "head_controller": ("HEAD_JOINT0", "HEAD_JOINT1"),
    "torso_controller": ("CHEST_JOINT0",),
    "larm_torso_controller": (
        "CHEST_JOINT0",
        "LARM_JOINT0",
        "LARM_JOINT1",
        "LARM_JOINT2",
        "LARM_JOINT3",
        "LARM_JOINT4",
        "LARM_JOINT5",
    ),
    "rarm_torso_controller": (
        "CHEST_JOINT0",
        "RARM_JOINT0",
        "RARM_JOINT1",
        "RARM_JOINT2",
        "RARM_JOINT3",
        "RARM_JOINT4",
        "RARM_JOINT5",
    ),
}


@dataclass(frozen=True)
class ControllerTarget:
    controller_name: str
    action_name: str
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]


class JSKNextageROS2RobotInterface(NextageROS2RobotInterface):
    @property
    def fullbody_controller(self):
        return self._controller(
            "fullbody_controller",
            [
                getattr(self.robot, name)
                for name in CONTROLLER_JOINT_GROUPS["fullbody_controller"]
            ],
        )


@contextmanager
def nextage_init(controller_timeout: float = 5.0, cancel_on_interrupt: bool = True):
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init(args=None)

    nextage = JSKNextageOpenGripper()
    ri = JSKNextageROS2RobotInterface(nextage, controller_timeout=controller_timeout)
    executor = MultiThreadedExecutor()
    executor.add_node(ri)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        yield nextage, ri
    except KeyboardInterrupt:
        if cancel_on_interrupt:
            try:
                ri.cancel_angle_vector()
            except Exception as exc:
                ri.get_logger().error(f"Failed to cancel motion: {exc}")
        raise
    finally:
        executor.shutdown()
        spin_thread.join()
        ri.destroy_node()
        if owns_rclpy and rclpy.ok():
            rclpy.shutdown()


class NextageTrajectoryCommander(Node):
    def __init__(self, targets: Iterable[ControllerTarget], server_timeout: float) -> None:
        super().__init__("skrobot_nextage_commander")
        self._server_timeout = server_timeout
        self._action_clients = {
            target.controller_name: ActionClient(
                self, FollowJointTrajectory, target.action_name
            )
            for target in targets
        }

    def wait_for_servers(self) -> None:
        for controller_name, client in self._action_clients.items():
            self.get_logger().info(f"Waiting for {controller_name} action server")
            if not client.wait_for_server(timeout_sec=self._server_timeout):
                raise RuntimeError(
                    f"{controller_name} action server did not become available "
                    f"within {self._server_timeout:.1f}s"
                )

    def send_targets(
        self,
        targets: Iterable[ControllerTarget],
        start_delay: float,
        duration: float,
    ) -> bool:
        stamp = (self.get_clock().now() + Duration(seconds=start_delay)).to_msg()
        result_futures = []

        for target in targets:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.header.stamp = stamp
            goal.trajectory.joint_names = list(target.joint_names)

            point = JointTrajectoryPoint()
            point.positions = list(target.positions)
            point.velocities = [0.0] * len(target.positions)
            point.time_from_start = Duration(seconds=duration).to_msg()
            goal.trajectory.points = [point]

            client = self._action_clients[target.controller_name]
            goal_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f"{target.controller_name} rejected the goal")
                return False

            result_futures.append((target.controller_name, goal_handle.get_result_async()))

        ok = True
        for controller_name, result_future in result_futures:
            rclpy.spin_until_future_complete(self, result_future)
            response = result_future.result()
            result = response.result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"{controller_name} failed: error_code={result.error_code} "
                    f"error_string={result.error_string!r}"
                )
                ok = False

        return ok
