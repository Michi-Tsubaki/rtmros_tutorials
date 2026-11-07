#!/usr/bin/env python3
"""Message conversion at the ROS 2 side of the hrpsys / ROS 1 bridge.

Hardware interfaces belong to upstream topic_based_hardware_interfaces.
This node only adapts messages and bounds outgoing real-robot trajectories.
A fault latches until this launch is restarted.
"""
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import WrenchStamped
from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SIDES = {'larm': 'left', 'rarm': 'right'}
AXES = ('force.x', 'force.y', 'force.z', 'torque.x', 'torque.y', 'torque.z')
FULL = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1'] + [
    f'{arm}_JOINT{i}' for arm in ('LARM', 'RARM') for i in range(6)]


class TopicAdapter(Node):
    def __init__(self):
        super().__init__('admittance_transport')
        self.arms = self.declare_parameter('arms', ['larm', 'rarm']).value
        self.require_enable = self.declare_parameter('require_enable', False).value
        self.positions, self.wrenches, self.wrench_times = {}, {}, {}
        self.joint_time = self.command_time = self.enable_time = 0.
        self.enabled = self.fault = False
        self.active, self.anchor, self.last_sent, self.commands = set(), {}, {}, {}
        self.activation_stamp = 0
        self.activation_time = self.sent_time = 0.
        self.started = False
        self.chest = None
        root = ET.parse(Path(get_package_share_directory('nextage_tutorials'))/'urdf/JSKNextageOpenGripper.urdf').getroot()
        self.limits = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
                       for j in root.findall('joint') if j.find('limit') is not None}
        self.pub = self.create_publisher(JointTrajectory, '/fullbody_controller/command', 1)
        self.joint_pub = self.create_publisher(JointState, '/admittance/joint_states', 1)
        self.names_pub = self.create_publisher(StatisticsNames, '/admittance/nextage_ft/names', 1)
        self.values_pub = self.create_publisher(StatisticsValues, '/admittance/nextage_ft/values', 1)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(JointState, '/admittance/joint_commands', self.command, 1)
        self.create_subscription(Bool, '/admittance/gui_enable', self.enable, 1)
        self.create_subscription(ControllerManagerActivity, '/admittance/controller_manager/activity',
                                 self.activity, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.stop_client = self.create_client(SwitchController, '/admittance/controller_manager/switch_controller')
        for arm in self.arms:
            self.create_subscription(WrenchStamped, f'/{SIDES[arm]}_ft/force_torque_compensated',
                                     lambda msg, arm=arm: self.wrench(arm, msg), qos_profile_sensor_data)
        self.create_timer(.04, self.tick)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec

    def fresh_message(self, msg):
        return -.1 <= (self.get_clock().now().nanoseconds-self.stamp(msg))/1e9 < .25

    def joints(self, msg):
        if not self.fresh_message(msg) or len(msg.name) != len(msg.position):
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in FULL):
            return
        self.positions, self.joint_time = positions, time.monotonic()
        # Upstream only exports position states; do not send velocity/effort fields.
        self.joint_pub.publish(JointState(header=msg.header, name=FULL,
                                         position=[positions[n] for n in FULL]))

    def wrench(self, arm, msg):
        w = msg.wrench
        values = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
        if (self.fresh_message(msg) and msg.header.frame_id == arm.upper()+'_FTSENSOR_SENSOR_Link'
                and all(math.isfinite(v) for v in values)):
            self.wrenches[arm], self.wrench_times[arm] = values, time.monotonic()

    def enable(self, msg):
        self.enabled, self.enable_time = msg.data, time.monotonic()

    def activity(self, msg):
        active = {arm for arm in self.arms if any(
            c.name == arm+'_admittance_controller' and c.state.id == 3 for c in msg.controllers)}
        if active != self.active:
            for arm in active-self.active:
                for i in range(6):
                    name = f'{arm.upper()}_JOINT{i}'
                    self.anchor.pop(name, None)
                    self.last_sent.pop(name, None)
            if not self.active and active:
                self.chest = None
                self.started = False
                self.activation_time = self.sent_time = time.monotonic()
            self.commands = {}
            self.activation_stamp = self.stamp(msg)
            self.active = active

    def command(self, msg):
        if (self.fresh_message(msg) and self.stamp(msg) >= self.activation_stamp
                and len(msg.name) == len(msg.position)):
            self.commands = dict(zip(msg.name, msg.position))
            self.command_time = time.monotonic()

    def fail(self, reason):
        if self.fault:
            return
        self.fault = True
        self.get_logger().error('Transport stopped: '+reason+'. Restart bringup to reset.')
        if self.stop_client.service_is_ready():
            req = SwitchController.Request()
            req.deactivate_controllers = [arm+'_admittance_controller' for arm in self.active]
            req.strictness = 1
            req.timeout.sec = 3
            if req.deactivate_controllers:
                self.stop_client.call_async(req)

    def tick(self):
        now = time.monotonic()
        ft_fresh = all(now-self.wrench_times.get(arm, 0.) < .25 for arm in self.arms)
        if ft_fresh:
            # CMTopicSystem consumes named state values, including sensor interfaces.
            names = StatisticsNames(names_version=1, names=[
                f'state_interface.{SIDES[arm]}_ft_sensor/{axis}' for arm in self.arms for axis in AXES])
            values = StatisticsValues(names_version=1, values=[v for arm in self.arms for v in self.wrenches[arm]])
            names.header.stamp = values.header.stamp = self.get_clock().now().to_msg()
            self.names_pub.publish(names)
            self.values_pub.publish(values)
        if not self.active or self.fault:
            return
        if now-self.joint_time >= .25 or not ft_fresh:
            if not self.started and now-self.activation_time < 2.:
                return  # DDS discovery: no trajectory before all fresh inputs arrive.
            return self.fail('stale joint/FT data')
        self.started = True
        if self.chest is None:
            self.chest = self.positions['CHEST_JOINT0']
        if self.require_enable and (not self.enabled or now-self.enable_time > .3):
            return self.fail('GUI enable lease expired')
        if self.pub.get_subscription_count() == 0 or self.count_publishers('/fullbody_controller/command') > 1:
            return self.fail('command bridge missing or another command publisher is running')
        if abs(self.positions['CHEST_JOINT0']-self.chest) > .005:
            return self.fail('chest moved')
        if now-self.command_time >= .25:
            return self.fail('stale controller commands')
        if not self.commands:
            return  # Wait for the first command generated after the activity event.
        targets = self.positions.copy()
        for arm in self.active:
            if any(abs(v) > (15. if i < 3 else 1.5) for i, v in enumerate(self.wrenches[arm])):
                return self.fail('external wrench exceeds commissioning limit')
            for i in range(6):
                name = f'{arm.upper()}_JOINT{i}'
                value = self.commands.get(name, math.nan)
                low, high = self.limits[name]
                self.anchor.setdefault(name, self.positions[name])
                self.last_sent.setdefault(name, self.positions[name])
                if (not math.isfinite(value) or not low <= value <= high
                        or abs(value-self.anchor[name]) > .10 or abs(value-self.positions[name]) > .05):
                    return self.fail('joint command exceeds commissioning envelope')
                step = .1*min(now-self.sent_time, .08)
                value = max(self.last_sent[name]-step, min(self.last_sent[name]+step, value))
                targets[name] = max(self.positions[name]-.01, min(self.positions[name]+.01, value))
        msg = JointTrajectory(joint_names=FULL)
        point = JointTrajectoryPoint(positions=[targets[n] for n in FULL])
        point.time_from_start.nanosec = 100000000
        msg.points = [point]
        self.pub.publish(msg)
        self.last_sent.update(targets)
        self.sent_time = now


def main():
    rclpy.init()
    node = TopicAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
