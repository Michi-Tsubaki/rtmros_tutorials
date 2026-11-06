#!/usr/bin/env python3
"""Integration check on an isolated ROS domain. Never uses the live robot domain.
Run after building: python3 test/check_admittance_transport.py
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

# Set before importing rclpy; subprocesses inherit it too.
os.environ['ROS_DOMAIN_ID'] = '179'
import rclpy
from controller_manager_msgs.srv import SwitchController, ListControllers
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


def main():
    rclpy.init()
    node = rclpy.create_node('transport_integration_test')
    assert not node.get_publishers_info_by_topic('/joint_states'), 'Test domain is already in use'
    names = ['CHEST_JOINT0', 'HEAD_JOINT0', 'HEAD_JOINT1'] + [
        f'{arm}_JOINT{i}' for arm in ('LARM', 'RARM') for i in range(6)]
    q = [0., 0., 0.] + [.02, -.03, -2., -.13, .5, -.39] + [.12, .02, -1.96, .41, .39, .09]
    initial = q.copy()
    commands = []
    js = node.create_publisher(JointState, '/joint_states', 10)
    ft = node.create_publisher(WrenchStamped, '/left_ft/force_torque_compensated', 10)
    publish_ft = True
    force = 0.

    def receive(message):
        values = dict(zip(message.joint_names, message.points[0].positions))
        q[:] = [values[name] for name in names]
        commands.append(q.copy())

    node.create_subscription(JointTrajectory, '/fullbody_controller/command', receive, 10)

    def tick():
        stamp = node.get_clock().now().to_msg()
        state = JointState(); state.header.stamp = stamp; state.name = names; state.position = q
        js.publish(state)
        if publish_ft:
            wrench = WrenchStamped(); wrench.header.stamp = stamp
            wrench.header.frame_id = 'LARM_FTSENSOR_SENSOR_Link'; wrench.wrench.force.x = force
            ft.publish(wrench)

    node.create_timer(.01, tick)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    client = node.create_client(SwitchController, '/admittance/controller_manager/switch_controller')
    listing = node.create_client(ListControllers, '/admittance/controller_manager/list_controllers')

    def call(client, request):
        future = client.call_async(request); deadline = time.monotonic()+10
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert future.done(), 'Service timed out'
        return future.result()

    source_launch = Path(__file__).resolve().parents[1] / 'launch/admittance_bringup.launch.py'
    log = open('/tmp/nextage-transport-integration.log', 'w')
    process = subprocess.Popen(['ros2', 'launch', str(source_launch), 'arm:=larm'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            spin(.1)
            if listing.service_is_ready():
                controllers = call(listing, ListControllers.Request()).controller
                if any(c.name == 'larm_admittance_controller' and c.state == 'inactive' for c in controllers):
                    break
        else:
            raise AssertionError('Controller did not configure; see /tmp/nextage-transport-integration.log')
        assert not commands, 'Inactive controller sent commands'
        request = SwitchController.Request(); request.activate_controllers = ['larm_admittance_controller']
        request.strictness = 2
        assert call(client, request).ok
        spin(1.)
        assert len(commands) > 5, 'No active transport output'
        force = 1.
        spin(3.)
        travel = max(abs(a-b) for c in commands for a,b in zip(c, initial))
        step = max(abs(a-b) for x,y in zip(commands, commands[1:]) for a,b in zip(x,y))
        assert .0001 < travel <= .10, travel
        assert step <= .0081, step
        assert all(c[:3] == initial[:3] and c[9:] == initial[9:] for c in commands)
        publish_ft = False  # Joint feedback and command subscription remain alive.
        spin(.7)
        count = len(commands)
        spin(.4)
        assert len(commands) == count, 'Commands continued after wrench timeout'
        controllers = call(listing, ListControllers.Request()).controller
        assert any(c.name == 'larm_admittance_controller' and c.state == 'inactive' for c in controllers)
        print(json.dumps({'commands': count, 'max_travel_rad': travel,
                          'max_step_rad': step, 'stale_wrench_stopped': True}))
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5)
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
