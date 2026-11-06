#!/usr/bin/env python3
"""TouchState -> skrobot IK -> NEXTAGE, with a viser debug view.

The Touch HID/OpenHaptics driver owns button toggling. Do not toggle again here.
All lengths below are metres, all joint/rotation angles are radians.
"""
import argparse
import time

from nextage_tutorials.uv_environment import add_uv_site_packages

add_uv_site_packages()

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Duration
from control_msgs.action import ParallelGripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from touch_msgs.msg import TouchState
from skrobot.coordinates import CascadedCoords, Coordinates
from skrobot.coordinates.math import quaternion2matrix, matrix2quaternion, rotation_distance
from skrobot.viewers import ViserViewer
from nextage_tutorials.jsk_nextage_model import JSKNextageOpenGripper
from nextage_tutorials.skrobot_nextage import CONTROLLER_JOINT_GROUPS

# Same values/mapping as euslisp/jsk-nextage-teleop.l.
POSITION_SCALE = 2.0
TOUCH_FRAME_MAP = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
RATE = 50.0
SEND_TIME = 1.0 / RATE
MAX_JOINT_SPEED = 1.0  # rad/s, same default as agentic_mp
INPUT_TIMEOUT = 0.25
JOINT_STATE_TIMEOUT = 0.5
MAX_JOINT_STEP = np.deg2rad(10)
INITIAL_SPEED = 0.3
INITIAL_TIMEOUT = 30.0
INITIAL_TOLERANCE = np.deg2rad(0.5)


def raised_initial_pose(robot, tips):
    """Configured reset pose with both tool tips raised 10 cm in world Z."""
    before = robot.angle_vector().copy()
    try:
        robot.reset_pose()
        for arm, tip in zip((robot.larm, robot.rarm), tips):
            target = Coordinates(pos=tip.worldpos() + [0., 0., 0.10], rot=tip.worldrot().copy())
            result = arm.inverse_kinematics(target, move_target=tip, rotation_axis=True,
                                            stop=100, revert_if_fail=True)
            if (result is False or result is None
                    or np.linalg.norm(tip.worldpos() - target.worldpos()) > 0.001
                    or rotation_distance(tip.worldrot(), target.worldrot()) > np.deg2rad(1)):
                raise RuntimeError('Cannot solve initial pose: reset pose + both arms 10 cm up')
        return robot.angle_vector().copy()
    finally:
        robot.angle_vector(before)


def publish_arm_command(publisher, joint_names, positions):
    trajectory = JointTrajectory()  # Zero stamp: start on receipt, including ROS 1 bridge.
    trajectory.joint_names = list(joint_names)
    trajectory.points = [JointTrajectoryPoint(
        positions=[float(value) for value in positions],
        time_from_start=Duration(nanosec=round(SEND_TIME * 1e9)))]
    publisher.publish(trajectory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left-topic', default='/left_ft/state')
    parser.add_argument('--right-topic', default='/right_ft/state')
    parser.add_argument('--touch-units', choices=['mm', 'm'], default='mm')
    parser.add_argument('--no-open-browser', action='store_true')
    parser.add_argument('--skip-initial-pose', action='store_true',
                        help='Start teleoperation at measured pose without the startup move')
    args = parser.parse_args()
    unit_scale = 0.001 if args.touch_units == 'mm' else 1.0

    # Keep the ROS context alive through Ctrl-C cleanup/cancel requests.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('nextage_touch_hid_teleop')
    # spin_once runs callbacks on this thread: no model mutation from ROS threads.
    samples = {'left': (None, 0.0), 'right': (None, 0.0)}

    def left_callback(msg):
        samples['left'] = (msg, time.monotonic())

    def right_callback(msg):
        samples['right'] = (msg, time.monotonic())

    node.create_subscription(TouchState, args.left_topic, left_callback, qos_profile_sensor_data)
    node.create_subscription(TouchState, args.right_topic, right_callback, qos_profile_sensor_data)
    # Require a command subscriber plus fresh feedback for every commanded joint.
    # Topic discovery alone also finds publishers that are no longer producing data.
    joint_feedback = {}

    def joint_callback(msg):
        now = time.monotonic()
        for name, position in zip(msg.name, msg.position):
            if np.isfinite(position):
                joint_feedback[name] = (position, now)
            else:
                joint_feedback.pop(name, None)

    node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data)
    command_pub = node.create_publisher(JointTrajectory, '/fullbody_controller/command', 1)
    joint_names = CONTROLLER_JOINT_GROUPS['fullbody_controller']
    try:
        robot = JSKNextageOpenGripper()

        # Left pinch: NEJINEJI -> gripper base -> gripper tip (same as nt-new-pinch).
        left_tip = CascadedCoords(parent=robot.LARM_NEJINEJI_Link)
        left_tip.translate([0.031905903, 0.068529998, 0.03632414])
        left_tip.rotate(-np.pi / 2, 'x')
        left_tip.translate([0, 0, 0.06743])
        # Right: original EusLisp end-coords, then the needle_holder tip ops.
        # Do not use the Python model's default gripper end-coords for this tool.
        right_tip = CascadedCoords(parent=robot.RARM_JOINT5_Link)
        right_tip.translate([-0.190, 0, 0])
        right_tip.rotate(np.pi / 2, 'y')
        right_tip.rotate(np.pi / 2, 'z')
        right_tip.translate([0, 0, 0.190])
        right_tip.translate([0, 0, -0.102])
        right_tip.rotate(np.deg2rad(33), 'z')
        right_tip.rotate(np.deg2rad(88.7), 'y')
        right_tip.translate([0, -0.023, 0.090])

        # Convex hulls make skrobot's FCL check fast enough for an interactive loop.
        # Keep visual meshes unchanged. Hulls are conservative (can over-report).
        # Group fixed links and hand internals with their nearest body joint.
        body_links = {j.child_link.name for j in robot.joint_list
                      if j.name.startswith(('LARM_JOINT', 'RARM_JOINT', 'HEAD_JOINT', 'CHEST_JOINT'))}
        rigid_group = {}
        for link in robot.link_list:
            if link.collision_mesh is not None:
                link.collision_mesh = link.collision_mesh.convex_hull
            ancestor = link
            while ancestor.parent_link is not None:
                if ancestor.name in body_links:
                    break
                ancestor = ancestor.parent_link
            rigid_group[link.name] = ancestor.name
        ignored_pairs = set()
        for a in robot.link_list:
            for b in robot.link_list:
                ga, gb = rigid_group[a.name], rigid_group[b.name]
                parent_a = getattr(robot, ga).parent_link
                parent_b = getattr(robot, gb).parent_link
                if (ga == gb or (parent_a and rigid_group[parent_a.name] == gb)
                        or (parent_b and rigid_group[parent_b.name] == ga)):
                    ignored_pairs.add(tuple(sorted((a.name, b.name))))
        # The conservative torso hull encloses the neck mounting region.
        ignored_pairs.add(('CHEST_JOINT0_Link', 'HEAD_JOINT1_Link'))

        # Explicit hand controller mapping; same motor angles/efforts as Lisp.
        hands = {}
        hands['left'] = ActionClient(node, ParallelGripperCommand,
                '/lhand/gripper_gripper_controller/gripper_cmd')
        hands['right'] = ActionClient(node, ParallelGripperCommand,
                '/rhand/needle_holder_gripper_controller/gripper_cmd')
        hand_previous = {'left': False, 'right': False}
        hand_futures = {'left': None, 'right': None}
        hand_handles = {'left': None, 'right': None}
        hand_status = {'left': 'idle', 'right': 'idle'}

        viewer = ViserViewer(enable_ik=False)
        viewer.add(robot)
        # ViserViewer exposes its viser server via _server in this checkout.
        gui = viewer._server.gui
        scene = viewer._server.scene
        with gui.add_folder('Touch HID teleop', expand_by_default=True):
            gui.add_markdown('Gray: toggle open/close. White: toggle arm lock.\n\n'
                             'Large axes: target. Small axes: model tool tip. '
                             'Robot mesh shows IK, not measured hardware feedback.')
            enabled = gui.add_checkbox('Send to robot', initial_value=True)
            connection_text = gui.add_text('Robot connection', initial_value='Waiting', disabled=True)
            recalibrate = gui.add_button('Recalibrate at current pose')
            status = gui.add_text('Command', initial_value='Preview only', disabled=True)
            collision_text = gui.add_text('Collision pairs', initial_value='', disabled=True)
            loop_text = gui.add_text('Loop ms', initial_value='', disabled=True)
        panels, target_frames, tip_frames = {}, {}, {}
        for side in ('left', 'right'):
            with gui.add_folder(side, expand_by_default=True):
                panels[side] = gui.add_markdown('Waiting for TouchState')
            target_frames[side] = scene.add_frame('/' + side + '/target', axes_length=0.10, axes_radius=0.003)
            tip_frames[side] = scene.add_frame('/' + side + '/tip', axes_length=0.05, axes_radius=0.002)
        # Joint sliders would mutate the IK model on a GUI thread. Display only.
        for slider in viewer._joint_sliders.values():
            slider.disabled = True
        calibration = {'left': None, 'right': None}
        requests = {'calibrate': False}

        @recalibrate.on_click
        def on_recalibrate(_):
            requests['calibrate'] = True

        viewer.show(open_browser=not args.no_open_browser)
        last_angles = robot.angle_vector().copy()
        was_enabled = False
        initialized = args.skip_initial_pose
        initial_target = raised_initial_pose(robot, (left_tip, right_tip)) if not initialized else None
        initial_started = None
        try:
            while rclpy.ok() and viewer.is_active:
                start = time.monotonic()
                # Drain a bounded number of callbacks without blocking the loop.
                for _ in range(4):
                    rclpy.spin_once(node, timeout_sec=0.0)
                connected = command_pub.get_subscription_count() > 0 and all(
                    name in joint_feedback and time.monotonic() - joint_feedback[name][1] < JOINT_STATE_TIMEOUT
                    for name in joint_names)
                connection_text.value = 'Connected' if connected else 'Disconnected: preview only'
                if not initialized:
                    if not connected or not enabled.value:
                        if initial_started is not None:
                            enabled.value = False
                        initial_started = None
                        status.value = 'INIT waiting for connection / Send to robot'
                    else:
                        if initial_started is None:
                            initial_started = start
                        for name in joint_names:
                            getattr(robot, name).joint_angle(joint_feedback[name][0])
                        measured = robot.angle_vector().copy()
                        indices = [robot.joint_list.index(getattr(robot, name)) for name in joint_names]
                        error = initial_target[indices] - measured[indices]
                        if np.max(np.abs(error)) <= INITIAL_TOLERANCE:
                            initialized = True
                            last_angles = measured.copy()
                            status.value = 'Initial pose reached; waiting for Touch input'
                            node.get_logger().info('Initial pose reached: reset pose + both arms 10 cm up')
                        else:
                            candidate = measured.copy()
                            candidate[indices] += np.clip(error, -INITIAL_SPEED / RATE, INITIAL_SPEED / RATE)
                            robot.angle_vector(candidate)
                            try:
                                _, pairs = robot.self_collision_check()
                                pairs = pairs - ignored_pairs
                                collision_text.value = str(sorted(pairs))
                                safe = not pairs
                            except (ImportError, ValueError) as exc:
                                safe = False
                                collision_text.value = str(exc)
                            fresh = command_pub.get_subscription_count() > 0 and all(
                                time.monotonic() - joint_feedback[name][1] < JOINT_STATE_TIMEOUT
                                for name in joint_names)
                            if not safe or not fresh or start - initial_started > INITIAL_TIMEOUT:
                                enabled.value = False
                                initial_started = None
                                status.value = 'INIT BLOCKED: collision / stale feedback / timeout'
                            else:
                                publish_arm_command(command_pub, joint_names, candidate[indices])
                                status.value = f'INIT: reset pose + 10 cm up ({np.rad2deg(np.max(np.abs(error))):.1f} deg remaining)'
                    viewer.redraw()
                    loop_text.value = f'{(time.monotonic() - start) * 1000:.1f}'
                    time.sleep(max(0, 1 / RATE - (time.monotonic() - start)))
                    continue
                touch_ready = all(msg is not None and time.monotonic() - stamp <= INPUT_TIMEOUT
                                  for msg, stamp in samples.values())
                sending = bool(enabled.value) and connected and touch_ready
                # Missing input at startup waits; loss during operation still disarms.
                if was_enabled and connected and not touch_ready:
                    enabled.value = False
                if requests['calibrate'] or (sending and not was_enabled):
                    if connected:
                        for name in CONTROLLER_JOINT_GROUPS['fullbody_controller']:
                            getattr(robot, name).joint_angle(joint_feedback[name][0])
                    calibration = {'left': None, 'right': None}
                    last_angles = robot.angle_vector().copy()
                    requests['calibrate'] = False
                # Start every solve at the last accepted pose; failed IK never accumulates.
                robot.angle_vector(last_angles)
                before = last_angles.copy()
                all_ok, any_active = True, False
                reports = {}
                for side, arm, tip in (('left', robot.larm, left_tip),
                                       ('right', robot.rarm, right_tip)):
                    msg, received = samples[side]
                    age = time.monotonic() - received
                    if msg is None or age > INPUT_TIMEOUT:
                        panels[side].content = f'**Waiting / stale** (age {age:.3f} s)'
                        # Either missing device blocks both arms in this dual-arm script.
                        all_ok = False
                        calibration[side] = None
                        continue
                    p, q = msg.pose.position, msg.pose.orientation
                    position = np.array([p.x, p.y, p.z])
                    quat = np.array([q.w, q.x, q.y, q.z])
                    if not np.isfinite(np.r_[position, quat]).all() or np.linalg.norm(quat) < 1e-8:
                        panels[side].content = '**Invalid pose / quaternion**'
                        all_ok = False
                        continue
                    # nt-touch-coords: same frame map for translation and rotation.
                    touch = Coordinates(
                        pos=POSITION_SCALE * unit_scale * (TOUCH_FRAME_MAP @ position),
                        rot=TOUCH_FRAME_MAP @ quaternion2matrix(quat / np.linalg.norm(quat)) @ TOUCH_FRAME_MAP.T)
                    touch.rotate(np.pi / 2, 'z')
                    if calibration[side] is None:
                        calibration[side] = (tip.worldpos() - touch.worldpos(),
                                             tip.worldrot() @ touch.worldrot().T)
                    offset_pos, offset_rot = calibration[side]
                    target = Coordinates(pos=touch.worldpos() + offset_pos,
                                         rot=offset_rot @ touch.worldrot())
                    target_frames[side].position = target.worldpos()
                    target_frames[side].wxyz = matrix2quaternion(target.worldrot())
                    # White button: the driver already toggles msg.locked.
                    ik_status = 'locked'
                    if not msg.locked:
                        any_active = True
                        result = arm.inverse_kinematics(target, move_target=tip,
                            rotation_axis=True, stop=10, revert_if_fail=True)
                        ik_status = 'OK' if result is not False and result is not None else 'FAIL'
                        all_ok &= ik_status == 'OK'
                    error_mm = np.linalg.norm(tip.worldpos() - target.worldpos()) * 1000
                    error_deg = np.rad2deg(rotation_distance(tip.worldrot(), target.worldrot()))
                    reports[side] = (msg, age, position, target, ik_status, error_mm, error_deg)

                # skrobot collision check; exclude only the documented mounting pairs.
                try:
                    collision, pairs = robot.self_collision_check()
                    pairs = pairs - ignored_pairs
                    collision = bool(pairs)
                    collision_text.value = str(sorted(pairs))
                except (ImportError, ValueError) as exc:
                    collision = True
                    collision_text.value = str(exc)
                work_ok = left_tip.worldpos()[2] > -0.2 and right_tip.worldpos()[2] > -0.2
                candidate = robot.angle_vector().copy()
                step_ok = np.max(np.abs(candidate - before)) <= MAX_JOINT_STEP
                # Also check after IK/collision: slow calculations must not send stale input.
                all_ok &= all(time.monotonic() - stamp <= INPUT_TIMEOUT
                              for _, stamp in samples.values())
                accepted = all_ok and not collision and work_ok and step_ok
                # Preview can show the rejected candidate, but next solve starts from before.
                if accepted:
                    last_angles = candidate.copy()
                reason = ('input / IK' if not all_ok else 'collision' if collision else
                          'z < -0.2 m' if not work_ok else 'joint step > 10 deg' if not step_ok else
                          'locked' if not any_active else 'ready')
                # Recheck after the solve in case the controller/feedback disappeared.
                if sending and (command_pub.get_subscription_count() == 0 or any(
                        time.monotonic() - joint_feedback[name][1] >= JOINT_STATE_TIMEOUT
                        for name in CONTROLLER_JOINT_GROUPS['fullbody_controller'])):
                    sending = False
                    connection_text.value = 'Disconnected: preview only'
                if sending and accepted and any_active:
                    # agentic_mp flush_sessions: limit each tick from measured feedback.
                    current = np.array([joint_feedback[name][0] for name in joint_names])
                    target = np.array([getattr(robot, name).joint_angle() for name in joint_names])
                    command = current + np.clip(target - current,
                                                -MAX_JOINT_SPEED / RATE, MAX_JOINT_SPEED / RATE)
                    publish_arm_command(command_pub, joint_names, command)
                    status.value = 'Arm command published (topic; no acceptance feedback)'
                else:
                    status.value = ('SEND BLOCKED: ' if sending else 'PREVIEW: ') + reason
                # Gray button: edge of close_gripper, independent of the white arm lock.
                for side in ('left', 'right'):
                    future = hand_futures[side]
                    if future is not None and future.done():
                        handle = future.result()
                        hand_handles[side] = handle if handle.accepted else None
                        if handle.accepted and not sending:
                            handle.cancel_goal_async()
                        hand_status[side] = 'accepted' if handle.accepted else 'REJECTED'
                        hand_futures[side] = None
                    if side not in reports:
                        continue
                    msg, age, position, target, ik_status, err_mm, err_deg = reports[side]
                    if sending and accepted and msg.close_gripper != hand_previous[side]:
                        if hand_futures[side] is None and hands[side].server_is_ready():
                            goal = ParallelGripperCommand.Goal()
                            if side == 'left':
                                goal.command.name = ['lhand_gripper_joint']
                                goal.command.position = [-1.5 if msg.close_gripper else 0.0]
                                goal.command.effort = [0.50 if msg.close_gripper else 0.30]
                            else:
                                goal.command.name = ['rhand_needle_holder_joint']
                                goal.command.position = [1.0 if msg.close_gripper else -1.87]
                                goal.command.effort = [0.0]
                            hand_futures[side] = hands[side].send_goal_async(goal)
                            hand_previous[side] = msg.close_gripper
                            hand_status[side] = 'submitted'
                        else:
                            hand_status[side] = 'waiting for action server / response'
                    if not sending:
                        hand_previous[side] = msg.close_gripper
                    panels[side].content = (
                        f'age: {age:.3f} s / locked: **{msg.locked}** / close: **{msg.close_gripper}**\n\n'
                        f'raw ({args.touch_units}): {position.round(3)}\n\n'
                        f'quat (wxyz): {[round(v, 4) for v in (msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)]}\n\n'
                        f'target (m): {target.worldpos().round(4)}\n\n'
                        f'IK: **{ik_status}** / error: {err_mm:.2f} mm, {err_deg:.2f} deg\n\n'
                        f'hand: {hand_status[side]}')
                # Topic commands cannot be cancelled; stop publishing on faults.
                # Each trajectory lasts 20 ms. Recovery requires GUI enable (and rebase).
                if sending and not accepted:
                    enabled.value = False
                if (sending and not accepted) or (was_enabled and not sending):
                    for handle in hand_handles.values():
                        if handle is not None:
                            handle.cancel_goal_async()
                was_enabled = sending and accepted
                for side, tip in (('left', left_tip), ('right', right_tip)):
                    tip_frames[side].position = tip.worldpos()
                    tip_frames[side].wxyz = matrix2quaternion(tip.worldrot())
                viewer.redraw()
                loop_text.value = f'{(time.monotonic() - start) * 1000:.1f}'
                time.sleep(max(0, 1 / RATE - (time.monotonic() - start)))
        finally:
            for future in hand_futures.values():
                if future is None:
                    continue
                for _ in range(20):
                    if future.done() or not rclpy.ok():
                        break
                    rclpy.spin_once(node, timeout_sec=0.05)
                if future.done():
                    handle = future.result()
                    if handle.accepted:
                        handle.cancel_goal_async()
            for handle in hand_handles.values():
                if handle is not None:
                    handle.cancel_goal_async()
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
            viewer.close()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
