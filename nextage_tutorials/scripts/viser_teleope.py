#!/usr/bin/env python3

from __future__ import annotations

import argparse
from contextlib import nullcontext
import threading
import time

from nextage_tutorials.uv_environment import add_uv_site_packages


add_uv_site_packages()

import numpy as np

from skrobot.models import Nextage
from skrobot.viewers import ViserViewer
from skrobot.coordinates.math import quaternion2matrix
from skrobot.coordinates.math import rotation_distance

from nextage_tutorials.jsk_nextage_model import JSKNextageOpenGripper


IK_TARGET_POSITION_TOLERANCE = 0.01
IK_TARGET_ROTATION_TOLERANCE = np.deg2rad(5.0)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Interactive NEXTAGE end-effector teleoperation with ViserViewer."
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="Add GUI buttons that send the current viewer pose to the robot.",
    )
    parser.add_argument(
        "--no-sync",
        action="store_true",
        help="Do not initialize the viewer from the latest joint_states.",
    )
    parser.add_argument(
        "--controller-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for FollowJointTrajectory action servers.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Default trajectory duration when sending to the robot.",
    )
    parser.add_argument(
        "--start-time",
        type=float,
        default=0.5,
        help="Default trajectory start delay when sending to the robot.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=10.0,
        help="Timeout for the optional wait-after-send GUI checkbox.",
    )
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Print the viewer URL without opening a browser.",
    )
    parser.add_argument(
        "--no-grid",
        action="store_true",
        help="Hide the ViserViewer ground grid.",
    )
    return parser


def sync_from_robot(nextage: Nextage, ri):
    if ri is None:
        return
    if ri.update_robot_state(wait_until_update=True):
        nextage.angle_vector(ri.robot.angle_vector())
    else:
        reason = getattr(ri, "_timeout_reason", "joint state update failed")
        ri.get_logger().warn(f"Could not sync viewer from joint_states: {reason}")


def unresolved_ik_targets(viewer: ViserViewer):
    unresolved = []
    constrain_rotation = bool(
        getattr(getattr(viewer, "_ik_constrain_rotation", None), "value", False)
    )

    for robot_targets in getattr(viewer, "_ik_targets", {}).values():
        for group_name, target in robot_targets.items():
            control = target.get("control")
            end_coords = target.get("end_coords")
            if control is None or end_coords is None:
                continue

            target_pos = np.asarray(control.position, dtype=np.float64)
            current_pos = np.asarray(end_coords.worldpos(), dtype=np.float64)
            pos_error = float(np.linalg.norm(target_pos - current_pos))

            rot_error = 0.0
            if constrain_rotation:
                target_rot = quaternion2matrix(
                    np.asarray(control.wxyz, dtype=np.float64)
                )
                current_rot = np.asarray(end_coords.worldrot(), dtype=np.float64)
                rot_error = float(rotation_distance(target_rot, current_rot))

            if (
                pos_error > IK_TARGET_POSITION_TOLERANCE
                or rot_error > IK_TARGET_ROTATION_TOLERANCE
            ):
                unresolved.append((group_name, pos_error, rot_error))

    return unresolved


def add_robot_send_controls(
    viewer: ViserViewer,
    nextage: Nextage,
    ri,
    duration,
    start_time,
    wait_timeout,
):
    send_lock = threading.Lock()

    with viewer._server.gui.add_folder("NEXTAGE Teleoperation", expand_by_default=True):
        duration_input = viewer._server.gui.add_number(
            "Duration [s]", initial_value=duration, step=0.1
        )
        start_time_input = viewer._server.gui.add_number(
            "Start delay [s]", initial_value=start_time, step=0.1
        )
        wait_checkbox = viewer._server.gui.add_checkbox(
            "Wait for completion", initial_value=False
        )
        wait_timeout_input = viewer._server.gui.add_number(
            "Wait timeout [s]", initial_value=wait_timeout, step=0.5
        )
        status_text = viewer._server.gui.add_text("Status", initial_value="Ready")
        send_button = viewer._server.gui.add_button("Send current pose")
        cancel_button = viewer._server.gui.add_button("Cancel motion")

    def send_current_pose():
        if not send_lock.acquire(blocking=False):
            status_text.value = "Send already in progress"
            return
        try:
            unresolved_targets = unresolved_ik_targets(viewer)
            if unresolved_targets:
                group_name, pos_error, rot_error = max(
                    unresolved_targets, key=lambda item: (item[1], item[2])
                )
                message = (
                    "IK not solved: "
                    f"{group_name} pos_err={pos_error:.3f}m "
                    f"rot_err={np.rad2deg(rot_error):.1f}deg"
                )
                status_text.value = message
                ri.get_logger().warn(f"Blocked send because {message}")
                return

            av = nextage.angle_vector()
            status_text.value = "Sending..."
            ri.angle_vector(
                av,
                time=float(duration_input.value),
                start_time=float(start_time_input.value),
            )
            if wait_checkbox.value:
                ri.wait_interpolation(
                    timeout=float(wait_timeout_input.value),
                    wait_for_state_update=False,
                )
            status_text.value = "Sent"
        except Exception as exc:
            status_text.value = f"Send failed: {exc}"
            ri.get_logger().error(f"Failed to send viewer pose: {exc}")
        finally:
            send_lock.release()

    def cancel_motion():
        try:
            ri.cancel_angle_vector()
            status_text.value = "Cancel requested"
        except Exception as exc:
            status_text.value = f"Cancel failed: {exc}"
            ri.get_logger().error(f"Failed to cancel motion: {exc}")

    send_button.on_click(
        lambda _: threading.Thread(target=send_current_pose, daemon=True).start()
    )
    cancel_button.on_click(
        lambda _: threading.Thread(target=cancel_motion, daemon=True).start()
    )


def run_viewer(nextage: Nextage, ri, args: argparse.Namespace):
    if args.send and not args.no_sync:
        sync_from_robot(nextage, ri)

    viewer = ViserViewer(draw_grid=not args.no_grid, enable_ik=True)
    viewer.add(nextage)

    if args.send:
        add_robot_send_controls(
            viewer,
            nextage,
            ri,
            duration=args.duration,
            start_time=args.start_time,
            wait_timeout=args.wait_timeout,
        )

    viewer.show(open_browser=not args.no_open_browser)
    print("Drag the rarm/larm IK controls in the browser.", flush=True)
    if args.send:
        print("Use the 'Send current pose' button to command the robot.", flush=True)
    else:
        print("Viewer-only mode. Pass --send to enable robot command buttons.", flush=True)

    try:
        while viewer.is_active:
            time.sleep(0.1)
    except KeyboardInterrupt:
        if ri is not None:
            ri.cancel_angle_vector()
    finally:
        viewer.close()

    return 0


def main():
    args = build_parser().parse_args()

    if args.send:
        from nextage_tutorials.skrobot_nextage import nextage_init

        robot_context = nextage_init(
            controller_timeout=args.controller_timeout,
            cancel_on_interrupt=True,
        )
    else:
        robot_context = nullcontext((JSKNextageOpenGripper(), None))

    with robot_context as (nextage, ri):
        return run_viewer(nextage, ri, args)


if __name__ == "__main__":
    main()
