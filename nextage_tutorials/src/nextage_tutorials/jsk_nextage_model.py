from __future__ import annotations

from math import radians
from pathlib import Path

from nextage_tutorials.uv_environment import add_uv_site_packages


add_uv_site_packages()

from skrobot.coordinates import CascadedCoords
from skrobot.models import Nextage


PACKAGE_NAME = "nextage_tutorials"
URDF_NAME = "JSKNextageOpenGripper.urdf"


def package_share_path() -> Path:
    source_root = Path(__file__).resolve().parents[2]
    if (source_root / "urdf" / URDF_NAME).exists():
        return source_root

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return source_root


def jsk_nextage_open_gripper_urdf_path() -> str:
    path = package_share_path() / "urdf" / URDF_NAME
    if not path.exists():
        raise FileNotFoundError(path)
    return str(path)


class JSKNextageOpenGripper(Nextage):
    RESET_POSE = {
        "CHEST_JOINT0": 0.0,
        "HEAD_JOINT0": 0.0,
        "HEAD_JOINT1": 0.0,
        "LARM_JOINT0": radians(0.6),
        "LARM_JOINT1": 0.0,
        "LARM_JOINT2": radians(-100),
        "LARM_JOINT3": radians(-15.2),
        "LARM_JOINT4": radians(9.4),
        "LARM_JOINT5": radians(-3.2),
        "RARM_JOINT0": radians(-0.6),
        "RARM_JOINT1": 0.0,
        "RARM_JOINT2": radians(-100),
        "RARM_JOINT3": radians(15.2),
        "RARM_JOINT4": radians(9.4),
        "RARM_JOINT5": radians(3.2),
    }

    @property
    def default_urdf_path(self):
        return jsk_nextage_open_gripper_urdf_path()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._reset_gripper_end_coords()

    def reset_pose(self):
        angle_vector = [
            self.RESET_POSE.get(joint.name, 0.0)
            for joint in self.joint_list
        ]
        self.angle_vector(angle_vector)
        return self.angle_vector()

    def _reset_gripper_end_coords(self) -> None:
        if hasattr(self, "rhand_tip_link"):
            self.rarm_end_coords = CascadedCoords(
                parent=self.rhand_tip_link,
                name="rarm_end_coords",
            )
        if hasattr(self, "lhand_tip_link"):
            self.larm_end_coords = CascadedCoords(
                parent=self.lhand_tip_link,
                name="larm_end_coords",
            )
