import math
import os
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class FrameManager(Node):
    def __init__(self) -> None:
        super().__init__("frame_manager")

        self._broadcaster = StaticTransformBroadcaster(self)
        self._transforms = self._load_transforms()
        # self._broadcaster.sendTransform(self._transforms)

        self.get_logger().info(f"Published {len(self._transforms)} static frame(s)")
        for transform in self._transforms:
            self.get_logger().info(
                f"{transform.header.frame_id} -> {transform.child_frame_id}"
            )

    def _resolve_frames_file(self) -> str:
        return os.path.join(
            get_package_share_directory("welding_robot_application"),
            "config",
            "frames.yaml",
        )

    def _load_yaml(self) -> dict[str, Any]:
        path = self._resolve_frames_file()
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        if "frames" not in data or not isinstance(data["frames"], list):
            raise ValueError(f"{path} must contain a top-level 'frames' list")

        return data

    def _transform_from_config(self, frame_cfg: dict[str, Any]) -> TransformStamped:
        required = ("parent_frame", "child_frame", "translation", "rotation_rpy")
        missing = [key for key in required if key not in frame_cfg]
        if missing:
            raise ValueError(f"Frame entry missing required fields: {missing}")

        translation = frame_cfg["translation"]
        rotation_rpy = frame_cfg["rotation_rpy"]

        a = float(rotation_rpy["a"])
        b = float(rotation_rpy["b"])
        c = float(rotation_rpy["c"])
        if frame_cfg.get("angles_in_degrees", True):
            a = math.radians(a)
            b = math.radians(b)
            c = math.radians(c)

        qx, qy, qz, qw = quaternion_from_rpy(a, b, c)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(frame_cfg["parent_frame"])
        transform.child_frame_id = str(frame_cfg["child_frame"])
        transform.transform.translation.x = float(translation["x"])
        transform.transform.translation.y = float(translation["y"])
        transform.transform.translation.z = float(translation["z"])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _load_transforms(self) -> list[TransformStamped]:
        # data = self._load_yaml()
        transforms = [self._transform_from_config(frame) for frame in data["frames"]]
        if not transforms:
            raise ValueError("frames.yaml does not contain any frames")
        return transforms


# def main() -> None:
#     rclpy.init()
    # node = FrameManager()
    # try:
    #     rclpy.spin(node)
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
