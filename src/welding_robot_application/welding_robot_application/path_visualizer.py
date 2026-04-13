import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, PoseArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    rx: Optional[float] = None
    ry: Optional[float] = None
    rz: Optional[float] = None


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


class PathVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("path_visualizer")

        self.declare_parameter("toolpath_name", "beam_top_outer_right")
        self.declare_parameter("frame_id", "table_top_corner")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("point_scale", 0.015)
        self.declare_parameter("line_scale", 0.005)
        self.declare_parameter("arrow_length", 0.04)
        self.declare_parameter("arrow_shaft_diameter", 0.004)
        self.declare_parameter("arrow_head_diameter", 0.008)
        self.declare_parameter("angles_in_degrees", False)

        # self._pose_array_pub = self.create_publisher(PoseArray, "toolpath_pose_array", 10)
        # self._marker_array_pub = self.create_publisher(MarkerArray, "toolpath_markers", 10)

        # self._waypoints = self.load_waypoints()
        self._publish_messages()

        period = 1.0 / self.get_parameter("publish_rate_hz").value
        self.create_timer(period, self._publish_messages)

    def resolve_csv_path(self) -> str:
        toolpath_name = self.get_parameter("toolpath_name").get_parameter_value().string_value
        if os.path.isabs(toolpath_name):
            return toolpath_name if toolpath_name.endswith(".csv") else f"{toolpath_name}.csv"

        filename = toolpath_name if toolpath_name.endswith(".csv") else f"{toolpath_name}.csv"
        return os.path.join(
            get_package_share_directory("welding_robot_application"),
            "paths",
            filename,
        )

    def load_waypoints(self) -> List[Waypoint]:
        path = self.resolve_csv_path()
        waypoints: List[Waypoint] = []

        with open(path, newline="", encoding="utf-8") as csv_file:
            sample = csv_file.read(1024)
            csv_file.seek(0)
            try:
                has_header = csv.Sniffer().has_header(sample)
            except csv.Error:
                has_header = True

            if has_header:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    if not row:
                        continue
                    normalized_row = {
                        (key.strip().lower() if key is not None else key): value.strip()
                        for key, value in row.items()
                        if key is not None and value is not None
                    }
                    waypoints.append(
                        Waypoint(
                            x=float(normalized_row["x"]),
                            y=float(normalized_row["y"]),
                            z=float(normalized_row["z"]),
                            rx=self._optional_angle(normalized_row, "rx", "roll", "a"),
                            ry=self._optional_angle(normalized_row, "ry", "pitch", "b"),
                            rz=self._optional_angle(normalized_row, "rz", "yaw", "c"),
                        )
                    )
            else:
                reader = csv.reader(csv_file)
                for row in reader:
                    if not row:
                        continue
                    if len(row) < 3:
                        raise ValueError(f"Expected at least 3 columns, got {len(row)} in row {row}")
                    values = [float(value) for value in row]
                    waypoint = Waypoint(x=values[0], y=values[1], z=values[2])
                    if len(values) >= 6:
                        waypoint.rx = values[3]
                        waypoint.ry = values[4]
                        waypoint.rz = values[5]
                    waypoints.append(waypoint)

        if not waypoints:
            raise ValueError(f"No waypoints found in {path}")

        self.get_logger().info(f"Loaded {len(waypoints)} waypoint(s) from {path}")
        return waypoints

    @staticmethod
    def _optional_angle(row: dict, *keys: str) -> Optional[float]:
        value = None
        for key in keys:
            value = row.get(key)
            if value not in (None, ""):
                break
        return float(value) if value not in (None, "") else None

    # def pose_from_waypoint(self, waypoint: Waypoint) -> Pose:
    #     pose = Pose()
    #     pose.position.x = waypoint.x
    #     pose.position.y = waypoint.y
    #     pose.position.z = waypoint.z

    #     rx = waypoint.rx or 0.0
    #     ry = waypoint.ry or 0.0
    #     rz = waypoint.rz or 0.0

    #     if self.get_parameter("angles_in_degrees").get_parameter_value().bool_value:
    #         rx = math.radians(rx)
    #         ry = math.radians(ry)
    #         rz = math.radians(rz)

    #     qx, qy, qz, qw = quaternion_from_rpy(rx, ry, rz)
    #     pose.orientation.x = qx
    #     pose.orientation.y = qy
    #     pose.orientation.z = qz
    #     pose.orientation.w = qw
    #     return pose

    def build_pose_array(self) -> PoseArray:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        for waypoint in self._waypoints:
            msg.poses.append(self.pose_from_waypoint(waypoint))

        return msg

    def build_marker_array(self) -> MarkerArray:
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()
        point_scale = self.get_parameter("point_scale").value
        line_scale = self.get_parameter("line_scale").value
        arrow_length = self.get_parameter("arrow_length").value
        arrow_shaft_diameter = self.get_parameter("arrow_shaft_diameter").value
        arrow_head_diameter = self.get_parameter("arrow_head_diameter").value

        marker_array = MarkerArray()

        points_marker = Marker()
        points_marker.header.frame_id = frame_id
        points_marker.header.stamp = now
        points_marker.ns = "toolpath_points"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = point_scale
        points_marker.scale.y = point_scale
        points_marker.scale.z = point_scale
        points_marker.color = ColorRGBA(r=0.95, g=0.35, b=0.15, a=1.0)

        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = now
        line_marker.ns = "toolpath_line"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = line_scale
        line_marker.color = ColorRGBA(r=0.15, g=0.75, b=0.95, a=1.0)

        for waypoint in self._waypoints:
            # point = Point(x=waypoint.x, y=waypoint.y, z=waypoint.z)
            points_marker.points.append(point)
            line_marker.points.append(point)

        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)

        for index, waypoint in enumerate(self._waypoints, start=2):
            if waypoint.rx is None and waypoint.ry is None and waypoint.rz is None:
                continue

            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame_id
            arrow_marker.header.stamp = now
            arrow_marker.ns = "toolpath_orientation"
            arrow_marker.id = index
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose = self.pose_from_waypoint(waypoint)
            arrow_marker.scale.x = arrow_length
            arrow_marker.scale.y = arrow_shaft_diameter
            arrow_marker.scale.z = arrow_head_diameter
            arrow_marker.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=1.0)
            marker_array.markers.append(arrow_marker)

        return marker_array

    def _publish_messages(self) -> None:
        self._pose_array_pub.publish(self.build_pose_array())
        self._marker_array_pub.publish(self.build_marker_array())


# def main() -> None:
#     rclpy.init()
#     node = PathVisualizer()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
