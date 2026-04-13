import csv
import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    rx: Optional[float] = None
    ry: Optional[float] = None
    rz: Optional[float] = None


@dataclass
class YawCandidate:
    yaw_deg: float
    pose: Pose
    state: RobotState


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


def quaternion_multiply(
    q1: tuple[float, float, float, float], q2: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ToolpathExecutor(Node):
    def __init__(self) -> None:
        super().__init__("toolpath_executor")

        self.declare_parameter("toolpath_name", "beam_top_outer_right")
        self.declare_parameter("planning_frame", "table_top_corner")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("target_link", "welding_tcp")
        self.declare_parameter("use_toolpath_surface_normal", True)
        self.declare_parameter("search_reachable_yaw_sequence", True)
        self.declare_parameter("yaw_sample_step_degrees", 30.0)
        self.declare_parameter("ik_timeout_sec", 0.05)
        self.declare_parameter("move_to_start_pose", True)
        self.declare_parameter("settle_after_start_sec", 0.0)
        self.declare_parameter("wait_for_fresh_joint_state", True)
        self.declare_parameter("joint_state_freshness_sec", 0.5)
        self.declare_parameter("angles_in_degrees", False)
        self.declare_parameter("default_rx", 0.0)
        self.declare_parameter("default_ry", 0.0)
        self.declare_parameter("default_rz", 0.0)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("max_velocity_scaling_factor", 0.4)
        self.declare_parameter("max_acceleration_scaling_factor", 0.4)
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("jump_threshold", 0.0)
        self.declare_parameter("prismatic_jump_threshold", 0.0)
        self.declare_parameter("revolute_jump_threshold", 0.0)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("min_cartesian_fraction", 1.0)
        self.declare_parameter("orientation_tolerance_x", 0.02)
        self.declare_parameter("orientation_tolerance_y", 0.02)
        self.declare_parameter("orientation_tolerance_z", math.pi)
        self.declare_parameter("execute", True)
        self.declare_parameter("return_to_ready", True)

        self._move_group_client = ActionClient(self, MoveGroup, "move_action")
        self._cartesian_path_client = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self._ik_client = self.create_client(GetPositionIK, "compute_ik")
        self._last_joint_state: Optional[JointState] = None
        self._last_joint_state_wall_time = 0.0
        self._joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self._on_joint_state,
            10,
        )
        self._ready_joint_targets = {
            "fr3_joint1": 0.0,
            "fr3_joint2": -0.7853981633974483,
            "fr3_joint3": 0.0,
            "fr3_joint4": -2.356194490192345,
            "fr3_joint5": 0.0,
            "fr3_joint6": 1.5707963267948966,
            "fr3_joint7": 0.7853981633974483,
        }

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg
        self._last_joint_state_wall_time = time.monotonic()

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
    def _optional_angle(row: dict[str, str], *keys: str) -> Optional[float]:
        value = None
        for key in keys:
            value = row.get(key)
            if value not in (None, ""):
                break
        return float(value) if value not in (None, "") else None

    def waypoint_pose(self, waypoint: Waypoint) -> Pose:
        pose = Pose()
        pose.position.x = waypoint.x
        pose.position.y = waypoint.y
        pose.position.z = waypoint.z

        if self.get_parameter("use_toolpath_surface_normal").get_parameter_value().bool_value:
            qx, qy, qz, qw = self.surface_normal_quaternion(waypoint, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            return pose

        rx = waypoint.rx if waypoint.rx is not None else self.get_parameter("default_rx").value
        ry = waypoint.ry if waypoint.ry is not None else self.get_parameter("default_ry").value
        rz = waypoint.rz if waypoint.rz is not None else self.get_parameter("default_rz").value

        if self.get_parameter("angles_in_degrees").get_parameter_value().bool_value:
            rx = math.radians(rx)
            ry = math.radians(ry)
            rz = math.radians(rz)

        qx, qy, qz, qw = quaternion_from_rpy(rx, ry, rz)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def waypoint_base_quaternion(self, waypoint: Waypoint) -> tuple[float, float, float, float]:
        # If the CSV already provides a local frame tilt, use it as the seam frame.
        # Otherwise fall back to a flat toolpath frame whose +Z is the surface normal.
        rx = waypoint.rx if waypoint.rx is not None else self.get_parameter("default_rx").value
        ry = waypoint.ry if waypoint.ry is not None else self.get_parameter("default_ry").value
        rz = waypoint.rz if waypoint.rz is not None else self.get_parameter("default_rz").value

        if self.get_parameter("angles_in_degrees").get_parameter_value().bool_value:
            rx = math.radians(rx)
            ry = math.radians(ry)
            rz = math.radians(rz)

        return quaternion_from_rpy(rx, ry, rz)

    def surface_normal_quaternion(self, waypoint: Waypoint, yaw_deg: float) -> tuple[float, float, float, float]:
        # The waypoint frame +Z is the local surface normal.
        # Align TCP +Z with waypoint-frame -Z, then sample yaw about waypoint-frame +Z.
        waypoint_frame = self.waypoint_base_quaternion(waypoint)
        tool_alignment = quaternion_from_rpy(math.pi, 0.0, 0.0)
        yaw_about_local_z = quaternion_from_rpy(0.0, 0.0, math.radians(yaw_deg))
        return quaternion_multiply(quaternion_multiply(waypoint_frame, yaw_about_local_z), tool_alignment)

    def position_constraint_from_pose(self, pose: Pose) -> PositionConstraint:
        planning_frame = self.get_parameter("planning_frame").get_parameter_value().string_value
        target_link = self.get_parameter("target_link").get_parameter_value().string_value

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.get_parameter("cartesian_max_step").value]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = planning_frame
        position_constraint.link_name = target_link
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(pose)
        position_constraint.weight = 1.0
        return position_constraint

    def orientation_constraint_from_pose(self, pose: Pose, free_yaw: bool = True) -> OrientationConstraint:
        planning_frame = self.get_parameter("planning_frame").get_parameter_value().string_value
        target_link = self.get_parameter("target_link").get_parameter_value().string_value

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = planning_frame
        orientation_constraint.link_name = target_link
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.get_parameter("orientation_tolerance_x").value
        orientation_constraint.absolute_y_axis_tolerance = self.get_parameter("orientation_tolerance_y").value
        orientation_constraint.absolute_z_axis_tolerance = (
            self.get_parameter("orientation_tolerance_z").value if free_yaw
            else self.get_parameter("orientation_tolerance_y").value
        )
        orientation_constraint.weight = 1.0
        return orientation_constraint

    def make_path_constraints(self, poses: List[Pose]) -> Constraints:
        constraints = Constraints()
        if poses:
            constraints.orientation_constraints.append(self.orientation_constraint_from_pose(poses[0], free_yaw=True))
        return constraints

    def make_move_group_goal(self, pose: Pose) -> MoveGroup.Goal:
        request = MoveGroup.Goal()
        request.request.group_name = self.get_parameter("planning_group").get_parameter_value().string_value
        request.request.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        request.request.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        request.request.pipeline_id = self.get_parameter("pipeline_id").get_parameter_value().string_value
        request.request.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        request.request.max_velocity_scaling_factor = self.get_parameter("max_velocity_scaling_factor").value
        request.request.max_acceleration_scaling_factor = self.get_parameter("max_acceleration_scaling_factor").value
        request.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.position_constraints.append(self.position_constraint_from_pose(pose))
        constraints.orientation_constraints.append(self.orientation_constraint_from_pose(pose, free_yaw=False))
        request.request.goal_constraints.append(constraints)

        request.planning_options.plan_only = not self.get_parameter("execute").get_parameter_value().bool_value
        request.planning_options.look_around = False
        request.planning_options.replan = False
        return request

    def make_ready_goal(self) -> MoveGroup.Goal:
        request = MoveGroup.Goal()
        request.request.group_name = self.get_parameter("planning_group").get_parameter_value().string_value
        request.request.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        request.request.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        request.request.pipeline_id = self.get_parameter("pipeline_id").get_parameter_value().string_value
        request.request.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        request.request.max_velocity_scaling_factor = self.get_parameter("max_velocity_scaling_factor").value
        request.request.max_acceleration_scaling_factor = self.get_parameter("max_acceleration_scaling_factor").value
        request.request.start_state.is_diff = True

        constraints = Constraints()
        for joint_name, joint_value in self._ready_joint_targets.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_value
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        request.request.goal_constraints.append(constraints)

        request.planning_options.plan_only = not self.get_parameter("execute").get_parameter_value().bool_value
        request.planning_options.look_around = False
        request.planning_options.replan = False
        return request

    def move_to_start_pose(self, pose: Pose) -> bool:
        self.get_logger().info("Waiting for MoveIt move_action server for start pose move...")
        self._move_group_client.wait_for_server()

        goal = self.make_move_group_goal(pose)
        send_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Start pose MoveGroup goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error("Start pose MoveGroup goal did not return a result")
            return False

        result = action_result.result
        if result.error_code.val != 1:
            self.get_logger().error(
                f"Failed to move to start pose with code {result.error_code.val}: "
                f"{result.error_code.message}"
            )
            return False

        self.get_logger().info("Reached toolpath start pose")
        return True

    def move_to_ready_pose(self) -> bool:
        self.get_logger().info("Returning to ready joint posture")
        self._move_group_client.wait_for_server()

        goal = self.make_ready_goal()
        send_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Ready-pose MoveGroup goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error("Ready-pose MoveGroup goal did not return a result")
            return False

        result = action_result.result
        if result.error_code.val != 1:
            self.get_logger().error(
                f"Failed to return to ready pose with code {result.error_code.val}: "
                f"{result.error_code.message}"
            )
            return False

        self.get_logger().info("Returned to ready joint posture")
        return True

    def wait_for_state_sync(self, context: str) -> None:
        settle_after_start_sec = self.get_parameter("settle_after_start_sec").value
        wait_for_fresh_joint_state = self.get_parameter("wait_for_fresh_joint_state").get_parameter_value().bool_value

        if settle_after_start_sec > 0.0:
            self.get_logger().info(
                f"Waiting {settle_after_start_sec:.2f}s for Gazebo/controller state to settle before {context}"
            )
            time.sleep(settle_after_start_sec)
        if wait_for_fresh_joint_state:
            self.get_logger().info(f"Waiting for a fresh joint state before {context}")
            self.wait_for_fresh_joint_state()

    def make_cartesian_request(self, poses: List[Pose]) -> GetCartesianPath.Request:
        request = GetCartesianPath.Request()
        request.header.frame_id = self.get_parameter("planning_frame").get_parameter_value().string_value
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.group_name = self.get_parameter("planning_group").get_parameter_value().string_value
        request.link_name = self.get_parameter("target_link").get_parameter_value().string_value
        request.waypoints = poses
        request.max_step = self.get_parameter("cartesian_max_step").value
        request.jump_threshold = self.get_parameter("jump_threshold").value
        request.prismatic_jump_threshold = self.get_parameter("prismatic_jump_threshold").value
        request.revolute_jump_threshold = self.get_parameter("revolute_jump_threshold").value
        request.avoid_collisions = self.get_parameter("avoid_collisions").get_parameter_value().bool_value
        request.path_constraints = self.make_path_constraints(poses)
        request.max_velocity_scaling_factor = self.get_parameter("max_velocity_scaling_factor").value
        request.max_acceleration_scaling_factor = self.get_parameter("max_acceleration_scaling_factor").value
        request.cartesian_speed_limited_link = self.get_parameter("target_link").get_parameter_value().string_value
        request.max_cartesian_speed = 0.0
        return request

    def make_ik_request(self, pose: Pose, seed_state: Optional[RobotState]) -> GetPositionIK.Request:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.get_parameter("planning_group").get_parameter_value().string_value
        request.ik_request.robot_state = seed_state if seed_state is not None else RobotState()
        if seed_state is None:
            request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = self.get_parameter("avoid_collisions").get_parameter_value().bool_value
        request.ik_request.ik_link_name = self.get_parameter("target_link").get_parameter_value().string_value

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.get_parameter("planning_frame").get_parameter_value().string_value
        pose_stamped.pose = pose
        request.ik_request.pose_stamped = pose_stamped

        timeout = Duration()
        ik_timeout = self.get_parameter("ik_timeout_sec").value
        timeout.sec = int(ik_timeout)
        timeout.nanosec = int((ik_timeout - timeout.sec) * 1_000_000_000)
        request.ik_request.timeout = timeout
        return request

    def solve_ik(self, pose: Pose, seed_state: Optional[RobotState]) -> Optional[RobotState]:
        request = self.make_ik_request(pose, seed_state)
        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or response.error_code.val != 1:
            return None
        return response.solution

    @staticmethod
    def joint_cost(state_a: RobotState, state_b: RobotState) -> float:
        names_a = list(state_a.joint_state.name)
        pos_a = list(state_a.joint_state.position)
        pos_b_map = dict(zip(state_b.joint_state.name, state_b.joint_state.position))
        cost = 0.0
        for name, value_a in zip(names_a, pos_a):
            value_b = pos_b_map.get(name)
            if value_b is None:
                continue
            cost += abs(wrap_to_pi(value_b - value_a))
        return cost

    def choose_reachable_pose_sequence(self, waypoints: List[Waypoint]) -> Optional[List[Pose]]:
        yaw_step = self.get_parameter("yaw_sample_step_degrees").value
        if yaw_step <= 0.0:
            self.get_logger().error("yaw_sample_step_degrees must be > 0")
            return None

        yaw_values: List[float] = []
        yaw = 0.0
        while yaw < 360.0 - 1e-6:
            yaw_values.append(yaw)
            yaw += yaw_step

        candidate_layers: List[List[YawCandidate]] = []
        self.get_logger().info(
            f"Searching reachable yaw sequence with {len(yaw_values)} yaw sample(s) per waypoint "
            f"at {yaw_step:.1f} deg spacing"
        )

        for index, waypoint in enumerate(waypoints, start=1):
            layer: List[YawCandidate] = []
            for yaw_deg in yaw_values:
                pose = Pose()
                pose.position.x = waypoint.x
                pose.position.y = waypoint.y
                pose.position.z = waypoint.z
                qx, qy, qz, qw = self.surface_normal_quaternion(waypoint, yaw_deg)
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw

                seed_states: List[Optional[RobotState]] = [None]
                if index > 1 and candidate_layers[index - 2]:
                    previous_layer = candidate_layers[index - 2]
                    previous_sorted = sorted(
                        previous_layer,
                        key=lambda candidate: abs(wrap_to_pi(math.radians(candidate.yaw_deg - yaw_deg))),
                    )
                    seed_states = [candidate.state for candidate in previous_sorted]

                solution = None
                for seed_state in seed_states:
                    solution = self.solve_ik(pose, seed_state)
                    if solution is not None:
                        break
                if solution is not None:
                    layer.append(YawCandidate(yaw_deg=yaw_deg, pose=pose, state=solution))

            if not layer:
                self.get_logger().error(f"No reachable yaw candidate found for waypoint {index}")
                return None

            self.get_logger().info(
                f"Waypoint {index}: {len(layer)}/{len(yaw_values)} yaw candidate(s) are reachable"
            )
            candidate_layers.append(layer)

        costs: List[List[float]] = [[math.inf] * len(layer) for layer in candidate_layers]
        parents: List[List[int]] = [[-1] * len(layer) for layer in candidate_layers]

        for j, candidate in enumerate(candidate_layers[0]):
            costs[0][j] = 0.0

        for i in range(1, len(candidate_layers)):
            for j, candidate in enumerate(candidate_layers[i]):
                for k, previous in enumerate(candidate_layers[i - 1]):
                    transition_cost = self.joint_cost(previous.state, candidate.state)
                    total_cost = costs[i - 1][k] + transition_cost
                    if total_cost < costs[i][j]:
                        costs[i][j] = total_cost
                        parents[i][j] = k

        last_index = min(range(len(candidate_layers[-1])), key=lambda idx: costs[-1][idx])
        if math.isinf(costs[-1][last_index]):
            self.get_logger().error("Could not find a connected reachable yaw sequence across the toolpath")
            return None

        selected_indices = [last_index]
        for i in range(len(candidate_layers) - 1, 0, -1):
            selected_indices.append(parents[i][selected_indices[-1]])
        selected_indices.reverse()

        selected_poses: List[Pose] = []
        selected_yaws: List[float] = []
        for layer, selected_index in zip(candidate_layers, selected_indices):
            selected = layer[selected_index]
            selected_poses.append(selected.pose)
            selected_yaws.append(selected.yaw_deg)

        self.get_logger().info(
            "Selected seam yaw sequence (deg): " + ", ".join(f"{yaw:.0f}" for yaw in selected_yaws)
        )
        return selected_poses

    def execute_trajectory(self, trajectory) -> bool:
        self.get_logger().info("Waiting for MoveIt execute_trajectory action...")
        self._execute_trajectory_client.wait_for_server()

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error("ExecuteTrajectory did not return a result")
            return False

        result = action_result.result
        if result.error_code.val != 1:
            self.get_logger().error(
                f"Trajectory execution failed with code {result.error_code.val}: "
                f"{result.error_code.message}"
            )
            return False

        self.get_logger().info("Cartesian trajectory executed successfully")
        return True

    def wait_for_fresh_joint_state(self) -> bool:
        freshness_sec = self.get_parameter("joint_state_freshness_sec").value
        deadline = time.monotonic() + max(1.0, freshness_sec + 1.0)

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_joint_state is None:
                continue
            age = time.monotonic() - self._last_joint_state_wall_time
            if age <= freshness_sec:
                self.get_logger().info(f"Fresh joint state received {age:.3f}s ago")
                return True

        self.get_logger().warn("Did not receive a fresh joint state before Cartesian planning")
        return False

    def run(self) -> None:
        waypoints = self.load_waypoints()
        use_surface_normal = self.get_parameter("use_toolpath_surface_normal").get_parameter_value().bool_value
        search_reachable_yaw = self.get_parameter("search_reachable_yaw_sequence").get_parameter_value().bool_value
        if use_surface_normal and search_reachable_yaw:
            self.get_logger().info("Waiting for MoveIt compute_ik service...")
            self._ik_client.wait_for_service()
            self.get_logger().info("Connected to compute_ik")
            poses = self.choose_reachable_pose_sequence(waypoints)
            if poses is None:
                return
        else:
            poses = [self.waypoint_pose(waypoint) for waypoint in waypoints]
        move_to_start = self.get_parameter("move_to_start_pose").get_parameter_value().bool_value

        self.get_logger().info("Waiting for MoveIt compute_cartesian_path service...")
        self._cartesian_path_client.wait_for_service()
        self.get_logger().info("Connected to compute_cartesian_path")
        if use_surface_normal:
            self.get_logger().info(
                "Using toolpath surface-normal mode: TCP +Z aligned with planning-frame -Z and yaw left free"
            )

        cartesian_poses = poses
        if move_to_start and poses:
            self.get_logger().info("Moving to first toolpath waypoint before Cartesian execution")
            if not self.move_to_start_pose(poses[0]):
                return
            self.wait_for_state_sync("Cartesian planning")
            cartesian_poses = poses[1:]

        if not cartesian_poses:
            self.get_logger().info("Toolpath contains only the start pose; nothing left for Cartesian interpolation")
            return

        request = self.make_cartesian_request(cartesian_poses)
        self.get_logger().info(
            f"Requesting one Cartesian path through {len(cartesian_poses)} waypoint(s) with max_step="
            f"{request.max_step:.4f}"
        )

        response_future = self._cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, response_future)
        response = response_future.result()
        if response is None:
            self.get_logger().error("compute_cartesian_path did not return a response")
            return

        if response.error_code.val != 1:
            self.get_logger().error(
                f"Cartesian path planning failed with code {response.error_code.val}: "
                f"{response.error_code.message}"
            )
            return

        min_fraction = self.get_parameter("min_cartesian_fraction").value
        self.get_logger().info(f"Cartesian path fraction: {response.fraction:.3f}")
        if response.fraction < min_fraction:
            self.get_logger().error(
                f"Cartesian path only covered {response.fraction:.3f}, below required {min_fraction:.3f}"
            )
            return

        num_points = len(response.solution.joint_trajectory.points)
        self.get_logger().info(f"Cartesian path planned successfully with {num_points} trajectory point(s)")

        if not self.get_parameter("execute").get_parameter_value().bool_value:
            self.get_logger().info("Execution disabled; stopping after Cartesian planning")
            return

        if self.execute_trajectory(response.solution):
            self.get_logger().info("Finished Cartesian toolpath execution")
            if self.get_parameter("return_to_ready").get_parameter_value().bool_value:
                self.wait_for_state_sync("return-to-ready planning")
                self.move_to_ready_pose()


def main() -> None:
    rclpy.init()
    node = ToolpathExecutor()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
