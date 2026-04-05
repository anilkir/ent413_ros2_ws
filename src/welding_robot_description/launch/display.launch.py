from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("welding_robot_description")

    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "welding_robot.xacro"]
    )

    rviz_config_file = PathJoinSubstitution(
        [pkg_share, "launch", "rviz.rviz"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str
        )
    }

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
        ),
    ])