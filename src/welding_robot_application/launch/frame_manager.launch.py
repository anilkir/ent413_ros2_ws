from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="welding_robot_application",
                executable="frame_manager",
                output="screen",
            )
        ]
    )
