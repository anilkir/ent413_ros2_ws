from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # toolpath_name = LaunchConfiguration("toolpath_name")
    # toolpath_frame = LaunchConfiguration("toolpath_frame")
    angles_in_degrees = LaunchConfiguration("angles_in_degrees")

    return LaunchDescription(
        [
            # DeclareLaunchArgument("toolpath_name", default_value="beam_top_outer_right"),
            # DeclareLaunchArgument("toolpath_frame", default_value="table_top_corner"),
            DeclareLaunchArgument("angles_in_degrees", default_value="false"),
            Node(
                package="welding_robot_application",
                executable="path_visualizer",
                output="screen",
                parameters=[
                    {
                        # "toolpath_name": toolpath_name,
                        # "frame_id": toolpath_frame,
                        "angles_in_degrees": angles_in_degrees,
                    }
                ],
            ),
        ]
    )
