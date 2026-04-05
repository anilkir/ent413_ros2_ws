from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    toolpath_name = LaunchConfiguration("toolpath_name")
    toolpath_frame = LaunchConfiguration("toolpath_frame")
    angles_in_degrees = LaunchConfiguration("angles_in_degrees")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("welding_robot_moveit_config"), "launch", "demo.launch.py"]
            )
        )
    )

    csv_planner = Node(
        package="welding_robot_application",
        executable="path_visualizer",
        output="screen",
        parameters=[
            {
                "toolpath_name": toolpath_name,
                "frame_id": toolpath_frame,
                "angles_in_degrees": angles_in_degrees,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("toolpath_name", default_value="beam_top_outer_right"),
            DeclareLaunchArgument("toolpath_frame", default_value="table_top_corner"),
            DeclareLaunchArgument("angles_in_degrees", default_value="false"),
            moveit_demo,
            csv_planner,
        ]
    )
