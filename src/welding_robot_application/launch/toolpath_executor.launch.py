from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    toolpath_name = LaunchConfiguration("toolpath_name")
    toolpath_frame = LaunchConfiguration("toolpath_frame")
    use_toolpath_surface_normal = LaunchConfiguration("use_toolpath_surface_normal")
    angles_in_degrees = LaunchConfiguration("angles_in_degrees")
    execute = LaunchConfiguration("execute")

    return LaunchDescription(
        [
            DeclareLaunchArgument("toolpath_name", default_value="beam_top_outer_right"),
            DeclareLaunchArgument("toolpath_frame", default_value="table_top_corner"),
            DeclareLaunchArgument("use_toolpath_surface_normal", default_value="true"),
            DeclareLaunchArgument("angles_in_degrees", default_value="false"),
            DeclareLaunchArgument("execute", default_value="true"),
            Node(
                package="welding_robot_application",
                executable="toolpath_executor",
                output="screen",
                parameters=[
                    {
                        "toolpath_name": toolpath_name,
                        "planning_frame": toolpath_frame,
                        "use_toolpath_surface_normal": use_toolpath_surface_normal,
                        "angles_in_degrees": angles_in_degrees,
                        "execute": execute,
                    }
                ],
            ),
        ]
    )
