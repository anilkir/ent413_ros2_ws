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
    settle_after_start_sec = LaunchConfiguration("settle_after_start_sec")
    wait_for_fresh_joint_state = LaunchConfiguration("wait_for_fresh_joint_state")
    joint_state_freshness_sec = LaunchConfiguration("joint_state_freshness_sec")
    use_sim_time = LaunchConfiguration("use_sim_time")
    max_velocity_scaling_factor = LaunchConfiguration("max_velocity_scaling_factor")
    max_acceleration_scaling_factor = LaunchConfiguration("max_acceleration_scaling_factor")

    return LaunchDescription(
        [
            DeclareLaunchArgument("toolpath_name", default_value="beam_top_outer_right"),
            DeclareLaunchArgument("toolpath_frame", default_value="table_top_corner"),
            DeclareLaunchArgument("use_toolpath_surface_normal", default_value="true"),
            DeclareLaunchArgument("angles_in_degrees", default_value="false"),
            DeclareLaunchArgument("execute", default_value="true"),
            DeclareLaunchArgument("settle_after_start_sec", default_value="2.0"),
            DeclareLaunchArgument("wait_for_fresh_joint_state", default_value="true"),
            DeclareLaunchArgument("joint_state_freshness_sec", default_value="0.5"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.4"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.4"),
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
                        "settle_after_start_sec": settle_after_start_sec,
                        "wait_for_fresh_joint_state": wait_for_fresh_joint_state,
                        "joint_state_freshness_sec": joint_state_freshness_sec,
                        "use_sim_time": use_sim_time,
                        "max_velocity_scaling_factor": max_velocity_scaling_factor,
                        "max_acceleration_scaling_factor": max_acceleration_scaling_factor,
                    }
                ],
            ),
        ]
    )
