from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    moveit_config = (
        MoveItConfigsBuilder("welding_robot")
        .robot_description(file_path="config/welding_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/welding_robot.srdf")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # RViz
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_full_config = PathJoinSubstitution(
        [FindPackageShare("welding_robot_moveit_config"), "launch", "moveit.rviz"]
    )
    rviz_empty_config = PathJoinSubstitution(
        [FindPackageShare("welding_robot_moveit_config"), "launch", "moveit_empty.rviz"]
    )
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(tutorial_mode),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    return LaunchDescription(
        [
            tutorial_arg,
            rviz_node,
            rviz_node_tutorial,
        ]
    )
