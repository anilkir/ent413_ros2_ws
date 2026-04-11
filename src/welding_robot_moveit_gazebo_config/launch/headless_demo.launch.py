from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("welding_robot_bringup")
    world = LaunchConfiguration("world")
    show_camera = LaunchConfiguration("show_camera")
    camera_topic = LaunchConfiguration("camera_topic")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("welding_robot_bringup"),
                    "launch",
                    "gazebo_control_headless.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world": world,
            "show_camera": show_camera,
            "camera_topic": camera_topic,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("welding_robot_moveit_gazebo_config"), "launch", "moveit_gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [bringup_share, "worlds", "welding_camera_demo.sdf"]
                ),
            ),
            DeclareLaunchArgument("show_camera", default_value="false"),
            DeclareLaunchArgument("camera_topic", default_value="/overhead_camera/image_raw"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value="moveit.rviz"),
            gazebo_launch,
            TimerAction(period=4.0, actions=[moveit_launch]),
        ]
    )
