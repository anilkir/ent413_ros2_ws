import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("welding_robot_bringup")
    description_share = FindPackageShare("welding_robot_description")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    bringup_share_root = os.path.join(get_package_prefix("welding_robot_bringup"), "share")
    description_share_root = os.path.join(
        get_package_prefix("welding_robot_description"), "share"
    )
    franka_share_root = os.path.join(get_package_prefix("franka_description"), "share")

    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    qt_platform = LaunchConfiguration("qt_platform")
    software_rendering = LaunchConfiguration("software_rendering")
    show_camera = LaunchConfiguration("show_camera")
    camera_topic = LaunchConfiguration("camera_topic")

    xacro_file = PathJoinSubstitution([bringup_share, "urdf", "welding_robot_gazebo.xacro"])
    default_world = PathJoinSubstitution([bringup_share, "worlds", "welding_camera_demo.sdf"])
    gui_config = PathJoinSubstitution([bringup_share, "config", "gazebo_gui.config"])
    rviz_config = PathJoinSubstitution([description_share, "launch", "rviz.rviz"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                world,
                " --gui-config ",
                gui_config,
                " --render-engine-gui ogre -r",
            ]
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("qt_platform", default_value="xcb"),
            DeclareLaunchArgument("software_rendering", default_value="true"),
            DeclareLaunchArgument("show_camera", default_value="false"),
            DeclareLaunchArgument("camera_topic", default_value="/overhead_camera/image_raw"),
            SetEnvironmentVariable("QT_QPA_PLATFORM", qt_platform),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                "1",
                condition=IfCondition(software_rendering),
            ),
            SetEnvironmentVariable(
                "QT_OPENGL",
                "software",
                condition=IfCondition(software_rendering),
            ),
            SetEnvironmentVariable(
                "QT_QUICK_BACKEND",
                "software",
                condition=IfCondition(software_rendering),
            ),
            SetEnvironmentVariable(
                "MESA_LOADER_DRIVER_OVERRIDE",
                "llvmpipe",
                condition=IfCondition(software_rendering),
            ),
            SetEnvironmentVariable(
                "GALLIUM_DRIVER",
                "llvmpipe",
                condition=IfCondition(software_rendering),
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [bringup_share_root, ":", description_share_root, ":", franka_share_root],
            ),
            gazebo,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=["-name", "welding_robot", "-topic", "robot_description"],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    "/ee_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/ee_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
            ),
            Node(
                package="image_tools",
                executable="showimage",
                name="overhead_camera_viewer",
                output="screen",
                remappings=[("image", camera_topic)],
                condition=IfCondition(show_camera),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
