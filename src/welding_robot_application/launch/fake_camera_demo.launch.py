from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    publish_rate = LaunchConfiguration("publish_rate")

    fake_camera_publisher = Node(
        package="welding_robot_application",
        executable="fake_camera_publisher",
        name="fake_camera_publisher",
        output="screen",
        parameters=[
            {
                "width": width,
                "height": height,
                "publish_rate": publish_rate,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="320"),
            DeclareLaunchArgument("height", default_value="240"),
            DeclareLaunchArgument("publish_rate", default_value="5.0"),
            fake_camera_publisher,
        ]
    )
