from setuptools import find_packages, setup


package_name = "welding_robot_application"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/application_demo.launch.py",
                "launch/frame_manager.launch.py",
                "launch/path_visualizer.launch.py",
                "launch/toolpath_executor.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/paths",
            [
                "paths/beam_top_outer_right.csv",
                "paths/beam_top_outer_left.csv",
                "paths/beam_bottom_outer_right.csv",
                "paths/beam_bottom_outer_left.csv",
            ],
        ),
        ("share/" + package_name + "/config", ["config/frames.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anilkir",
    maintainer_email="anilkir@example.com",
    description="Application package for CSV path planning and industrial base-frame publication.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_camera_publisher = welding_robot_application.fake_camera_publisher:main",
            "path_visualizer = welding_robot_application.path_visualizer:main",
            "frame_manager = welding_robot_application.frame_manager:main",
            "toolpath_executor = welding_robot_application.toolpath_executor:main",
        ],
    },
)
