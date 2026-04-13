# ENT413 Welding Robot ROS 2 Workspace

This repository is a ROS 2 Jazzy workspace for the ENT413 welding robot labs. It combines robot description, MoveIt planning, Gazebo simulation, and application-side tooling for CSV-based seam visualization and execution.

The workspace is organized as a normal `colcon` layout:

- `src/` contains the ROS 2 packages
- `build/`, `install/`, and `log/` are generated workspace artifacts

## Branches

- `main` is the primary working branch for the lab repository
- `solution` contains the corresponding completed solution state

This README is intentionally kept branch-neutral so the same top-level orientation applies in both places.

## Packages

- `franka_description`: upstream Franka robot meshes, xacros, and support files used by the FR3-based stack
- `lab_basics`: introductory Python ROS 2 package for basic lab exercises
- `welding_robot_description`: welding robot URDF/xacro description built around the Franka FR3 arm
- `welding_robot_moveit_config`: MoveIt and RViz configuration for local planning with fake or mock control
- `welding_robot_bringup`: Gazebo bringup, simulated sensors, and Gazebo-backed controller launches
- `welding_robot_moveit_gazebo_config`: MoveIt configuration that plans and executes against the Gazebo controllers
- `welding_robot_application`: application-side tools for frame publication, toolpath visualization, fake camera demos, and seam execution

## Recommended Flow

For a typical teaching or demo sequence:

1. Start with `welding_robot_description` to explain the robot model, links, joints, and frames.
2. Use `welding_robot_moveit_config` for MoveIt planning in RViz.
3. Use `welding_robot_bringup` for Gazebo simulation, cameras, and simulated control.
4. Use `welding_robot_moveit_gazebo_config` when you want MoveIt to execute through the Gazebo-backed controllers.
5. Use `welding_robot_application` for CSV toolpaths, static environment frames, and seam-following demos.

## Build

From the workspace root:

```bash
cd ~/ent413_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source ~/ent413_ros2_ws/install/setup.bash
```

To rebuild only one package while iterating:

```bash
colcon build --packages-select welding_robot_application
source ~/ent413_ros2_ws/install/setup.bash
```

## Common Entry Points

Visualize the robot description:

```bash
ros2 launch welding_robot_description display.launch.py
```

Launch MoveIt in RViz:

```bash
ros2 launch welding_robot_moveit_config demo.launch.py
```

Launch Gazebo with the welding cell cameras:

```bash
ros2 launch welding_robot_bringup gazebo_camera_demo.launch.py
```

Launch MoveIt against Gazebo-backed control:

```bash
ros2 launch welding_robot_moveit_gazebo_config demo.launch.py
```

Launch the application demo with toolpath visualization:

```bash
ros2 launch welding_robot_application application_demo.launch.py \
  toolpath_name:=beam_top_outer_right
```

## Package Documentation

More detailed package-level instructions live in:

- `src/welding_robot_application/README.md`
- `src/welding_robot_bringup/README.md`
- `src/welding_robot_moveit_config/README.md`
- `src/welding_robot_moveit_gazebo_config/README.md`
- `src/franka_description/README.md`
