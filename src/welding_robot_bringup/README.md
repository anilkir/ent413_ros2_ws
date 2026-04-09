# Welding Robot Gazebo Bringup

This package is the separate Gazebo teaching path for the welding robot.

Use it when you want to show:

- the robot inside a Gazebo world
- the simulated overhead camera
- controllers running inside Gazebo through `gz_ros2_control`

Do not use this package for the RViz and MoveIt planning lesson. That remains in `welding_robot_moveit_config`.

## Run Gazebo Camera Demo

This demo starts Gazebo with the welding cell and overhead camera bridge.

```bash
cd /home/anilkir/ent413_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch welding_robot_bringup gazebo_camera_demo.launch.py
```

The overhead camera topics are:

- `/overhead_camera/image_raw`
- `/overhead_camera/camera_info`

## Run Gazebo Control Demo

This demo starts Gazebo with `gz_ros2_control` and loads the arm controllers inside Gazebo.

```bash
cd /home/anilkir/ent413_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch welding_robot_bringup gazebo_control_demo.launch.py
```

If old Gazebo or ROS processes are still running, clear them first:

```bash
pkill -f gz
pkill -f controller_manager
pkill -f robot_state_publisher
pkill -f parameter_bridge
pkill -f 'ros2 launch'
```

## Gazebo Vs MoveIt

Use `welding_robot_moveit_config` when teaching motion planning.

- RViz + MoveIt is mainly kinematic.
- The robot state comes from fake or mock `ros2_control`.
- It is the cleaner path for planning, frames, groups, and trajectories.
- It is the better first lesson because it has fewer moving parts.

Use `welding_robot_bringup` when teaching simulation and sensors.

- Gazebo adds a 3D world, rendering, sensors, and physics.
- The camera image is rendered in simulation and bridged back into ROS 2.
- In `gazebo_control_demo.launch.py`, the joint controllers live inside Gazebo through `gz_ros2_control`.
- This path is better for showing how simulated sensors and simulated controllers behave in a world.

## Recommended Teaching Order

1. Start with `welding_robot_description` to explain links, joints, and frames.
2. Use `welding_robot_moveit_config` to teach RViz and MoveIt planning.
3. Use `welding_robot_bringup` as a separate module to teach Gazebo, cameras, and simulation controllers.
