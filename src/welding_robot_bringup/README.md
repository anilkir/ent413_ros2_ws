# Welding Robot Gazebo Bringup

This package owns the Gazebo side of the welding robot stack.

Use it when you want to show:

- the robot inside a Gazebo world
- the simulated cameras
- controllers running inside Gazebo through `gz_ros2_control`
- headless Gazebo simulation for VM-safe use

Use `welding_robot_moveit_gazebo_config` when you want MoveIt to plan and
execute against the Gazebo controllers. This package still owns the Gazebo
launches, worlds, robot simulation xacros, sensors, and controller setup.

## Run Gazebo Camera Demo

This demo starts Gazebo with the welding cell and camera bridges.

```bash
cd /home/anilkir/ent413_ros2_ws/src
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch welding_robot_bringup gazebo_camera_demo.launch.py
```

If the Gazebo 3D window is blank or all grey in VirtualBox, keep the default
`software_rendering:=true`. On a machine with working GPU acceleration, you can
switch back to hardware rendering with:

```bash
ros2 launch welding_robot_bringup gazebo_camera_demo.launch.py software_rendering:=false
```

The default standalone `image_tools/showimage` window is disabled. View the
camera topics in RViz with `Image` displays instead.

The camera topics are:

- `/overhead_camera/image_raw`
- `/overhead_camera/camera_info`
- `/ee_camera/image_raw`
- `/ee_camera/camera_info`

## Run Gazebo Control Demo

This demo starts Gazebo with `gz_ros2_control` and loads the arm controllers inside Gazebo.

```bash
cd /home/anilkir/ent413_ros2_ws/src
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch welding_robot_bringup gazebo_control_demo.launch.py
```

This launch file also defaults to `software_rendering:=true` for VirtualBox and
other environments with unreliable OpenGL acceleration.

## Run Headless Gazebo Control Demo

Use this when Gazebo's GUI is broken in VirtualBox or you only need the real
Gazebo controllers and camera topics.

```bash
cd /home/anilkir/ent413_ros2_ws/src
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch welding_robot_bringup gazebo_control_headless.launch.py
```

This launch still provides:

- Gazebo physics
- `gz_ros2_control` controllers
- `/overhead_camera/*` topics
- `/ee_camera/*` topics

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
- The camera images are rendered in simulation and bridged back into ROS 2.
- In `gazebo_control_demo.launch.py`, the joint controllers live inside Gazebo through `gz_ros2_control`.
- In `gazebo_control_headless.launch.py`, the same controllers and camera topics run without the Gazebo GUI.
- This path is better for showing how simulated sensors and simulated controllers behave in a world.

Use `welding_robot_moveit_gazebo_config` when teaching planning against the real
Gazebo-backed controllers.

- It provides a separate MoveIt instance that does not touch `welding_robot_moveit_config`.
- It layers MoveIt and RViz on top of the Gazebo launches from this package.
- Its `headless_demo.launch.py` uses `welding_robot_bringup/gazebo_control_headless.launch.py`.

## Recommended Teaching Order

1. Start with `welding_robot_description` to explain links, joints, and frames.
2. Use `welding_robot_moveit_config` to teach RViz and MoveIt planning.
3. Use `welding_robot_bringup` as a separate module to teach Gazebo, cameras, and simulation controllers.
4. Use `welding_robot_moveit_gazebo_config` to teach planning and execution against the Gazebo controllers.
