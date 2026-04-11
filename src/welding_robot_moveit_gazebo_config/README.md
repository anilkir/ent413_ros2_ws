# Welding Robot MoveIt Gazebo Config

This package launches a separate MoveIt instance that executes trajectories
through the Gazebo-backed `welding_robot_arm_controller`.

It does not modify or depend on the fake-controller launch path inside
`welding_robot_moveit_config`.

`welding_robot_bringup` owns the Gazebo simulation launches. This package owns
the MoveIt configuration and the combined launches that add MoveIt / RViz on top
of the Gazebo bringup package.

## Launch Gazebo And MoveIt Together

```bash
cd /home/anilkir/ent413_ros2_ws/src
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch welding_robot_moveit_gazebo_config demo.launch.py
```

If Gazebo's 3D GUI crashes or stays grey in VirtualBox, use the headless variant
instead. This starts `welding_robot_bringup/launch/gazebo_control_headless.launch.py`
and then starts RViz / MoveIt from this package:

```bash
ros2 launch welding_robot_moveit_gazebo_config headless_demo.launch.py
```

The standalone `image_tools/showimage` viewer is disabled by default. View the
camera topics in RViz with `Image` displays instead:

- `/overhead_camera/image_raw`
- `/overhead_camera/camera_info`
- `/ee_camera/image_raw`
- `/ee_camera/camera_info`

## Launch MoveIt Against An Already-Running Gazebo Control Demo

Start Gazebo first:

```bash
ros2 launch welding_robot_bringup gazebo_control_demo.launch.py
```

Then start the Gazebo-connected MoveIt instance:

```bash
ros2 launch welding_robot_moveit_gazebo_config moveit_gazebo.launch.py
```

## Launch MoveIt Against An Already-Running Headless Gazebo Control Demo

Start headless Gazebo first:

```bash
ros2 launch welding_robot_bringup gazebo_control_headless.launch.py
```

Then start the Gazebo-connected MoveIt instance:

```bash
ros2 launch welding_robot_moveit_gazebo_config moveit_gazebo.launch.py
```
