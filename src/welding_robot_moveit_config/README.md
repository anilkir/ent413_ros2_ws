## Welding Robot MoveIt Config

Manual MoveIt configuration for the `welding_robot_description` package.

This package uses the FR3 arm from `franka_description`, a fixed torch TCP named `welding_tcp`, and a fake `ros2_control` joint trajectory controller for local planning and RViz testing.

For the separate Gazebo teaching path, use `welding_robot_bringup`.

- `welding_robot_moveit_config`: RViz and MoveIt planning with fake or mock control
- `welding_robot_bringup`: Gazebo world, simulated camera, and Gazebo-based controllers
