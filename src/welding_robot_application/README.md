# welding_robot_application

Application-side package for the welding robot lab. This package provides:

- toolpath visualization in RViz
- fixed environment frame publication
- a synthetic camera stream for perception demos
- toolpath planning and execution through MoveIt

## Package structure

### Nodes

- `path_visualizer`
  Loads a CSV toolpath from `paths/` and publishes:
  `/toolpath_pose_array` as `geometry_msgs/PoseArray`
  `/toolpath_markers` as `visualization_msgs/MarkerArray`

- `frame_manager`
  Loads `config/frames.yaml` and publishes all configured static transforms with `StaticTransformBroadcaster`.

- `fake_camera_publisher`
  Publishes a synthetic overhead camera stream for demos:
  `/overhead_camera/image_raw`
  `/overhead_camera/camera_info`
  `/overhead_camera/fov_marker`

- `toolpath_executor`
  Loads a selected toolpath, chooses reachable yaw orientations when needed, moves to the first seam point, computes a Cartesian path through the rest of the seam, executes it, and then returns the robot to the `ready` posture.

### Launch files

- `application_demo.launch.py`
  Starts your MoveIt demo together with the toolpath visualizer.

- `path_visualizer.launch.py`
  Starts only the path visualizer.

- `frame_manager.launch.py`
  Starts only the frame manager.

- `toolpath_executor.launch.py`
  Starts the seam executor.

### Data files

- `paths/`
  Student-facing seam CSVs.

- `config/frames.yaml`
  Static environment frames such as `industrial_base`, `table_top_corner`, and `workpiece_frame`.

## Build and source

From the workspace root:

```bash
cd ~/ent413_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select welding_robot_application
source ~/ent413_ros2_ws/install/setup.bash
```

If you update Python source, rebuild and re-source before testing again.

## Toolpath CSV format

Minimum format:

```text
x,y,z
```

Optional local orientation columns:

```text
x,y,z,rx,ry,rz
```

Accepted aliases:

```text
x,y,z,roll,pitch,yaw
```

```text
x,y,z,a,b,c
```

Conventions:

- `x, y, z` are in meters
- `rx, ry, rz` are rotations about the local frame axes
- `a, b, c` are treated as aliases for the same three rotations
- angles are radians by default
- set `angles_in_degrees:=true` if the file uses degrees

Current toolpath files:

- [beam_top_outer_right.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_top_outer_right.csv)
- [beam_top_outer_left.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_top_outer_left.csv)
- [beam_bottom_outer_right.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_bottom_outer_right.csv)
- [beam_bottom_outer_left.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_bottom_outer_left.csv)

## Frame configuration

The frame manager always loads:

- [frames.yaml](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/config/frames.yaml)

Use it with:

```bash
ros2 run welding_robot_application frame_manager
```

or:

```bash
ros2 launch welding_robot_application frame_manager.launch.py
```

## Visualization

Run the visualizer directly:

```bash
ros2 run welding_robot_application path_visualizer --ros-args \
  -p toolpath_name:=beam_top_outer_right
```

Or via launch:

```bash
ros2 launch welding_robot_application path_visualizer.launch.py \
  toolpath_name:=beam_top_outer_right
```

Useful parameters:

- `toolpath_name`
- `frame_id`
- `angles_in_degrees`
- `publish_rate_hz`
- `point_scale`
- `line_scale`

For RViz, add:

- `MarkerArray` on `/toolpath_markers`
- `PoseArray` on `/toolpath_pose_array`

`/toolpath_markers` is usually the better display because it shows both points and the connecting line.

## Fake camera demo

Run directly:

```bash
ros2 run welding_robot_application fake_camera_publisher
```

Useful parameters:

- `width`
- `height`
- `publish_rate`

This is intended for classroom demos of image topics, camera info, and simple sensor visualization without needing a real camera.

## Combined application demo

This starts your MoveIt demo together with toolpath visualization:

```bash
ros2 launch welding_robot_application application_demo.launch.py \
  toolpath_name:=beam_top_outer_right
```

Useful arguments:

- `toolpath_name`
- `toolpath_frame`
- `angles_in_degrees`

## Toolpath executor

### What it does

The executor is intended for seam-following rather than independent point-to-point planning.

Default behavior:

1. Load the selected CSV toolpath.
2. If surface-normal mode is enabled, treat each waypoint frame `+Z` as the local surface normal.
3. Align TCP `+Z` with waypoint-frame `-Z`.
4. Sample yaw around the local `+Z` axis and use IK to choose a reachable low-cost yaw sequence across the seam.
5. Move to the first seam waypoint.
6. Compute one Cartesian path through the remaining seam waypoints.
7. Execute that Cartesian trajectory.
8. Return to the MoveIt `ready` posture.

### Typical execution

```bash
ros2 launch welding_robot_application toolpath_executor.launch.py \
  toolpath_name:=beam_top_outer_right \
  toolpath_frame:=table_top_corner \
  execute:=true
```

Plan only:

```bash
ros2 run welding_robot_application toolpath_executor --ros-args \
  -p toolpath_name:=beam_top_outer_right \
  -p planning_frame:=table_top_corner \
  -p execute:=false
```

Disable the return-to-ready motion:

```bash
ros2 run welding_robot_application toolpath_executor --ros-args \
  -p toolpath_name:=beam_top_outer_right \
  -p return_to_ready:=false
```

### Important parameters

- `toolpath_name`
  Selected seam CSV.

- `planning_frame`
  Frame in which the CSV points are interpreted. The current default is `table_top_corner`.

- `use_toolpath_surface_normal`
  If `true`, the executor derives tool orientation from the toolpath frame or CSV local frame instead of using each CSV orientation as a fully fixed pose.

- `search_reachable_yaw_sequence`
  If `true`, the executor samples yaw candidates and picks a reachable low-joint-change sequence before Cartesian planning.

- `yaw_sample_step_degrees`
  Spacing between sampled yaw candidates. Default `30.0`.

- `move_to_start_pose`
  If `true`, use normal planning to reach the first seam point before Cartesian interpolation.

- `cartesian_max_step`
  Cartesian interpolation step size in meters. Smaller values can help if Cartesian path fraction is low.

- `min_cartesian_fraction`
  Minimum acceptable Cartesian completion fraction. Default `1.0`.

- `avoid_collisions`
  Keep collision checking enabled during IK and Cartesian path generation.

- `execute`
  If `false`, stop after planning instead of executing.

- `return_to_ready`
  If `true`, perform one final MoveIt motion back to the SRDF `ready` posture after the seam completes.

### When to use CSV orientation columns

Top seams can often work with position-only CSVs.

Bottom seams currently rely on their stored local tilt in the CSV. In surface-normal mode, the executor now uses those local orientation columns as the seam frame, then samples yaw around that local frame's `+Z`.

### Common failure modes

- `No reachable yaw candidate found for waypoint 1`
  The starting seam pose is not reachable for any sampled yaw, or the local seam tilt is wrong.

- `Cartesian path only covered <fraction>`
  The straight-line seam is only partially feasible. First try reducing `cartesian_max_step`.

- Start move succeeds but the seam fails immediately
  Usually indicates a poor seam yaw choice, a singularity, or a collision-constrained segment.

## Recommended workflow for the lab

1. Build and source the workspace.
2. Start the MoveIt demo.
3. Start `frame_manager` if your environment frames are needed.
4. Run `path_visualizer` and verify the seam in RViz.
5. Run `toolpath_executor` in `execute:=false` mode first if you are testing a new seam.
6. Run again with `execute:=true` once the seam behavior looks correct.

## Relevant files

- [README.md](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/README.md)
- [setup.py](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/setup.py)
- [package.xml](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/package.xml)
- [frames.yaml](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/config/frames.yaml)
- [toolpath_executor.py](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/welding_robot_application/toolpath_executor.py)
- [path_visualizer.py](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/welding_robot_application/path_visualizer.py)
- [frame_manager.py](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/welding_robot_application/frame_manager.py)
- [fake_camera_publisher.py](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/welding_robot_application/fake_camera_publisher.py)
