# welding_robot_application

This package is the application-side package for classroom demos around the welding robot. It currently focuses on two things:

- visualizing toolpaths stored in CSV files
- publishing fixed environment frames from a single YAML file

## Main Parts

- `welding_robot_application/path_visualizer.py`
  Reads a toolpath CSV and publishes RViz displays.
  Topics:
  `/toolpath_pose_array` as `geometry_msgs/PoseArray`
  `/toolpath_markers` as `visualization_msgs/MarkerArray`

- `welding_robot_application/frame_manager.py`
  Publishes all static TF frames defined in `config/frames.yaml` using `StaticTransformBroadcaster`.

- `paths/`
  Stores student-facing toolpath CSV files.
  Example:
  [beam_top_outer_right.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_top_outer_right.csv)
  Additional seam files:
  [beam_top_outer_left.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_top_outer_left.csv)
  [beam_bottom_outer_right.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_bottom_outer_right.csv)
  [beam_bottom_outer_left.csv](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/paths/beam_bottom_outer_left.csv)

- `config/frames.yaml`
  Stores fixed environment frames such as `industrial_base`, `workpiece_frame`, and `table_top_corner`.
  File:
  [frames.yaml](/home/anilkir/ent413_ros2_ws/src/welding_robot_application/config/frames.yaml)

- `launch/application_demo.launch.py`
  Starts your existing welding robot MoveIt demo and the CSV toolpath visualizer together.

- `launch/frame_manager.launch.py`
  Starts only the static frame manager.

## CSV format

Use a header row with:

```text
x,y,z
```

or with optional orientation columns:

```text
x,y,z,rx,ry,rz
```

You can also use the aliases:

```text
x,y,z,roll,pitch,yaw
```

or:

```text
x,y,z,a,b,c
```

Units:

- `x, y, z` are meters.
- `rx, ry, rz` are rotations about the reference frame `x, y, z` axes.
- `a, b, c` are accepted as aliases for the same three rotations.
- By default the angles are interpreted as radians.
- Set the node parameter `angles_in_degrees:=true` if your CSV uses degrees.

If orientation columns are present, the visualizer publishes them in the `PoseArray` and also shows green arrow markers in RViz.

## Toolpath Selection

The visualizer parameter is named `toolpath_name`.

It accepts either:

- `beam_top_outer_right`
- `beam_top_outer_right.csv`

Both resolve to the same file inside `paths/`.

The default reference frame for the visualizer is now `table_top_corner`, so the beam seam CSV files are expressed relative to that frame rather than `world`.

## Typical usage

Start the application demo:

```bash
ros2 launch welding_robot_application application_demo.launch.py
```

Visualize a CSV path:

```bash
ros2 run welding_robot_application path_visualizer --ros-args -p toolpath_name:=beam_top_outer_right
```

Launch just the visualizer:

```bash
ros2 launch welding_robot_application path_visualizer.launch.py
```

If your CSV angles are in degrees:

```bash
ros2 run welding_robot_application path_visualizer --ros-args \
  -p toolpath_name:=beam_top_outer_right \
  -p angles_in_degrees:=true
```

You can also choose the toolpath from the combined launch file:

```bash
ros2 launch welding_robot_application application_demo.launch.py toolpath_name:=beam_top_outer_right
```

Publish multiple static frames from YAML:

```bash
ros2 run welding_robot_application frame_manager
```

Or with the launch file:

```bash
ros2 launch welding_robot_application frame_manager.launch.py
```

The node always loads `share/welding_robot_application/config/frames.yaml`.

## RViz Setup

For toolpath visualization, add either of these displays in RViz:

- `MarkerArray` on `/toolpath_markers`
- `PoseArray` on `/toolpath_pose_array`

`/toolpath_markers` is usually the more useful one because it shows both points and the connecting line.
