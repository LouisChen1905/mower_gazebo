# mower_gazebo

`mower_gazebo` provides a ROS 2 + Gazebo Classic simulation package for the mower robot. It installs custom Gazebo worlds, bundled models, and helper nodes that adapt Gazebo topics into the mower message interfaces used elsewhere in this workspace.

## What It Contains

- `gazebo_simulator_node`: converts mower planning commands into `/cmd_vel`, republishes odometry as mower location/GPS-style status topics, publishes fixed sensor TFs, and supports pose reset through Gazebo's `/set_entity_state` service.
- `visual_boundary_simulator_node`: generates `mower_msgs/msg/VisualBoundaryPoints` from the robot pose and a boundary description, with optional RViz markers for boundary and grass points in the camera field of view.
- `visual_obstacle_simulator_node`: clusters the front ToF point cloud into separate `mower_msgs/msg/VisualObs` objects on the front visual-obstacle topic.
- `launch/mower_gazebo.launch.py`: starts Gazebo Classic, loads the bundled world, and launches both nodes with default parameters.
- `worlds/mower_test.world`: flat test world with a 40 m x 40 m ground plane, a simple square collision boundary, the `mower_robot` model, and a visual `boundary_points` model.
- `models/mower_robot/model.sdf`: differential-drive mower model with rear drive wheels, front caster wheels, a Gazebo ROS diff-drive plugin, a front ToF ray sensor, and a camera.
- `worlds/mower_ackermann_4wd.world`: flat test world that loads the `ackermann_4wd_robot` model.
- `models/ackermann_4wd_robot/model.sdf`: Ackermann-steered mower model with four driven wheels and a package-local Gazebo plugin for `/cmd_vel` and `/odom`.
- `models/boundary_points/model.sdf`: static visualized boundary segments included in the default world.
- `scripts/generate_boundary_model.py`: regenerates `models/boundary_points/model.sdf` from ENU boundary points.

## Build

From the workspace root:

```bash
colcon build --packages-select mower_gazebo
source install/setup.bash
```

This package depends on ROS 2, `gazebo_ros`, `mower_msgs`, and `mower_wrapper_client`.

## Regenerate Boundary Model

Rebuild the Gazebo boundary visualization from the ENU boundary file:

```bash
python3 mower_gazebo/scripts/generate_boundary_model.py
```

The generator assumes the source file is ENU and converts it to the Gazebo/map convention used by this workspace:

```text
map x = ENU north
map y = -ENU east
map z = ENU up
```

## Run

Launch the default simulation:

```bash
ros2 launch mower_gazebo mower_gazebo.launch.py
```

Launch a different world:

```bash
ros2 launch mower_gazebo mower_gazebo.launch.py world:=/absolute/path/to/world.sdf
```

Launch the Ackermann + 4WD model:

```bash
ros2 launch mower_gazebo mower_gazebo.launch.py \
  world:=$(ros2 pkg prefix mower_gazebo)/share/mower_gazebo/worlds/mower_ackermann_4wd.world \
  model_name:=ackermann_4wd_robot
```

Useful runtime interfaces inferred from the code:

- Gazebo diff-drive consumes `/cmd_vel` and publishes `/odom`.
- `gazebo_simulator_node` subscribes to mower planning on `TOPIC_PLANNING` and publishes mower status topics such as location, RTK/GPS, motor state, safety state, and dock pole state.
- `visual_boundary_simulator_node` publishes `TOPIC_VISUAL_BOUNDARY_POINTS` plus debug markers on `boundary_in_fov` and `grass_in_fov`.
- `visual_obstacle_simulator_node` subscribes to `/mower_tof/points` and publishes `mower_msgs/msg/VisualObs` on `TOPIC_VISUAL_OBS_FRONT` / `visual_obstacle_front`.
- Publish `geometry_msgs/msg/Pose` to `reset_mower_pose` to reposition the mower model in Gazebo.

## Main Assumptions And Limitations

- The launch file is written for Gazebo Classic (`gazebo`, `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`), not modern Gazebo Sim.
- The default visual-boundary inputs use hard-coded absolute paths:
  - `/home/chensi/boundary_points.txt`
  - `/home/chensi/boundary_tangential_vectors.txt`
  - `/home/chensi/polygon_vertices.txt`
- If the boundary point files are missing or invalid, `visual_boundary_simulator_node` falls back to a synthetic square boundary with `boundary_half_size` and computed tangential vectors.
- The mower/Gazebo bridge uses a fixed ENU-to-Gazebo mapping (`east = -y`, `north = x`) and can only emit GPS-like data after receiving a location anchor on `TOPIC_LOCATION_ANCHOR`.
- The published safety, dock, and cutter status are simulated placeholders rather than detailed physics-based sensor outputs.
- The package currently ships no plugin/controller config beyond the launch-time defaults in `mower_gazebo.launch.py`.
