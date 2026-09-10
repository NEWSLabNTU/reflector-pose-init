# Getting started

This guide takes a new user from a checkout to a working synthetic detection.
It does not require a LiDAR, a map, or Autoware.

## Prerequisites

You need:

- a ROS 2 installation with `colcon` and `rviz2`;
- this repository inside a ROS 2 workspace's `src/` directory;
- Python 3 with NumPy and PyYAML, installed through the workspace/package
  dependencies.

The Autoware bridge additionally needs the message packages used by
`reflective_pose_autoware`. If those packages are unavailable, build through
`reflective_pose_ros` and use the detector without the bridge.

## Build and source

From the workspace root:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to reflective_pose_autoware
source install/setup.bash
```

For a non-Autoware setup:

```bash
colcon build --packages-up-to reflective_pose_ros
source install/setup.bash
```

The packages have separate responsibilities:

| Package | Use |
|---|---|
| `reflective_pose_core` | ROS-free detection, configuration, geometry, and map anchoring |
| `reflective_pose_sim` | synthetic VLP-32C scenes used by the desk test |
| `reflective_pose_cli` | ROS-free command-line map anchoring entry point |
| `reflective_pose_ros` | live detector node, launch files, RViz, and offline debug viewer |
| `reflective_pose_autoware` | vehicle-speed gate and Autoware initialization service handoff |

Confirm that the overlay exposes the commands:

```bash
ros2 pkg executables reflective_pose_ros
ros2 pkg executables reflective_pose_cli
ros2 pkg executables reflective_pose_autoware
```

You should see `board_detector_node`, `anchor_debug_viewer`,
`anchor-map-to-board`, and `board_pose_initializer` in the corresponding
lists.

## First run: synthetic detection

Start the detector and synthetic scene publisher together:

```bash
ros2 launch reflective_pose_ros simulated_scene.launch.xml rviz:=true
```

The default `board` scene publishes synthetic VLP-32C scans, a static
`base_link -> velodyne` transform, and a board at a known pose. The detector
accumulates the configured number of scans and should publish one board pose.

Try the failure cases too:

```bash
ros2 launch reflective_pose_ros simulated_scene.launch.xml \
    scene:=two_boards rviz:=true

ros2 launch reflective_pose_ros simulated_scene.launch.xml \
    scene:=distractors rviz:=true
```

The first should stop with an ambiguous result; the second should report no
candidate. These are useful checks that the rejection and safety behavior is
working before real data is involved.

For details about the RViz displays and synthetic parameters, continue to the
[desk test](desk-test.md). To use a real point cloud, read
[configuring the detector](configuring.md) and [live detection](live-detection.md).

## ROS-free installation

The core detector and map anchoring CLI do not import ROS. When only offline
tools are needed, install the two packages from the repository checkout:

```bash
python3 -m pip install -e packages/reflective_pose_core \
    -e packages/reflective_pose_cli
```

The CLI still needs a YAML configuration and an intensity-bearing `.ply` or
`.pcd`; follow [map anchoring](map-anchoring.md).
