# Run live LiDAR detection

Use this workflow to run only the generic detector node against a real or
simulated point cloud. It publishes a pose but does not call Autoware or any
localization service, so it is the safe first step on a vehicle.

## Before starting

The input must satisfy all of these conditions:

- `ros.input_topic` carries `sensor_msgs/PointCloud2` with `x`, `y`, `z`, and
  `intensity` fields;
- the message `header.frame_id` matches `ros.sensor_frame`;
- TF can resolve `ros.base_frame <- ros.sensor_frame`;
- the board is visible and the vehicle is stationary while scans accumulate;
- the shared `board.pose_in_map`, dimensions, and runtime profile are correct.

On a vehicle, set `ros.twist_topic` and its speed threshold. Without that
motion guard, the detector cannot know that stacked scans were collected while
the vehicle moved and the board may be smeared.

Prepare a configuration as described in
[configuring the detector](configuring.md), then build and source the
workspace:

```bash
colcon build --packages-up-to reflective_pose_ros
source install/setup.bash
```

## Start the detector

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/home/you/reflective_pose.yaml
```

The launch file starts `board_detector_node`. It waits for TF, accumulates
`ros.accumulate_scans` scans, runs the runtime detector, and stops after a
successful detection or terminal ambiguity/failure.

The detector-only launch publishes a pose and calls no service. To perform the
Autoware handoff, use the separate workflow in
[Autoware initialization](autoware-initialization.md).

## Inspect the output

With the default launch name and namespace:

```bash
ros2 topic echo /board_detector/board_pose \
    --qos-durability transient_local
ros2 topic echo /diagnostics
```

The pose is `geometry_msgs/PoseWithCovarianceStamped` in `map`. The covariance
reflects range, plane-fit quality, and whether the observed board edges
constrain the estimated centre.

Open the supplied RViz layout in another terminal:

```bash
rviz2 -d "$(ros2 pkg prefix reflective_pose_ros)/share/reflective_pose_ros/rviz/board_detector.rviz"
```

The detector publishes latched debug topics below its node namespace:

- `~/debug/board_points` — accepted board points, or ambiguous candidates;
- `~/debug/board_pose` — the detected board pose in the sensor frame;
- `~/debug/rejected` — one marker per rejected cluster;
- `/diagnostics` — state, attempt, and detection counts.

If the node is launched inside a namespace, replace `/board_detector` in the
topic examples with that namespace and node name. Use `ros2 topic list` when
the resolved name is uncertain.

## Understand the states

- `wait_tf`: the node has not yet found the configured static transform;
- `accumulate`: scans are being collected;
- `detected`: a pose was published;
- `failed`: an ambiguity or terminal detector error occurred.

`NO_CANDIDATE` retries another accumulation batch. `AMBIGUOUS` is terminal:
the node refuses to choose between two board-shaped reflectors. Correct the
scene or configuration and restart the node.

## First checks when no pose appears

1. `ros2 topic echo` the configured input topic and confirm it is receiving
   messages with intensity.
2. Check TF with `ros2 run tf2_ros tf2_echo base_link velodyne` using the actual
   configured frame names.
3. Confirm the detector is reading the intended YAML file from its startup log.
4. Look at `/diagnostics` and the rejected-cluster markers.
5. Run the [desk test](desk-test.md) to separate a software/configuration
   problem from a sensor or calibration problem.

For a recorded real scan, use [rosbag validation](rosbag-validation.md).
