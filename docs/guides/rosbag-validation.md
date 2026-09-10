# Rosbag validation

Run this before enabling the initializer on a vehicle. Use a bag containing a
stationary view of the board.

## Preconditions

- Input is `sensor_msgs/PointCloud2` with `x`, `y`, `z` and `intensity`.
- The cloud's `header.frame_id` matches `ros.sensor_frame`.
- A static TF from `ros.base_frame` to `ros.sensor_frame` is available. Use the
  recorded `/tf_static`, or publish the sensor's calibrated transform when the
  bag lacks it.
- The vehicle is stationary while scans accumulate.

Detector gates, board dimensions, mounting height and scan count are deployment
settings. Set them for the site before validating — see
[configuration](../configuration.md).

## Procedure

A bag's topic and frame rarely match the vehicle's. The topic is a remap and
the frame a ROS parameter, so both are launch arguments; the detector file
(`config_file`) describes the board and the gates and is the one the anchoring
tool shares, so it is left alone here:

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    input_topic:=/the/bags/points \
    config_file:=/path/to/detector.yaml \
    params_file:=/path/to/board_detector.param.yaml   # sensor_frame, accumulate_scans
```

The shipped `params_file` says `velodyne`; a bag whose `frame_id` differs needs
a copy with `sensor_frame` changed.

Play the bag in another terminal:

```bash
ros2 bag play /path/to/rosbag --clock
```

`--clock` is for RViz and other simulated-time nodes. The node accumulates a
configured number of received scans, not a bag-time interval.

The detector alone publishes `~/board_pose` and calls no service, so this is
safe against a running stack. `just fake-tf`, `just launch` and `just rviz` are
shortcuts for a local setup; read the `justfile` and set its topic, frame and
calibration for yours first.

## What success looks like

```bash
ros2 topic echo /diagnostics
```

- `~/debug/board_points` contains only accepted board points.
- `~/board_pose` appears, carrying the pose and its covariance.
- The log reports range, point count, extents, centre constraints, the
  confidence with its terms, and the computed map `x`, `y`, `z` and yaw.
- Diagnostics reach `OK` with state `detected`, and carry `confidence` and
  `confidence.*` keys on every attempt.

Record each run: bag name, measured board distance, configured dimensions and
height, calculated pose, independently expected pose, pass or fail. That makes
calibration and map changes comparable across sessions.

## Failure triage

| Result | Behaviour | First checks |
|---|---|---|
| `NO_CANDIDATE` | retries each accumulation batch | intensity field and band, topic and frame, TF, range and height gates, the rejected-cluster labels |
| `AMBIGUOUS` | publishes nothing for that batch, never picks, retries the next batch | a second reflector, reflective sign or tape, board dimensions, the `candidate_N` diagnostic values and the RViz candidate labels |
| `low confidence` | publishes nothing for that batch, retries the next batch | the `confidence.*` diagnostic values name the weak term: a hidden edge (`edges`), a smeared or clipped extent (`extent`), too few or too many returns (`density`) |
| no pose published at all | detector never converged | run the [desk test](desk-test.md) to separate a config problem from a data problem |
| `service unavailable` | the Autoware node fails after a 5 s wait | start the localization stack; check `/localization/initialize` |

No outcome is terminal: every batch is judged on its own, so a transient
second reflector or a weak view costs one batch, not the run. `/diagnostics`
is `OK` only while the *last* batch published a pose; a board found and then
lost reads `WARN` with the reason.

Debug topics are transient-local. Read them against diagnostics and the current
log, since latched markers can otherwise look current.

## On the vehicle

```bash
ros2 launch reflective_pose_autoware board_pose_initializer.launch.xml
```

This one calls `/localization/initialize`. Start it only after the map and NDT
localization stack expose that service:

```bash
ros2 service list | grep '^/localization/initialize$'
```

Requires an anchored map or a surveyed `board.pose_in_map`, exact sensor TF, and
the board mounted where the map was built. Set `twist_topic` so the detector
discards scans taken while the cart is moving. This launch is standalone: the
parent vehicle launch must start map loading separately.
