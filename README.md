# reflective_pose_detector

Finds a retroreflective board of known size and known position in a LiDAR scan,
and derives the sensor's pose in the map from it. Intended as a cold-start pose
source where GNSS is unavailable — indoors, under cover.

A cold-start pose source, not a localizer: it publishes a pose for every
batch of scans in which it finds the board with enough confidence, and nothing
for the batches in which it does not. Whoever consumes the pose decides when
the vehicle is stationary enough to act on one.

New users: start with the [guide index](docs/guides/README.md), then follow the
setup, configuration, runtime, map, and validation guide that matches your
workflow.

## Packages

| Package | Role | ROS | Autoware |
|---|---|---|---|
| `reflective_pose_core` | detector, map anchoring, geometry, point-cloud IO | no | no |
| `reflective_pose_sim` | VLP-32C scan simulator and test scenes | no | no |
| `reflective_pose_cli` | `anchor-map-to-board` | no | no |
| `reflective_pose_ros` | detector node, launch, RViz, debug viewer | yes | no |
| `reflective_pose_autoware` | velocity gate, `/localization/initialize` | yes | yes |

## Build

In a ROS 2 workspace:

```bash
colcon build --packages-up-to reflective_pose_autoware
source install/setup.bash
```

`--packages-up-to reflective_pose_ros` stops short of the Autoware bridge and
needs no Autoware packages present.

Without ROS, for the detector and the offline tools:

```bash
pip install -e packages/reflective_pose_core -e packages/reflective_pose_cli
```

## ROS 2

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml
```

Subscribes to `~/input/pointcloud` (`sensor_msgs/PointCloud2`, needs
`intensity`; remap it), accumulates `accumulate_scans` of them, detects, and
publishes:

| Topic | Type |
|---|---|
| `~/board_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `~/debug/*` | markers and clouds for RViz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

It calls no service, so it is safe to run against a live stack. Without hardware,
see the [desk test](docs/guides/desk-test.md).

The board contract and detection gates are in the shared detector file. Frames,
scan accumulation, and the motion guard are ordinary node parameters; the
input topic is a launch remap. To select a different detector contract, copy
the file and pass it:

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/path/to/my.yaml
```

## CLI

Anchors a SLAM cloud to the board, so the map and the vehicle's startup pose
guess share one reference frame.

```bash
ros2 run reflective_pose_cli anchor-map-to-board \
    slam_export.ply -o /path/to/map --dry-run   # inspect
ros2 run reflective_pose_cli anchor-map-to-board \
    slam_export.ply -o /path/to/map             # write
```

Writes `pointcloud_map.pcd`, `board_anchor.yaml`, `board_polygon.osm` and
`map_projector_info.yaml`. Full procedure and failure debugging:
[map anchoring](docs/guides/map-anchoring.md).

## Autoware

`reflective_pose_ros` publishes a pose and stops there. `reflective_pose_autoware`
is the part that acts on it: it gates on `autoware_vehicle_msgs/VelocityReport`,
spends an attempt budget, applies the fallback policy, and calls
`autoware_localization_msgs/InitializeLocalization` with `method=AUTO` so NDT align
refines the guess.

Both nodes together:

```bash
ros2 launch reflective_pose_autoware board_pose_initializer.launch.xml
```

Start it only once the localization stack exposes the service:

```bash
ros2 service list | grep '^/localization/initialize$'
```

To drive a non-Autoware stack instead, subscribe to `~/board_pose` and leave
`reflective_pose_autoware` out of the build. That split is the reason the
detector carries no Autoware dependency.

## Configuration

One file per reader. The detector file is what the detector looks for; both
the node and the anchoring tool read it, so the map and the runtime guess
cannot drift apart:

```
packages/reflective_pose_core/reflective_pose_core/data/detector.yaml
```

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/path/to/detector.yaml input_topic:=/my/points
anchor-map-to-board cloud.ply -o map/ --config /path/to/detector.yaml
```

Where each node is plugged in — frames, accumulation, the motion guard, the
Autoware handoff policy — is ordinary ROS parameters, shipped as
`board_detector.param.yaml` and `board_pose_initializer.param.yaml`. The
anchoring tool's floor-fit knobs are its own flags.

The detector file keeps shared sensor settings at `detector`, then separates
the independent `detector.runtime` and `detector.map` policies. Runtime
`height_min`/`height_max` are absolute heights above physical ground. Runtime
points are transformed with the real `base_link <- sensor` TF, then
`detector.runtime.base_link_height_above_ground` is added only for point and
candidate height checks; the TF itself remains unchanged for range, viewpoint,
and pose calculations.

Map clouds are floor-levelled with ground at `z=0`. Map mode has no
`height_min`/`height_max`: the required `detector.map.aabb` is the explicit
spatial crop and sole point-Z filter; the map range gate is separate and
defaults to unbounded. Bounds are inclusive. Use `-.inf` for an unbounded lower
side and `.inf` for an unbounded upper side. Null or omitted bounds are invalid.

`board.pose_in_map[2]` is the board's target coordinate in the output map.
`detector.runtime.board_centre_height` and
`detector.map.board_centre_height` are independent ground-relative candidate
height policies; they are not aliases for the board map pose. Keep the map
policy value aligned with the physical board height, and keep
`pose_in_map[2]` aligned with the map's surveyed target.

**Set `twist_topic` before running on a vehicle.** Stacking scans assumes a
stationary sensor; without a motion source the detector cannot tell and will
measure a smeared board.

Every key: [configuration](docs/configuration.md).

## Tests

```bash
just test
```

The three ROS-free packages need only `PYTHONPATH`; the two ROS ones need
`/opt/ros` sourced.

## Documentation

- [Guides](docs/guides/README.md) — task-oriented recipes and recommended
  reading order
- [Getting started](docs/guides/getting-started.md) — build and run the first
  synthetic detection
- [Configuring the detector](docs/guides/configuring.md) — create and select a
  deployment YAML
- [Live detection](docs/guides/live-detection.md) — run against LiDAR and RViz
- [Autoware initialization](docs/guides/autoware-initialization.md) — hand the
  pose to `/localization/initialize`
- [Debugging](docs/guides/debugging.md) — inspect live and offline failures
- [Map anchoring](docs/guides/map-anchoring.md) — produce an anchored map
- [Rosbag validation](docs/guides/rosbag-validation.md) — validate a real scan
- [Design](docs/design/reflective_pose_detector.md) — the package split and why
- [Configuration](docs/configuration.md)
- [Desk test](docs/guides/desk-test.md) — no hardware, synthetic scenes
