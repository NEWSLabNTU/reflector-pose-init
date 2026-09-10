# Configuring the detector

The detector uses one YAML file for the board contract, runtime detector,
offline map anchoring, ROS wiring, covariance, and (optionally) Autoware
handoff policy. This guide explains how to create and choose that file; the
[configuration reference](../configuration.md) is the authoritative list of
keys and defaults.

## Create a user-owned file

Start from the canonical file in the checkout:

```bash
cp src/localization/reflective_pose_detector/\
packages/reflective_pose_core/reflective_pose_core/data/reflective_pose.yaml \
    ~/reflective_pose.yaml
$EDITOR ~/reflective_pose.yaml
```

Do not edit the installed copy under `share/` as a deployment configuration.
The `reflective_pose_ros` source package carries a synchronized copy so its
launch files have package data, but a user-owned file makes the selected
configuration explicit and survives package rebuilds.

The loader rejects unknown keys. This is intentional: a misspelled threshold
must fail at startup instead of looking like a setting that had an effect.

## Select the file

Pass the same file to every process that participates in one deployment:

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/home/you/reflective_pose.yaml

ros2 launch reflective_pose_autoware board_pose_initializer.launch.xml \
    config_file:=/home/you/reflective_pose.yaml

ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply \
    -o /path/to/map --config /home/you/reflective_pose.yaml --dry-run
```

The ROS nodes expose one parameter, `config_file`. The CLI uses `--config`.
Leaving the ROS argument empty selects the packaged default, which is useful
for the desk test but should not be mistaken for a vehicle deployment file.

## Configure in this order

### 1. Shared board contract

Set these before tuning any detector gate:

```yaml
board:
  pose_in_map: [x, y, z, roll, pitch, yaw]
  width: 0.6
  height: 0.97
```

`width` and `height` describe the reflective face in metres. `pose_in_map` is
the board centre in the final anchored `map` frame; angles are radians. The
runtime pose calculation and offline map anchoring both read these values, so
do not create a second copy under a node-specific section.

### 2. Runtime vehicle profile

Runtime point heights are measured in `base_link`, after the configured TF
transform from the LiDAR frame. On this vehicle the floor is approximately one
wheel radius below the rear-axle `base_link` origin:

```yaml
detector:
  runtime:
    height_reference: base_link
    floor_height_in_frame: -0.265
    board_centre_height: null
    height_min: 0.5
    height_max: 1.5
```

When `board_centre_height` is `null`, it is derived from the shared board
height and `floor_height_in_frame`. Set it explicitly if the vehicle's frame
or floor datum does not follow that convention.

The runtime profile also owns single-scan range, clustering, extent, planarity,
verticality, centre-height, and density settings. Runtime `height_min/max` are
not map settings and must not be copied into the map block.

### 3. Map profile and spatial crop

Map points are first levelled against the fitted floor. The map profile is
therefore independent of the runtime profile:

```yaml
detector:
  map:
    height_reference: map_floor
    board_centre_height: 1.0
    aabb:
      min: [null, null, 0.5]
      max: [null, null, 1.5]
    cluster_tolerance: 0.05
    cluster_min_points: 60
    planarity_max_thickness: 0.08
    density_check_enabled: false
```

`aabb` is an inclusive point filter in the leveled, floor-zero `map_debug`
frame. `null` on an individual coordinate means that side is unbounded. The
old `detector.map.height_min` and `detector.map.height_max` settings are not
accepted; use the AABB Z bounds instead. The AABB restricts detector input but
does not crop floor fitting or the cloud written to the anchored map.

The map's XY origin and heading are inherited from the SLAM export. They are
not made canonical by floor levelling, so choose finite XY bounds from a debug
view and expect to retune them when the export changes. `board_centre_height`
is still a candidate-centre gate; it is not a replacement for the AABB.

### 4. ROS wiring and motion protection

Set the cloud topic and frame names to match the live system:

```yaml
ros:
  input_topic: /sensing/lidar/top/pointcloud_raw_ex
  sensor_frame: velodyne
  base_frame: base_link
  accumulate_scans: 10
  twist_topic: /vehicle/status/velocity_status
  max_speed_for_accumulation: 0.05
```

The cloud must contain `x`, `y`, `z`, and `intensity`. A static TF from
`base_frame` to `sensor_frame` must be available. Accumulating scans assumes
the vehicle is stationary; set `twist_topic` on a vehicle so moving batches
are discarded instead of producing smeared board geometry. An empty topic is
appropriate for a bench only.

### 5. Autoware handoff

The `autoware` block controls what happens after the detector publishes a
pose: speed threshold, attempt budget, fallback policy, and the
`/localization/initialize` service name. It does not change board detection.
Keep `fallback_to_user_defined_pose: false` unless a reviewed fallback policy
exists.

## Check the result

Use the [desk test](desk-test.md) after changing the file. For a real sensor,
run the detector alone first using [live detection](live-detection.md). For a
map, use `--dry-run` and the debug dump described in
[debugging detection](debugging.md) before writing artifacts.

If the runtime and offline results disagree, first verify that both commands
received the same file, then check the frame-specific profiles. The board
dimensions and global board pose should be shared; height, crop, clustering,
and geometry gates are intentionally mode-specific.
