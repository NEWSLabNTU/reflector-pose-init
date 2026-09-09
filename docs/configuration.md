# Configuration

One file, read by every package:

```
packages/reflective_pose_core/reflective_pose_core/data/reflective_pose.yaml
```

Both ROS nodes declare exactly one parameter, `config_file`. Empty means the
packaged default. The CLI takes `--config`.

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/path/to/reflective_pose.yaml
anchor-map-to-board cloud.pcd -o map/ --config /path/to/reflective_pose.yaml
```

`reflective_pose_ros` installs a copy to its `share/config/` for launch files to
reference. A test asserts the two are byte-identical, so edit the canonical file
and rebuild — never the copy.

An unknown key is an error, not a warning. A misspelled threshold otherwise
reads exactly like a threshold that had no effect.

## Sections

| Section | Read by | Holds |
|---|---|---|
| `board` | everything | the board's size and where it is in the map |
| `detector` | core | detection gates |
| `anchor` | core | floor fit for offline anchoring |
| `covariance` | ros | the guess covariance published with the pose |
| `ros` | ros | topics, frames, accumulation |
| `autoware` | autoware | handoff policy |

Two values are derived rather than repeated, because both have caused drift:

- `board.width`, `board.height` and `board.centre_height` are written once and
  fanned out to both the detector and the anchoring tool. The runtime pose guess
  and the map it is checked against cannot disagree.
- `detector.scan_count` follows `ros.accumulate_scans`. The expected return
  count scales with the number of stacked scans, so a node accumulating ten
  against a detector assuming one rejects every real board as ten times too
  dense.

## board

Shared truth. Both the runtime node and the offline anchoring tool read it.

| Key | Default | Meaning |
|---|---|---|
| `pose_in_map` | `[0, 0, 1.3, 0, 0, 0]` | `[x, y, z, roll, pitch, yaw]`, radians, rotation `Rz(yaw) @ Ry(pitch) @ Rx(roll)` |
| `width` | `0.6` | reflective face width, metres |
| `height` | `0.97` | reflective face height, metres |
| `centre_height` | `1.0` | expected centre height above the local floor, metres |

`centre_height` and `pose_in_map[2]` are different things: the first is the
physical mounting height the detector expects, the second is a map coordinate.
Keep them equal for a floor-level map; differ deliberately only when the map
frame carries an offset.

## detector

| Key | Default | Meaning |
|---|---|---|
| `intensity_threshold` | `240.0` | retroreflector band cutoff |
| `range_min` / `range_max` | `3.0` / `18.0` | usable range, metres |
| `height_min` / `height_max` | `0.5` / `1.5` | accepted candidate centre height |
| `cluster_tolerance` | `0.05` | clustering distance, metres |
| `cluster_min_points` | `60` | smallest cluster considered |
| `extent_tolerance` | `[0.8, 1.5]` | accepted fraction of nominal size |
| `planarity_max_thickness` | `0.08` | plane-fit thickness limit, metres |
| `verticality_max_dot` | `0.25` | how far off vertical the normal may be |
| `centre_height_tolerance` | `0.30` | slack on `board.centre_height` |
| `density_max_ratio` | `1.4` | upper bound on returns vs expected |
| `density_check_enabled` | `true` | whether the density gate runs |
| `azimuth_step_rad` | `0.0035` | 0.2 deg at 600 rpm / 10 Hz |

Three of these are measurements rather than tunings, and the comments in the
YAML say so:

- **`intensity_threshold: 240`** — the VLP-32C reports calibrated reflectivity,
  0-100 diffuse and 101-255 reserved for retroreflectors. This is a sensor
  contract.
- **`range_min: 3.0`** — below 3 m the board falls into the sparse lower
  elevation band, where the 9.4 degree gap between the -25.0 and -15.6 degree
  beams disconnects its bottom third from the rest of the cluster. Measured in
  simulation.
- **`density_max_ratio`** — an upper bound only. Yaw, occlusion and dropout all
  legitimately reduce the return count; nothing legitimately inflates it.

## anchor

Floor fit for the offline tool.

| Key | Default | Meaning |
|---|---|---|
| `floor_band` | `0.3` | metres above the lowest points to fit within |
| `floor_percentile` | `2.0` | percentile taken as "the lowest points" |
| `floor_inlier` | `0.05` | refit tolerance, metres |
| `floor_refits` | `3` | refit iterations |
| `max_floor_tilt_deg` | `10.0` | refuse a cloud tilted more than this |

## covariance

Read by `reflective_pose_ros`, which computes the covariance: it is a property
of the detection, not of the stack the pose is handed to.

| Key | Default |
|---|---|
| `sigma_xy_base` | `0.15` |
| `sigma_xy_per_metre` | `0.03` |
| `sigma_z` | `0.10` |
| `sigma_yaw_base` | `0.05` |
| `sigma_roll_pitch` | `0.02` |
| `safety_factor` | `2.0` |
| `unconstrained_axis_sigma` | `1.0` |

Loose on purpose. The service is called with `method=AUTO`, so NDT align refines
the guess; an over-tight covariance makes it search too small a window, while an
over-loose one costs a few hundred milliseconds.

## ros

| Key | Default | Meaning |
|---|---|---|
| `input_topic` | `/sensing/lidar/top/pointcloud_raw_ex` | `sensor_msgs/PointCloud2` with `intensity` |
| `sensor_frame` | `velodyne` | must match the cloud's `header.frame_id` |
| `base_frame` | `base_link` | static TF to `sensor_frame` must exist |
| `accumulate_scans` | `10` | scans stacked per detection attempt |
| `twist_topic` | `""` | motion guard source; empty disables it |
| `max_speed_for_accumulation` | `0.05` | m/s above which scans are discarded |

**Set `twist_topic` on a vehicle.** Stacking scans assumes a stationary sensor —
nothing deskews them — so a batch taken while the cart rolls is smeared and the
board's extents measure wrong. Empty is correct on a bench and wrong on a
vehicle; the node logs a warning when it is empty. `nav_msgs/Odometry` and
`geometry_msgs/TwistStamped` are both accepted.

## autoware

| Key | Default | Meaning |
|---|---|---|
| `initialize_service` | `/localization/initialize` | the service called on success |
| `max_speed_for_init` | `0.05` | m/s above which no pose is acted on |
| `max_attempts` | `5` | failed attempts before the fallback policy runs |
| `fallback_to_user_defined_pose` | `false` | send `board.pose_in_map` when attempts run out |
| `pose_wait_timeout` | `0.0` | seconds to wait for a pose before spending an attempt; `0` disables |

Keep `fallback_to_user_defined_pose` false unless there is an explicit, reviewed
fallback policy. A silent fallback to a fixed pose turns a detector failure into
a mislocalization report three weeks later.

`pose_wait_timeout` exists because detection and the service call are separate
processes: an attempt is one *pose acted on*, so a detector that never detects
spends no attempts and the fallback never runs.

## Changing the site

Moving the board, changing its face dimensions, or rebuilding the map
invalidates the old pose. Re-anchor the map or resurvey `board.pose_in_map`,
then repeat [rosbag validation](guides/rosbag-validation.md).
