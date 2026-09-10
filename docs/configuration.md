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
ros2 run reflective_pose_cli anchor-map-to-board cloud.pcd -o map/ \
    --config /path/to/reflective_pose.yaml
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

- `board.width`, `board.height` and `board.pose_in_map` are written once and
  fanned out to both detector modes and the anchoring tool. The runtime pose
  guess and the map it is checked against cannot disagree.
- `detector.runtime` and `detector.map` are separate policies. Runtime heights
  are measured in `base_link`; map heights are measured after the CLI fits and
  levels the map floor. Their clustering, extent and planarity gates can be
  tuned independently.
- `detector.map.aabb` is an optional map-only spatial crop. It is never applied
  by the runtime node, and it does not change the board shape or
  `board.pose_in_map` shared by the two modes.
- The runtime detector's `scan_count` follows `ros.accumulate_scans`. The expected return
  count scales with the number of stacked scans, so a node accumulating ten
  against a detector assuming one rejects every real board as ten times too
  dense.

## board

Shared truth. Both the runtime node and the offline anchoring tool read it.

| Key | Canonical YAML / library fallback | Meaning |
|---|---|---|
| `pose_in_map` | `[0, 0, 1.3, 0, 0, 0]` / same | `[x, y, z, roll, pitch, yaw]`, radians, rotation `Rz(yaw) @ Ry(pitch) @ Rx(roll)` |
| `width` | `0.6` / `0.6` | reflective face width, metres |
| `height` | `0.6` / `0.97` | reflective face height, metres |

## detector

The detector section has sensor-wide values and two independent gate profiles.

| Key | Canonical YAML / library fallback | Meaning |
|---|---|---|
| `intensity_threshold` | `150.0` / `110.0` | retroreflector band cutoff |
| `azimuth_step_rad` | `0.0035` / `0.0035` | 0.2 deg at 600 rpm / 10 Hz; used by the return-density and edge-observation models |
| `mean_elevation_step_rad` | omitted / `0.0225` | fallback spacing used for the vertical edge-observation margin; the VLP-32C elevation table, not this mean, drives expected density |

The values in the table are the canonical YAML values or the resolved library
fallback when the key is omitted. They are not necessarily the same as the
constructor defaults in `reflective_pose_core.detector.DetectorParams`; those
defaults exist for direct library callers, while the packaged YAML is the
deployment configuration.

### Detection order

The gates run in this order. A cluster stops at its first failing gate, so the
first rejection reason is the most useful one to investigate.

```text
map AABB (map anchoring only)
  -> intensity, range and height point gates
  -> total point-count check
  -> voxel connected-component clustering
  -> planarity
  -> verticality
  -> board width and height
  -> candidate centre height
  -> return density (when enabled)
  -> pose and observed-edge flags
  -> OK, NO_CANDIDATE or AMBIGUOUS
```

The map AABB is applied before the core detector and is not a runtime gate.
The point gates remove individual points. Clustering then forms components; a
component smaller than `cluster_min_points` is discarded. The remaining gates
operate on one component at a time. Exactly one surviving component gives
`OK`; none gives `NO_CANDIDATE`; more than one gives `AMBIGUOUS` and the
detector intentionally refuses to choose.

`runtime` contains `range_min`, `range_max`, `height_min`, and `height_max`.
The map policy uses `map.aabb` for its spatial bounds, then shares the
remaining candidate and geometry gates: `board_centre_height`,
`cluster_tolerance`, `cluster_min_points`, `extent_tolerance`,
`planarity_max_thickness`, `verticality_max_dot`, `centre_height_tolerance`,
`density_max_ratio`, and `density_check_enabled`.

### Canonical gate values by mode

The packaged YAML currently resolves these detector policies. `runtime` and
`map` are intentionally different; `null` and `inf` below are meaningful
values with different semantics, not missing documentation. In YAML, use
`-.inf` for an unbounded lower AABB bound and `.inf` for an unbounded upper
bound. Keep `board_centre_height: null` only when the runtime value should be
derived automatically.

| Key | Runtime | Map | Interpretation |
|---|---:|---:|---|
| `height_reference` | `base_link` | `map_floor` | Frame in which candidate height is evaluated |
| `range_min` / `range_max` | `3.0` / `18.0 m` | `0.0` / `inf m` | Point range band; map mode normally leaves it unbounded |
| `height_min` / `height_max` | `0.5` / `1.5 m` | AABB `z: 0.5` / `1.1 m` | Runtime point band; map Z filtering is owned by `map.aabb` |
| `board_centre_height` | `null` → `1.035 m` | `1.0 m` | Expected candidate-centroid height; runtime value is derived from the board pose and floor offset |
| `cluster_tolerance` | `0.20 m` | `0.05 m` | Voxel clustering resolution |
| `cluster_min_points` | `60` | `60` | Minimum total/component point count |
| `extent_tolerance` | `[0.8, 1.5]` | `[0.8, 1.5]` | Lower/upper multipliers of the shared board dimensions |
| `planarity_max_thickness` | `0.08 m` | `0.08 m` | Maximum fitted-plane thickness |
| `verticality_max_dot` | `0.25` | `0.25` | Maximum absolute normal-to-gravity dot product |
| `centre_height_tolerance` | `0.30 m` | `0.30 m` | Allowed candidate-centroid height error |
| `density_max_ratio` | `1.4` | `1.4` | Maximum observed/expected return ratio |
| `density_check_enabled` | `true` | `false` | Runtime uses the single-sensor model; merged maps skip it |

`runtime.height_reference` is `base_link`. Set `runtime.floor_height_in_frame`
to the floor's z coordinate in `base_link`; for this vehicle, the rear-axle
`base_link` convention makes it approximately `-wheel_radius` (`-0.265 m`).
When `runtime.board_centre_height` is `null`, the loader derives it as
`board.pose_in_map[2] + floor_height_in_frame`. This accounts for the wheel
radius without pretending every vehicle's `base_link` is at its axle. This
shortcut assumes the canonical map z datum is the floor directly below the
board; set an explicit runtime centre height when using another datum.

`map.height_reference` is `map_floor`. The CLI first fits the floor and levels
it to z=0, so the Z bounds in `map.aabb` and `map.board_centre_height` are
map-local tuning values. The source PLY's arbitrary z origin is not used.
`map.range_max` is infinite and density is off by default because a merged map
has no single sensor origin or scan count.

`map.aabb` may be omitted or set to `null` to disable the crop, or be a mapping
with inclusive `min` and `max` `[x, y, z]` bounds. Use `-.inf` on a lower
coordinate and `.inf` on an upper coordinate when that side is unbounded.
These coordinates are in the levelled, floor-zero `map_debug` frame: after
floor fitting, before the final board-based map placement. The source PLY's XY
origin and heading are still arbitrary, so the box may need retuning for each
map export. The box is applied only to detection; floor fitting, the
room-centre viewpoint, and the output map still use the full cloud. Map
`height_min` and `height_max` are rejected to prevent a duplicate Z filter;
runtime height bounds remain valid in `base_link`. An AABB under
`detector.runtime` is rejected.

`height_min` and `height_max` filter individual reflective points in the named
height frame. `board_centre_height` is a later candidate-centre gate; it does
not change the frame or the shared board pose.

### Point gates

These gates run before clustering. Their bounds are inclusive: a value equal
to a limit is kept.

| Key | What is measured | Units and frame | Tuning effect |
|---|---|---|---|
| `intensity_threshold` | `intensity >= threshold` | calibrated sensor reflectivity | Higher removes dimmer returns; lower admits more diffuse reflective objects. For the VLP-32C this is primarily a sensor contract, not a general-purpose tuning knob. |
| `range_min`, `range_max` | Euclidean norm of each input point | metres; sensor frame at runtime, leveled map input for map mode | Narrowing the interval removes points before they can form clusters. Runtime `range_min: 3.0` is a measured VLP-32C working-range limit. |
| `height_min`, `height_max` | transformed point `z` | metres in `base_link` for runtime | Narrowing the interval removes reflective points above or below the expected board band. Map Z bounds belong in `detector.map.aabb`; map `height_min/max` keys are rejected. |

`intensity_threshold`, `range_min/max`, and runtime `height_min/max` are point
filters, not board-level checks. A board can fail later even when many of its
points pass these filters.

### Clustering gates

| Key | What is measured | Units | Tuning effect |
|---|---|---|---|
| `cluster_tolerance` | Voxel size and 26-connected-neighbour distance used to form components | metres in detector-input coordinates | Larger values join nearby reflectors and bridge gaps; smaller values split sparse boards. Tune for the largest across-ring spacing, not the dense within-ring spacing. |
| `cluster_min_points` | Minimum points both for the whole post-point-gate set and for each component | count | Higher suppresses small/noisy objects but can remove distant or partially observed boards. |

The implementation is voxel connected-component clustering, an O(N)
approximation of Euclidean clustering. `cluster_tolerance` is therefore not
an exact pairwise radius, even though it has the same metre-scale role.

### Candidate geometry gates

These gates operate after a component has formed. The board dimensions used in
the formulas come from the shared `board.width` and `board.height`, which must
describe the reflective face rather than its frame or mounting hardware.

| Key | Contract | Units/frame | If loosened |
|---|---|---|---|
| `extent_tolerance: [lo, hi]` | Observed projected width must satisfy `lo * board.width <= width <= hi * board.width`; height uses the same rule with `board.height` | dimensionless factors; observed extents are metres in the fitted board plane | A lower `lo` accepts more occluded/dropout boards; a higher `hi` accepts larger blobs and nearby merged reflectors. Failures are `bad_width` or `bad_height`. |
| `planarity_max_thickness` | Square root of the smallest covariance eigenvalue of the component | metres | Higher accepts thicker/noisier/non-planar clusters; failure is `not_planar`. |
| `verticality_max_dot` | `abs(board_normal · gravity_up)` | dimensionless, from 0 to 1 | Higher accepts more board tilt. `0` is a vertical board; `1` is horizontal. A value of `0.25` permits approximately 14.5 degrees of board tilt. Failure is `not_vertical`. |
| `board_centre_height` | Component centroid height compared with the expected board centre height | metres in the policy frame | This value is the expected height; it is not loosened directly. Use `centre_height_tolerance` to widen the band. |
| `centre_height_tolerance` | `abs(component_centroid_height - board_centre_height)` | metres in `base_link` for runtime or `map_floor` for map mode | Higher accepts more mounting or floor-height error; failure is `bad_mount_height`. |
| `density_max_ratio` | `number_of_component_points / expected_board_return_count` | dimensionless ratio | Higher accepts unusually dense clusters. It is an upper bound only; low density is expected under yaw, occlusion, and dropout. Failure is `too_dense`. |
| `density_check_enabled` | Whether the density ratio check runs | boolean | Set false for merged maps or when no single sensor/scan count describes the input. |

For example, with `board.width: 0.6`, `board.height: 0.6`, and
`extent_tolerance: [0.8, 1.5]`, the accepted observed dimensions are:

```text
width:  0.8 * 0.6 ... 1.5 * 0.6 = 0.48 ... 0.90 m
height: 0.8 * 0.6 ... 1.5 * 0.6 = 0.48 ... 0.90 m
```

The extents are the min-to-max spread of points after projection onto the
fitted board `right` and `up` axes. They are not the raw sensor-frame X/Y
spread, and `extent_tolerance` does not affect clustering.

The density model uses the VLP-32C elevation table, configured board
dimensions, range, board height, and `scan_count`. Runtime `scan_count` is
derived from `ros.accumulate_scans`; a ten-scan accumulation must be evaluated
with `scan_count: 10`. Map mode resolves it to one and disables density by
default because a merged map has no single scan count.

The final observed-edge flags are not another acceptance gate. They record
whether the scan reached each board edge so the pose covariance can identify
an unconstrained axis. `edge_margin_scale` is an internal detector constant,
not a YAML setting.

Three of these are measurements rather than tunings, and the comments in the
YAML say so:

- **`intensity_threshold: 150`** — the VLP-32C reports calibrated reflectivity,
  0-100 diffuse and 101-255 reserved for retroreflectors. This is a sensor
  contract.
- **`runtime.range_min: 3.0`** — below 3 m the board falls into the sparse lower
  elevation band, where the 9.4 degree gap between the -25.0 and -15.6 degree
  beams disconnects its bottom third from the rest of the cluster. Measured in
  simulation.
- **`runtime.density_max_ratio`** — an upper bound only. Yaw, occlusion and dropout all
  legitimately reduce the return count; nothing legitimately inflates it.

## anchor

Floor fit for the offline tool.

| Key | Canonical YAML | Meaning |
|---|---|---|
| `floor_band` | `0.3` | metres above the lowest points to fit within |
| `floor_percentile` | `2.0` | percentile taken as "the lowest points" |
| `floor_inlier` | `0.05` | refit tolerance, metres |
| `floor_refits` | `3` | refit iterations |
| `max_floor_tilt_deg` | `10.0` | refuse a cloud tilted more than this |

## covariance

Read by `reflective_pose_ros`, which computes the covariance: it is a property
of the detection, not of the stack the pose is handed to.

| Key | Canonical YAML |
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

| Key | Canonical YAML | Meaning |
|---|---|---|
| `input_topic` | `/sensing/lidar/vlp32/velodyne_points` | `sensor_msgs/PointCloud2` with `intensity` |
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

| Key | Canonical YAML | Meaning |
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
