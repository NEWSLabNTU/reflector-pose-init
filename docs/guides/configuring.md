# Configuring the detector

The detector file is a small, shared contract: it says what board the system
expects, which point-cloud gates define a candidate, and what covariance to
attach to a published pose. Both `board_detector_node` and
`anchor-map-to-board` read it, so runtime detection and offline map anchoring
cannot quietly use different board dimensions or gates.

Node wiring and Autoware handoff policy are separate ROS parameter files. The
anchoring tool's floor-fit settings are command-line flags. The
[configuration reference](../configuration.md) is the complete list of keys
and defaults; this guide focuses on choosing values and diagnosing gates.

## Create a user-owned detector file

Start from the packaged core default:

```bash
cp packages/reflective_pose_core/reflective_pose_core/data/detector.yaml \
    ~/detector.yaml
$EDITOR ~/detector.yaml
```

When the package is installed, the same file is available below the core
package's `share/config/` directory. Keep a user-owned copy for deployment so
package rebuilds do not overwrite the file you selected.

The loader rejects unknown sections and keys. A misspelled threshold must fail
at startup instead of looking like a setting that took effect.

## Select the file

Pass the same detector file to the live node and the map tool:

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/home/you/detector.yaml \
    params_file:=/path/to/board_detector.param.yaml

ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply \
    -o /path/to/map --config /home/you/detector.yaml --dry-run
```

The combined Autoware launch also takes this file, while its two nodes receive
their own parameter files:

```bash
ros2 launch reflective_pose_autoware board_pose_initializer.launch.xml \
    config_file:=/home/you/detector.yaml \
    detector_params_file:=/path/to/board_detector.param.yaml \
    initializer_params_file:=/path/to/board_pose_initializer.param.yaml
```

An empty `config_file` selects the packaged `detector.yaml`. That is convenient
for the desk test; use an explicit path on a vehicle or when producing a map.

## The detector file

The file has exactly three sections:

```yaml
board:
  pose_in_map: [0.0, 0.0, 1.3, 0.0, 0.0, 0.0]
  width: 0.6
  height: 0.6
  centre_height: 1.3

detector:
  intensity_threshold: 100.0
  range_min: 3.0
  range_max: 18.0
  height_min: 0.5
  height_max: 1.65
  cluster_tolerance: 0.15
  cluster_min_points: 60
  extent_tolerance: [0.8, 1.5]
  planarity_max_thickness: 0.08
  verticality_max_dot: 0.25
  centre_height_tolerance: 0.30
  density_max_ratio: 1.4
  density_check_enabled: true
  azimuth_step_rad: 0.0035
  map_aabb:
    min: [-.inf, -.inf, 0.5]
    max: [.inf, .inf, 1.65]

covariance:
  sigma_xy_base: 0.15
  sigma_xy_per_metre: 0.03
  sigma_z: 0.10
  sigma_yaw_base: 0.05
  sigma_roll_pitch: 0.02
  safety_factor: 2.0
  unconstrained_axis_sigma: 1.0
```

`board.width`, `board.height`, and `board.centre_height` are copied into the
typed detector and anchoring parameters by the loader. Set them once under
`board`; do not create runtime-only or map-only copies.

`board.centre_height` is the board centre above the local floor. It is not
necessarily the same quantity as `board.pose_in_map[2]`, which is a coordinate
in the final map frame. They are normally equal for a floor-level map.

## Map AABB

`detector.map_aabb` is an optional, inclusive spatial crop for map anchoring.
It is evaluated after floor levelling in the floor-zero `map_debug` frame:

```yaml
detector:
  map_aabb:
    min: [-.inf, -5.0, 0.5]
    max: [12.0, .inf, 1.65]
```

Use `-.inf` only for an unbounded lower side and `.inf` only for an unbounded
upper side. This is the canonical YAML spelling; do not use `null` for an
individual coordinate. Omit `map_aabb` when no crop is needed. An explicit
`map_aabb: null` disables the entire optional crop; it is different from an
unbounded side and is retained only as a convenient compatibility spelling.

The AABB filters only the points passed to board detection. Floor fitting,
viewpoint calculation, and the anchored output cloud still use the full map.
The live detector does not apply this map-only crop.

The map AABB is not a replacement for the detector's height or centre-height
gates. The former removes points before clustering; the latter checks the
candidate's measured centre after geometry is fitted. Keep the AABB broad
enough to contain the board and inspect `map_debug` coordinates when it yields
`no points inside map AABB`.

## How the gates are applied

Detection is staged. The first failing stage determines which setting to
inspect:

1. **Point gates:** keep points with `intensity >= intensity_threshold`,
   `range_min <= range <= range_max`, and
   `height_min <= height <= height_max`. Runtime heights are in `base_link`;
   map heights are in the levelled, floor-zero frame.
2. **Clustering:** join nearby kept points using `cluster_tolerance`; discard
   clusters smaller than `cluster_min_points`.
3. **Planarity:** fit a plane and require RMS thickness to be at most
   `planarity_max_thickness`.
4. **Verticality:** require `abs(normal · up) <= verticality_max_dot`; a
   vertical board has a normal perpendicular to up, so its value is near zero.
5. **Extent:** project the cluster onto board-aligned horizontal and vertical
   axes and compare its measured width and height with the nominal board.
6. **Mounting height:** require the fitted centre height to be within
   `board.centre_height ± centre_height_tolerance`.
7. **Density:** when enabled, reject only clusters whose return count is above
   `density_max_ratio` times the sensor-model expectation. Dropout and
   occlusion reduce counts legitimately, so this is not a minimum-return gate.
8. **Confidence:** a lone geometric survivor gets one confidence scalar. The
   ROS node publishes it only when it is at least `min_confidence`; the offline
   tool prints the value. Ambiguous and low-confidence batches are retried by
   the node rather than selecting a risky winner.

The live node injects its `accumulate_scans` value as `scan_count` when loading
the detector file. The expected density therefore scales with the number of
stacked scans; `scan_count` is not a separate YAML tuning knob.

## Tune from the first failing gate

| Observation | Inspect first | Typical action | Risk of loosening |
|---|---|---|---|
| `n_after_gates` is too small | intensity, range, height frame and bounds | Correct the sensor contract or widen the point band | Diffuse returns and unrelated reflectors enter clustering |
| `n_clusters` is zero or the board is split | `cluster_tolerance`, `cluster_min_points`, scan motion | Bridge adjacent LiDAR rings or lower the minimum only for a known sparse view | Separate reflectors merge |
| `not_planar` | merged clusters, map noise, motion smear | Fix clustering or motion first; then raise the thickness limit only for measured noise | Walls and merged objects can pass |
| `not_vertical` | TF, board mounting, `verticality_max_dot` | Fix frame/mounting assumptions; widen only for an allowed tilt | Horizontal surfaces become candidates |
| `bad_width` / `bad_height` | board dimensions, occlusion, extent values | Correct `board.width/height`; adjust extent factors only for systematic observation bias | A wrong size makes another object look plausible |
| `bad_mount_height` | floor datum, `board.centre_height`, tolerance | Fix the frame or shared height; widen tolerance only for known variation | Reflectors at other heights survive |
| `too_dense` | scan count, motion, merged clusters, density model | Make `accumulate_scans` match the batch and fix merging/motion; then review the upper ratio | Large unrelated clusters survive |
| `AMBIGUOUS` | visible reflectors and map AABB | Remove the second candidate or constrain the map crop | Picking one can shift the entire map |

### `extent_tolerance` in detail

`extent_tolerance` is `[lower_factor, upper_factor]`, a pair of multipliers
rather than a metre-valued tolerance. For a `0.6 m` board and
`[0.8, 1.5]`, each measured dimension must be between `0.48 m` and `0.90 m`:

```text
lower = lower_factor * nominal_dimension
upper = upper_factor * nominal_dimension
```

The lower factor handles under-observation from occlusion, dropout, scan
smearing, or a split cluster. The upper factor limits merged clusters and an
incorrectly small nominal board. The width and height use the same pair, and
the lower and upper factors can be changed independently. Change them only
after confirming that the board is one cluster and that its configured size is
correct.

### Confidence terms

The confidence scalar is a weighted mean of planarity, extent, density, edge
observation, and range. The `edges` term has double weight because a hidden
edge biases the estimated centre; `range` has half weight because covariance
already grows with distance. `/diagnostics` reports each term as
`confidence.<name>`, and the offline debug dump stores the same values and the
resolved map gates.

## Node wiring and Autoware policy

The detector node's separate `board_detector.param.yaml` contains:

| Key | Meaning |
|---|---|
| `sensor_frame` / `base_frame` | cloud frame and frame used for runtime heights |
| `accumulate_scans` | scans per detection attempt; becomes detector `scan_count` |
| `twist_topic` | optional `Odometry` or `TwistStamped` motion source |
| `max_speed_for_accumulation` | speed above which an accumulated batch is discarded |

Set `twist_topic` on a vehicle. Stacking scans assumes a stationary sensor;
an empty topic is useful for a bench but allows motion-smearing in deployment.

The separate `board_pose_initializer.param.yaml` contains the service name,
speed gate, attempt budget, fallback policy, and optional pose wait timeout.
It is not part of board detection and must not be added under `detector`.

## Validate changes

After each meaningful change, run the [desk test](desk-test.md), then validate
a stationary [rosbag](rosbag-validation.md). For a map, use
`--dry-run --dump-debug` and the [debugging guide](debugging.md) before writing
map artifacts.

Moving the board, changing its face dimensions, or rebuilding the map
invalidates the old pose. Re-anchor the map or resurvey `board.pose_in_map`,
then repeat [rosbag validation](rosbag-validation.md).
