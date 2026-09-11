# Configuration

One file per reader.

| File | Read by | Holds |
|---|---|---|
| `detector.yaml` | `board_detector_node` (`config_file`), `anchor-map-to-board` (`--config`) | the board, the detection gates, the guess covariance |
| `board_detector.param.yaml` | `board_detector_node`, as ROS parameters | frames, accumulation, the motion guard |
| `board_pose_initializer.param.yaml` | `board_pose_initializer`, as ROS parameters | the Autoware handoff policy |
| flags of `anchor-map-to-board` | the tool | the floor fit |

The detector file is the one that matters for correctness: it keeps shared
sensor settings together while giving runtime detection and offline map
anchoring separate policies. The two modes do not see equivalent clouds:
runtime points are a sensor view in `base_link`, while map points are a merged,
floor-levelled cloud. The other settings describe where each node is plugged
in, and arrive the way every other ROS node takes such things. The input cloud
is neither: `board_detector_node` subscribes to `~/input/pointcloud`, and the
launch file remaps it.

Packaged defaults:

```
packages/reflective_pose_core/reflective_pose_core/data/detector.yaml
packages/reflective_pose_ros/config/board_detector.param.yaml
packages/reflective_pose_autoware/config/board_pose_initializer.param.yaml
```

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/path/to/detector.yaml \
    params_file:=/path/to/board_detector.param.yaml \
    input_topic:=/sensing/lidar/top/pointcloud_raw_ex
anchor-map-to-board cloud.ply -o map/ --config /path/to/detector.yaml
```

An unknown key in the detector file is an error, not a warning: a misspelled
threshold otherwise reads exactly like a threshold that had no effect. A file
carrying a `ros:`, `autoware:` or `anchor:` section is from the earlier
six-section layout and is refused with a message saying where each moved.

## detector.yaml

Three sections.

| Section | Holds |
|---|---|
| `board` | the board's size and where it is in the map |
| `detector` | detection gates |
| `covariance` | the guess covariance published with the pose |

`board.width` and `board.height` are shared geometry. `board.pose_in_map` is the
board's target pose in the output map; its `z` value is a map coordinate, not a
detector height-policy setting. The runtime and map policies each have their
own ground-relative `board_centre_height`, because each path has its own
height datum and tuning.

`detector.scan_count` is not in the file. The node passes its own
`accumulate_scans` to the loader, because the expected return count scales with
the number of stacked scans: a node accumulating ten against a detector
assuming one rejects every real board as ten times too dense. The anchoring
tool leaves it at one.

### board

| Key | Default | Meaning |
|---|---|---|
| `pose_in_map` | `[0, 0, 1.3, 0, 0, 0]` | `[x, y, z, roll, pitch, yaw]`, radians, rotation `Rz(yaw) @ Ry(pitch) @ Rx(roll)` |
| `width` | `0.6` | reflective face width, metres |
| `height` | `0.6` | reflective face height, metres |

`pose_in_map[2]` is the board's target `z` in the map produced by anchoring. It
is independent from both detector policies' expected board centre heights.
With a correctly floor-levelled map (`z=0` at ground), it should normally agree
numerically with `detector.map.board_centre_height`, but the loader does not
derive either value from the other.

### detector

The top level contains settings shared by both policies:

| Key | Default | Meaning |
|---|---|---|
| `intensity_threshold` | `100.0` | retroreflector band cutoff |
| `azimuth_step_rad` | `0.0035` | sensor azimuth resolution, radians |
| `mean_elevation_step_rad` | `0.0225` | sensor elevation step used by the density model, radians |
| `min_confidence` | `0.6` | below this, a surviving runtime cluster is not published |

The policy-specific gates are independent. Matching defaults do not make the
two clouds share a hidden flat parameter set.

### detector.runtime

Runtime heights are ground-relative even though the input points and the TF
transform are expressed relative to `base_link`:

| Key | Default | Meaning |
|---|---|---|
| `base_link_height_above_ground` | `0.265` | added to transformed `base_link` z only for height checks |
| `range_min` / `range_max` | `3.0` / `18.0` | usable range, metres |
| `height_min` / `height_max` | `0.5` / `1.65` | ground-relative point-height band, metres |
| `cluster_tolerance` | `0.15` | clustering distance, metres |
| `cluster_min_points` | `60` | smallest cluster considered |
| `board_centre_height` | `1.3` | expected board centre above physical ground, metres |
| `extent_tolerance` | `[0.8, 1.5]` | accepted fraction of nominal size |
| `planarity_max_thickness` | `0.08` | plane-fit thickness limit, metres |
| `verticality_max_dot` | `0.25` | how far off vertical the normal may be |
| `centre_height_tolerance` | `0.30` | slack on this policy's `board_centre_height` |
| `density_max_ratio` | `1.4` | upper bound on returns vs expected |
| `density_check_enabled` | `true` | whether the density gate runs |

The node transforms with the real `base_link <- sensor` TF. It computes
`height_above_ground = z_base_link + base_link_height_above_ground` for the
runtime point band and candidate-centre check. The offset does not alter the
TF, range, viewpoint, density geometry, or returned pose. This scalar assumes
the `base_link` z axis is aligned with gravity; if roll/pitch is meaningful,
use a gravity-aligned height frame instead of a scalar offset.

### detector.map

Map points are gravity-levelled and translated so the fitted physical ground is
`z=0`. The required AABB is the explicit map spatial crop and sole point-height
filter; the separate map range gate defaults to unbounded. Map mode has no
`height_min` or `height_max` keys:

| Key | Default | Meaning |
|---|---|---|
| `aabb` | required | inclusive crop in the levelled, floor-zero `map_debug` frame; its z slab is the map point-height filter |
| `range_min` / `range_max` | `0.0` / `.inf` | map range policy, metres |
| `cluster_tolerance` | `0.15` | clustering distance, metres |
| `cluster_min_points` | `60` | smallest cluster considered |
| `board_centre_height` | `1.3` | expected board centre above physical ground, metres |
| `extent_tolerance` | `[0.8, 1.5]` | accepted fraction of nominal size |
| `planarity_max_thickness` | `0.08` | plane-fit thickness limit, metres |
| `verticality_max_dot` | `0.25` | how far off vertical the normal may be |
| `centre_height_tolerance` | `0.30` | slack on this policy's `board_centre_height` |
| `density_max_ratio` | `1.4` | upper bound on returns vs expected |
| `density_check_enabled` | `false` | whether the density gate runs |

The runtime detector applies intensity, range and ground-relative height point
gates before clustering. Map anchoring applies `detector.map.aabb` first, then
the map policy's intensity/range gates; its resolved detector height bounds are
unbounded. Both modes then apply clustering, planarity, verticality, projected
extent, candidate centre height, and the policy's density setting. A cluster
that passes all of them is a candidate. The ROS node then applies
`min_confidence`; the core detector does not suppress a geometric survivor
based on confidence.

`verticality_max_dot` means `abs(normal · up) <= limit`, so a vertical board has
a value near zero. `board_centre_height` in either policy is ground-relative;
it is a candidate-level plausibility check, not a second map point crop.

#### Map AABB

`detector.map.aabb` is required and is applied only by `anchor-map-to-board`,
after floor levelling and before the detector receives its map points. Bounds
are inclusive. Use signed infinity as the canonical YAML spelling for open
sides:

```yaml
detector:
  map:
    aabb:
      min: [-.inf, -.inf, 0.5]
      max: [.inf, .inf, 1.65]
```

Use `-.inf` for an unbounded lower bound and `.inf` for an unbounded upper
bound. Do not use `null`, and do not omit `detector.map.aabb`; an AABB with
unbounded sides is written explicitly with signed infinities. Null or omitted
bounds are rejected. The AABB is the sole map spatial/Z input filter. Floor
fitting, viewpoint calculation, and the anchored output cloud always use the
full input map.

Most of these are measurements rather than tunings, and the comments in the
YAML say so:

- **`intensity_threshold: 100`** — the VLP-32C reports calibrated reflectivity,
  0-100 diffuse and 101-255 reserved for retroreflectors. 100 is the bottom
  edge of that band, so this is a sensor contract — and, unlike the 240 it
  replaced, one the board reaches: over the replay bag no scan cleared
  `cluster_min_points` at 240, and every scan does at 100. The evidence table
  is beside the key. The cost is that everything retroreflective now passes
  this gate, and the geometry, the confidence gate and a non-terminal
  `AMBIGUOUS` are what sort it.
- **`detector.runtime.cluster_tolerance: 0.15`** and
  **`detector.runtime.height_max: 1.65`** — measured together
  on the replay bag. The board sits 0.66 m below the cart's sensor, in the
  part of the fan where ring spacing is 0.1 m at 5 m and more below 4 m; at
  the old 0.05 it split into ring stripes in every one of 235 batches, and
  0.10 still lost a ring at 3 m. 0.15 bridges the rings but also the 0.1 m
  gap to a second retroreflective band directly above the board (top at
  1.6 m, band 1.7 to 1.9 m), which merged into the cluster in 2 of 9 batches
  and lifted the centre 6 to 9 cm; `detector.runtime.height_max: 1.65` keeps
  that band out. The equivalent map Z filtering belongs in the z bounds of
  `detector.map.aabb`.
  The old 1.5 clipped the top 0.1 m of the board itself.
- **`range_min: 3.0`** — below 3 m the board falls into the sparse lower
  elevation band, where the 9.4 degree gap between the -25.0 and -15.6 degree
  beams disconnects its bottom third from the rest of the cluster. Measured in
  simulation.
- **`density_max_ratio`** — an upper bound only. Yaw, occlusion and dropout all
  legitimately reduce the return count; nothing legitimately inflates it.

#### `min_confidence`

Every cluster that survives the gates carries one confidence in `[0, 1]`,
computed in `reflective_pose_core.detector.confidence_terms` from what the
gates already measured, re-expressed as how far inside each gate the cluster
landed:

| Term | Weight | 1 means | 0 means |
|---|---|---|---|
| `planarity` | 1 | residual under half `planarity_max_thickness` | at the gate |
| `extent` | 1 | nominal size (an under-read of one sample spacing is forgiven) | at the edge of `extent_tolerance` |
| `density` | 1 | the return count the sensor model predicts | none, or at `density_max_ratio` |
| `edges` | 2 | all four bounding edges observed | none |
| `range` | 0.5 | at `range_min` | at `range_max` |

`edges` weighs double because a hidden edge is the one defect that biases the
*centre* — by up to half the hidden width — rather than merely widening the
covariance. `range` weighs half because the covariance already grows with it.

The node publishes no pose below `min_confidence`, and `/diagnostics` says
`low confidence 0.xx < 0.yy` with every term as a key/value; the next batch is
judged afresh. Nothing is terminal. The default 0.6 sits between the two
populations the simulator produces: a clean board scores 0.86 to 0.93 across
3 to 15 m (yaw to 60 degrees, blooming or 70 % dropout still 0.79 to 0.90),
while a board with one edge hidden scores 0.55. The replay bag's stationary
detections score 0.85 to 0.89. The offline tool prints the same number, so a
map anchored to a weak detection is visible as such.

### covariance

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

Loose on purpose. The pose is handed on as a guess for a scan matcher to refine
(Autoware's initializer is called with `method=AUTO`); an over-tight covariance
makes it search too small a window, while an over-loose one costs a few hundred
milliseconds.

## board_detector.param.yaml

Ordinary `ros__parameters`, under `/**`. A test asserts the shipped file names
exactly the parameters the node declares, with the node's own defaults.

| Key | Default | Meaning |
|---|---|---|
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

## board_pose_initializer.param.yaml

Same shape, same test.

| Key | Default | Meaning |
|---|---|---|
| `initialize_service` | `/localization/initialize` | the service called on success |
| `max_speed_for_init` | `0.05` | m/s above which no pose is acted on |
| `max_attempts` | `5` | failed attempts before the fallback policy runs |
| `fallback_to_user_defined_pose` | `false` | send an empty request when attempts run out, so Autoware uses its own user-defined pose |
| `pose_wait_timeout` | `0.0` | seconds to wait for a pose before spending an attempt; `0` disables |

Keep `fallback_to_user_defined_pose` false unless there is an explicit, reviewed
fallback policy. A silent fallback to a fixed pose turns a detector failure into
a mislocalization report three weeks later.

`pose_wait_timeout` exists because detection and the service call are separate
processes: an attempt is one *pose acted on*, so a detector that never detects
spends no attempts and the fallback never runs.

## anchor-map-to-board flags

The floor fit is a property of one run of one tool, so it is flags, with
`AnchorParams` as the defaults. There is deliberately no flag for the board or
the gates: those come from `--config`, the same file the vehicle loads.

| Flag | Default | Meaning |
|---|---|---|
| `--floor-band` | `0.3` | metres above the lowest points to fit within |
| `--floor-percentile` | `2.0` | percentile taken as "the lowest points" |
| `--floor-inlier` | `0.05` | refit tolerance, metres |
| `--floor-refits` | `3` | refit iterations |
| `--max-floor-tilt-deg` | `10.0` | refuse a cloud tilted more than this |

## Changing the site

Moving the board, changing its face dimensions, or rebuilding the map
invalidates the old pose. Re-anchor the map or resurvey `board.pose_in_map`,
then repeat [rosbag validation](guides/rosbag-validation.md).
