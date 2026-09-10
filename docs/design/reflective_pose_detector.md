# reflective_pose_detector — package split and configuration

Status: proposed, 2026-09-08. Supersedes the single-package layout inherited
from `golfcart_board_initializer`.

## Why split at all

The package does four jobs that have four different audiences:

- a **detector**, which is geometry over a point cloud and needs nothing from ROS
- a **ROS node**, which is wiring: topics, TF, timers, diagnostics
- an **offline CLI**, run by a person at a terminal against a `.pcd` on disk
- an **Autoware handoff**, which is one service call and one gating rule

Today all four live in one `ament_python` package whose `package.xml` depends on
`tier4_localization_msgs` and `autoware_vehicle_msgs`. That means a person who
wants to detect a retroreflective board in a point cloud must install Autoware,
and a stack that is not Autoware cannot use the detector at all. Neither is a
consequence of the algorithm; both are consequences of the packaging.

The split is already latent in the source. Of 4052 lines, only three modules
import ROS at all — `node.py`, `debug_viz.py`, `scene_publisher.py` — and only
two symbols anywhere are Autoware-specific.

## Packages

```
packages/
  reflective_pose_core/        ROS-free library.  pip-installable.
  reflective_pose_sim/         ROS-free VLP-32C simulator.  test/desk use.
  reflective_pose_cli/         ROS-free terminal tools.
  reflective_pose_ros/         Generic ROS node, launch, RViz, debug viewer.
  reflective_pose_autoware/    Autoware bridge.  The only Autoware dependency.
```

| package | modules today | lines | depends on |
|---|---|---|---|
| `core` | `detector`, `anchor`, `geometry`, `pointcloud_io`, `vlp32`, `config` | 1347 | numpy, pyyaml |
| `sim` | `simulation/{vlp32_sim,scenes}` | 529 | numpy, core |
| `cli` | `anchor_cli` | 263 | core |
| `ros` | `node` (most), `debug_viz`, `scene_publisher` | ~750 | rclpy, core, sim |
| `autoware` | new, extracted from `node` | ~150 | ros, `tier4_localization_msgs`, `autoware_vehicle_msgs` |

`core` and `cli` and `sim` carry a `pyproject.toml` **and** a `package.xml` with
`ament_python`, so `colcon build` sees them and `pip install` also works. That is
what makes the repo self-contained: someone can clone it, `pip install -e
packages/reflective_pose_core packages/reflective_pose_cli`, and anchor a map
with no ROS on the machine at all.

`sim` is its own package rather than a `core` extra because 529 lines of
synthetic-scan generation should not ship to the vehicle inside the runtime
library. `core`'s tests and `ros`'s `scene_publisher` both take it as a test or
optional dependency.

## Node responsibility: detect, then decide

The current `node.py` does both halves. It is split on the seam that already
exists in it:

**`reflective_pose_ros`** — detects and publishes. Subscribes to the cloud, looks
up TF, accumulates scans, runs `core.detect_board`, and publishes

- `~/board_pose` — `geometry_msgs/PoseWithCovarianceStamped`, the vehicle pose
- `~/debug/*` — the existing marker, cloud and pose debug topics
- `/diagnostics` — as today

It knows nothing about what anyone does with that pose.

**`reflective_pose_autoware`** — decides and acts. Subscribes to `~/board_pose`,
gates on `autoware_vehicle_msgs/VelocityReport` (`max_speed_for_init`), applies
`max_attempts` and the `fallback_to_user_defined_pose` policy, and calls
`tier4_localization_msgs/InitializeLocalization`.

Putting the *policy* — when is it safe to initialize, how many times do we try,
what happens when we fail — in the Autoware package rather than the generic one
is deliberate. `VelocityReport` is an Autoware type, so a generic node could not
gate on it without a converter; and "is the vehicle stopped enough to trust a
cold-start pose" is a question about the vehicle stack, not about the board.

The cost is one topic hop on a path that runs once at cold start. It buys a
detector node that a non-Autoware stack can run unmodified.

## Configuration

One canonical YAML, sectioned by consumer. Today's file is a flat
`/**: ros__parameters` block that `config.py` parses by hand specifically to
avoid importing ROS — the shape is ROS's, the reader is not. The sections below
are the four kinds of setting already mixed in that file, separated:

```yaml
board:                        # shared truth: runtime AND offline anchoring
  pose_in_map: [0.0, 0.0, 1.300, 0.0, 0.0, 0.0]   # x y z roll pitch yaw, radians
  width: 0.6
  height: 0.97

detector:                     # core
  intensity_threshold: 240.0
  azimuth_step_rad: 0.0035
  runtime:                     # heights in base_link
    height_reference: base_link
    floor_height_in_frame: -0.265
    board_centre_height: null  # derive from board pose plus floor offset
    range_min: 3.0
    range_max: 18.0
    cluster_tolerance: 0.05
    cluster_min_points: 60
    extent_tolerance: [0.8, 1.5]
    planarity_max_thickness: 0.08
    verticality_max_dot: 0.25
    density_max_ratio: 1.4
    density_check_enabled: true
  map:                        # heights in the fitted map floor frame
    height_reference: map_floor
    board_centre_height: 1.0
    aabb:                      # optional [min, max] crop in map_debug
      min: [null, null, 0.5]
      max: [null, null, 1.5]
    # To restrict XY as well, replace the nulls with finite map_debug values:
    #   min: [-5.0, -3.0, 0.0]
    #   max: [ 5.0,  3.0, 2.0]
    range_min: 0.0
    range_max: .inf
    cluster_tolerance: 0.05
    cluster_min_points: 60
    extent_tolerance: [0.8, 1.5]
    planarity_max_thickness: 0.08
    verticality_max_dot: 0.25
    density_max_ratio: 1.4
    density_check_enabled: false

anchor:                       # core, offline path
  # floor fit and anchoring inputs

ros:                          # reflective_pose_ros
  input_topic: /sensing/lidar/top/pointcloud_raw_ex
  sensor_frame: velodyne
  base_frame: base_link
  accumulate_scans: 10

autoware:                     # reflective_pose_autoware
  initialize_service: /localization/initialize
  max_speed_for_init: 0.05
  max_attempts: 5
  fallback_to_user_defined_pose: false
  sigma_xy_base: 0.15
  sigma_xy_per_metre: 0.03
  sigma_z: 0.10
  sigma_yaw_base: 0.05
  covariance_safety_factor: 2.0
```

`core` reads `board:`, `detector:` and `anchor:` and ignores the rest. Each ROS
node reads `board:` plus its own section. The CLI reads `board:`, `detector:` and
`anchor:`.

**The ROS nodes declare one parameter, `config_file`, not thirty.** The node
resolves the path, hands it to `core.load_config`, and that is the only loader in
the repo.

This is a real trade-off and it is worth stating plainly. It gives up per-key
`ros2 param set` at runtime and the Autoware convention of overriding individual
parameters from a launch file. It gains one schema, one validator, and one file
that a library with no ROS dependency can read without knowing what
`ros__parameters` means. The present code already pays for the current shape by
hand-parsing around it.

Comments in the current file carry measurements — the 101-255 retroreflector
band, the 3 m minimum from the 9.36 degree beam gap, why `density_max_ratio` is
an upper bound only. **Those comments move with their keys.** They are the reason
the values are what they are.

### Where the file lives

`packages/reflective_pose_core/data/reflective_pose.yaml` is canonical.

- `core.default_config_path()` resolves: `$REFLECTIVE_POSE_CONFIG`, then the
  installed package data, then the repo checkout.
- `reflective_pose_ros` installs a copy to its `share/` for launch files.
- A test asserts the two are byte-identical, so the copy cannot drift silently.

## The CLI stays ROS-free

`anchor_cli --rviz` currently imports `rclpy` inside a function to publish debug
topics. That keeps the module importable without ROS but makes the *package* need
ROS for its most useful failure-path flag.

Instead: `--dump-debug PATH` writes an `.npz` of the levelled cloud, per-cluster
points and rejection records. `reflective_pose_ros` gains an
`anchor_debug_viewer` entry point that replays that file onto the same topics,
drawn by the same `debug_viz` code, so a map-cloud debug run still looks
identical to the live node's.

```
# In a sourced ROS 2 workspace:
$ ros2 run reflective_pose_cli anchor-map-to-board cloud.pcd -o map/ \
    --dump-debug /tmp/anchor.npz
$ ros2 run reflective_pose_ros anchor_debug_viewer /tmp/anchor.npz
```

Two steps instead of one, in exchange for a CLI package that installs and runs
with no ROS present. In that ROS-free installation, call the console script
directly as `anchor-map-to-board`; the `ros2 run reflective_pose_cli` prefix is
the ROS workspace form shown above.

## Entry points

| command | package | was |
|---|---|---|
| `anchor-map-to-board` | cli | `anchor_map_to_board` |
| `board_detector_node` | ros | part of `board_pose_initializer` |
| `board_scene_publisher` | ros | same |
| `anchor_debug_viewer` | ros | `anchor_cli --rviz` |
| `board_pose_initializer` | autoware | the Autoware half of the old node |

The Autoware-facing name is kept for `reflective_pose_autoware`, because that is
the node an Autoware launch file names.

## Tests

The eight test files split with their modules. `test/conftest.py`'s `sys.path`
insertion goes away once each package is a real installable distribution.
`test_debug_viz.py` is the only ROS-importing test and moves to `ros`.

## Migration

1. Rename the submodule path: `src/localization/golfcart_board_initializer` to
   `src/localization/reflective_pose_detector`, update `.gitmodules` and the 12
   references in 5 docs in the parent repo.
2. Create the five package skeletons; move modules unchanged.
3. Rewrite `config.py` to the sectioned schema; port the YAML with its comments.
4. Split `node.py` on the detect/decide seam.
5. Replace `--rviz` with `--dump-debug` plus the viewer.
6. Split the tests; delete the `conftest.py` path hack.

Steps 2 to 6 are behaviour-preserving except for the config format and the
`--rviz` interface, which are the two things this document is proposing to
change.

## Not doing

- Renaming the GitHub repository. `reflector-pose-init` stays unless asked; only
  the submodule *path* changes here.
- Rewriting the detector. The geometry, the thresholds and the measurements
  behind them are unchanged; this is a packaging and configuration change.
- Reviving anything the parent repo retired in `84b33f6` beyond what the
  submodule already contains.
