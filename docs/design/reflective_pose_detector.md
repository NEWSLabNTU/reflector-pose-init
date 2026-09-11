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
`autoware_localization_msgs` and `autoware_vehicle_msgs`. That means a person who
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
| `autoware` | new, extracted from `node` | ~150 | ros, `autoware_localization_msgs`, `autoware_vehicle_msgs` |

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
- `~/debug/*` — marker, cloud and pose debug topics, among them
  `~/debug/board_outline`: the detected board as two rectangles in the sensor
  frame, the configured size around the detected centre (`nominal`) and the
  measured extents with each edge coloured by whether it was observed
  (`measured`)
- `/diagnostics` — as today

It knows nothing about what anyone does with that pose.

**`reflective_pose_autoware`** — decides and acts. Subscribes to `~/board_pose`,
gates on `autoware_vehicle_msgs/VelocityReport` (`max_speed_for_init`), applies
`max_attempts` and the `fallback_to_user_defined_pose` policy, and calls
`autoware_localization_msgs/InitializeLocalization`.

Putting the *policy* — when is it safe to initialize, how many times do we try,
what happens when we fail — in the Autoware package rather than the generic one
is deliberate. `VelocityReport` is an Autoware type, so a generic node could not
gate on it without a converter; and "is the vehicle stopped enough to trust a
cold-start pose" is a question about the vehicle stack, not about the board.

The cost is one topic hop on a path that runs once at cold start. It buys a
detector node that a non-Autoware stack can run unmodified.

## Configuration

One detector YAML is shared by the two consumers that need the board contract:
`board_detector_node` and `anchor-map-to-board`. It has three sections, and
`config.py` parses it without importing ROS. Wiring belongs to ROS parameter
files and floor fitting belongs to CLI flags, so no consumer has to ignore
unrelated sections.

```yaml
board:                        # shared truth: runtime AND offline anchoring
  pose_in_map: [0.0, 0.0, 1.300, 0.0, 0.0, 0.0]   # x y z roll pitch yaw, radians
  width: 0.6
  height: 0.6
  centre_height: 1.3

detector:                     # core
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
  min_confidence: 0.6
  map_aabb:
    min: [-.inf, -.inf, 0.5]
    max: [.inf, .inf, 1.65]

covariance:                   # reflective_pose_ros, the guess covariance
  sigma_xy_base: 0.15
  sigma_xy_per_metre: 0.03
  sigma_z: 0.10
  sigma_yaw_base: 0.05
  safety_factor: 2.0
  unconstrained_axis_sigma: 1.0
```

That is the whole file. It is read by the two things that run the detector:
`board_detector_node` (parameter `config_file`) and `anchor-map-to-board`
(`--config`). Nothing else reads it and nothing else is in it.

`detector.map_aabb` is a map-only inclusive crop, evaluated after floor
levelling in the floor-zero `map_debug` frame. Use `-.inf` for an unbounded
lower side and `.inf` for an unbounded upper side. The crop filters detector
input, while floor fitting, viewpoint calculation, and the anchored output
cloud use the full map. The live detector ignores this field.

**Revised 2026-09-10: one file per reader.** The first cut of this design put
the ROS wiring (`ros:`), the Autoware handoff policy (`autoware:`) and the
floor-fit knobs (`anchor:`) in the same document, so that one file was read by
three kinds of consumer and each ignored the sections that were not its own.
That coupling was the objection: a file for many packages. The sections moved
to where each consumer already keeps its settings —

- `board_detector_node` takes its wiring (frames, `accumulate_scans`, the
  motion guard) as ordinary ROS parameters, shipped as
  `board_detector.param.yaml`. The input cloud is a remap of
  `~/input/pointcloud`, not a parameter.
- `board_pose_initializer` takes the handoff policy as ROS parameters,
  shipped as `board_pose_initializer.param.yaml`, and reads no detector file
  at all.
- `anchor-map-to-board` takes the floor fit as flags, with `AnchorParams` as
  the defaults, and builds its inputs with `core.anchor_params(config, ...)`.

A file from the six-section layout is refused by name, with the message saying
where each section went. `DetectorParams.scan_count` is passed to the loader by
the node, since the node is the one thing that knows how many scans it stacks.

What stays from the original trade-off: the board and the gates are still one
schema, one validator, and one file a ROS-free library reads — and still not
overridable per key from a launch file, on purpose, because the map was
anchored against that file.

Comments in the current file carry measurements — the 101-255 retroreflector
band, the 3 m minimum from the 9.36 degree beam gap, why `density_max_ratio` is
an upper bound only. **Those comments move with their keys.** They are the reason
the values are what they are.

### Where the file lives

`packages/reflective_pose_core/reflective_pose_core/data/detector.yaml` is the
packaged default.

- `core.default_config_path()` resolves: `$REFLECTIVE_POSE_CONFIG`, then the
  installed package data, then the repo checkout.
- `reflective_pose_core` installs it to its own `share/config/`, which is what
  the launch files point at. No copy in another package.
- The two param files ship with the package whose node reads them, and a test
  in each package asserts the file names exactly the parameters the node
  declares.

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
$ anchor-map-to-board cloud.pcd -o map/ --dump-debug /tmp/anchor.npz
$ ros2 run reflective_pose_ros anchor_debug_viewer /tmp/anchor.npz
```

Two steps instead of one, in exchange for a CLI package that installs and runs
with no ROS present.

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
