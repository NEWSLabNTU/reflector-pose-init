# Phase 1 — split the single package into five

Design: [../design/reflective_pose_detector.md](../design/reflective_pose_detector.md)

Status: in progress, opened 2026-09-08.

The design is agreed. This phase is the execution: five packages, one config
file, a node split on the detect/decide seam, and a CLI that no longer needs ROS.

Everything here is behaviour-preserving **except** two deliberate interface
changes, called out at P3 and P5: the config file format, and `--rviz` becoming
`--dump-debug` plus a viewer.

## Work items

### P1 — repository skeleton and the config contract

The contract every other item codes against, so it lands first and alone.

- `packages/` with five directories, each carrying `pyproject.toml`; `core`,
  `sim` and `cli` additionally carry `package.xml` with `ament_python` so colcon
  sees them and `pip install -e` also works.
- `packages/reflective_pose_core/data/reflective_pose.yaml` — the canonical
  config, ported from `config/board_initializer.param.yaml` **with its comments**.
  Sections: `board`, `detector`, `anchor`, `ros`, `autoware`.
- `reflective_pose_core.config` — `load_config(path) -> Config`, plus
  `default_config_path()` resolving `$REFLECTIVE_POSE_CONFIG`, then installed
  package data, then the repo checkout.

**Done when:** `load_config` round-trips the ported YAML into the existing
`DetectorParams` and `AnchorParams` dataclasses with the same values the current
flat file produces. A test asserts that equivalence against the old file, so the
port cannot silently change a threshold.

### P2 — `reflective_pose_core`

Move, do not rewrite: `detector.py`, `anchor.py`, `geometry.py`,
`pointcloud_io.py`, `vlp32.py`. Tests move with them.

**Done when:** `pytest packages/reflective_pose_core` passes with no ROS on
`PYTHONPATH`, and nothing under `core` imports `rclpy` or any `*_msgs`.

### P3 — `reflective_pose_sim`

Move `simulation/{vlp32_sim,scenes}.py` and the tests that drive them.

**Done when:** core's tests still pass taking `sim` as a test dependency, and
`sim` is absent from any runtime dependency list.

### P4 — `reflective_pose_cli`

Move `anchor_cli.py`. Replace `--rviz` with `--dump-debug PATH`, writing an
`.npz` of the levelled cloud, per-cluster points and rejection records.

**This is one of the two interface changes.** The failure path is exactly when
the picture is wanted, so the dump must carry everything the old inline viewer
drew, not a summary.

**Done when:** `pip install packages/reflective_pose_core packages/reflective_pose_cli`
into a venv with no ROS; the direct console script
`anchor-map-to-board --help` and a dry-run anchor both work there.

### P5 — `reflective_pose_ros`

The generic half of `node.py`, plus `debug_viz.py`, `scene_publisher.py`, the
launch files and the RViz configs. New `anchor_debug_viewer` entry point
replaying P4's `.npz` through `debug_viz` onto the same topics.

The node detects and publishes `~/board_pose`
(`geometry_msgs/PoseWithCovarianceStamped`), `~/debug/*` and `/diagnostics`. It
declares one parameter, `config_file`.

**Done when:** `package.xml` names no Autoware package, and the node runs against
`board_scene_publisher` with no Autoware installed.

### P6 — `reflective_pose_autoware`

The decide half: gate on `autoware_vehicle_msgs/VelocityReport`, apply
`max_attempts` and `fallback_to_user_defined_pose`, call
`tier4_localization_msgs/InitializeLocalization`. Entry point keeps the name
`board_pose_initializer`, because that is what an Autoware launch file names.

**Done when:** it is the only package whose `package.xml` mentions Autoware, and
the desk test — scene publisher, detector node, this node — still reaches a
service call.

### P7 — integration

- Parent repo launch and docs point at the new package and entry point names.
- The old flat `config/board_initializer.param.yaml` is deleted, not left beside
  the new one.
- `README.md` rewritten for five packages.

**Done when:** `colcon build` is green from a clean workspace and the parent
repo's `just build` still succeeds.

## Parallelism

P1 is serial and blocks everything. P2 through P6 touch disjoint directories and
can run together; each owns exactly one `packages/<name>/` and must not edit
another's. Shared files — the top-level `README.md`, `.gitignore`, the old
`golfcart_board_initializer/` tree — are removed in P7, not by the parallel
items, so two items cannot race to delete the same file.

## Risks

**The config port is where a number can silently change.** The current file's
comments record measurements: the 101-255 retroreflector band, the 3 m minimum
that follows from the 9.36 degree beam gap, why `density_max_ratio` is an upper
bound only. P1's equivalence test exists for this and is not optional.

**`board_pose_in_map` is shared truth** between runtime init and offline
anchoring. It moves to `board.pose_in_map` and both paths must read that one key;
a port that gives the CLI and the node separate copies reintroduces exactly the
drift the current file's comment warns about.

**The detect/decide split changes when the service is called.** The generic node
publishes on every successful detection; the Autoware node applies the attempt
budget. Verify the retry and fallback behaviour end to end, not just that both
nodes start.
