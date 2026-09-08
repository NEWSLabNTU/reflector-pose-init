# golfcart_board_initializer

Cold-start localization indoors from a retroreflective board, replacing GNSS as
the initial-pose source.

Design: [docs/design/board_pose_initializer.md](../../../docs/design/board_pose_initializer.md)
Phase: [docs/roadmaps/3-indoor-e-board-initializer.md](../../../docs/roadmaps/3-indoor-e-board-initializer.md)

The node detects the board in an accumulated stationary scan, composes the
vehicle pose in the map frame, and calls `/localization/initialize` with
`method=AUTO` — the board supplies a guess and NDT align refines it.

This package is a one-shot cold-start initializer, not a continuous localizer.
It must be started while the cart can see its one known board and is stationary.

## Layout

| Path | Role |
|------|------|
| `golfcart_board_initializer/detector.py` | Detection. Pure numpy, no `rclpy`. |
| `golfcart_board_initializer/geometry.py` | Pose composition and covariance. Pure numpy. |
| `golfcart_board_initializer/vlp32.py` | Beam table, read from the Nebula calibration with an embedded fallback. |
| `golfcart_board_initializer/anchor.py` | Offline map anchoring. Pure numpy. |
| `golfcart_board_initializer/pointcloud_io.py` | PLY and PCD, intensity preserved. Pure numpy. |
| `golfcart_board_initializer/simulation/` | Synthetic VLP-32C scans and scenes. |
| `golfcart_board_initializer/node.py` | ROS wiring, state machine, diagnostics. |
| `golfcart_board_initializer/scene_publisher.py` | Publishes synthetic scans for desk testing. |
| `golfcart_board_initializer/debug_viz.py` | Marker/cloud builders shared by the live node and `anchor_cli --rviz`. |
| `rviz/board_initializer.rviz` | RViz layout for the live/simulated-scene debug topics. |
| `rviz/anchor_debug.rviz` | RViz layout for `anchor_map_to_board --rviz`. |

`detector.py` and `geometry.py` import no ROS. That is what lets the tests run
with nothing installed, and it lets the offline map-anchoring step share the code
that runs at runtime.

## Running the tests

```bash
cd src/localization/golfcart_board_initializer
python3 -m pytest test
```

No ROS, no hardware, no bag. 57 tests, roughly 12 seconds on current dev host.

## Desk test with synthetic scans

```bash
ros2 launch golfcart_board_initializer simulated_scene.launch.xml
ros2 launch golfcart_board_initializer simulated_scene.launch.xml scene:=two_boards
ros2 launch golfcart_board_initializer simulated_scene.launch.xml scene:=distractors
```

Add `rviz:=true` to open RViz with `rviz/board_initializer.rviz`: the raw scan
coloured by intensity over a fixed 0–255 range — the retroreflector band above
100 then separates visually — the detected board points in green, a green arrow
along the board normal, every rejected cluster labelled with its reason, and the
dry-run initial pose with its covariance.

An ambiguous result draws **both** candidates in green with red `AMBIGUOUS
candidate N` labels. Every debug topic is cleared at the start of each attempt:
they are latched, so without that a stale detection from a previous run keeps
drawing and reads as a current one. The `Board pose` display is off by default
for the same reason — a latched `PoseStamped` cannot be retracted, so the arrow
marker carries that pose instead.

`dry_run` defaults to true there: the composed pose is published on
`~/debug/initial_pose` instead of calling the service, so the whole path runs
without a localization stack. Expected outcomes are a detection, an ambiguity
abort, and a clean no-candidate respectively.

`ros2 launch` under a shell `timeout` can leave the publisher running. A stale
publisher feeding a second scene into the same topic looks exactly like a
detector bug — check `pgrep -f board_scene_publisher` before believing one.

## Real rosbag validation

Run this before enabling the initializer on a vehicle. Use a bag containing a
stationary view of the board.

### Preconditions

- Input is `sensor_msgs/PointCloud2` with `x`, `y`, `z`, and `intensity`
  fields.
- Cloud `header.frame_id` matches configured `sensor_frame`.
- Static TF from `base_frame` to `sensor_frame` is available. Use recorded
  `/tf_static`, or publish the sensor's calibrated transform when the bag lacks
  it.
- Vehicle is stationary while scans accumulate.

Detector limits, board dimensions, mounting height, and scan count are
deployment settings in `config/board_initializer.param.yaml`; set them for the
site before validation.

### Procedure

1. Source built workspace. Identify bag point-cloud topic and frame.
2. Start calibrated static TF only when bag does not provide it.
3. Start node in dry-run mode.
4. Play bag.
5. Inspect RViz, diagnostics, and log.

Start node with bag topic:

```bash
ros2 launch golfcart_board_initializer board_initializer.launch.xml \
  dry_run:=true input_topic:=/your/lidar/topic
```

Play bag in another terminal:

```bash
ros2 bag play /path/to/rosbag --clock
```

`--clock` is for RViz and other simulated-time nodes. This node accumulates a
configured number of received scans, not a bag-time interval.

For local real-data workflow, `just fake-tf`, `just launch`, and `just rviz`
provide shortcuts. Inspect `justfile` and change its topic/frame/calibration for
your setup before use.

### What success looks like

Inspect RViz with `rviz/board_initializer.rviz` and diagnostics:

```bash
ros2 topic echo /diagnostics
```

Expected result:

- `/board_pose_initializer/debug/board_points` contains only accepted board
  points.
- `/board_pose_initializer/debug/initial_pose` appears. In dry-run mode it is
  published instead of calling `/localization/initialize`.
- Node log reports range, point count, extents, centre constraints, and computed
  map `x`, `y`, `z`, and yaw.
- Diagnostics `localization: board_pose_initializer` reaches `OK` with state
  `done`.

Record each field result with bag name, measured board distance, configured
board dimensions/height, calculated pose, independently expected pose, and
pass/fail. This makes calibration or map changes comparable across sessions.

### Failure behavior and triage

| Result | Node behavior | First checks |
|---|---|---|
| `NO_CANDIDATE` | Retries after each 10-scan attempt; fails after `max_attempts` (default 5). | Intensity field/band, topic and frame, TF, range/height gates, rejected-cluster labels. |
| `AMBIGUOUS` | Fails immediately; never chooses a candidate. | Second reflector, reflective sign/tape, board dimensions, RViz candidate labels. |
| `pose initializer service unavailable` | Fails after a 5 s service wait. | Start Autoware localization stack; verify `/localization/initialize`. |
| `DONE` in dry run | Stops after publishing one pose. | Expected; restart node for another attempt. |

`FAILED` is terminal. Correct setup, then restart the node; it does not retry
after failure. Debug topics are transient-local. Interpret them with diagnostics
and current logs, since latched markers can otherwise look current.

## Anchoring a map to the board

A SLAM cloud sits in an arbitrary frame — its origin is wherever the vehicle
happened to be for the first scan. The anchoring tool fixes the frame to the
board instead.

### Getting a map

1. Build and inspect a SLAM map; export its cloud as `.ply` or `.pcd` with the
   `intensity` field preserved.
2. Run a dry run. Confirm detected board dimensions, floor tilt, and transform.
3. Run anchoring without `--dry-run`; use resulting `pointcloud_map.pcd` as map
   source.
4. Merge `board_polygon.osm` into route `lanelet2_map.osm`, then tile cloud with
   `autoware_pointcloud_divider` for Autoware deployment.

```bash
# Inspect board detection and transform; writes nothing.
ros2 run golfcart_board_initializer anchor_map_to_board \
  /path/to/slam_export.ply -o /path/to/map \
  --config /path/to/board_initializer.param.yaml --dry-run

# Write anchored map artifacts.
ros2 run golfcart_board_initializer anchor_map_to_board \
  /path/to/slam_export.ply -o /path/to/map \
  --config /path/to/board_initializer.param.yaml
```

### Command options

| Argument | Default | Purpose |
|---|---|---|
| `cloud` | required | Input `.ply` or `.pcd`; must carry `intensity`. |
| `-o`, `--output-dir` | required | Directory for generated map artifacts. |
| `--name` | `pointcloud_map.pcd` | Output cloud filename. |
| `--config` | package `board_initializer.param.yaml` | Shared runtime/anchoring board parameters. |
| `--dry-run` | off | Report result; do not write files. |
| `--rviz` | off | Publish debug topics and hold the process open for RViz inspection. See below. |
| `--rviz-frame` | `map_debug` | `frame_id` for the `--rviz` topics; set RViz's Fixed Frame to match. |

`--config` is source of truth. It supplies detector gates, physical board
dimensions, and `board_pose_in_map`; do not duplicate them as CLI overrides.
Omit it only when the installed package config is intended. Pass the exact YAML
loaded by `board_pose_initializer` when building a deployment map.

Writes the anchored `pointcloud_map.pcd`, `board_anchor.yaml` (the transform, so
a rebuild can be compared against it), `board_polygon.osm` (the board's Lanelet2
landmark), and `map_projector_info.yaml` with `projector_type: Local`.

Detection reuses `detector.py`, so the board pose defining the map and the board
pose the vehicle computes at startup come from identical code — a detector bias
cancels instead of appearing as a localization error. Two board-shaped
retroreflectors in the map abort the run rather than picking one, since anchoring
to the wrong object shifts the whole map with no later symptom.

### Debugging a failed anchor

When the tool cannot find the board, the exception message alone
(`no board found in the map (N retroreflective clusters: ...)`) is not enough to
triage — it flattens every rejection into one line. Every run that reaches
detection, successful or not, now also prints a per-cluster breakdown to
stderr:

```
error: no board found in the map (60 retroreflective clusters: ...)
  1834 point(s) passed the intensity/range/height gates, 60 cluster(s) formed, 0 survived every gate
  rejected clusters:
      1. bad_width        0.41 m           n= 812  centroid=(  6.20,   1.05,   1.62)
      2. not_planar       thickness 0.061 m n=  34  centroid=(  9.80,  -3.40,   1.10)
      ...
```

`reason` and `centroid` are enough to tell a wrongly-gated board apart from an
exit sign or a second reflector — but matching 60 centroids against the cloud
by hand is still slow. Add `--rviz`:

```bash
ros2 run golfcart_board_initializer anchor_map_to_board \
  /path/to/slam_export.ply -o /path/to/map \
  --config /path/to/board_initializer.param.yaml --dry-run --rviz
```

This publishes, latched, under `/anchor_map_to_board/debug/`:

- `map_cloud` — the full cloud in the gravity-levelled frame detection actually
  ran on, coloured by intensity, so the retroreflector band is visible the same
  way it is in `rviz/board_initializer.rviz`'s "Raw scan" display.
- `board_points` — the accepted board's points (or every ambiguous candidate's).
- `rejected` — one text marker per rejected cluster, at its centroid, with
  reason and point count; a green arrow along the accepted board's normal when
  one was found; red `AMBIGUOUS candidate N` labels when more than one
  survived.

Open with `rviz2 -d rviz/anchor_debug.rviz` (Fixed Frame `map_debug`, matching
the default `--rviz-frame`). The process stays alive after printing until
Ctrl+C — it does not need a localization stack, a bag, or even a successful
anchor: this is the intended way to inspect a `NO_CANDIDATE` or `AMBIGUOUS`
failure, not just a successful run. It reuses the exact marker/cloud-building
code the live node uses for its own `~/debug/*` topics (`debug_viz.py`), so a
rejection reads the same whether it happened on a live scan or an offline map.

### Map contract

- **Anchored map:** tool places board exactly at YAML `board_pose_in_map`.
  Format is `[x, y, z, roll, pitch, yaw]`; angles are radians and rotation is
  `Rz(yaw) @ Ry(pitch) @ Rx(roll)`. Generated `map_projector_info.yaml` must
  use `projector_type: Local`.
- **Height fields differ:** `board_centre_height` is detector's expected physical
  mounting height above local floor. `board_pose_in_map[2]` is configured map
  coordinate. Keep them equal for a floor-level map; deliberately differ only
  when map frame has an offset.
- **Infrastructure change:** moving board, changing its face dimensions, or
  rebuilding map invalidates old pose/configuration. Re-anchor map or resurvey
  `board_pose_in_map`, then repeat real-bag validation.

## Deployment parameters

Configured in [`config/board_initializer.param.yaml`](config/board_initializer.param.yaml:1):

This file is source of truth for deployed values. Python defaults and design-doc
examples intentionally support other sites and must not be copied as current
cart settings.

| Parameter | Default | Description & Tuning Notes |
|---|---|---|
| `board_centre_height` | `1.6` | Expected board centre height in `base_link`, metres. |
| `board_width` / `board_height` | `0.6` / `0.6` | Physical reflective-face dimensions, metres. Current site configuration. |
| `intensity_threshold` | `110.0` | VLP-32C retroreflector return band threshold (101–255). |
| `range_min` / `range_max` | `3.0` / `18.0` | Valid sensor range bounds for detection (avoid <3m due to sparse VLP-32C bottom beam gap). |
| `height_min` / `height_max` | `1.0` / `2.2` | Accepted candidate-centre height in `base_link`, metres. |
| `planarity_max_thickness` | `0.04` | Maximum eigenvalue thickness ($\sqrt{\lambda_3}$) for plane fit. Relaxed to 0.04m to accommodate real-world hardware point scatter. |
| `board_pose_in_map` | `[0, 0, 1.6, 0, 0, 0]` | Board map pose: `[x, y, z, roll, pitch, yaw]`, angles in radians. |
| `accumulate_scans` / `max_attempts` | `10` / `5` | Scan count per attempt / attempts before terminal failure. |
| `fallback_to_user_defined_pose` | `false` | Keep false unless an explicit, reviewed fallback policy exists. |

## On the vehicle

```bash
ros2 launch golfcart_board_initializer board_initializer.launch.xml
```

Start this node only after the map/NDT localization stack exposes
`/localization/initialize`. Keep `gnss_enabled:=false`. Default launch uses
`dry_run:=false`, so success calls that service with `method=AUTO`; NDT refines
the board pose. Verify service availability before startup:

```bash
ros2 service list | grep '^/localization/initialize$'
```

Requires anchored map or surveyed `board_pose_in_map`, exact sensor TF, and
board mounted where map was built. This package launch is standalone; parent
vehicle launch must start map loading and pose initialization separately.
