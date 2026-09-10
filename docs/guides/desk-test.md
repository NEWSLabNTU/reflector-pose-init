# Desk test with synthetic scans

No hardware, no bag, no localization stack. `reflective_pose_sim` renders
VLP-32C scans of a synthetic room and `board_scene_publisher` puts them on the
wire, along with the static `base_link -> velodyne` transform the node needs.

This is the first functional check after [building and sourcing the workspace](getting-started.md).
It uses the packaged configuration unless `config_file` is supplied.

```bash
ros2 launch reflective_pose_ros simulated_scene.launch.xml
ros2 launch reflective_pose_ros simulated_scene.launch.xml scene:=two_boards
ros2 launch reflective_pose_ros simulated_scene.launch.xml scene:=distractors
```

Expected outcomes, in order: a detection, an ambiguity abort, and a clean
no-candidate.

With the default node name, inspect the result and diagnostics from another
terminal:

```bash
ros2 topic echo /board_detector/board_pose \
    --qos-durability transient_local
ros2 topic echo /diagnostics
```

To test a user-owned configuration:

```bash
ros2 launch reflective_pose_ros simulated_scene.launch.xml \
    config_file:=/home/you/detector.yaml rviz:=true
```

Add `rviz:=true` to open RViz with `rviz/board_detector.rviz`:

- the raw scan coloured by intensity over a fixed 0-255 range, so the
  retroreflector band above 100 separates visually
- the detected board points in green, with an arrow along the board normal
- every rejected cluster labelled with its reason
- the published pose with its covariance

An ambiguous result draws **both** candidates in green with red
`AMBIGUOUS candidate N` labels.

Only the detector runs here. It publishes `~/board_pose` and calls no service,
so nothing needs `/localization/initialize` to exist.

## Two things that look like detector bugs and are not

**Stale latched markers.** Every debug topic is cleared at the start of each
attempt. They are latched, so without that a detection from a previous run keeps
drawing and reads as a current one. The `Board pose` display is off by default
for the same reason: a latched `PoseStamped` cannot be retracted, so the arrow
marker carries that pose instead.

**A surviving scene publisher.** `ros2 launch` under a shell `timeout` can leave
the publisher running. A stale publisher feeding a second scene into the same
topic looks exactly like a detector bug — check `pgrep -f board_scene_publisher`
before believing one.
