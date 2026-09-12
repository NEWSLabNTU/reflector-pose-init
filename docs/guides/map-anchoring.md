# Anchoring a map to the board

A SLAM cloud sits in an arbitrary frame — its origin is wherever the vehicle
happened to be for the first scan. Anchoring fixes the frame to the board
instead, so the map and the vehicle's startup pose guess share one reference.

Detection reuses the same detector the node runs, so the board pose defining the
map and the board pose the vehicle computes at startup come from identical code:
a detector bias cancels instead of appearing as a localization error.

## Procedure

1. Build a SLAM map and export its cloud as `.ply` or `.pcd` **with the
   `intensity` field preserved**.
2. Dry-run the tool. Confirm the detected dimensions, the floor tilt and the
   transform.
3. Run it for real; use the resulting `pointcloud_map.pcd` as the map source.
4. Merge `board_polygon.osm` into the route's `lanelet2_map.osm`, then tile the
   cloud with `autoware_pointcloud_divider`.

```bash
# inspect; writes nothing
ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply -o /path/to/map \
    --config /path/to/detector.yaml --dry-run

# write the map artifacts
ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply -o /path/to/map \
    --config /path/to/detector.yaml
```

## Options

| Argument | Default | Purpose |
|---|---|---|
| `cloud` | required | input `.ply` or `.pcd`; must carry `intensity` |
| `-o`, `--output-dir` | required | directory for the generated artifacts |
| `--name` | `pointcloud_map.pcd` | output cloud filename |
| `--config` | packaged `detector.yaml` | board, detector gates, covariance |
| `--floor-band`, `--floor-percentile`, `--floor-inlier`, `--floor-refits`, `--max-floor-tilt-deg` | `AnchorParams` | the floor fit; see [configuration](../configuration.md) |
| `--dry-run` | off | report the result, write nothing |
| `--dump-debug PATH` | off | write an `.npz` for `anchor_debug_viewer` |

`--config` is the source of truth. It supplies the detector gates, the physical
board dimensions and `board.pose_in_map`; there are deliberately no CLI
overrides for them. Pass the exact file the vehicle will load when building a
deployment map.

## Restrict the map search

The detector file requires a map AABB. For a large map, narrow its inclusive
crop in the same detector file:

```yaml
detector:
  map:
    aabb:
      min: [-.inf, -5.0, 0.5]
      max: [12.0, .inf, 1.65]
```

The bounds are inclusive and are evaluated in the floor-levelled,
floor-zero `map_debug` frame. Use `-.inf` for an unbounded lower side and
`.inf` for an unbounded upper side. Do not use `null` or omit the AABB; a full
map search is written explicitly with signed infinities. The crop filters only
the detector input: floor fitting, viewpoint calculation, and the output map
still use the complete cloud. The live detector ignores this map-only setting.

## What it writes

- `pointcloud_map.pcd` — the anchored cloud
- `board_anchor.yaml` — the transform, so a rebuild can be compared against it
- `board_polygon.osm` — the board as a Lanelet2 landmark
- `map_projector_info.yaml` — with `projector_type: Local`

Two board-shaped retroreflectors in the map abort the run rather than picking
one. Anchoring to the wrong object shifts the whole map with no later symptom.

## Debugging a failure

Every run that reaches detection prints a per-cluster breakdown to stderr:

```
error: no board found in the map (60 retroreflective clusters: ...)
  1834 point(s) passed the intensity/range/height gates, 60 cluster(s) formed, 0 survived every gate
  rejected clusters:
      1. bad_width        0.41 m           n= 812  centroid=(  6.20,   1.05,   1.62)
      2. not_planar       thickness 0.061 m n=  34  centroid=(  9.80,  -3.40,   1.10)
```

`reason` and `centroid` distinguish a wrongly-gated board from an exit sign or a
second reflector, but matching sixty centroids against the cloud by hand is
slow. Dump and view instead:

```bash
ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply -o /path/to/map --dry-run \
    --dump-debug /tmp/anchor.npz

ros2 run reflective_pose_ros anchor_debug_viewer /tmp/anchor.npz
rviz2 -d "$(ros2 pkg prefix reflective_pose_ros --share)/rviz/anchor_debug.rviz"    # Fixed Frame: map_debug
```

The viewer publishes, latched:

- `map_cloud` — the full cloud in the gravity-levelled frame detection actually
  ran on, coloured by intensity, so the retroreflector band is visible
- `board_points` — the accepted board's points, or every ambiguous candidate's
- `rejected` — one text marker per rejected cluster at its centroid, with reason
  and point count; an arrow along the accepted board's normal when there was
  one; red `AMBIGUOUS candidate N` labels when more than one survived

The dump is written on the failure paths too, including the "board not at the
configured pose" exit — which is the case it is most for. The viewer needs no
localization stack, no bag and no successful anchor, and it reuses the drawing
code the live node uses for its own `~/debug/*` topics, so a rejection reads the
same offline as it does on the vehicle.

The CLI itself has no ROS dependency; that is why the drawing happens in a
separate viewer rather than inline.

## Map contract

- The tool places the board exactly at `board.pose_in_map`. Format is
  `[x, y, z, roll, pitch, yaw]`, angles in radians, rotation
  `Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
- `map_projector_info.yaml` must use `projector_type: Local`.
- `detector.runtime.board_centre_height` and
  `detector.map.board_centre_height` are ground-relative candidate-height
  policies. `board.pose_in_map[2]` is the target coordinate in the output map;
  it is independent, but should normally match the map policy height when the
  output map has ground at z=0.
- Moving the board, changing its face dimensions, or rebuilding the map
  invalidates the old pose. Re-anchor or resurvey, then repeat
  [rosbag validation](rosbag-validation.md).
