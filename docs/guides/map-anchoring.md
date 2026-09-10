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

After building and sourcing the ROS 2 workspace, run the CLI through its ROS 2
package with `ros2 run reflective_pose_cli anchor-map-to-board`:

```bash
# inspect; writes nothing
ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply -o /path/to/map \
    --config /path/to/reflective_pose.yaml --dry-run

# write the map artifacts
ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply -o /path/to/map \
    --config /path/to/reflective_pose.yaml
```

## Options

| Argument | Default | Purpose |
|---|---|---|
| `cloud` | required | input `.ply` or `.pcd`; must carry `intensity` |
| `-o`, `--output-dir` | required | directory for the generated artifacts |
| `--name` | `pointcloud_map.pcd` | output cloud filename |
| `--config` | packaged `reflective_pose.yaml` | board and detector settings |
| `--dry-run` | off | report the result, write nothing |
| `--dump-debug PATH` | off | write an `.npz` for `anchor_debug_viewer` |

`--config` is the source of truth. It supplies the map detector policy, the
shared physical board dimensions, and `board.pose_in_map`; there are
deliberately no CLI overrides for them. Pass the exact file the vehicle will
load when building a deployment map. The runtime policy is read from the same
file but is not applied to the map.

If the exported map contains too much of the surrounding building, set the
optional `detector.map.aabb` in that YAML file:

```yaml
detector:
  map:
    aabb:
      min: [-5.0, -3.0, 0.0]
      max: [ 5.0,  3.0, 2.0]
```

The bounds are inclusive and use the levelled, floor-zero `map_debug` frame —
after floor fitting and before final board placement. They are not raw PLY
coordinates and are not final anchored-map coordinates. Floor fitting, the
room-centre viewpoint, and the exported map still use the full cloud; only
detection input is cropped. Because floor levelling does not establish a
canonical XY origin or heading, choose the bounds from a debug view and expect
to retune them when the SLAM export frame changes. Set an individual coordinate
to `null` when that side should be unbounded, and use `aabb: null` to disable
the entire crop.

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
rviz2 -d packages/reflective_pose_ros/rviz/anchor_debug.rviz    # Fixed Frame: map_debug
```

The viewer publishes, latched:

- `map_cloud` — the full cloud in the gravity-levelled frame detection actually
  came from, coloured by intensity, so the retroreflector band is visible. It
  remains full even when `detector.map.aabb` crops detector input.
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
- `detector.map.board_centre_height` is a map-local gate measured from the
  fitted floor. `detector.runtime.board_centre_height` is a separate gate in
  `base_link`; it may be derived from the shared board height and the vehicle's
  floor-to-`base_link` offset.
- `detector.map.aabb`, when enabled, is a map-only inclusive spatial gate in
  the levelled, floor-zero `map_debug` frame. It does not alter floor fitting,
  viewpoint calculation, or the full cloud written to the map.
- `board.pose_in_map[2]` is the authoritative board placement in the anchored
  map and is shared with runtime initialization. It is not used as a substitute
  for the map's trial-tuned height gate.
- Moving the board, changing its face dimensions, or rebuilding the map
  invalidates the old pose. Re-anchor or resurvey, then repeat
  [rosbag validation](rosbag-validation.md).
