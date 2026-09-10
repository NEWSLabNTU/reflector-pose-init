# Debug detection failures

Use this guide when the detector publishes no pose, reports ambiguity, or the
offline anchoring tool rejects a map. The fastest path is to make the detector
show the points and the reason each cluster was rejected.

## Start with the right layer

- Use the [desk test](desk-test.md) to verify the software path without a
  sensor.
- Use [live detection](live-detection.md) to inspect a real topic without
  calling Autoware.
- Use [rosbag validation](rosbag-validation.md) to replay a stationary real
  scan.
- Use [map anchoring](map-anchoring.md) for a merged map; its debug dump and
  viewer are separate from the live detector topics.

Do not tune runtime and map settings interchangeably. Runtime heights are in
`base_link`; map AABB bounds and the map candidate-height gate after levelling
are in the floor-zero `map_debug` frame.

## Inspect a live detector

Launch the detector and open the supplied RViz layout:

```bash
ros2 launch reflective_pose_ros board_detector.launch.xml \
    config_file:=/home/you/reflective_pose.yaml

rviz2 -d "$(ros2 pkg prefix reflective_pose_ros)/share/reflective_pose_ros/rviz/board_detector.rviz"
```

Check the input and diagnostics:

```bash
ros2 topic hz /sensing/lidar/vlp32/velodyne_points
ros2 topic echo /diagnostics
ros2 topic list | grep board_detector
```

The RViz displays show accepted board points, the board normal, and rejection
labels. An `AMBIGUOUS candidate N` label means more than one cluster passed all
board gates; the node intentionally refuses to choose one.

## Capture an offline failure

The map CLI can write a debug dump even when anchoring fails. Always use the
complete ROS 2 executable invocation:

```bash
ros2 run reflective_pose_cli anchor-map-to-board slam_export.ply \
    -o /tmp/anchored-map \
    --config /home/you/reflective_pose.yaml \
    --dry-run --dump-debug /tmp/anchor.npz
```

Replay the dump through the ROS viewer:

```bash
ros2 run reflective_pose_ros anchor_debug_viewer /tmp/anchor.npz

rviz2 -d "$(ros2 pkg prefix reflective_pose_ros)/share/reflective_pose_ros/rviz/anchor_debug.rviz"
```

The viewer publishes the full leveled cloud on `map_cloud`, not only the
points inside the AABB. `board_points` contains the accepted board or every
ambiguous candidate; `rejected` contains cluster labels. The debug dump also
records the AABB frame, bounds, and the number of points inside it.

The AABB is evaluated in the leveled, floor-zero `map_debug` frame. It is not
raw PLY coordinates and not the final anchored map frame. Floor fitting,
viewpoint calculation, and the output cloud use the full input map.

## Interpret common outcomes

| Symptom | Meaning | First checks |
|---|---|---|
| No pose and `wait_tf` | The detector cannot transform sensor points | `ros.sensor_frame`, `ros.base_frame`, and static TF |
| No pose and `accumulate` | Too few scans or no candidate yet | input topic, `accumulate_scans`, intensity, runtime gates |
| `no candidate` | Reflective clusters failed a gate | rejection reason, height frame, extents, planarity, board dimensions |
| `AMBIGUOUS` | Multiple board-shaped clusters survived | signs/reflectors in view, map AABB, XY crop, geometry gates |
| `no points inside map AABB` | The crop misses the leveled cloud | inspect `map_debug` coordinates and use `-.inf`/`.inf` for unbounded lower/upper sides |
| Floor fit tilted too far | The lowest points do not describe the floor | floor band/percentile, map export, multiple floor levels |
| Board found but wrong pose | Shared map contract or orientation is wrong | `board.pose_in_map`, board dimensions, anchored map, viewpoint |

## Read rejection labels

Each rejected cluster gets one label at its centroid. The label contains the
first geometry gate that rejected that cluster and, when available, the
measured value. Compare that value with the resolved configuration for the
same runtime or map profile.

| Reason | Label measurement | Meaning | Check first |
|---|---|---|---|
| `not_planar` | `thickness X m` | The cluster is thicker than `planarity_max_thickness` after fitting a plane | merged objects, map noise, scan motion, then the thickness limit |
| `not_vertical` | `|n.up| X` | The fitted board normal is too aligned with gravity; `0` is a vertical board and `1` is horizontal | TF, board mounting, and `verticality_max_dot` |
| `degenerate_up` | no measurement | The fitted normal leaves no usable gravity-up direction in the board plane | degenerate or nearly horizontal geometry and the preceding verticality setting |
| `bad_width` | `X m` | Projected width is outside `extent_tolerance * board.width` | reflective-face width, occlusion, cluster splitting/merging |
| `bad_height` | `X m` | Projected height is outside `extent_tolerance * board.height` | reflective-face height, vertical occlusion, height filtering |
| `bad_mount_height` | `X m` | The cluster centroid height is outside `board_centre_height ± centre_height_tolerance` | height frame, floor datum, derived centre height |
| `too_dense` | `ratio X` | The cluster has more points than `density_max_ratio` times the sensor-model expectation | accumulated scan count, motion, merged clusters, density setting |

For example, with a `0.6 m` board and `extent_tolerance: [0.8, 1.5]`, a
`bad_width 0.41 m` rejection is below the `0.48 m` lower bound. Do not fix
that by changing planarity or density: inspect occlusion, scan accumulation,
and the configured board width first.

The labels are per-cluster, not a summary of the entire scan. If there are no
labels, the failure happened before cluster geometry: inspect the point count
after intensity/range/height gates, `cluster_min_points`, and the input topic
or frame. The detector does not report later measurements for a cluster that
already failed an earlier gate.

For offline map runs, `--dump-debug` also stores the resolved map parameters,
the measured rejection value, and the accepted lower/upper limits in the NPZ
debug file. This is useful when the YAML contains separate runtime and map
profiles; do not compare a map rejection against runtime limits.

## Tune in a safe order

1. Confirm the cloud contains intensity and that the intensity threshold leaves
   the board visible.
2. Confirm the frame and floor convention before changing height values.
3. Use the AABB to remove distant map context; its bounds are a map-only
   spatial filter.
4. Tune clustering for the point density of the input.
5. Tune planarity and board geometry only after the board forms one cluster.
6. Re-run the desk or rosbag workflow after each meaningful configuration
   change.

The detector's rejection reason is more useful than lowering every threshold.
For example, `bad_width` or `not_planar` points to a geometry/map-density
problem, while an empty AABB points to a coordinate-frame problem.
