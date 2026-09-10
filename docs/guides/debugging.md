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
ros2 topic hz /sensing/lidar/top/pointcloud_raw_ex
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
| `no points inside map AABB` | The crop misses the leveled cloud | inspect `map_debug` coordinates and use `null` for an unbounded side |
| Floor fit tilted too far | The lowest points do not describe the floor | floor band/percentile, map export, multiple floor levels |
| Board found but wrong pose | Shared map contract or orientation is wrong | `board.pose_in_map`, board dimensions, anchored map, viewpoint |

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
