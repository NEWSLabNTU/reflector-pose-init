# User guides

These are task-oriented recipes. Start with the guide that matches what you are
trying to do; the parameter tables and design notes are linked separately when
you need detail.

## Recommended path

1. [Getting started](getting-started.md) — build the workspace and run the
   synthetic detector once.
2. [Configuring the detector](configuring.md) — create a user-owned YAML file
   for a vehicle, board, and map.
3. [Desk test](desk-test.md) — exercise success, ambiguity, and rejection
   without hardware.
4. [Live detection](live-detection.md) — run the detector against LiDAR and
   inspect its published pose.
5. [Map anchoring](map-anchoring.md) — turn a SLAM export into the anchored
   map consumed by localization.
6. [Debugging detection](debugging.md) — inspect failures in RViz, live or
   offline.
7. [Autoware initialization](autoware-initialization.md) — connect a detected
   pose to `/localization/initialize`.
8. [Rosbag validation](rosbag-validation.md) — replay a stationary real scan
   before enabling the vehicle path.

## I want to…

| Goal | Guide |
|---|---|
| Build and verify the packages | [Getting started](getting-started.md) |
| Make a configuration file | [Configuring the detector](configuring.md) |
| Test without a sensor | [Desk test](desk-test.md) |
| Run against a live point cloud | [Live detection](live-detection.md) |
| Anchor a `.ply` or `.pcd` map | [Map anchoring](map-anchoring.md) |
| Understand a rejected board | [Debugging detection](debugging.md) |
| Send the pose to Autoware | [Autoware initialization](autoware-initialization.md) |
| Validate a rosbag | [Rosbag validation](rosbag-validation.md) |

## References

- [Configuration reference](../configuration.md) — complete key meanings,
  defaults, frames, and validation rules.
- [Design](../design/reflective_pose_detector.md) — package boundaries and
  the reasoning behind the runtime/offline split.

Unless a guide says otherwise, commands assume that the ROS 2 underlay and
this workspace overlay have already been sourced. Complete commands include
`ros2 run` or `ros2 launch`; the package executable names are not shell commands
until the workspace is built and sourced.
