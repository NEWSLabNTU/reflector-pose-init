# Initialize Autoware localization

This workflow connects a successful board detection to Autoware's localization
initializer. It is deliberately separate from the generic detector:
`reflective_pose_ros` produces the pose, while `reflective_pose_autoware`
decides whether the vehicle is stopped and calls the initialization service.

If you only need to inspect a pose, follow [live detection](live-detection.md)
instead.

## Preconditions

Before starting this launch:

- the localization map is already loaded;
- the map was anchored to the same board placement, or `board.pose_in_map` is
  an explicitly surveyed replacement;
- the localization stack exposes the configured initialization service;
- the configured LiDAR topic and static TF are available;
- the vehicle is stationary and the velocity source is publishing.

Check the service first:

```bash
ros2 service list | grep '^/localization/initialize$'
```

The default launch also expects `autoware_vehicle_msgs/VelocityReport` on
`/vehicle/status/velocity_status`. The initializer uses that velocity to reject
poses observed while the vehicle is moving.

## Start the combined path

```bash
ros2 launch reflective_pose_autoware board_pose_initializer.launch.xml \
    config_file:=/home/you/detector.yaml \
    detector_params_file:=/path/to/board_detector.param.yaml \
    initializer_params_file:=/path/to/board_pose_initializer.param.yaml
```

This starts both nodes under the `/localization` namespace:

- `reflective_pose_ros/board_detector_node` subscribes to LiDAR, detects the
  board, and publishes `/localization/board_detector/board_pose`;
- `reflective_pose_autoware/board_pose_initializer` subscribes to that latched
  pose, applies its speed and attempt policy, and calls
  `initialize_service`.

Both nodes receive the same detector file. The shared `board:` block keeps the
board placement used by detection consistent with the map. The detector wiring
and initializer policy are separate ROS parameter files so each node owns only
the settings it can interpret.

## What happens after detection

1. The detector accumulates the configured number of scans while the vehicle
   is stopped.
2. It publishes one pose in `map` with a covariance.
3. The initializer checks current speed and whether the pose was accumulated
   before a recent motion event.
4. It sends `InitializeLocalization` with `method=AUTO`, allowing NDT align to
   refine the detector's pose guess.
5. A successful service response changes the initializer state to `done`.

The detector may publish a pose even when the Autoware service is unavailable;
that is a detector success but not a completed localization initialization.

## Monitor the handoff

```bash
ros2 topic echo /localization/board_detector/board_pose \
    --qos-durability transient_local
ros2 topic echo /diagnostics
ros2 service list | grep '^/localization/initialize$'
```

The two nodes publish diagnostics with their own state. A detector diagnostic
stuck in `wait_tf` or `accumulate` is a perception/setup problem. A detector
pose with an initializer state such as `service unavailable` is an Autoware
handoff problem.

## Configure retry and fallback policy

The initializer's ROS parameter file controls the handoff, not the detector:

```yaml
/**:
  ros__parameters:
    initialize_service: /localization/initialize
    max_speed_for_init: 0.05
    max_attempts: 5
    fallback_to_user_defined_pose: false
    pose_wait_timeout: 0.0
```

Keep `fallback_to_user_defined_pose` disabled unless sending the configured
`board.pose_in_map` is an explicitly reviewed safety decision. A fixed fallback
can turn a perception failure into a plausible but wrong localization.

One subtlety follows from the package split: detector failures do not become
initializer attempts because the initializer never sees a pose. If the board
is never detected, use the detector's diagnostics and RViz rejection markers;
do not expect `max_attempts` or the fallback policy to resolve it.

## If initialization fails

- **No board pose:** follow [live detection](live-detection.md) and
  [debugging detection](debugging.md).
- **Pose published, service unavailable:** start the localization stack or
  correct `initialize_service` in `board_pose_initializer.param.yaml`.
- **Pose ignored while moving:** stop the vehicle and check both the velocity
  topic and the detector's `twist_topic` motion guard in
  `board_detector.param.yaml`.
- **Service rejects the pose:** verify that the map was anchored with the same
  board dimensions and `board.pose_in_map`, then inspect the covariance and
  NDT/map configuration.
