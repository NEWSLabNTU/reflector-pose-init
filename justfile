# ============================================================================
# Shortcuts for speeding up development
# ============================================================================

# lidar setting
lidar_topic := "/velodyne_points"
lidar_frame_id := "velodyne"

# Default recipe: show all available commands
default:
    @just --list

# Publish baselink->LiDAR, parameters mirrored from src/param/autoware_individual_params/individual_params/config/default/golfcart_sensor_kit/sensor_kit_calibration.yaml
fake-tf:
    #!/usr/bin/env bash
    ros2 run tf2_ros static_transform_publisher \
        --x 0.46 --y 0.0 --z 1.96 \
        --roll 0.0 --pitch 0.0 --yaw -0.05 \
        --frame-id base_link \
        --child-frame-id {{lidar_frame_id}}

# The detector alone. Publishes ~/board_pose; calls no service, so it is safe
# against a running stack. This is what `dry_run:=true` used to mean.
launch:
    #!/usr/bin/env bash
    ros2 launch reflective_pose_ros board_detector.launch.xml \
        input_topic:={{lidar_topic}}

# Detector plus the Autoware handoff. This one CALLS /localization/initialize.
launch-autoware:
    #!/usr/bin/env bash
    ros2 launch reflective_pose_autoware board_pose_initializer.launch.xml \
        input_topic:={{lidar_topic}}

rviz:
    #!/usr/bin/env bash
    rviz2 -d packages/reflective_pose_ros/rviz/board_detector.rviz

# Every package's tests, without a ROS workspace: the three ROS-free packages
# need only PYTHONPATH, the two ROS ones need /opt/ros sourced.
test:
    #!/usr/bin/env bash
    set -e
    root="$(pwd)/packages"
    export PYTHONPATH="$root/reflective_pose_core:$root/reflective_pose_sim:$root/reflective_pose_cli:$root/reflective_pose_ros:$root/reflective_pose_autoware:${PYTHONPATH:-}"
    for pkg in core sim cli ros autoware; do
        d="$root/reflective_pose_$pkg"
        ls "$d"/test/*.py >/dev/null 2>&1 || { printf '%-26s (no tests)\n' "$pkg"; continue; }
        printf '%-26s ' "$pkg"
        (cd "$d" && python3 -m pytest test -q 2>&1 | tail -1)
    done
