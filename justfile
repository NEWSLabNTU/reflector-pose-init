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

launch:
    #!/usr/bin/env bash
    ros2 launch golfcart_board_initializer board_initializer.launch.xml \
        dry_run:=true \
        input_topic:={{lidar_topic}}

rviz:
    #!/usr/bin/env bash
    rviz2 -d rviz/board_initializer.rviz