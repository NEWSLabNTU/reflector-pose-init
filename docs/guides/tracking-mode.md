# Tracking a moving board

`board_tracking_node` finds the board in **every scan** and publishes where it
is relative to the vehicle. It was written for AutoSDV's coach-board pursuit
lab: a person holds a 0.6 m retroreflective square in front of a 1/10-scale
car and walks it, and the car follows at about 2 m.

It is not the initializer with a switch flipped. The two nodes share the
detector, the detector file format, the verdict and the debug markers, and
differ in everything else:

| | `board_detector_node` (initializer) | `board_tracking_node` |
|---|---|---|
| Question | where is the vehicle, given a surveyed board? | where is the board, relative to me? |
| Scans per detection | `accumulate_scans` (stacked) | 1, always |
| While moving | discards scans (motion guard) | the normal case |
| Output | vehicle pose, `PoseWithCovarianceStamped`, `map` | board pose, `PoseStamped`, `base_link` |
| Output stamp | processing time | the scan's stamp |
| Output QoS | latched (transient local) | volatile |
| Rate | as often as a batch completes | every scan, ~10 Hz |

A latched board position handed to a follower that subscribes late is a
position from the past, and stacking scans from a moving car smears the
board; those are why it is a separate node rather than a mode. The design
reasoning is in the module docstring of `tracking_node.py`.

## Run it

```bash
ros2 launch reflective_pose_ros board_tracking.launch.xml \
    profile:=autosdv_robin_w \
    input_topic:=/sensing/lidar/robin_w/pointcloud_raw \
    output_topic:=/perception/coach/board
```

| Launch argument | Default | Meaning |
|---|---|---|
| `profile` | `detector` | a detector file shipped by `reflective_pose_core`: `autosdv_vlp16`, `autosdv_robin_w`, or the initializer's `detector` |
| `config_file` | the profile's path | any detector file; overrides `profile` |
| `params_file` | `board_tracking.param.yaml` | the node's wiring |
| `input_topic` | `/sensing/lidar/top/pointcloud_raw_ex` | `PointCloud2` with `intensity` |
| `output_topic` | `/board_tracking/board` | where the board pose goes |
| `node_name` | `board_tracking` | |

Feed it the driver's own cloud (`/sensing/lidar/robin_w/pointcloud_raw` or the
VLP-16's `/sensing/lidar/top/pointcloud_raw_ex` equivalent), not a cropped or
downsampled one: the intensity field and the full density are what the gates
were set against.

### What is published

`~/board` (remap with `output_topic`), `geometry_msgs/PoseStamped`, only for a
scan whose single surviving candidate clears `min_confidence`:

- `header.frame_id` is `target_frame` (default `base_link`);
- `header.stamp` is the **scan's** stamp, so a consumer can measure and
  predict across the latency;
- `pose.position` is the board centre; range is `hypot(x, y)`, bearing
  `atan2(y, x)`, positive to the left;
- `pose.orientation` is the board frame: x the board normal pointing back at
  the car, y to the board's left as the car sees it, z up.

Nothing is published for a scan with no candidate, two candidates, or a
low-confidence one. The consumer owns the timeout; this node does not repeat
the last pose.

`/diagnostics`, once a second, `perception: board_tracking`:

| Key | Meaning |
|---|---|
| `scan_rate_hz`, `detection_rate_hz` | over the last `diagnostics_window` seconds |
| `processing_ms_mean`, `processing_ms_max` | callback time per scan |
| `scan_age_ms_mean`, `scan_age_ms_max` | now minus the scan stamp, at publication: driver plus transport plus processing |
| `confidence`, `confidence.*`, `range_m`, ... | the last scan's verdict, as for the initializer |

The level is OK at or above `min_detection_rate`, WARN below it (with the last
scan's verdict as the reason), and ERROR when no scan arrived in the window.

### Parameters

`board_tracking.param.yaml`; a test holds it to `TrackingParams`.

| Key | Default | Meaning |
|---|---|---|
| `sensor_frame` | `""` | empty takes each cloud's `header.frame_id` |
| `base_frame` | `base_link` | height datum of the detector's gates; static TF to the sensor, looked up once |
| `target_frame` | `""` | frame of the published pose; empty is `base_frame`, anything else is looked up at the scan stamp |
| `publish_debug` | `true` | `~/debug/rejected`, `~/debug/board_points`, `~/debug/board_outline` every scan |
| `diagnostics_window` | `2.0` | seconds |
| `min_detection_rate` | `5.0` | Hz; below it `/diagnostics` is WARN |

## The AutoSDV profiles

Two detector files in `reflective_pose_core/data/`, installed beside the
packaged `detector.yaml`:

| | `autosdv_robin_w.yaml` | `autosdv_vlp16.yaml` |
|---|---|---|
| `sensor` | `robin_w` | `vlp16` |
| board | 0.6 x 0.6 m | 0.6 x 0.6 m |
| centre gate | 1.05 +/- 0.35 m (0.7 to 1.4) | 0.9 +/- 0.5 m (0.4 to 1.4) |
| point band | 0.35 to 1.75 m | 0.1 to 1.75 m |
| range | 1.0 to 11.0 m | 1.0 to 11.0 m |
| `cluster_tolerance` | 0.15 m | 0.25 m |
| `cluster_min_points` | 40 | 15 |
| `extent_sampling_slack` | on | on |
| `verticality_max_dot` | 0.35 (~20 deg lean) | 0.35 |
| density gate | **off** (beam model approximate) | on |
| `intensity_threshold` | 100, **unmeasured** | 100, **unmeasured** |
| `base_link_height_above_ground` | 0.30, **unmeasured** | 0.30, **unmeasured** |

**The mounting height lives in the profile, not the TF.** AutoSDV's
`sensor_kit_calibration.yaml` puts every LiDAR at `z = 0` relative to
`base_link`, which is unmeasured. The detector's height gates read TF `z` plus
`base_link_height_above_ground`, so the profile sets the offset to the
LiDAR's height above the ground (about 0.30 m on a car 0.262 m tall) and the
two add up correctly. When the calibration gets a measured `z`, set the offset
to `0.0`: Autoware's `base_link` is on the ground. Until then the published
`pose.position.z` is relative to a `base_link` that sits at the LiDAR, about
0.3 m lower than true; range and bearing are unaffected.

## Hold the board where the LiDAR can see it

This decides more than any threshold. The LiDAR is about 0.30 m off the
ground, and its vertical field of view is a cone around that height:

| Sensor | Vertical FOV | Reaches at 2 m | Chest-height board (centre 1.0 m) | Knee-height board (centre 0.55 m) |
|---|---|---|---|---|
| Robin-W (model) | +/-35 deg | -1.1 to 1.70 m | fully in view from 1.43 m | fully in view from 0.8 m |
| VLP-16 | +/-15 deg | -0.24 to 0.84 m | top in view from 3.7 m | fully in view from 2.05 m |
| VLP-16 Hi-Res | +/-10 deg | -0.05 to 0.65 m | top in view from 5.7 m | fully in view from 3.1 m |

Detection in simulation, five seeds per range, bearings -16 to +16 deg, with
the shipped profile (the Hi-Res row uses the VLP-16 profile with
`sensor: vlp16_hires`):

| Sensor | Chest height (1.0 m) | Knee height (0.55 m) |
|---|---|---|
| Robin-W | 1.5 to 10 m, every scan | outside the profile's 0.7-1.4 m gate; at 0.75 m, 1.5 to 10 m |
| VLP-16 | **3.0 m** to 8.5 m; gaps at 9-9.5 m | **1.5 m** to 9.5 m |
| VLP-16 Hi-Res | **4.5 m** to 10 m | **2.5 m** to 10 m |

So:

- **Robin-W car:** hold the board at chest height. Above about 1.35 m its top
  leaves the fan inside 2 m.
- **VLP-16 car:** hold the board low, centre about 0.55 m (bottom edge near
  0.25 m). At chest height the car is blind to it at the 2 m standoff, and
  sees it only once it is 3 m away -- a follower would close in until it lost
  the board, then stop. The VLP-16 profile's centre gate accepts both heights,
  so nothing changes in software; the TA changes their grip.
- **Which VLP-16 is on the car?** AutoSDV's `VLP16.param.yaml` decodes with
  Nebula's `VLP16_hires.yaml` (+/-10 deg). If the unit is a Hi-Res, set
  `sensor: vlp16_hires` in a copy of the profile and expect the Hi-Res row
  above: no detection inside 2.5 m even at knee height. If it is a standard
  VLP-16, the driver is decoding with the wrong table and every elevation it
  publishes is two-thirds of the true one; fix the driver first.

The Robin-W numbers rest on an assumed symmetric +/-35 deg field. If a bag
shows the upper limit is lower, the chest-height board loses its top edge at
2 m first.

## What a single scan costs

`reflective_pose_sim.benchmark` times `detect_board` on the tracking scene
(board, five distractors, a 60 x 40 m hall), single-threaded:

```bash
OMP_NUM_THREADS=1 python3 -m reflective_pose_sim.benchmark --sensor robin_w \
    --config "$(ros2 pkg prefix reflective_pose_core)/share/reflective_pose_core/config/autosdv_robin_w.yaml"
```

Measured on an AMD Ryzen 9 9950X, one core, milliseconds per scan:

| Scene | Points/scan | Before (d61a929) | After |
|---|---|---|---|
| Robin-W, board at 1.5 m | 219k | 12.2 | 2.2 |
| Robin-W, board at 2 m | 219k | 10.0 | 1.8 |
| Robin-W, board at 8 m | 219k | 6.8 | 1.2 |
| VLP-16, board at 2 m | 27k | 1.6 | 0.53 |

The whole callback (reading the `PointCloud2`, detecting, publishing),
measured in the node on the same machine: Robin-W 3.9 ms per scan without
debug output and 4.5 ms with it; VLP-16 0.85 and 1.4 ms. Reading the cloud is
about half of the Robin-W figure.

These are a desktop's. A Jetson Orin's Cortex-A78AE cores are several times
slower per thread on this kind of numpy work; even at 5x, a Robin-W scan is
about 25 ms of a 100 ms budget. Run the benchmark on the vehicle before
trusting either number there, and turn `publish_debug` off if the CPU is
contended. The Robin-W model's 219k points per scan is an upper bound (192
lines x 1200 columns, no misses); a real frame is likely smaller.

## Desk test

```bash
ros2 launch reflective_pose_ros simulated_tracking.launch.xml                     # Robin-W, walking
ros2 launch reflective_pose_ros simulated_tracking.launch.xml \
    sensor:=vlp16 profile:=autosdv_vlp16 centre_height:=0.55                      # VLP-16, knee height
ros2 launch reflective_pose_ros simulated_tracking.launch.xml scene:=tracking_distractors
```

`scene:=walking` (the default) walks the board away from `range_m` at
`speed_mps` with a +/-10 deg sway, and starts over every 4 s;
`scene:=tracking` holds it still. Watch it with

```bash
ros2 topic hz /board_tracking/board
ros2 topic echo /diagnostics
```

Expected: about 10 Hz on `~/board` and `tracking at 10.0 Hz` on
`/diagnostics`; with `tracking_distractors`, nothing on `~/board` and a WARN
naming each rejected cluster.

## What the field bags must measure

Every number in the two profiles that is not geometry is a guess until a bag
of the real board on the real car says otherwise. Record, per LiDAR type, the
raw driver cloud plus `/tf_static`:

1. **Static board at 2, 4, 6 and 8 m**, held at the height the lab will use,
   facing the car, ~10 s each; plus 2 m at 30 and 60 deg of yaw.
2. **Walking**, away from 2 m to 8 m and back, at walking pace.
3. **No board**: the same site with the TA standing in place, wearing whatever
   the TA will wear.

From them, set:

| Value | How |
|---|---|
| `intensity_threshold` | the intensity histogram of the board's returns against everything else; the lowest value that keeps the board whole at 8 m |
| `base_link_height_above_ground` | a tape measure to the LiDAR's optical centre (or fix the calibration and set 0) |
| Robin-W vertical FOV and line spacing | the elevation of each scan line across the static board; replaces the `robin_w` model in `sensors.py`, then turn its density gate on |
| VLP-16 model | the unit's label; then the driver calibration (`VLP16.yaml` or `VLP16_hires.yaml`) and `sensor:` |
| `cluster_tolerance`, `cluster_min_points` | the board's cluster at 8 m must be one cluster with margin |
| `verticality_max_dot`, `centre_height_tolerance` | how far the TA actually leans and raises the board while walking |
| detection rate and latency | `/diagnostics` while replaying the walk; `processing_ms_*` on the vehicle's own CPU |

Replay a bag through the node exactly as the car will run it:

```bash
ros2 launch reflective_pose_ros board_tracking.launch.xml \
    profile:=autosdv_robin_w input_topic:=/sensing/lidar/robin_w/pointcloud_raw &
ros2 bag play <bag>
```

On a replay the scans carry the recording's stamps, so `scan_age_ms_*` is
meaningless; the rates and `processing_ms_*` are not.

Or replay it offline, scan by scan, without a running graph:

```bash
ros2 run reflective_pose_ros board_tracking_report <bag> --lidar vlp16     # or robin-w
ros2 run reflective_pose_ros board_tracking_report <bag> --profile my.yaml --topic /points
```

`board_tracking_report` (logic in `reflective_pose_core.tracking_report`)
prints the detection rate per range bin with the gate behind each miss, the
detection time per scan, the board's intensity against the background with a
suggested `intensity_threshold` (the middle of the gap between the two), and
the detected centre height against the centre-height gate. A static-board bag
is the input it is built for: a miss borrows the range of the nearest
detection in time. `--synthesize OUT --lidar vlp16 --centre-height 0.55`
writes such a bag from `reflective_pose_sim`, which is how the tool is tested.

## Known limits

- **One board in view.** Two candidates in one scan is `AMBIGUOUS` and
  publishes nothing. That includes a large retroreflective sign partly hidden
  behind the board, whose visible part can be board-sized; keep the chase
  clear of signage.
- **No association across scans.** Each scan is judged alone; holding on to
  the board between scans, predicting it, and timing out are the consumer's
  job.
- **No deskewing.** A scan is treated as instantaneous. The board spans a few
  degrees of a 360 deg sweep, a few milliseconds, so this costs centimetres at
  walking pace, not more.
- **Far range on a VLP-16.** Past about 8.5 m only one or two rings cross a
  0.6 m board, and a plane through them is ill-defined.
