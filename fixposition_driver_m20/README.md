# fixposition_driver_m20

The Deep Robotics M20 localization interface for the Fixposition ROS2 driver. See
[../M20.md](../M20.md) for the design and the interface contract; this file documents the code and
the parameters.

## Layout

| File | Contents | ROS? |
|---|---|---|
| `deskew.hpp/cpp` | cloud layout parsing, IMU rate integration, the in-place motion compensation | no |
| `status_monitor.hpp/cpp` | input liveness, sticky failure flags, the initial pose hold, the verdict | no |
| `status_contract.hpp/cpp` | the wire contract: the pure encoder that numbers `/LOCATION_STATUS` as the OEM's specification v1.2.0 does | no |
| `time_utils.hpp/cpp` | stamp arithmetic, sensor-to-host clock retiming | no |
| `m20_params.hpp` | the parameter struct | no |
| `odometry_source_policy.hpp` | which odometry feed each mode follows, and what the other one may still do | no |
| `m20_localization.hpp/cpp` | publishers, subscribers, the deskew worker, parameter loading | yes |

The first six are deliberately ROS-free and are what most unit tests exercise; only
`m20_localization` touches rclcpp, and `test_m20_localization` covers its parameter loading and
subscriptions on a real node.

## Threads

| Thread | Does |
|---|---|
| driver parser | `OnOdometry(sample, SENSOR_ENU)` → in `fpa_odomenu` mode publishes `/ODOM` immediately; in `subscribe` mode only marks the RTK input |
| node executor | lidar / IMU / leg-odometry callbacks, the followed `/ODOM` in `subscribe` mode (`OnOdometry(sample, SUBSCRIBED_ODOM)`), initial poses, the status fallback timer, the startup watchdog |
| deskew worker | one scan at a time: deskew, publish `/LOC_BODY_POINTS`, publish the scheduled status |

Each mutex is a leaf — none is ever held while taking another:

| Mutex | Guards |
|---|---|
| `state_mutex_` | newest stamp and body velocity |
| `imu_mutex_` | the IMU window (copied out before deskewing, so the callback is never blocked by it) |
| `cloud_mutex_` | the single pending scan slot |
| `status_mutex_` | the 2 Hz publish schedule |
| `StatusMonitor::hold_mutex_` | the initial pose hold |

Liveness and error flags live in atomics inside `StatusMonitor`, so any thread can mark them.

## Odometry feeds

Two feeds reach `OnOdometry()`, and each call names its own (`OdomFeed`). `DecideOdomInput()` in
`odometry_source_policy.hpp` decides what becomes of a sample. Each mode follows exactly one feed, and
the other one never takes its place.

| Mode | Feed | Published | Stamp and velocity reference | Marks the RTK input | Odometry alive, initialized, counted |
|---|---|---|---|---|---|
| `fpa_odomenu` | the sensor's ENU output (`SENSOR_ENU`) | `/ODOM` and the handler alias | yes | yes | yes |
| `subscribe` | the followed `/ODOM` (`SUBSCRIBED_ODOM`) | the handler alias | yes | yes | yes |
| `subscribe` | the sensor's ENU output (`SENSOR_ENU`) | no | no | yes, when valid and finite | no |

A sample from the followed feed that is marked invalid is dropped; one with a non-finite value is
dropped and raises `numeric_error`. In `subscribe` mode an invalid or non-finite sample from the
sensor is dropped without either, because it is not the localization that failed.

## Parameters

All under the `m20.` prefix. Defaults are the M20's field-validated values.

| Parameter | Default | Meaning |
|---|---|---|
| `enable` | `false` | master switch; nothing is created when false |
| `odom_source` | `fpa_odomenu` | `fpa_odomenu` publishes `/ODOM` from the sensor; `subscribe` follows an `/ODOM` published by a SLAM pipeline and takes only an RTK liveness mark from the sensor (see [Odometry feeds](#odometry-feeds)) |
| `odom_topic` / `odom_frame_id` / `odom_child_frame_id` | `/ODOM` / `map` / `""` | the OEM's own convention, including the empty child frame |
| `odom_depth` / `odom_reliable` | `10` / `true` | `/ODOM` and `/LOCATION_STATUS` QoS |
| `zero_covariance` | `true` | the OEM strips covariance and downstream expects that |
| `retime_to_host_clock` / `retime_jump_sec` | `true` / `0.5` | sensor-to-host clock correction and the residual that counts as a clock step |
| `lidar_topic` / `imu_topic` / `motion_info_topic` | `/LIDAR/POINTS` / `/IMU` / `/MOTION_INFO` | inputs |
| `loc_body_points_topic` / `loc_body_frame_id` | `/LOC_BODY_POINTS` / `base_link` | deskewed cloud output |
| `loc_body_depth` / `loc_body_reliable` | `5` / `false` | cloud QoS; make it reliable if the consumer subscribes reliably |
| `deskew_enable` / `deskew_use_linear_velocity` | `true` / `true` | motion compensation and its translation term |
| `deskew_time_quantum_sec` | `0.001` | rotation lookup bucket; 0 evaluates one rotation per point |
| `lidar_to_base_xyzrpy` | six zeros | lidar→base extrinsic; identity on the M20, whose URDF transforms are all identity |
| `location_status_topic` / `location_status_rate_hz` | `/LOCATION_STATUS` / `2.0` | OEM contract |
| `initialpose_topic` | `/initialpose` | initial poses to report as initializing (see [Initial pose](#initial-pose)); empty disables |
| `initialpose_settle_sec` | `2.0` | how long the followed odometry must flow without a break after an initial pose; must be finite and > 0, or startup fails |
| `matching_odom_topic` | `""` | scan-matching odometry to monitor in SLAM mode; empty means matching health is always OK |
| `wheelspeed_enable` / `wheelspeed_scale_factor` / `wheelspeed_use_y` | `true` / `1000.0` / `false` | leg odometry → wheelspeed aiding (m/s → mm/s) |
| `motion_info_max_rate_hz` | `50.0` | cap on forwarding; the sensor needs no more |
| `timeouts.{lidar,imu,leg_odom,rtk,matching}_sec` | `0.5` / `0.3` / `0.5` / `1.0` / `0.5` | how long an input may be silent before it counts as missing |
| `timeouts.error_hold_sec` | `5.0` | how long a failure keeps being reported |
| `overtime_threshold_sec` | `0.15` | above the 0.1 s scan period, so jitter does not flag the status |
| `timestamp_skew_threshold_sec` | `1.0` | odometry/lidar clock disagreement that counts as a timestamp error |
| `boot_reinit_grace_sec` / `boot_reinit_max` | `45.0` / `5` | startup deafness self-heal; 0 disables |

## Status semantics

`StatusMonitor` reaches one verdict and one state per field; `status_contract.hpp` numbers them as
the OEM's "山猫M20 定位输出话题与格式说明 v1.2.0" does, which OEM firmware 120 (localization
baseline 3.5.13+) reads. Downstream treats `total_status` 0 and 2 as usable. OEM firmware before 120
reads a different numbering, which this driver no longer sends; a robot on such firmware stays on a
2.x release.

| Monitor state | Sent |
|---|---|
| `total_status`: no pose from the followed odometry yet | 7 initializing |
| `total_status`: an initial pose is settling ([Initial pose](#initial-pose)) | 7 initializing |
| `total_status`: normal | 0 |
| `total_status`: low quality — a monitored input missing or a failure held | 2 minor fault |
| `total_status`: lost — the followed odometry stopped or a non-finite value arrived | 3 lost |
| `input_val.{lidar,imu,leg_odom,rtk}`: arriving within its timeout | 1 normal |
| `input_val.{lidar,imu,leg_odom,rtk}`: silent past its timeout, or never received | 2 no data |
| `input_val.{tag,reflective_strip}`: neither is fitted on this robot | 0 not used |
| `loss_val.odom_error`: the followed odometry stopped | 2 hard loss |
| `loss_val.{leg_odom_error,matching_error,pub_error}` | 1 minor |
| `exec_val.{overtime,numeric_error,timestamp_error}` | 1 |

A loss or execution field that is not raised is 0. The lidar, the IMU, the leg odometry and the RTK
input are monitored in every configuration; a not-used input never degrades the verdict. The
specification defines a few values this driver never sends: `total_status` 1 (undefined), 4 and 6
(reserved) and 5 (lost, needs an initial pose — the driver cannot tell whether the pipeline would
recover without one), and input 3 (abnormal — nothing here tells an abnormal input from a missing
one).

### Initial pose

Unless `m20.initialpose_topic` is empty, the module subscribes it
(`geometry_msgs/PoseWithCovarianceStamped`, reliable, volatile, depth 10). It does not interpret the
pose; the localization pipeline applies it.
From each receipt `total_status` is 7 until the followed odometry has been flowing — as
`loss_val.odom_error` judges it, with `timeouts.rtk_sec` — without a break for
`m20.initialpose_settle_sec`, counted from its first sample after the pose. A break restarts the
count, and so does another pose. Then the usual verdict applies again, and a later stop of the
odometry is an ordinary loss (3). The other fields are reported as usual throughout, and the rule is
the same in both odometry source modes.

Settling is judged from the odometry because that is all the driver can see: LIO-SLAM's pose graph
acknowledges only a pose it has installed or queued, and rejects an operator's pose without an
acknowledgement while it is aligned to GNSS.

### Odometry and the RTK input

`input_val.rtk` says that odometry is arriving, not that GNSS is healthy. In `fpa_odomenu` mode it
follows the sensor's valid ENU output. In `subscribe` mode it means "the pipeline's `/ODOM` or the
sensor's ENU output is arriving": either one keeps it set. Arriving means accepted, not merely
received: a sensor output counts only while it is valid and finite, and the pipeline's `/ODOM` only
when it is finite and every publication enabled for it completed without an error. Finite covers
the position, the orientation and both velocities. A pipeline that has stopped therefore shows in
`loss_val.odom_error` and `total_status`, not in this flag. Both the flag and `odom_error` use
`timeouts.rtk_sec`.

What `subscribe` mode reports with the lidar, the IMU and the leg odometry arriving, scan-matching
monitoring off (`matching_odom_topic` empty) or its input arriving, no failure being held, no
initial pose settling, and the timeouts expired. A monitored scan-matching input that has gone
silent turns normal into a minor fault (0 → 2); an initial pose that is still settling makes
`total_status` 7 in every row:

| Situation | `total_status` | `loss_val.odom_error` | `input_val.rtk` |
|---|---|---|---|
| the pipeline's `/ODOM` and the sensor's ENU output both arriving | 0 | 0 | 1 |
| the pipeline stopped, the sensor still has a valid ENU pose | 3 | 2 | 1 |
| the pipeline publishing, the sensor without a valid ENU pose or disconnected | 0 | 0 | 1 |
| both stopped after the pipeline had published | 3 | 2 | 2 |
| nothing from the pipeline yet, the sensor has a valid ENU pose | 7 | 2 | 1 |
