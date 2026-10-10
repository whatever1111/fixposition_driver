# fixposition_driver_m20

The Deep Robotics M20 localization interface for the Fixposition ROS2 driver. See
[../M20.md](../M20.md) for the design and the interface contract; this file documents the code and
the parameters.

## Layout

| File | Contents | ROS? |
|---|---|---|
| `deskew.hpp/cpp` | cloud layout parsing, IMU rate integration, the in-place motion compensation | no |
| `status_monitor.hpp/cpp` | input liveness, sticky failure flags, the `total_status` verdict | no |
| `time_utils.hpp/cpp` | stamp arithmetic, sensor-to-host clock retiming | no |
| `m20_params.hpp` | the parameter struct | no |
| `odometry_source_policy.hpp` | which odometry feed each mode follows, and what the other one may still do | no |
| `m20_localization.hpp/cpp` | publishers, subscribers, the deskew worker, parameter loading | yes |

The first five are deliberately ROS-free and are what the unit tests exercise; only
`m20_localization` touches rclcpp.

## Threads

| Thread | Does |
|---|---|
| driver parser | `OnOdometry(sample, SENSOR_ENU)` → in `fpa_odomenu` mode publishes `/ODOM` immediately; in `subscribe` mode only marks the RTK input |
| node executor | lidar / IMU / leg-odometry callbacks, the followed `/ODOM` in `subscribe` mode (`OnOdometry(sample, SUBSCRIBED_ODOM)`), the status fallback timer, the startup watchdog |
| deskew worker | one scan at a time: deskew, publish `/LOC_BODY_POINTS`, publish the scheduled status |

Each mutex is a leaf — none is ever held while taking another:

| Mutex | Guards |
|---|---|
| `state_mutex_` | newest stamp and body velocity |
| `imu_mutex_` | the IMU window (copied out before deskewing, so the callback is never blocked by it) |
| `cloud_mutex_` | the single pending scan slot |
| `status_mutex_` | the 2 Hz publish schedule |

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
| `matching_odom_topic` | `""` | scan-matching odometry to monitor in SLAM mode; empty means matching health is always OK |
| `wheelspeed_enable` / `wheelspeed_scale_factor` / `wheelspeed_use_y` | `true` / `1000.0` / `false` | leg odometry → wheelspeed aiding (m/s → mm/s) |
| `motion_info_max_rate_hz` | `50.0` | cap on forwarding; the sensor needs no more |
| `timeouts.{lidar,imu,leg_odom,rtk,matching}_sec` | `0.5` / `0.3` / `0.5` / `1.0` / `0.5` | how long an input may be silent before it counts as missing |
| `timeouts.error_hold_sec` | `5.0` | how long a failure keeps being reported |
| `overtime_threshold_sec` | `0.15` | above the 0.1 s scan period, so jitter does not flag the status |
| `timestamp_skew_threshold_sec` | `1.0` | odometry/lidar clock disagreement that counts as a timestamp error |
| `boot_reinit_grace_sec` / `boot_reinit_max` | `45.0` / `5` | startup deafness self-heal; 0 disables |

## Status semantics

`total_status` follows the OEM specification: `0` uninitialized, `1` normal, `2` low quality, `3` lost.

The sub-field flags (`exec_val`, `loss_val`, `input_val`) have **no published specification**. This
implementation uses the values the field-validated Python bridge used — errors `0` ok / `1` error,
inputs `1` present / `0` missing — defined as constants at the top of `status_monitor.hpp`. If the
OEM ever documents them, that is the only place to change.

`tag` and `reflective_strip` are always reported missing: neither is fitted on this robot.

`input_val.rtk` says that odometry is arriving, not that GNSS is healthy. In `fpa_odomenu` mode it
follows the sensor's valid ENU output. In `subscribe` mode it means "the pipeline's `/ODOM` or the
sensor's ENU output is arriving": either one keeps it set. Arriving means accepted, not merely
received: a sensor output counts only while it is valid and finite, and the pipeline's `/ODOM` only
when it is finite and every publication enabled for it completed without an error. Finite covers
the position, the orientation and both velocities. A pipeline that has stopped therefore shows in
`loss_val.odom_error` and `total_status`, not in this flag. Both the flag and `odom_error` use
`timeouts.rtk_sec`.

What `subscribe` mode reports, with the lidar, the IMU and the leg odometry arriving, scan-matching
monitoring off (`matching_odom_topic` empty) or its input arriving, no failure being held, and the
timeouts expired. A monitored scan-matching input that has gone silent turns a `total_status` of `1`
into `2`:

| Situation | `total_status` | `loss_val.odom_error` | `input_val.rtk` |
|---|---|---|---|
| the pipeline's `/ODOM` and the sensor's ENU output both arriving | 1 | 0 | 1 |
| the pipeline stopped, the sensor still has a valid ENU pose | 3 | 1 | 1 |
| the pipeline publishing, the sensor without a valid ENU pose or disconnected | 1 | 0 | 1 |
| both stopped after the pipeline had published | 3 | 1 | 0 |
| nothing from the pipeline yet, the sensor has a valid ENU pose | 0 | 1 | 1 |
