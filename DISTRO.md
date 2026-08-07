# Which branch for which ROS 2 distro

| branch       | distro | Fast DDS | deployment on the robot          |
|--------------|--------|----------|----------------------------------|
| `m20-foxy`   | Foxy   | 2.0.x    | native (`colcon` on the robot)   |
| `m20-humble` | Humble | 2.6.x    | container (`fslam-m20:<arch>`)   |
| `m20-jazzy`  | Jazzy  | 2.14.x   | not started                      |

The source tree is deliberately distro-portable (`__has_include` for tf2 headers,
`ConstSharedPtr` callbacks, `if(COMMAND rosidl_get_typesupport_target)`, explicit
C++17, `shared_future`), so these branches should stay nearly identical and
cherry-picking between them should be trivial. Keep it that way: prefer a portable
construct over a branch-local edit.

## What actually differs per branch

| difference | foxy | humble |
|---|---|---|
| `drdds` | system package (vendored copy has `COLCON_IGNORE`) | vendored copy is built |
| SDK `bagwriter.cpp` | must be removed before building — see below | builds fine |
| packaging | native colcon workspace | docker image layered on `fslam-humble` |

## The SDK caveat (Foxy)

`fixposition-sdk/fpsdk_ros2/src/bagwriter.cpp` includes
`rosbag2_storage/storage_options.hpp`, which Foxy places under `rosbag2_cpp/`.
The driver never uses `BagWriter`, so the build tooling deletes that source (plus
`bagwriter.hpp` and `ext/rosbag2_cpp_writer.hpp`) before compiling the SDK. It is
a submodule, so the deletion lives in the deploy repo's build script rather than
here.

## Why Foxy exists at all

The M20's OEM lidar driver publishes `/LIDAR/POINTS` from a DDS participant bound
to `127.0.0.1` only. Fast DDS 2.6 (Humble) stopped announcing a loopback locator
for participants that also have real interfaces, so a Humble process cannot
complete discovery with that writer — while Foxy's 2.0 can, and does, with no
special configuration. Measured on ChongQing_106: a Foxy process using the OEM's
own `/opt/robot/fastdds.xml` sees `pubs=2` and receives ~9.3 Hz of cloud.

Do not try to solve this inside a Humble process with a second, loopback-only
participant. That was tried: it crashes the OEM lidar process on every match
(~12 restarts/minute), each dead process leaks its `fast_datasharing_*` segments,
and `/dev/shm` fills until the whole OEM stack crash-loops.
