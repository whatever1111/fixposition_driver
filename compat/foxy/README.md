# Foxy SDK build copy

For SDK `d27853e2b6bffbc32f7d264deb97dfbb689252a5`, apply
`no-bagwriter.patch` only to a private SDK source copy used for a Foxy build:

```sh
test "$ROS_DISTRO" = foxy
sha256sum compat/foxy/no-bagwriter.patch
patch --batch --forward --fuzz=0 -d "$SDK_BUILD_COPY" -p1 < compat/foxy/no-bagwriter.patch
```

Stop on patch failure. Record the SDK SHA and patch SHA with the build evidence.
Do not patch the submodule checkout or apply this patch on Humble.

The patch removes ROS2 BagWriter and its rosbag2 includes because Foxy's
rosbag2 API is incompatible. It preserves ROS2 time conversion, logging,
message conversion, and `fpsdk_common/test/ros2_test.cpp` (ConvTime).
Build `fpsdk_common` and the driver; SDK tools that require BagWriter are
outside this compatibility path. Humble keeps the upstream BagWriter.

For a separate no-ROS common build, clear `ROS_VERSION` and `ROS_PACKAGE_PATH`
and set both `FPSDK_USE_ROS1=OFF` and `FPSDK_USE_ROS2=OFF`.
