# ============================================================================
# Binary-only release image for the M20 fixposition driver.
#
# One Dockerfile serves every ROS 2 distro branch:
#   docker build -f release/Dockerfile --build-arg ROS_DISTRO=foxy   --target runtime ...
#   docker build -f release/Dockerfile --build-arg ROS_DISTRO=humble --target runtime ...
#
# Stages:
#   compiled — ROS base + toolchain. Builds the Fixposition SDK (plain cmake,
#              PROJ off, tests off) and the driver workspace (colcon, tests
#              off), sources still in place.
#   builder  — compiled, with the staged tree at /opt/fixposition pruned down
#              to binaries: no headers, no C/C++ sources, no python bindings,
#              no dev files.
#   runtime  — ROS base + runtime libs, carrying ONLY the pruned /opt/fixposition.
#              A guard step fails the build if anything source-like survived.
#   test     — compiled + the fixposition_driver_m20 unit tests, built and run.
#              Fails unless every test passed and all the expected ones ran:
#                docker build -f release/Dockerfile --build-arg ROS_DISTRO=... --target test .
#              Last in the file and not an ancestor of runtime, so no runtime
#              build runs it: BuildKit builds only what the target depends on,
#              the legacy builder only the stages up to the target.
#
# The runtime stage is what gets published; release artifacts must never
# contain source code. Always name the target: a build without --target
# produces the last stage, which is test.
# ============================================================================
ARG ROS_DISTRO=foxy

FROM ros:${ROS_DISTRO} AS compiled
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential cmake git python3-colcon-common-extensions \
        libboost-dev libboost-date-time-dev libyaml-cpp-dev zlib1g-dev libeigen3-dev \
        nlohmann-json3-dev libssl-dev pkg-config \
        ros-${ROS_DISTRO}-rosbag2-cpp \
        ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-eigen ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Older distros ship json.hpp without the cmake package config the SDK asks for.
RUN if [ -z "$(find /usr -name nlohmann_jsonConfig.cmake -print -quit)" ]; then \
        mkdir -p /usr/lib/cmake/nlohmann_json && \
        { echo 'add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)'; \
          echo 'set_target_properties(nlohmann_json::nlohmann_json PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "/usr/include")'; \
          echo 'set(nlohmann_json_FOUND TRUE)'; \
        } > /usr/lib/cmake/nlohmann_json/nlohmann_jsonConfig.cmake; \
    fi

WORKDIR /w
COPY . /w/repo

# Patch only the private SDK copy on Foxy; retain ROS2 time/logging helpers.
RUN cp -a /w/repo/fixposition-sdk /w/sdk && \
    git -C /w/repo/fixposition-sdk rev-parse HEAD && \
    rm -rf /w/sdk/.git && \
    if [ "${ROS_DISTRO}" = "foxy" ]; then \
        sha256sum /w/repo/compat/foxy/no-bagwriter.patch && \
        git -C /w/sdk apply --check /w/repo/compat/foxy/no-bagwriter.patch && \
        git -C /w/sdk apply /w/repo/compat/foxy/no-bagwriter.patch; \
    fi

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
        cmake -S /w/sdk/fpsdk_common -B /w/build_fpsdk_common \
              -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_INSTALL_PREFIX=/opt/fixposition/fpsdk \
              -DCMAKE_PREFIX_PATH=/opt/fixposition/fpsdk \
              -DFPSDK_USE_PROJ=OFF -DFPSDK_USE_FFMPEG=OFF -DFPSDK_USE_BZ2=OFF \
              -DFPSDK_USE_ROS1=OFF -DFPSDK_USE_ROS2=ON -DFPSDK_BUILD_TESTING=OFF && \
        cmake --build /w/build_fpsdk_common --parallel "$(nproc)" && \
        cmake --install /w/build_fpsdk_common

# Driver workspace: exactly the packages a ROS 2 deployment needs. The ROS 1
# driver and the SDK tree stay out of colcon's sight. The vendored drdds is
# built here so the image is self-contained; robot-native deployments on Foxy
# use the system drdds instead (the release workflow strips it from the Foxy
# native tarball).
RUN mkdir -p /w/ws/src && \
    for p in fixposition_driver_lib fixposition_driver_msgs fixposition_driver_ros2 \
             fixposition_driver_m20 rtcm_msgs drdds; do \
        cp -a /w/repo/${p} /w/ws/src/; \
    done

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /w/ws && \
    colcon build --install-base /opt/fixposition/install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DCMAKE_PREFIX_PATH=/opt/fixposition/fpsdk \
                     -DBUILD_TESTING=OFF


FROM compiled AS builder
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Prune to binaries only. What remains: shared libs, executables, msg/launch/
# config share files, and colcon's generated environment scripts.
RUN rm -rf /w/repo /w/sdk /w/ws /w/build_* && \
    find /opt/fixposition -depth -type d \
        \( -name include -o -name 'python3*' -o -name cmake -o -name pkgconfig \) -exec rm -rf {} + && \
    find /opt/fixposition -type f \
        \( -name '*.h' -o -name '*.hpp' -o -name '*.hh' -o -name '*.hxx' \
        -o -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o -name '*.a' \) -delete && \
    echo "${ROS_DISTRO}" > /opt/fixposition/.ros_distro

COPY release/env.sh release/entrypoint.sh /opt/fixposition/
RUN chmod +x /opt/fixposition/env.sh /opt/fixposition/entrypoint.sh


FROM ros:${ROS_DISTRO} AS runtime
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.source="https://github.com/whatever1111/fixposition_driver"
LABEL org.opencontainers.image.description="M20 fixposition driver, binary-only (${ROS_DISTRO})"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rosbag2-cpp \
        ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-eigen ros-${ROS_DISTRO}-tf2-geometry-msgs \
        "$(if [ "${ROS_DISTRO}" = "foxy" ]; then echo libyaml-cpp0.6; else echo libyaml-cpp0.7; fi)" \
        "$(if [ "${ROS_DISTRO}" = "foxy" ]; then echo libboost-date-time1.71.0; else echo libboost-date-time1.74.0; fi)" \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/fixposition /opt/fixposition

# Guard: refuse to produce an image with source aboard, and make sure the
# executable actually resolves every shared library in this runtime.
RUN bad="$(find /opt/fixposition \( -type d -name include \) -print -o \
        \( -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' \
           -o -name '*.h' -o -name '*.hpp' -o -name '*.hh' -o -name '*.hxx' \) \) -print -o \
        \( -type f -name '*.py' ! -name '_local_setup_util*' \) -print)" && \
    if [ -n "${bad}" ]; then echo "source files in release image:"; echo "${bad}"; exit 1; fi && \
    source /opt/fixposition/env.sh && \
    exe=/opt/fixposition/install/fixposition_driver_ros2/lib/fixposition_driver_ros2/fixposition_driver_ros2_exec && \
    test -x "${exe}" && \
    if ldd "${exe}" | grep 'not found'; then echo "unresolved libraries"; exit 1; fi && \
    if ldd /opt/fixposition/install/fixposition_driver_m20/lib/libfixposition_driver_m20.so | grep 'not found'; then \
        echo "unresolved libraries (m20)"; exit 1; fi

# Config is the product's business (fslam etc.) — mount it, don't bake it:
#   docker run --network host -v /path/to/your.yaml:/config/fixposition.yaml <image>
# On the M20 robot: only --network host (or none). Never the default bridge.
ENTRYPOINT ["/opt/fixposition/entrypoint.sh"]
CMD ["ros2", "run", "fixposition_driver_ros2", "fixposition_driver_ros2_exec", \
     "--ros-args", "--params-file", "/config/fixposition.yaml"]


FROM compiled AS test
ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Same workspace and install base as the build above, reconfigured with the
# tests on, so only the driver library and M20 test executables are new.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /w/ws && \
    colcon build --install-base /opt/fixposition/install \
        --packages-select fixposition_driver_lib fixposition_driver_m20 \
        --cmake-force-configure \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
                     -DCMAKE_PREFIX_PATH=/opt/fixposition/fpsdk \
                     -DBUILD_TESTING=ON

# colcon test exits 0 when a test fails; the verdict is colcon test-result,
# which exits non-zero on any failure or error. It also exits 0 when nothing
# ran at all, so each expected gtest executable must show up in its listing
# with at least one test and none failed, errored or skipped.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /w/ws && \
    colcon test --install-base /opt/fixposition/install \
        --packages-select fixposition_driver_lib fixposition_driver_m20 \
        --event-handlers console_direct+ && \
    colcon test-result --all --verbose | tee /w/test-result.txt && \
    for t in test_deskew test_status_monitor test_time_utils test_odometry_source_policy \
             fixposition_driver_lib_tcp_reconnect_test; do \
        if ! grep -E "/${t}\.gtest\.xml: [1-9][0-9]* tests?, 0 errors, 0 failures, 0 skipped" \
                /w/test-result.txt; then \
            echo "m20 tests: no clean result for ${t}"; exit 1; \
        fi; \
    done
