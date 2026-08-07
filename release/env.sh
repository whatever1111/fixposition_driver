#!/bin/bash
# Environment for the binary-only M20 fixposition driver release.
# Source this from any location; every path is relative to this file.
#
#   source /opt/fixposition/env.sh
#   ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec \
#       --ros-args --params-file /path/to/your.yaml

_fp_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_fp_distro="$(cat "${_fp_here}/.ros_distro")"

# ROS setup scripts reference unset variables; tolerate callers running `set -u`.
_fp_had_u=0
case $- in *u*) _fp_had_u=1; set +u ;; esac
# shellcheck disable=SC1091
source "/opt/ros/${_fp_distro}/setup.bash"
# shellcheck disable=SC1091
source "${_fp_here}/install/setup.bash"
if [ "${_fp_had_u}" -eq 1 ]; then set -u; fi

export LD_LIBRARY_PATH="${_fp_here}/fpsdk/lib:${LD_LIBRARY_PATH:-}"
unset _fp_here _fp_distro _fp_had_u
