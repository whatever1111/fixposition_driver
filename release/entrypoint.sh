#!/bin/bash
set -e
# shellcheck disable=SC1091
source /opt/fixposition/env.sh

# 镜像不带配置,靠 bind-mount 注入。忘了挂 -v 的话,rcl 只会甩一句
# "Error opening YAML file" 再打一屏 C++ 栈,根本看不出是缺文件 —— 所以先自己查。
# The image ships no config; it is bind-mounted in. Without the mount, rcl only
# says "Error opening YAML file" and dumps a C++ stack trace, which does not
# point at the missing mount at all — so check for it ourselves first.
CONFIG="${FP_CONFIG:-/config/fixposition.yaml}"
if [[ "$*" == *"${CONFIG}"* ]] && [[ ! -f "${CONFIG}" ]]; then
  echo "ERROR: config not found at ${CONFIG}" >&2
  echo "" >&2
  echo "This image ships no configuration — mount one in:" >&2
  echo "  docker run --rm --network host -v /dev/shm:/dev/shm \\" >&2
  echo "      -v /path/to/your.yaml:${CONFIG} <image>" >&2
  echo "" >&2
  echo "Both --network host and -v /dev/shm:/dev/shm are required, or the driver" >&2
  echo "runs but silently receives nothing (Fast DDS shared memory lives in /dev/shm)." >&2
  exit 2
fi

exec "$@"
