#!/usr/bin/env bash
set -eo pipefail

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # ROS setup scripts may reference unset AMENT/COLCON tracing variables.
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

export PYTHONPATH="/workspace/wuji-retargeting:${PYTHONPATH:-}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

exec "$@"
