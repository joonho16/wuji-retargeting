#!/usr/bin/env bash
set -eo pipefail

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # ROS setup scripts may reference unset AMENT/COLCON tracing variables.
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

if [[ -f /opt/gen3_kortex_overlay/share/kortex_driver/local_setup.bash ]]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/gen3_kortex_overlay/share/kortex_driver/local_setup.bash
  set -u
fi

if [[ -f /opt/gen3_servo_overlay/share/moveit_servo/local_setup.bash ]]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/gen3_servo_overlay/share/moveit_servo/local_setup.bash
  set -u
fi

if [[ -f /opt/gen3_twist_overlay/share/picknik_twist_controller/local_setup.bash ]]; then
  set +u
  source /opt/gen3_twist_overlay/share/picknik_twist_controller/local_setup.bash
  set -u
fi

export PYTHONPATH="/workspace/wuji-retargeting:${PYTHONPATH:-}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

exec "$@"
