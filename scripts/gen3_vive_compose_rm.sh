#!/usr/bin/env bash
# Run Gen3 VIVE Docker Compose stacks without leaving compose containers behind.
# Docker Compose does not support `up --rm`, so this wrapper runs `up` attached
# and always tears the project down on exit/Ctrl-C.
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/gen3_vive_compose_rm.sh <profile> [service-or-up-args...]

Profiles:
  receiver     UDP receiver only
  wrapper      dry-run mapper/wrapper only
  servo        fake Gen3 Servo + dry-run wrapper (no receiver/deadman helper)
  full-fake    receiver + fake Gen3 Servo + dry-run wrapper + dry-run deadman
  full-real    receiver + real Gen3 Servo + real-gated wrapper
  real-robot   real Gen3 Servo + real-gated wrapper, without receiver
  deadman      dry-run deadman helper only
  tools        interactive/tooling services
  test         unit/compile verification
  debug        UDP tcpdump helper

Examples:
  scripts/gen3_vive_compose_rm.sh full-fake
  scripts/gen3_vive_compose_rm.sh receiver
  scripts/gen3_vive_init_real_calib.sh
  GEN3_ROBOT_IP=192.168.1.10 \
    GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib \
    GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml \
    scripts/gen3_vive_compose_rm.sh full-real

Notes:
  - This is the compose-up equivalent of --rm: on exit it runs
    `docker compose down --remove-orphans` for this compose project.
  - If any attached service exits, the whole stack is stopped and cleaned up.
  - Real-robot profiles fail before Docker starts unless the repo/host
    calibration directory exists, /calib/<file> maps to a real file, and that
    file contains `real_robot_calibration_reviewed: true`.
  - Do not pass -d/--detach; detached mode would be cleaned up immediately.
  - One-shot tooling commands should still use `docker compose run --rm ...`.
USAGE
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

profile=$1
shift

case "$profile" in
  receiver|wrapper|servo|full-fake|full-real|real-robot|deadman|tools|test|debug)
    ;;
  *)
    echo "Unknown Gen3 VIVE compose profile: $profile" >&2
    usage >&2
    exit 2
    ;;
esac

for arg in "$@"; do
  case "$arg" in
    -d|--detach)
      echo "Detached mode is not supported by this --rm-style wrapper." >&2
      echo "Run attached and stop with Ctrl-C so cleanup can remove containers." >&2
      exit 2
      ;;
  esac
done

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
compose_file="$repo_root/docker-compose.gen3-vive.yml"

preflight_real_robot() {
  local robot_ip=${GEN3_ROBOT_IP:-}
  local calib_container=${GEN3_VIVE_CALIBRATION:-}
  local calib_dir=${GEN3_VIVE_CALIBRATION_DIR:-./example/config}
  local calib_rel
  local calib_host_dir
  local calib_host_file

  if [[ -z "$robot_ip" || "$robot_ip" == "0.0.0.0" ]]; then
    echo "GEN3_ROBOT_IP must be set for $profile before starting real robot services." >&2
    exit 2
  fi

  if [[ -z "$calib_container" ]]; then
    echo "GEN3_VIVE_CALIBRATION must point to reviewed real calibration YAML under /calib." >&2
    echo "Example: GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml" >&2
    exit 2
  fi

  case "$calib_container" in
    /calib/*)
      calib_rel=${calib_container#/calib/}
      ;;
    *)
      echo "GEN3_VIVE_CALIBRATION must be inside the compose calibration mount: /calib/<file>." >&2
      echo "Got: $calib_container" >&2
      exit 2
      ;;
  esac

  if [[ -z "$calib_rel" || "$calib_rel" == *..* ]]; then
    echo "Refusing unsafe GEN3_VIVE_CALIBRATION path: $calib_container" >&2
    exit 2
  fi

  case "$calib_dir" in
    /*) calib_host_dir=$calib_dir ;;
    *) calib_host_dir=$repo_root/$calib_dir ;;
  esac
  calib_host_file=$calib_host_dir/$calib_rel

  if [[ ! -d "$calib_host_dir" ]]; then
    echo "GEN3_VIVE_CALIBRATION_DIR does not exist on the host: $calib_host_dir" >&2
    echo "Create it or point GEN3_VIVE_CALIBRATION_DIR at the directory containing $calib_rel." >&2
    exit 2
  fi

  if [[ ! -f "$calib_host_file" ]]; then
    echo "Calibration file is missing on the host/repo checkout: $calib_host_file" >&2
    echo "Container path $calib_container maps to host file $calib_host_file." >&2
    echo "Create a repo-local working file with: scripts/gen3_vive_init_real_calib.sh" >&2
    exit 2
  fi

  if ! grep -Eq '^[[:space:]]*real_robot_calibration_reviewed:[[:space:]]*true([[:space:]]*(#.*)?)?$' "$calib_host_file"; then
    echo "Real robot calibration is not marked reviewed: $calib_host_file" >&2
    echo "Edit measured calibration values, then set: real_robot_calibration_reviewed: true" >&2
    exit 2
  fi
}

case "$profile" in
  full-real|real-robot)
    preflight_real_robot
    ;;
esac

# Keep cleanup scoped and predictable. Users may override COMPOSE_PROJECT_NAME
# when they intentionally need multiple same-profile stacks.
profile_slug=${profile//-/_}
export COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-gen3_vive_${profile_slug}}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  docker compose \
    --project-directory "$repo_root" \
    -f "$compose_file" \
    --profile "$profile" \
    down --remove-orphans
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cd "$repo_root"

docker compose \
  --project-directory "$repo_root" \
  -f "$compose_file" \
  --profile "$profile" \
  up --build --remove-orphans --abort-on-container-exit "$@"
