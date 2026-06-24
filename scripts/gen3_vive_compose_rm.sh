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
  GEN3_ROBOT_IP=192.168.1.10 scripts/gen3_vive_compose_rm.sh full-real
  scripts/gen3_vive_compose_rm.sh receiver

Notes:
  - This is the compose-up equivalent of --rm: on exit it runs
    `docker compose down --remove-orphans` for this compose project.
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
  up --build --remove-orphans "$@"
