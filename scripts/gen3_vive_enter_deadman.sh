#!/usr/bin/env bash
# Run the interactive Gen3 VIVE keyboard deadman helper in the ROS2 Docker image.
# Uses docker compose run --rm so the helper container is removed on exit.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
compose_file="$repo_root/docker-compose.gen3-vive.yml"

topic=${GEN3_VIVE_PHYSICAL_DEADMAN_TOPIC:-/gen3_vive/physical_deadman}
rate=${GEN3_VIVE_ENTER_DEADMAN_RATE_HZ:-20}

cd "$repo_root"
exec docker compose \
  --project-directory "$repo_root" \
  -f "$compose_file" \
  --profile tools \
  run --rm --build shell \
  bash -lc 'exec python3 example/arm_teleop/gen3_vive_enter_deadman.py --topic "$0" --rate-hz "$1" "${@:2}"' \
  "$topic" "$rate" "$@"
