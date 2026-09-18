#!/usr/bin/env bash
# Compatibility entrypoint for the focused A/B hold operator window.
# Uses docker compose run --rm so the helper container is removed on exit.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
compose_file="$repo_root/docker-compose.gen3-vive.yml"

cd "$repo_root"
export GEN3_OPERATOR_DIR=${GEN3_OPERATOR_DIR:-$repo_root/.local/gen3_operator}
mkdir -p "$GEN3_OPERATOR_DIR"
if [[ -z ${DISPLAY:-} ]]; then
  echo 'A/B hold requires the Linux desktop DISPLAY; run from a desktop terminal.' >&2
  exit 2
fi
operator_project=${COMPOSE_PROJECT_NAME:-wuji-retargeting}
status=0
docker compose \
  --project-directory "$repo_root" \
  --project-name "$operator_project" \
  -f "$compose_file" \
  --profile tools \
  run --rm --build operator \
  python3 -m example.arm_teleop.gen3_operator_keyboard "$@" || status=$?

if (( status == 42 )); then
  exec python3 "$script_dir/gen3_teleop_shutdown.py" \
    --repo "$repo_root" --operator-dir "$GEN3_OPERATOR_DIR" \
    --operator-project "$operator_project" \
    --tmux-session "${GEN3_MANUS_TMUX_SESSION:-}"
fi
exit "$status"
