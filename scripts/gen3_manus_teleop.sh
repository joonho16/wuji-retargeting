#!/usr/bin/env bash
# One operator entrypoint for the Gen3 VIVE + MANUS + WH110 teleop stack.
# This script wraps existing Compose/safety scripts; it does not bypass their
# real-hardware latches.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)

usage() {
  cat <<'USAGE'
Usage:
  scripts/gen3_manus_teleop.sh <command> [args...]

Safe / inspection commands:
  help                         Show this help
  status                       Show required tracked/private assets and git state
  compose-config               Validate root MANUS/WH110 compose.yaml
  compose-gen3-config          Validate docker-compose.gen3-vive.yml
  smoke                        Run safe root Docker smoke service (no hardware)
  init-calib [dir]             Create ignored Gen3 real calibration template

Gen3 VIVE commands:
  gen3 <profile> [args...]     Run scripts/gen3_vive_compose_rm.sh <profile>
                               Profiles include receiver, wrapper, servo,
                               full-fake, full-fake-rviz, full-real, real-robot,
                               deadman, tools, test, debug.
  deadman [args...]            Run interactive Gen3 keyboard deadman helper

MANUS / WH110 commands:
  manus-bridge                 Run MANUS SDK -> UDP bridge via MANUS Core
  manus-check                  Wait for one MANUS UDP frame (no hand motion)
  tuning                       Run MANUS tuning GUI (no hand motion)
  sim                          Run MANUS -> MuJoCo simulation (no hand motion)
  hand                         Run MANUS -> real WH110 hand service

Combined real run:
  full-real --print            Print the four commands without starting them
  full-real [--session NAME]   Start the four-command real stack in tmux windows

Common environment:
  HAND=left|right                         default: left
  MANUS_CORE_IP=<windows-manus-core-ip>    required for manus-bridge/full-real
  MANUS_PORT=15020                         host UDP port consumed by WH110 service
  MANUS_UDP_PORT=15020                     bridge UDP target, defaults to MANUS_PORT
  GEN3_ROBOT_IP=<robot-ip>                 required for Gen3 real robot/full-real
  GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib
  GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml
  CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES  required for real WH110

Private assets that must be supplied locally and are intentionally not committed:
  manus-sdk/ManusSDK_v3.1.1/               proprietary MANUS SDK extraction
  ~/.wuji                                  Wuji Studio per-device assets
  .local/gen3_vive_calib/real_gen3_vive_calibration.yaml

Examples:
  scripts/gen3_manus_teleop.sh status
  scripts/gen3_manus_teleop.sh init-calib
  scripts/gen3_manus_teleop.sh gen3 full-fake
  MANUS_CORE_IP=192.168.50.132 HAND=left scripts/gen3_manus_teleop.sh manus-bridge
  HAND=left MANUS_PORT=15020 CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
    scripts/gen3_manus_teleop.sh hand
  GEN3_ROBOT_IP=192.168.50.10 MANUS_CORE_IP=192.168.50.132 HAND=left \
    CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
    scripts/gen3_manus_teleop.sh full-real --print
USAGE
}

cd "$repo_root"

source_env_file() {
  local file=$1
  if [[ -f "$file" ]]; then
    set -a
    # shellcheck source=/dev/null
    source "$file"
    set +a
  fi
}

# Optional local env files. They are ignored; examples are tracked.
source_env_file .env.gen3-manus
source_env_file .env

q() { printf '%q' "$1"; }

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 127
  }
}

require_env() {
  local missing=0
  for name in "$@"; do
    if [[ -z "${!name:-}" ]]; then
      echo "Missing required environment: $name" >&2
      missing=1
    fi
  done
  (( missing == 0 )) || exit 2
}

set_full_real_defaults() {
  export HAND=${HAND:-left}
  export MANUS_PORT=${MANUS_PORT:-15020}
  export MANUS_UDP_PORT=${MANUS_UDP_PORT:-$MANUS_PORT}
  export MANUS_UDP_HOST=${MANUS_UDP_HOST:-127.0.0.1}
  export MANUS_HOST=${MANUS_HOST:-0.0.0.0}
  export GEN3_VIVE_CALIBRATION_DIR=${GEN3_VIVE_CALIBRATION_DIR:-.local/gen3_vive_calib}
  export GEN3_VIVE_CALIBRATION=${GEN3_VIVE_CALIBRATION:-/calib/real_gen3_vive_calibration.yaml}
}

calibration_host_file() {
  local calib_dir=${GEN3_VIVE_CALIBRATION_DIR:-.local/gen3_vive_calib}
  local calib_container=${GEN3_VIVE_CALIBRATION:-/calib/real_gen3_vive_calibration.yaml}
  local rel
  case "$calib_container" in
    /calib/*) rel=${calib_container#/calib/} ;;
    *) echo "GEN3_VIVE_CALIBRATION must be under /calib/<file>; got $calib_container" >&2; return 2 ;;
  esac
  [[ -n "$rel" && "$rel" != *..* ]] || { echo "Unsafe calibration path: $calib_container" >&2; return 2; }
  case "$calib_dir" in
    /*) printf '%s/%s\n' "$calib_dir" "$rel" ;;
    *) printf '%s/%s/%s\n' "$repo_root" "$calib_dir" "$rel" ;;
  esac
}

require_full_real_env() {
  set_full_real_defaults
  require_cmd docker
  require_cmd tmux
  require_env GEN3_ROBOT_IP MANUS_CORE_IP
  if [[ ${CONFIRM_REAL_WH110:-} != "I_UNDERSTAND_WH110_MOVES" ]]; then
    echo "Refusing real WH110 startup: set CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES after staged checks." >&2
    exit 64
  fi
  local calib_file
  calib_file=$(calibration_host_file)
  if [[ ! -f "$calib_file" ]]; then
    echo "Missing Gen3 real calibration: $calib_file" >&2
    echo "Run: scripts/gen3_manus_teleop.sh init-calib" >&2
    exit 65
  fi
  if ! grep -Eq '^[[:space:]]*real_robot_calibration_reviewed:[[:space:]]*true([[:space:]]*(#.*)?)?$' "$calib_file"; then
    echo "Calibration is not marked reviewed: $calib_file" >&2
    echo "Edit measured values and set real_robot_calibration_reviewed: true before real robot use." >&2
    exit 66
  fi
}

full_real_commands() {
  set_full_real_defaults
  local cdroot="cd $(q "$repo_root")"
  local gen3="$cdroot && GEN3_ROBOT_IP=$(q "${GEN3_ROBOT_IP:-<GEN3_ROBOT_IP>}") GEN3_VIVE_CALIBRATION_DIR=$(q "$GEN3_VIVE_CALIBRATION_DIR") GEN3_VIVE_CALIBRATION=$(q "$GEN3_VIVE_CALIBRATION") ./scripts/gen3_vive_compose_rm.sh full-real"
  local deadman="$cdroot && ./scripts/gen3_vive_enter_deadman.sh"
  local bridge="$cdroot && MANUS_BUILD_GRPC=1 MANUS_CORE_IP=$(q "${MANUS_CORE_IP:-<MANUS_CORE_IP>}") HAND=$(q "$HAND") MANUS_UDP_HOST=$(q "$MANUS_UDP_HOST") MANUS_UDP_PORT=$(q "$MANUS_UDP_PORT") docker compose --profile manus-sdk run --build --rm manus-sdk-udp-bridge-unicast"
  local hand="$cdroot && HAND=$(q "$HAND") MANUS_PORT=$(q "$MANUS_PORT") CONFIRM_REAL_WH110=$(q "${CONFIRM_REAL_WH110:-<CONFIRM_REAL_WH110>}") docker compose --profile real-wh110 --profile manus run --rm --service-ports real-wh110-manus"
  printf 'GEN3:   %s\n' "$gen3"
  printf 'DEADMAN:%s\n' " $deadman"
  printf 'MANUS:  %s\n' "$bridge"
  printf 'HAND:   %s\n' "$hand"
}

run_full_real() {
  local print_only=0
  local session=${GEN3_MANUS_TMUX_SESSION:-gen3-manus-teleop}
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --print|--dry-run) print_only=1; shift ;;
      --session) session=${2:?--session requires a name}; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *) echo "Unknown full-real option: $1" >&2; exit 2 ;;
    esac
  done

  if (( print_only )); then
    full_real_commands
    return 0
  fi

  require_full_real_env
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "tmux session already exists: $session" >&2
    echo "Attach with: tmux attach -t $(q "$session")" >&2
    exit 67
  fi

  local cdroot="cd $(q "$repo_root")"
  local cmd_gen3="$cdroot && GEN3_ROBOT_IP=$(q "$GEN3_ROBOT_IP") GEN3_VIVE_CALIBRATION_DIR=$(q "$GEN3_VIVE_CALIBRATION_DIR") GEN3_VIVE_CALIBRATION=$(q "$GEN3_VIVE_CALIBRATION") ./scripts/gen3_vive_compose_rm.sh full-real"
  local cmd_deadman="$cdroot && ./scripts/gen3_vive_enter_deadman.sh"
  local cmd_bridge="$cdroot && MANUS_BUILD_GRPC=1 MANUS_CORE_IP=$(q "$MANUS_CORE_IP") HAND=$(q "$HAND") MANUS_UDP_HOST=$(q "$MANUS_UDP_HOST") MANUS_UDP_PORT=$(q "$MANUS_UDP_PORT") docker compose --profile manus-sdk run --build --rm manus-sdk-udp-bridge-unicast"
  local cmd_hand="$cdroot && HAND=$(q "$HAND") MANUS_PORT=$(q "$MANUS_PORT") CONFIRM_REAL_WH110=$(q "$CONFIRM_REAL_WH110") docker compose --profile real-wh110 --profile manus run --rm --service-ports real-wh110-manus"

  tmux new-session -d -s "$session" -n gen3 "$cmd_gen3"
  tmux new-window -t "$session" -n deadman "$cmd_deadman"
  tmux new-window -t "$session" -n manus "$cmd_bridge"
  tmux new-window -t "$session" -n hand "$cmd_hand"
  tmux select-window -t "$session:gen3"
  echo "Started tmux session: $session"
  echo "Attach/detach: tmux attach -t $(q "$session")   (detach with Ctrl-b d)"
  tmux attach -t "$session"
}

status() {
  echo "Project: gen3-manus-teleop (package imports intentionally remain wuji_retargeting)"
  echo "Repo: $repo_root"
  echo
  echo "Git status:"
  git status --short --branch || true
  echo
  echo "Required tracked/recoverable files:"
  local required=(
    docker-compose.gen3-vive.yml
    .env.gen3-vive.example
    docker/gen3-vive/Dockerfile
    example/arm_teleop/gen3_vive_ros2_wrapper.py
    scripts/gen3_vive_compose_rm.sh
    scripts/gen3_manus_teleop.sh
    compose.yaml
    Dockerfile
  )
  for path in "${required[@]}"; do
    [[ -e "$path" ]] && printf '  ok  %s\n' "$path" || printf '  MISS %s\n' "$path"
  done
  echo
  echo "Private/ignored examples:"
  for path in .omx .local/gen3_vive_calib/real_gen3_vive_calibration.yaml manus-sdk/ManusSDK_v3.1.1 .env .env.gen3-manus .env.gen3-vive; do
    if git check-ignore -q "$path" 2>/dev/null; then
      printf '  ignored  %s\n' "$path"
    else
      printf '  CHECK    %s (not ignored)\n' "$path"
    fi
  done
}

cmd=${1:-help}
shift || true
case "$cmd" in
  help|-h|--help) usage ;;
  status) status ;;
  compose-config) docker compose --profile realsense --profile gui --profile real-wh110 --profile manus-sdk --profile manus config ;;
  compose-gen3-config) docker compose -f docker-compose.gen3-vive.yml --profile receiver --profile wrapper --profile servo --profile full-fake --profile full-fake-rviz --profile full-real --profile real-robot --profile deadman --profile tools --profile test --profile debug config ;;
  smoke) docker compose run --rm smoke ;;
  init-calib) exec "$script_dir/gen3_vive_init_real_calib.sh" "${1:-.local/gen3_vive_calib}" ;;
  gen3)
    [[ $# -ge 1 ]] || { echo "Usage: $0 gen3 <profile> [args...]" >&2; exit 2; }
    exec "$script_dir/gen3_vive_compose_rm.sh" "$@"
    ;;
  deadman) exec "$script_dir/gen3_vive_enter_deadman.sh" "$@" ;;
  manus-bridge)
    export HAND=${HAND:-left}
    export MANUS_UDP_PORT=${MANUS_UDP_PORT:-${MANUS_PORT:-15020}}
    export MANUS_BUILD_GRPC=1
    require_env MANUS_CORE_IP
    exec docker compose --profile manus-sdk run --build --rm manus-sdk-udp-bridge-unicast
    ;;
  manus-check) exec docker compose --profile manus run --rm --service-ports manus-check ;;
  tuning) exec docker compose --profile manus --profile gui run --rm --service-ports tuning-manus ;;
  sim) exec docker compose --profile manus --profile gui run --rm --service-ports teleop-sim-manus ;;
  hand)
    if [[ ${CONFIRM_REAL_WH110:-} != "I_UNDERSTAND_WH110_MOVES" ]]; then
      echo "Refusing real WH110 startup: set CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES after staged checks." >&2
      exit 64
    fi
    exec docker compose --profile real-wh110 --profile manus run --rm --service-ports real-wh110-manus
    ;;
  full-real) run_full_real "$@" ;;
  *) echo "Unknown command: $cmd" >&2; usage >&2; exit 2 ;;
esac
