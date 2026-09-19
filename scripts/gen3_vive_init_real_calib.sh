#!/usr/bin/env bash
# Create a repo-local real calibration working file without assuming host $HOME.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)
template="$repo_root/example/config/gen3_vive_real_calibration.template.yaml"
target_dir=${1:-.local/gen3_vive_calib}
case "$target_dir" in
  /*) target_dir_abs=$target_dir ;;
  *) target_dir_abs=$repo_root/$target_dir ;;
esac
target_file="$target_dir_abs/real_gen3_vive_calibration.yaml"

mkdir -p "$target_dir_abs"
if [[ -e "$target_file" ]]; then
  echo "Calibration file already exists, not overwriting: $target_file"
else
  cp "$template" "$target_file"
  echo "Created calibration working file: $target_file"
fi

cat <<MSG

Calibration working file (existing files are preserved):
  $target_file

New files use the saved Gen3 + left WH110 settings:
  VIVE axes: (-Z, -X, +Y); TCP hand-back offset: [0, -0.070, 0.120] m
  Linear/angular scale: 1.0; speed caps: 0.5 m/s, 100 deg/s
  real_robot_calibration_reviewed: true for this saved setup

Run full-real from the repo root with repo-relative paths:
  GEN3_ROBOT_IP=<robot-ip> \\
  GEN3_VIVE_CALIBRATION_DIR=${target_dir} \\
  GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml \\
  scripts/gen3_manus_teleop.sh full-real
MSG
