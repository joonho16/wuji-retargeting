#!/usr/bin/env bash
# Build the tracked MANUS Core -> Wuji UDP bridge against a locally mounted
# licensed MANUS SDK. This keeps proprietary SDK files out of Git while making
# the bridge itself reproducible from tracked source.
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
source_file=${MANUS_BRIDGE_SOURCE:-$script_dir/manus_wuji_udp_bridge.cpp}
compile_only=0
if [[ ${1:-} == "--compile-only" ]]; then
  compile_only=1
  shift
fi
output_file=${1:-${MANUS_BRIDGE_OUT:-/tmp/manus_wuji_udp_bridge}}
sdk_root=${MANUS_SDK_ROOT:-/opt/manus/ManusSDK_v3.1.1}

if [[ ! -f "$source_file" ]]; then
  echo "Missing tracked MANUS bridge source: $source_file" >&2
  exit 2
fi

if [[ ! -d "$sdk_root" ]]; then
  cat >&2 <<MSG
Missing MANUS SDK root: $sdk_root
Mount or extract the licensed SDK, e.g. ./manus-sdk/ManusSDK_v3.1.1,
or set MANUS_SDK_ROOT / MANUS_SDK_HOST_DIR for Docker Compose.
MSG
  exit 2
fi

sdk_api_dir=""
for candidate in \
  "$sdk_root/SDKMinimalClient_Linux/ManusSDK" \
  "$sdk_root/SDKClient_Linux/ManusSDK" \
  "$sdk_root/ROS2/ManusSDK"; do
  if [[ -f "$candidate/include/ManusSDK.h" && -f "$candidate/lib/libManusSDK.so" ]]; then
    sdk_api_dir=$candidate
    break
  fi
done

if [[ -z "$sdk_api_dir" ]]; then
  cat >&2 <<MSG
Could not find MANUS SDK headers/libs below: $sdk_root
Expected one of:
  SDKMinimalClient_Linux/ManusSDK/include + lib
  SDKClient_Linux/ManusSDK/include + lib
  ROS2/ManusSDK/include + lib
MSG
  exit 2
fi

mkdir -p "$(dirname -- "$output_file")"

if (( compile_only )); then
  g++ -std=c++17 -O2 -Wall -Wextra \
    "$source_file" \
    -I"$sdk_api_dir/include" \
    -pthread \
    -c -o "$output_file"
  cat <<MSG
Compiled tracked MANUS bridge object:
  source: $source_file
  sdk:    $sdk_api_dir
  output: $output_file
MSG
  exit 0
fi

g++ -std=c++17 -O2 -Wall -Wextra \
  "$source_file" \
  -I"$sdk_api_dir/include" \
  -L"$sdk_api_dir/lib" \
  -Wl,-rpath,"$sdk_api_dir/lib" \
  -Wl,--allow-shlib-undefined \
  -lManusSDK -pthread \
  -o "$output_file"

cat <<MSG
Built tracked MANUS bridge:
  source: $source_file
  sdk:    $sdk_api_dir
  output: $output_file
MSG
