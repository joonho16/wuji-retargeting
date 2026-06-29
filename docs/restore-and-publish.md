# Restore and personal-repo publish guide

This project is a conservative integration of upstream `wuji-retargeting` with
Gen3 VIVE teleop and MANUS/WH110 operator tooling. It is intended to be usable
after deleting the surrounding development workspace as long as the Git
repo plus external/private assets are restored.

## What is tracked

Tracked/recoverable assets include:

- Root MANUS/WH110 `Dockerfile` and `compose.yaml`
- Tracked MANUS SDK -> Wuji UDP bridge source under `tools/manus_bridge/`
- Gen3 VIVE ROS2 `docker-compose.gen3-vive.yml`
- `docker/gen3-vive/`
- `example/arm_teleop/`
- Gen3 VIVE configs under `example/config/gen3_vive_*`
- Gen3 VIVE scripts under `scripts/gen3_vive_*`
- Unified operator launcher `scripts/gen3_manus_teleop.sh`
- No-hardware tests under `tests/`
- Restore/operator docs
- `.env.gen3-vive.example` and `.env.gen3-manus.example`

## What is intentionally not tracked

These are private, generated, or licensed runtime assets:

- `.omx/` workflow logs/state
- `.local/`, especially `.local/gen3_vive_calib/real_gen3_vive_calibration.yaml`
- `manus-sdk/ManusSDK_v3.1.1/` and MANUS SDK zips/libraries
- `~/.wuji` Wuji Studio per-device assets
- `.env`, `.env.gen3-manus`, other local env files
- `__pycache__`, `.pyc`, build artifacts, logs

## Fresh clone restore

```bash
git clone --recurse-submodules <your-repo-url> gen3-manus-teleop
cd gen3-manus-teleop
git submodule update --init --recursive
```

Optional local environment defaults:

```bash
cp .env.gen3-manus.example .env.gen3-manus
# edit MANUS_CORE_IP, GEN3_ROBOT_IP, HAND, ports, etc.
```

Prepare private assets:

1. Extract the licensed MANUS SDK to `manus-sdk/ManusSDK_v3.1.1/`. The tracked bridge in `tools/manus_bridge/` will build against this local SDK; no SDK source or binaries are committed.
2. Ensure Wuji Studio has provisioned host assets under `~/.wuji` for WH110.
3. Generate Gen3 real calibration template:
   ```bash
   scripts/gen3_manus_teleop.sh init-calib
   ```
4. Edit `.local/gen3_vive_calib/real_gen3_vive_calibration.yaml` with measured
   values. Set `real_robot_calibration_reviewed: true` only after review.

Build/check:

```bash
scripts/gen3_manus_teleop.sh compose-config
scripts/gen3_manus_teleop.sh compose-gen3-config
scripts/gen3_manus_teleop.sh status
# Optional host-side source/API check when the licensed SDK is present locally:
MANUS_SDK_ROOT=manus-sdk/ManusSDK_v3.1.1 tools/manus_bridge/build_manus_wuji_udp_bridge.sh --compile-only /tmp/manus_wuji_udp_bridge.o
# Full bridge linking/running is normally done inside the Docker image, where the MANUS dependencies are installed.
```

## Operator entrypoint

Use one script for all common flows:

```bash
scripts/gen3_manus_teleop.sh help
scripts/gen3_manus_teleop.sh gen3 full-fake
scripts/gen3_manus_teleop.sh manus-check
scripts/gen3_manus_teleop.sh full-real --print
```

`full-real` starts four tmux windows only after real-hardware env and safety
latches are set.

## Naming and publishing

The runtime/project can be published under a new repository name such as
`gen3-manus-teleop`. The Python package remains `wuji_retargeting` for
compatibility with upstream imports, submodules, and existing scripts. A broad
package rename is intentionally out of scope for this safety-focused cleanup.

To publish later without pushing from this automation session:

```bash
# inspect current upstream remote
git remote -v

# keep upstream reference if desired
git remote rename origin upstream

# add your own empty GitHub repo
git remote add origin git@github.com:<you>/gen3-manus-teleop.git

# push yourself
git push -u origin main
```

If your default branch has a different name, replace `main` accordingly.
