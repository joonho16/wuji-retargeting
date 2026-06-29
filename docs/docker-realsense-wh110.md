# Docker Compose: RealSense + WH110 staged runbook

This runbook is for an Ubuntu x86_64 desktop host running `wuji-retargeting` in a
Docker container. It keeps the default path safe: `docker compose up` only runs a
non-motion smoke test. RealSense, GUI simulation/tuning, and real WH110 teleop are
separate opt-in stages.

## Safety model

- Default services do **not** mount `/dev`, do **not** open WH110 USB, and do
  **not** run `teleop_real.py`.
- `realsense-check`, `teleop-sim-realsense`, and `tuning-realsense` can read the
  RealSense camera but do not enable a real hand.
- `real-wh110` is the only service that can enable and move WH110. It is behind
  the `real-wh110` Compose profile and an additional confirmation variable:
  `CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES`.

Run the stages in order and stop if any earlier stage fails.

## 0. Host prerequisites

```bash
git clone --recurse-submodules https://github.com/wuji-technology/wuji-retargeting.git
cd wuji-retargeting
# If you already cloned without submodules:
git submodule update --init --recursive

docker --version
docker compose version
```

For GUI stages on X11 desktops, allow local Docker clients to use the display:

```bash
xhost +local:docker
# Optional cleanup after the session:
# xhost -local:docker
```

For RealSense, plug the camera into the host and ensure it is visible under USB
or video devices. The Compose services mount `/dev` and use cgroup rules for USB
and UVC character devices (`81` and `189`).

For WH110, provision the hand once with Wuji Studio on the host. The real-hardware
service mounts `${HOME}/.wuji` read-only into `/root/.wuji` because `wujihandpy`
uses those per-device assets. Identify the stable USB-CDC serial device before
running real teleop:

```bash
ls -l /dev/serial/by-id/ 2>/dev/null || true
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
```

The `real-wh110` profile grants serial cgroup access for common USB-CDC majors:
`166` (`ttyACM`) and `188` (`ttyUSB`). Keep the real profile disabled until the
camera and simulation/tuning stages are verified.

## 1. Build and safe smoke test

```bash
docker compose build
docker compose run --rm smoke
# Equivalent safe default; this runs only the smoke service:
docker compose up --abort-on-container-exit
```

The smoke test imports the container dependencies and verifies bundled model
assets. It does not instantiate `wujihandpy.Hand()` and does not run real teleop.
If it reports missing `wuji-description` assets, initialize submodules and rebuild.
Compose runs the source copied into the image; rebuild after changing code or
most files. The staged RealSense/WH110 services bind-mount `./example/config`,
so YAML tuning changes made on the host are visible without rebuilding.

Inside this Linux Docker image, the Compose GUI services call `python` directly.
The `mjpython` launcher is not installed in the image and is only needed for some
non-Linux/macOS viewer setups. The image pins `mediapipe==0.10.21` because newer
MediaPipe releases removed the `mp.solutions.hands` API used by the current
RealSense input adapter. GUI services also default to Mesa software rendering
and disable the DRI3 GLX path
(`LIBGL_ALWAYS_SOFTWARE=1`, `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`,
`LIBGL_DRI3_DISABLE=1`) to avoid Docker/host nouveau or NVIDIA GLX driver
mismatches.

## 2. RealSense enumeration, no real hand

```bash
docker compose --profile realsense run --rm realsense-check
```

Expected success prints at least one RealSense device name, serial, and firmware.
If no camera is attached, this command exits non-zero after printing
`RealSense devices found: 0`; that is a camera/USB staging issue, not a WH110
motion risk.

## 3. RealSense + MuJoCo simulation, no real hand

```bash
HAND=right docker compose --profile realsense --profile gui run --rm teleop-sim-realsense
```

This launches the existing command inside `example/`:

```bash
python teleop_sim.py --realsense --show-video --hand right
```

Use this stage to confirm the camera pose pipeline and MuJoCo visualization before
connecting or enabling WH110.

## 4. RealSense tuning GUI, no real hand

```bash
HAND=right docker compose --profile realsense --profile gui run --rm tuning-realsense
```

This launches:

```bash
python tuning_tool.py --realsense --show-video --hand right
```

Tune and verify retargeting behavior here before any real-hand run.
This service mounts `./example/config` read-write, so editing
`example/config/adaptive_analytical_video.yaml` on the host should auto-reload in
the tuning viewer. The simulation and real-hand services mount the same config
directory read-only to use the staged/tuned parameters.

## 5. Explicit WH110 real teleop

Only run this after stages 1-4 pass and the physical setup is clear of people and
obstructions. This command can enable and move the hand.

```bash
HAND=right \
HAND_SERIAL=<optional-wujihand-serial> \
CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
docker compose --profile real-wh110 run --rm real-wh110
```

If `HAND_SERIAL` is omitted, `wujihandpy.Hand()` will auto-select according to the
SDK behavior. When multiple WH110 devices are connected, set `HAND_SERIAL`.
The real-hand service shows the RealSense/MediaPipe video overlay by default
(`SHOW_VIDEO=1`). Set `SHOW_VIDEO=0` if you need to disable the GUI window.

The service runs the existing WH110 command:

```bash
CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
  python teleop_real.py --realsense --show-video --hand right --hand-model wh110 [--hand-serial ...]
```

If the service exits with a refusal message, confirm you set
`CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES` and that `${HOME}/.wuji` exists on
the host.

## Troubleshooting

- **`pyrealsense2` import/build failure**: use x86_64 and Python 3.11 as provided
  by this Dockerfile; rebuild without cache if the base image changed.
- **`wujihandpy` cannot find assets**: verify Wuji Studio populated
  `${HOME}/.wuji` and that Compose mounts it into `/root/.wuji`.
- **GUI window does not open**: verify `DISPLAY`, `/tmp/.X11-unix`, and `xhost
  +local:docker`; for Wayland-only sessions, switch to an X11 session or provide
  an XWayland-compatible setup.
- **RealSense device busy**: close `realsense-viewer` or other camera clients on
  the host, then rerun `realsense-check`.
- **WH110 not discovered**: check `/dev/serial/by-id`, `/dev/ttyACM*`, and
  `/dev/ttyUSB*` on the host; make sure the `real-wh110` profile is used so the
  serial device cgroup rules are active.
