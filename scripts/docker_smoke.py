#!/usr/bin/env python3
"""Non-motion Docker smoke test for wuji-retargeting.

This intentionally avoids importing or running teleop_real.py because that module
can construct the WH110 backend when executed. The smoke test only verifies that
container dependencies and bundled model assets are present.
"""

from __future__ import annotations

import importlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_ROOT = ROOT / "example"

REQUIRED_IMPORTS = (
    "wuji_retargeting",
    "mujoco",
    "mediapipe",
    "pyrealsense2",
)

# These are needed by existing teleop entrypoints at import time. Keeping them in
# the smoke test catches private/platform wheel issues before a user reaches the
# real hardware stage.
HARDWARE_PATH_IMPORTS = (
    "wujihandpy",
    "avp_stream",
)

# Import through example/ so the new device follows the same path as teleop scripts.
INPUT_DEVICE_IMPORTS = (
    "input_devices.manus_quantum",
)

REQUIRED_ASSETS = (
    "wuji_retargeting/wuji-description/hand/body/urdf/right.urdf",
    "wuji_retargeting/wuji-description/hand/body/mjcf/right.xml",
    "example/config/adaptive_analytical_video.yaml",
    "example/tuning_tool.py",
    "example/teleop_sim.py",
)


def check_imports(names: tuple[str, ...]) -> None:
    for name in names:
        importlib.import_module(name)
        print(f"import ok: {name}")


def check_assets(paths: tuple[str, ...]) -> None:
    for rel in paths:
        path = ROOT / rel
        if not path.exists():
            raise FileNotFoundError(
                f"required asset missing: {rel}; did you clone submodules with "
                "`git submodule update --init --recursive`?"
            )
        print(f"asset ok: {rel}")


def check_mediapipe_legacy_api() -> None:
    import mediapipe as mp

    if not hasattr(mp, "solutions") or not hasattr(mp.solutions, "hands"):
        version = getattr(mp, "__version__", "unknown")
        raise RuntimeError(
            "mediapipe legacy solutions API is unavailable "
            f"(version={version}); this repository's RealSense/MediaPipe input "
            "uses mp.solutions.hands, so keep the Docker image pinned to a "
            "compatible mediapipe release."
        )
    print(f"mediapipe legacy API ok: {mp.__version__} mp.solutions.hands")


def main() -> None:
    import sys

    if str(EXAMPLE_ROOT) not in sys.path:
        sys.path.insert(0, str(EXAMPLE_ROOT))

    check_imports(REQUIRED_IMPORTS)
    check_mediapipe_legacy_api()
    check_imports(HARDWARE_PATH_IMPORTS)
    check_imports(INPUT_DEVICE_IMPORTS)
    check_assets(REQUIRED_ASSETS)
    print("Docker smoke test passed: imports/assets only; no real hardware command ran.")


if __name__ == "__main__":
    main()
