# MANUS SDK local mount

This directory is intentionally kept out of Git except for this README and the
local `.gitignore`.

Place the proprietary MANUS Core SDK extraction here when you need the real
MANUS bridge services:

```text
manus-sdk/ManusSDK_v3.1.1/
```

The root `compose.yaml` mounts that directory into the container at
`/opt/manus/ManusSDK_v3.1.1` by default. The SDK zip, extracted libraries,
build outputs, and any MANUS vendor files are private/runtime assets and must
not be committed.

Expected restore flow after a fresh clone:

1. Obtain MANUS Core SDK 3.1.1 from MANUS through the normal licensed channel.
2. Extract it locally to `manus-sdk/ManusSDK_v3.1.1/`.
3. Keep this README tracked, but leave all SDK contents ignored.
4. Run `scripts/gen3_manus_teleop.sh manus-bridge` with `MANUS_CORE_IP` set.
