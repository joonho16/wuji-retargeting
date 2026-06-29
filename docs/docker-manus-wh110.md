# Docker MANUS bridge + WH110 staged runbook

This runbook adds a MANUS Quantum/Metaglove-compatible input path without
vendoring the proprietary MANUS SDK into the Docker image.  The container listens
for MediaPipe-order 21x3 hand keypoints over UDP JSON, then uses the existing
Wuji retargeting configs and teleop loops.

## What is installed, and what still comes from the MANUS SDK package?

The Docker image installs the MANUS Linux SDK system prerequisites from the
MANUS Linux guide: libudev, build-essential, libusb, zlib, ncurses, gdb,
OpenSSH server, ZMQ, libtool, git, and the MANUS-required gRPC/protobuf source
build (`gRPC v1.28.1`, protobuf from that gRPC tree).

The proprietary MANUS SDK package itself is still mounted at runtime instead of
copied into this repository/image. By default Compose expects it at:

```bash
./manus-sdk/ManusSDK_v3.1.1
```

Override that path when needed:

```bash
MANUS_SDK_HOST_DIR=/absolute/path/to/ManusSDK_v3.1.1 \
  docker compose --profile manus-sdk run --rm manus-sdk-shell
```

MANUS exposes glove data through its own SDK/Core stack and ROS2 packages. For
the Wuji retargeting loop, this repository tracks its own license-safe bridge
source under `tools/manus_bridge/`. The bridge builds against the mounted
licensed SDK at runtime and translates raw MANUS skeleton frames into the small
JSON protocol below. Use `manus-sdk-minimal-client` only for SDK/Core
connectivity checks, and use `manus-sdk-udp-bridge-unicast` for the real SDK ->
Wuji UDP bridge.

## UDP JSON protocol

Direct Python listener default: `127.0.0.1:15020/udp`.
Compose container listener: `0.0.0.0:15020/udp` internally, with the published host bind defaulting to `127.0.0.1:15020/udp` via `MANUS_PUBLISH_HOST=127.0.0.1`. Unauthenticated keypoint packets are therefore loopback-only by default.
Default sender target from the same host: `127.0.0.1:15020/udp` when the Compose
service is run with `--service-ports` or via `docker compose up`. If the MANUS bridge runs on another machine, set `MANUS_PUBLISH_HOST=0.0.0.0` for Compose or `--manus-host 0.0.0.0` for direct Python only on a controlled/trusted network.

Coordinates must be **MediaPipe Hands order**, shape `(21, 3)`.  Units are meters
by default; set `MANUS_UNIT=cm` or `MANUS_UNIT=mm` if your bridge emits other
units.

Accepted payload forms:

```json
{"left_fingers": [[0.0, 0.0, 0.02], "... 20 more ..."]}
```

```json
{"left": [[0.0, 0.0, 0.02], "..."], "right": [[0.0, 0.0, 0.02], "..."]}
```

```json
{"hand": "left", "keypoints": [[0.0, 0.0, 0.02], "... 20 more ..."]}
```

Frames expire after `MANUS_TIMEOUT` seconds (default `0.5`) and return zeros
until a fresh frame arrives.  Existing teleop loops skip zero poses, so stale
MANUS frames are not repeatedly commanded.

## Build and smoke test

```bash
docker compose build
docker compose run --rm smoke
```

The smoke service is non-motion: it imports dependencies and verifies assets
only.  It does not mount `/dev` and cannot enable WH110.

For a fast image build that installs apt-level MANUS packages but skips the
slow official gRPC/protobuf source build:

```bash
MANUS_BUILD_GRPC=0 docker compose build
```

Use the default `MANUS_BUILD_GRPC=1` for actual MANUS SDK remote-mode linking.

## MANUS SDK dependency and minimal-client check

Build the image with the MANUS Linux SDK dependencies:

```bash
docker compose --profile manus-sdk build
```

Drop into a shell with the SDK mounted, host networking, `/dev`, and `/run/udev`:

```bash
docker compose --profile manus-sdk run --rm manus-sdk-shell
```

Compile and run MANUS' minimal Linux SDK client:

```bash
docker compose --profile manus-sdk run --rm manus-sdk-minimal-client
```

If normal remote discovery sends packets but finds no host, try the unicast
discovery workaround:

```bash
MANUS_CORE_IP=<WINDOWS_MANUS_CORE_IP> \
  docker compose --profile manus-sdk run --rm manus-sdk-minimal-client-unicast
```

When the MANUS minimal client prompts for a connection type, choose remote mode
(`3`) for a Windows MANUS Core on the LAN. This diagnostic service temporarily
rewrites the SDK's `255.255.255.255:62687/udp` discovery packets to
`MANUS_CORE_IP:62687/udp` using an `iptables` DNAT rule, then removes that rule
on exit. It is meant for networks where unicast works but limited broadcast is
filtered before MANUS Core sees it. The tracked bridge also preserves the
discovered Core service port while
replacing only the discovered broadcast IP when `MANUS_CORE_IP` is set; for
example `255.255.255.255:51580` becomes `192.168.50.132:51580`.

Successful minimal-client diagnostics should show that the SDK discovered and
connected to MANUS Core. Exact wording is SDK-sample dependent; the tracked Wuji
bridge logs are shown in the next section.

## Real MANUS Core -> Wuji UDP bridge

Run this when MANUS Core is on the Windows machine and the Linux/Docker host is
on the same LAN:

```bash
MANUS_CORE_IP=<WINDOWS_MANUS_CORE_IP> HAND=left MANUS_UDP_PORT=15020 \
  docker compose --profile manus-sdk run --rm manus-sdk-udp-bridge-unicast
```

This service:

1. sets `MANUS_CONNECTION=remote` for the tracked bridge;
2. applies the same temporary unicast DNAT rule for
   `255.255.255.255:62687/udp -> MANUS_CORE_IP:62687/udp`;
3. preserves the MANUS-discovered Core service port while replacing only the
   bogus broadcast host address;
4. publishes Wuji-compatible UDP JSON to
   `MANUS_UDP_HOST:MANUS_UDP_PORT` (default `127.0.0.1:15020`).

Expected tracked-bridge logs:

```text
MANUS SDK discovery DNAT active: ...
MANUS UDP bridge target: 127.0.0.1:15020 hand=left
Built tracked MANUS bridge:
Starting tracked MANUS -> Wuji UDP bridge connection=remote hand=left
Using MANUS_CORE_IP=... for discovered host ...
Connected to MANUS Core via ...
Published MANUS UDP frames: 1
```

Keep this bridge running in terminal 1, then start `manus-check`,
`tuning-manus`, `teleop-sim-manus`, or `real-wh110-manus` in another terminal.

Do not set `MANUS_CORE_HOST` for this sample. MANUS SDK 3.1.1's public
`CoreSdk_ConnectToHost` call expects a `ManusHost` record produced by the SDK's
host-discovery flow. Supplying only an IP address is not a supported replacement
for discovery and can produce bogus connection targets.

If remote mode logs availability requests followed by `Received 0 packets`, the
container has sent discovery packets but no MANUS Core discovery response reached
it. This Compose service already uses host networking, matching MANUS' remote
Docker guidance. A syscall trace of MANUS SDK 3.1.1 shows remote discovery is
sent as UDP broadcast to:

```text
255.255.255.255:62687/udp
```

The payload starts with `Notice me Manus-Core-San!`.

When the unicast workaround is active and MANUS Core is reachable, the expected
reply starts with `Being noticed by Senpai!`, followed by a Core service
description and the actual TCP service port selected by MANUS Core.

Check the Windows/Core side instead:

1. MANUS Core is running on the Windows machine and the gloves/devices are
   visible there.
2. The MANUS license has the SDK feature enabled.
3. Windows network profile is private/trusted, not public.
4. Windows Defender Firewall allows MANUS Core on the private network, or is
   temporarily disabled only long enough to test discovery.
5. No VPN, guest Wi-Fi isolation, AP/client isolation, VLAN, or subnet routing
   rule blocks LAN broadcast/multicast discovery replies.
6. If using Docker Desktop/WSL2 instead of native Linux Docker, validate once
   from native Linux or directly on the host; host networking and broadcast
   discovery are materially less reliable through WSL/NAT layers.

### Remote discovery packet trace

On the Linux/Docker host, capture MANUS discovery traffic while starting the
minimal client in another terminal:

```bash
docker run --rm --network host --cap-add NET_RAW --cap-add NET_ADMIN \
  wuji-retargeting:realsense-wh110 bash -lc '
    apt-get update >/dev/null &&
    DEBIAN_FRONTEND=noninteractive apt-get install -y tcpdump >/dev/null &&
    exec tcpdump -ni any "udp port 62687 or host <WINDOWS_MANUS_CORE_IP>"
  '
```

Then run:

```bash
docker compose --profile manus-sdk run --rm manus-sdk-minimal-client
```

Choose remote mode (`3`).

Interpret the capture:

- Outbound Linux packets to `255.255.255.255.62687` are present, but Windows
  sees nothing in Wireshark with filter `udp.port == 62687`: the LAN/AP/switch,
  NIC, or Windows firewall path is dropping broadcast before MANUS Core can see
  it.
- Windows sees `Notice me Manus-Core-San!`, but sends no reply: MANUS Core is
  not listening/responding for SDK discovery, or the SDK/Core version/license
  state is wrong.
- Windows sends a reply, but Linux sees no inbound packet: return traffic is
  blocked on the Windows or Linux side.

The MANUS docs require a MANUS Bodypack or a MANUS license key with the SDK
feature enabled. They also note that Linux supports SDK integrated mode, while
remote mode needs a reachable MANUS Core installation and MANUS Core currently
only runs on Windows.

## MANUS UDP port check (no GUI, no robot)

Terminal 1:

```bash
HAND=left MANUS_PORT=15020 \
  docker compose --profile manus run --rm --service-ports manus-check
```

Terminal 2:

```bash
python scripts/manus_udp_mock.py --hand left --host 127.0.0.1 --port 15020 --frames 5
```

`--service-ports` matters: `docker compose run` does **not** publish service
ports by default.  You can also use `docker compose up manus-check` if you prefer
Compose-managed service ports. The port is bound to `127.0.0.1` by default; use `MANUS_PUBLISH_HOST=0.0.0.0` only for a trusted remote bridge. Direct host Python commands likewise bind to `127.0.0.1`; pass `--manus-host 0.0.0.0` only for a trusted remote bridge.

## Tuning GUI with MANUS input

Terminal 1 (real MANUS bridge):

```bash
MANUS_CORE_IP=<WINDOWS_MANUS_CORE_IP> HAND=left MANUS_UDP_PORT=15020 \
  docker compose --profile manus-sdk run --rm manus-sdk-udp-bridge-unicast
```

Alternative terminal 1 mock sender:

```bash
python scripts/manus_udp_mock.py --hand left --host 127.0.0.1 --port 15020
```

Terminal 2:

```bash
HAND=left MANUS_PORT=15020 \
  docker compose --profile manus --profile gui run --rm --service-ports tuning-manus
```

This runs:

```bash
cd example
python tuning_tool.py --manus --hand left --manus-host 0.0.0.0 --manus-port 15020
```

The tuning tool auto-selects `config/retarget_manus_left.yaml` or
`config/retarget_manus_right.yaml` when the default AVP config is still selected.
Edit the config on the host; the container mounts `./example/config` read-write
for tuning auto-reload.

## MuJoCo simulation with MANUS input

```bash
HAND=left MANUS_PORT=15020 \
  docker compose --profile manus --profile gui run --rm --service-ports teleop-sim-manus
```

This is still simulation only: no WH110 device is mounted or enabled.

## Real WH110 with MANUS input (opt-in only)

Only run this after the smoke, UDP check, tuning, and simulation stages above.
The service refuses to start unless you set the explicit confirmation variable.

```bash
HAND=left MANUS_PORT=15020 CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
  docker compose --profile real-wh110 --profile manus run --rm --service-ports real-wh110-manus
```

Optional WH110 serial selection:

```bash
HAND_SERIAL=<YOUR_WH110_SERIAL> \
HAND=left MANUS_PORT=15020 CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
  docker compose --profile real-wh110 --profile manus run --rm --service-ports real-wh110-manus
```

The service mounts `/dev` for WH110 USB-CDC serial access and mounts
`${HOME}/.wuji:/root/.wuji:ro` for Wuji Studio provisioned device assets.

## Direct host commands (no Docker)

```bash
cd example
python tuning_tool.py --manus --hand left --manus-host 127.0.0.1 --manus-port 15020
python teleop_sim.py --input manus --hand left --manus-host 127.0.0.1 --manus-port 15020
CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
  python teleop_real.py --input manus --hand left --hand-model wh110 --manus-host 127.0.0.1 --manus-port 15020
```

The direct host commands bind MANUS UDP to `127.0.0.1` by default. The Python entrypoint enforces the same WH110 confirmation latch as Compose before creating `Wh110Backend`. For a remote MANUS bridge, add `--manus-host 0.0.0.0` only on a controlled/trusted network.

## Real MANUS bridge mapping note

The tracked bridge in `tools/manus_bridge/manus_wuji_udp_bridge.cpp` maps MANUS
raw skeleton nodes into MediaPipe Hands landmark order `(21, 3)` and emits:

```json
{"hand":"left","keypoints":[[0.0,0.0,0.0], "... 20 more ..."]}
```

It uses the current practical mapping from MANUS chains to wrist/thumb/index/
middle/ring/pinky landmarks. If orange/cyan tuning skeletons show lateral finger
collapse or swapped joints, fix the SDK bridge coordinate/order mapping before
changing retargeting segment lengths.
