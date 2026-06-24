# Gen3 VIVE teleop runbook

This runbook covers the Linux-side Gen3 VIVE teleoperation path. The Windows
VIVE/OpenVR sender stays external and must not be patched from this repository.

## Data path

```text
external Windows sender
  role=gen3_tracker, frame=steamvr_world
  -> UDP JSON
  -> example/arm_teleop/vive_udp_receiver.py
  -> /vive/gen3_tracker/pose
  -> example/arm_teleop/gen3_vive_mapper.py
  -> example/arm_teleop/gen3_vive_ros2_wrapper.py
  -> /servo_node/delta_twist_cmds (MoveIt Servo TwistStamped)
  -> Gen3
```

The normal backend is MoveIt Servo `geometry_msgs/msg/TwistStamped`. Direct
Kortex twist is diagnostic-only, disabled by default, and not part of the normal
operator launch path. The wrapper reads the current TCP transform
`planning_frame -> tool_frame` (default `base_link -> end_effector_link`) and
uses VIVE displacement as a **target TCP offset**, not as a raw velocity. The
published Twist is the remaining target error, clamped by the configured speed
limits, so the robot stops when the TCP reaches the target even if the tracker
stays offset from neutral.

## Required calibration artifact

Before any non-dry-run execution, create a reviewed calibration YAML with this
shape:

```yaml
source_frame: steamvr_world
planning_frame: base_link
tracker_role: gen3_tracker
tracker_mount_frame: gen3_tool_tracker
T_planning_source:
  translation: [0.0, 0.0, 0.0]
  rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
T_tool_tracker_mount:
  translation: [0.0, 0.0, 0.0]
  rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
linear_scale: 1.0
angular_scale: 1.0
workspace_limits:
  x: [-0.25, 0.25]
  y: [-0.25, 0.25]
  z: [-0.20, 0.20]
max_linear_mps: 0.10
max_angular_radps: 0.25
stale_timeout_s: 0.10
```

The mapper refuses malformed calibration fields, non-unit quaternions,
non-finite values, inverted workspace limits, and non-positive gains/limits.

`T_planning_source.rotation_xyzw` is the rotation that maps SteamVR/source axes
into Gen3 planning-frame axes. This is the knob to fix cases like "tracker X
should become robot -Z". `T_tool_tracker_mount` is currently kept as a validated
calibration/reserved mount field; the relative target-pose controller does not
use it for the linear command path, and the practical first-pass value for
handheld tests is identity.

## Safety state machine

Command output is enabled only after all gates pass:

1. Valid receiver packet: `role=gen3_tracker`, `frame=steamvr_world`, finite
   `pos[3]`, finite unit `quat[4]`.
2. Calibration source frame and tracker role match the pose stream.
3. Pose is fresh; stale tracking emits a zero stop and requires recapture.
4. Deadman is pressed.
5. After every stop/stale/invalid/deadman release, recovery requires a fresh pose
   plus a **new deadman rising edge** to recapture neutral.
6. Real robot mode requires `--no-dry-run`, `--real-robot`,
   `--deadman-source physical_topic`, `--real-robot-latch`, and
   `--calibration-path`.
7. Real robot mode also requires live TF for the current tool pose. If
   `planning_frame -> tool_frame` is unavailable while deadman is pressed, the
   mapper emits `TOOL_POSE_UNAVAILABLE_STOP` and requires neutral recapture.
8. Synthetic deadman is dry-run only.

The ROS2 wrapper runs a watchdog timer (`--watchdog-period-s`, default `0.02`)
that calls the mapper even when no new pose arrives, so sender/tracking loss
reaches the stale-stop path. A physical deadman falling edge publishes an
immediate zero stop before waiting for the next pose.

## Staged bring-up

### Stage 0: local verification

If the host has no ROS2 installation, use the Docker Compose environment:

```bash
cd /workspace/wujihand/.omx/worktrees/gen3-vive-teleop

# Optional: edit defaults such as VIVE_UDP_PORT or ROS_DOMAIN_ID.
cp .env.gen3-vive.example .env

# Build the ROS2 Humble image for receiver/wrapper/Kinova MoveIt Servo tests.
docker compose -f docker-compose.gen3-vive.yml --profile tools build shell

# Run the same Python verification inside the ROS2 container.
docker compose -f docker-compose.gen3-vive.yml --profile test run --rm test
```

The image installs the Gen3 VIVE path plus the binary Kinova Gen3 MoveIt Servo
stack (`kortex_bringup`, Gen3 6/7DOF Robotiq MoveIt configs, `moveit_servo`,
`ros2_control`, and `rmw_cyclonedds_cpp`). It does not install the full Wuji
hand/camera/glove dependency set. The repository source is baked into the image
so the host does not need ROS2 or a Python environment; rebuild the image after
editing source files.

Long-running Compose stacks should be started through the repository wrapper, not
raw `docker compose up`. Compose has no `up --rm`;
`scripts/gen3_vive_compose_rm.sh` runs `up` attached, stops the whole stack if
any service exits, and always performs `docker compose down --remove-orphans`
when the stack exits or you press Ctrl-C. One-shot inspection commands still use
`docker compose run --rm ...`. Real-robot profiles fail before Docker starts
unless `GEN3_ROBOT_IP`, a repo/host calibration directory, the mapped
`GEN3_VIVE_CALIBRATION=/calib/<file>.yaml`, and
`real_robot_calibration_reviewed: true` are present.

Without Docker:

```bash
python3 -m unittest discover -s tests -v
git ls-files '*.py' | xargs python3 -m py_compile
python3 -m tabnanny \
  example/arm_teleop/vive_udp_receiver.py \
  example/arm_teleop/gen3_vive_mapper.py \
  example/arm_teleop/gen3_vive_ros2_wrapper.py \
  tests/*.py tests/helpers/*.py
```

### Stage 1: receiver only

Start the external sender with:

```text
role=gen3_tracker
frame=steamvr_world
```

On the Linux host, run the receiver and verify:

```bash
python3 example/arm_teleop/vive_udp_receiver.py --bind-port 9000
ros2 topic hz /vive/gen3_tracker/pose
ros2 topic echo /vive/gen3_tracker/pose --once
```

Stop if `header.frame_id` is not `steamvr_world`, topic is not
`/vive/gen3_tracker/pose`, packets are stale, or diagnostics show role/frame
mismatch.

Docker equivalent:

```bash
# Terminal A: receive Windows UDP on host network and publish ROS2 PoseStamped.
./scripts/gen3_vive_compose_rm.sh receiver

# Terminal B: inspect ROS2 topics without host ROS2 installed.
docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic hz /vive/gen3_tracker/pose'

docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic echo /vive/gen3_tracker/pose --once'

docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic echo /vive/gen3_tracker/status --once'
```

If no packets arrive:

```bash
docker compose -f docker-compose.gen3-vive.yml --profile debug run --rm udp-debug
```

### Stage 2: dry-run mapper/wrapper

Dry-run config should print without ROS2 imports or actuation:

```bash
python3 example/arm_teleop/gen3_vive_ros2_wrapper.py --print-config
```

Confirm:

- `dry_run=true`
- `real_robot=false`
- `real_robot_latch=false`
- `backend=moveit_servo_twist`
- `twist_topic=/servo_node/delta_twist_cmds`

Use dry-run tests and logs to confirm neutral capture, stale stop, clamp/filter,
and deadman release behavior before enabling any robot-side Servo stack.

Docker dry-run wrapper:

```bash
# Terminal A: keep receiver running.
./scripts/gen3_vive_compose_rm.sh receiver

# Terminal B: start mapper/wrapper with synthetic deadman and debug twist topic.
./scripts/gen3_vive_compose_rm.sh wrapper

# Terminal C: inspect generated stop/command status and debug twist.
docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic echo /gen3_vive/status'

docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic echo /gen3_vive/debug_twist'
```

`example/config/gen3_vive_dry_run_calibration.yaml` is identity calibration for
container smoke tests only. It is not a reviewed real-robot calibration.

### Stage 3: MoveIt Servo simulation / no hardware motion

For a host with no ROS2 install, run the full fake stack with one Compose
command:

```bash
./scripts/gen3_vive_compose_rm.sh full-fake
```

This starts:

- `receiver`: Windows UDP JSON -> `/vive/gen3_tracker/pose`
- `gen3-servo-fake`: Kinova Gen3 MoveIt + ros2_control fake hardware + Servo
- `wrapper-servo-dry-run`: VIVE pose -> `/servo_node/delta_twist_cmds`
- `deadman-hold`: dry-run-only Bool publisher on `/gen3_vive/physical_deadman`

Check from another terminal, still without host ROS2:

```bash
docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 node list | sort'

docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic info -v /servo_node/delta_twist_cmds'

docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic info -v /joint_trajectory_controller/joint_trajectory'

docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'ros2 topic echo --once --full-length --field data /gen3_vive/status std_msgs/msg/String'

# Before pressing the real deadman, verify current TCP TF is available:
docker compose -f docker-compose.gen3-vive.yml --profile tools run --rm shell \
  bash -lc 'timeout 5 ros2 run tf2_ros tf2_echo base_link end_effector_link'
```

Expected wiring:

- `/servo_node` exists.
- `/servo_node/start_servo` service exists and autostart returns success.
- `/servo_node/delta_twist_cmds` has publisher `gen3_vive_mapper_wrapper` and
  subscriber `servo_node`.
- `/joint_trajectory_controller/joint_trajectory` has publisher `servo_node`
  and subscriber `joint_trajectory_controller`.

If no VIVE pose has arrived yet, `/gen3_vive/status` should report a stop reason
such as `NO_FRESH_POSE_STOP`. In real robot mode, wait for
`"tool_pose_available":true` before pressing the physical deadman. Once the
Windows sender is publishing and deadman is held, a fresh pose plus deadman
rising edge captures neutral and then enables commands.

### Stage 4: real robot gated run

Only after Stage 0-3 pass and the physical E-stop/deadman are tested:

```bash
python3 -m example.arm_teleop.gen3_vive_ros2_wrapper \
  --spin \
  --no-dry-run \
  --real-robot \
  --deadman-source physical_topic \
  --physical-deadman-topic /gen3_vive/physical_deadman \
  --real-robot-latch \
  --calibration-path /path/to/gen3_vive_calibration.yaml
```

Docker equivalent, including receiver, wrapper, and real Gen3 MoveIt Servo stack
in one Compose profile. Use a repo-local calibration working directory so the
command does not depend on this agent container's `$HOME` or your host `$HOME`.
If `.local/gen3_vive_calib/real_gen3_vive_calibration.yaml` is missing, the
first `full-real` run creates a template and exits before starting Docker:

```bash
GEN3_ROBOT_IP=192.168.1.10 \
GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib \
GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml \
./scripts/gen3_vive_compose_rm.sh full-real

# Then edit the generated file with measured calibration values:
# .local/gen3_vive_calib/real_gen3_vive_calibration.yaml
# Only after review, set this inside the YAML:
# real_robot_calibration_reviewed: true

# Run the same full-real command again.
```

Keep the tracker near neutral and press the physical deadman once to capture
neutral. Release the deadman to stop; re-pressing after release recaptures
neutral rather than resuming from an old offset. While held, moving the tracker
sets a target TCP offset from the captured TCP pose. The robot should stop after
its TCP reaches that target; it should no longer require returning the tracker to
the first pose to stop.

## Stop conditions

Stop immediately on any of:

- role or frame mismatch
- stale `/vive/gen3_tracker/pose`
- unexpected non-zero twist after deadman release
- missing current TCP TF in real robot mode (`TOOL_POSE_UNAVAILABLE_STOP`)
- missing or synthetic deadman in real robot mode
- missing `real_robot_latch`
- calibration mismatch or unreviewed calibration changes
- direct Kortex backend selected outside a diagnostic dry-run

## Sender contract reminder

The sender computer is managed separately. Linux-side code refuses bad packets
locally; do not edit sender-side code from this repository to paper over receiver
diagnostics.
