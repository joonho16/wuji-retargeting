# Gen3 VIVE teleop runbook

> **Real-hardware path updated:** `full-real` and `real-robot` now run native Twist.
> Use [the native runbook](gen3-native-teleop.md) and [HOWTO](../HOWTO.md).
> The Servo architecture and real-wrapper descriptions below are historical;
> `full-fake` / `full-fake-rviz` remain available for legacy Servo simulation.

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
# Per-axis tracker-driven tool offset limits, relative to the neutral tool pose
# captured when deadman changes false -> true.
# Optional: omit workspace_limits or set it to null to disable offset clamping.
workspace_limits:
  x: [-0.25, 0.25]
  y: [-0.25, 0.25]
  z: [-0.20, 0.20]
max_linear_mps: 0.10
max_angular_radps: 0.25
# Optional stationary-jitter suppression.  0 disables; start real-robot tests
# around 0.003 m and 0.02 rad if VIVE/TF noise creates joint buzz.
linear_deadband_m: 0.003
angular_deadband_rad: 0.02
stale_timeout_s: 0.10
```

The mapper refuses malformed calibration fields, non-unit quaternions,
non-finite values, inverted workspace limits, and non-positive gains/limits.

`workspace_limits` is optional. Omit the entire field or set
`workspace_limits: null` to follow tracker-driven target offsets without a
position box. When supplied, all three axis ranges must be valid; an empty or
partial mapping is rejected. Speed limits, deadbands, deadman and stale-input
stops still apply. MoveIt Servo joint, collision and singularity checks are
independent of this mapper setting. Configuration is read at startup; rebuild
the Docker image after mapper code changes and restart the mapper after editing
the calibration YAML.

`T_planning_source.rotation_xyzw` is the rotation that maps SteamVR/source axes
into Gen3 planning-frame axes. This is the knob to fix cases like "tracker X
should become robot -Z". In the native path, `T_tool_tracker_mount` defines the
robot control point relative to the actual TCP: `T_base_control = T_base_tool *
T_tool_tracker_mount`. Translation is TCP-to-control-point in TCP axes, metres.
The native controller follows this point and shifts its Twist back to the TCP,
including the rotation-induced linear velocity. Identity retains TCP following.
The legacy Servo mapper does not consume this mount field.

## Per-joint velocity and acceleration limits

`GEN3_JOINT_LIMITS_CONFIG` selects the additional arm cap file, defaulting
to `example/config/gen3_joint_limits.yaml`. Velocity is in **rad/s** and
acceleration is in **rad/s²**.
The default file has no additional caps. For example, to limit joints 1 and 2:

```yaml
robot_soft_limit_mode: ANGULAR_TRAJECTORY
joint_limits:
  joint_1: {max_velocity: 0.3, max_acceleration: 1.0}
  joint_2: {max_acceleration: 0.8}
```

Unlisted joints retain the model/robot limits. `joint_1` through `joint_6` are
valid for both robots; `joint_7` requires `GEN3_DOF=7`. Unknown joints, duplicate
YAML keys, disabled limits and non-positive/non-finite values stop startup.
Velocity and acceleration overrides are independent. Gripper limits are separate.

Before launching any driver or Servo actions, the stack calculates each cap:

- Fake hardware: `min(URDF velocity, enabled MoveIt velocity, optional file cap)`.
- Real hardware: also include both the robot hard limit and the selected robot
  soft limit. The stack reads `ControlConfig.GetKinematicHardLimits()` and
  `GetKinematicSoftLimits()` through an authenticated Kortex TCP session, validates
  the returned joint count and values, and converts deg/s to rad/s.
- Acceleration: `min(enabled MoveIt acceleration, robot hard/soft acceleration,
  optional file cap)`, converting deg/s² to rad/s². Fake hardware omits robot
  discovery. Every joint must have a valid bound. Profiles without joint
  acceleration values are rejected; use an angular profile with those values.

The resulting values are passed to the Servo node as
`robot_description_planning.joint_limits.<joint>.max_velocity`; startup logs show
every source and final cap, including `max_acceleration`.
Larger file values cannot raise existing ceilings.
Servo's `override_velocity_scaling_factor` must remain `0.0`; other values are
rejected because Humble can otherwise bypass its joint-bound scaling.

The Docker image builds a pinned Humble 2.5.10 Servo overlay because stock
Humble's Butterworth filter does **not** enforce acceleration bounds. After
Servo calculates joint velocities and applies speed limits, the overlay bounds
each velocity change by `max_acceleration * publish_period` and integrates from
the previous published position reference. It handles continuous-joint angle
wrapping and bypasses the old Butterworth position filter to avoid adding a
second source of lag. `low_latency_mode` must stay false for the fixed period.
Startup requires overlay version 4, including the continuous-joint IK fix and
removal of the 0.1 rad tracking-error latch. Real hardware also requires the Kortex cyclic-failure
guard overlay (version 1).
An absent or older overlay stops startup before launching any driver. The startup log must include
`Gen3 per-joint reference acceleration limiting enabled (Butterworth bypassed)`.

The published reference and feedback history remain continuous across ±π, but
continuous-joint **IK seeds** are normalized into [-π, π]. Feeding unwrapped
seeds to KDL reproduced a non-finite approximate IK solution after motion began
near the angle boundary, followed by `Gen3 acceleration limiter rejected an
invalid joint reference`. This message means invalid input, not an acceleration
cap violation. Version 2 fixes the seed conversion, checks IK results for finite
values and holds the last reference on an invalid result. Repeated diagnostics
are throttled to once per second. This reproduction does not establish the
hardware cause of a subsequent Joint Fault.

Collision/singularity/position safety holds, zero commands and deadman release
take priority over acceleration smoothing; restart ramps from zero commanded
velocity. These are limits on the discrete **Servo position references**. They
are not a hard physical acceleration guarantee: controller interpolation,
delivery timing and the robot's tracking response still need feedback checks.
The change does not establish the cause of a previous Joint Fault.
If joint feedback exceeds `incoming_command_timeout`, Servo stops advancing
the reference. Fresh timestamps with a stationary joint are a separate case:
faulted hardware can continue publishing joint feedback while rejecting motion.
Version 4 removes the added 0.1 rad tracking-error check and its restart latch.
The legacy `gen3_max_joint_tracking_error` YAML key is ignored. A reference/feedback
gap alone no longer triggers that stop; this removal does not resolve the hardware
Joint Fault reported before the tracking-error message. Joint speed/acceleration
bounds, collision/singularity/position holds, stale-feedback handling and the
Kortex communication-failure guard remain active.

The position-limit warning checks the **computed reference**, not just the
current measured angle. Its added diagnostic reports measured/reference angles,
requested velocity, model bounds and margin. For this 7-DOF model, joint 4 has
bounds ±2.57 rad and the configured 0.1 rad margin stops outward commands near
±2.47 rad (141.52°).

Per-joint acceleration saturation can change the Cartesian path
and response while ramping; the limiter is not a path planner.

Soft limits are stored by control mode. `robot_soft_limit_mode` defaults to
`ANGULAR_TRAJECTORY`; it can also select `CARTESIAN_TRAJECTORY`,
`ANGULAR_JOYSTICK`, or `CARTESIAN_JOYSTICK`. This explicitly chooses the stored
profile to impose in Servo; it does not change the robot's control mode or assume
that the low-level Kortex driver enforces that high-level profile. The lookup
does not change robot limits or send motion commands.

Real discovery is mandatory: connection, authentication, SDK, timeout, mode or
invalid-limit errors stop startup instead of falling back to model-only limits.
`GEN3_LIMIT_QUERY_TIMEOUT_S` (default 15 seconds) bounds the whole query including
SDK connection/cleanup. `GEN3_LIMIT_QUERY_USERNAME` and
`GEN3_LIMIT_QUERY_PASSWORD` default to the Kinova factory credentials (`admin`).
These are lookup credentials only; driver authentication remains configured by
the installed Kinova launch package. Credentials are not printed or passed as
command-line arguments. The Docker image installs the pinned official Kortex
2.x Python SDK. Host ROS installations also require the same Servo overlay;
installing only the Python SDK does not enable acceleration limiting.

To edit caps without rebuilding the image, copy the file into the existing
calibration mount and set its container path:

```bash
cp example/config/gen3_joint_limits.yaml .local/gen3_vive_calib/gen3_joint_limits.yaml
# Edit .local/gen3_vive_calib/gen3_joint_limits.yaml, then launch with:
export GEN3_JOINT_LIMITS_CONFIG=/calib/gen3_joint_limits.yaml
```

Use `GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib` as in real bring-up below.
Rebuild the image once for this code/SDK change. Limits are a startup snapshot:
restart the Servo stack after changing the YAML, selected profile or robot's
stored limits. This applies to the Servo teleoperation path; the separately
launched `move_group` and other command sources keep their own configuration.
Actual hardware speed and acceleration tracking still require verification
through joint feedback.

The output regression test launches fake hardware and checks acceleration during
steps/reversals, zero-command holds, collision holds and loss of joint feedback:

```bash
docker run --rm --network none wuji-retargeting:gen3-vive-ros2-humble \
  python3 tests/helpers/gen3_servo_acceleration_runtime.py
docker run --rm --network none wuji-retargeting:gen3-vive-ros2-humble \
  python3 tests/helpers/gen3_servo_cartesian_runtime.py
docker run --rm --network none wuji-retargeting:gen3-vive-ros2-humble \
  python3 tests/helpers/gen3_kortex_cyclic_runtime.py
```

The Cartesian regression runs only Servo with synthetic wrapped joint feedback
and tracking lag. It exercises 1.0/1.5 rad/s angular commands and reversals near
±π, checking finite outputs, angle continuity and reference speed/acceleration
bounds. It also introduces a reference/feedback gap above 0.1 rad, checks that
zero commands hold, and verifies subsequent motion works without a restart.
The Kortex test invokes the
actual driver/SDK without initialization or a robot connection and verifies that
read/write failure stays latched. All runtime tests refuse containers with a
non-loopback network interface.

The Kortex overlay retains driver 0.2.3 and C++ SDK 2.5. A cyclic read/write
exception is logged and returned to ros2_control as `ERROR`. It does not perform
another `RefreshFeedback` RPC from the exception handler or reuse the failed
session. Reactivation requires a process restart. Driver logging defaults to
INFO instead of forcing DEBUG on the 1 kHz loop. This contains the observed
uncaught `Future already retrieved` failure; it does not repair the underlying
network/SDK timeout or prove a hardware Joint Fault is resolved.

If motion is slow but vibrates only while the tracker moves, do not assume a
stronger low-pass filter will fix it. The mapper's default `low_pass_alpha=1.0`
does no input smoothing. Its pose-follow command is proportional to remaining
position/orientation error with an implicit gain of 1/s, so small errors already
produce small speeds even with a high speed cap. Acceleration limiting bounds
abrupt command changes; it does not diagnose feedback latency, tracker motion
noise, controller interpolation or hardware issues.

API references: [Gen3 ControlConfig](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/doc/markdown/summary_pages/ControlConfig.md),
[KinematicLimits units](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/doc/markdown/messages/ControlConfig/KinematicLimits.md).

## Real-time scheduling for the hardware loop

The real Gen3 service grants `SYS_NICE`, an `rtprio` limit of 99, and unlimited
`memlock`, following the [ros2_control Humble Docker configuration](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html#determinism).
The controller manager requests `SCHED_FIFO` priority 50 for its 1 kHz hardware
loop. Without these permissions, container root still receives
`Operation not permitted` and the loop runs with ordinary scheduling, making it
more vulnerable to timing jitter. `memlock` permits memory locking; it does not
itself enable real-time scheduling or lock memory automatically.

These are container creation settings, so recreate the real Gen3 service to
apply them; an image rebuild or `docker restart` alone is insufficient. They
apply only to `gen3-servo-real`, not the receiver, wrapper or fake hardware.
For a check that never launches the robot, run:

```bash
docker compose -f docker-compose.gen3-vive.yml --profile real-robot \
  run --rm --no-deps --entrypoint python3 gen3-servo-real -c \
  'import os; os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(50)); print("SCHED_FIFO:", os.sched_getscheduler(0), "priority:", os.sched_getparam(0).sched_priority); os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))'
```

Expected output is `SCHED_FIFO: 1 priority: 50`. This grants scheduling
permission; it does not guarantee that every control cycle meets its deadline
or that Kortex UDP feedback cannot time out. If timeouts persist, retain the
fault bits before clearing faults and measure loop latency and network loss.

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

Stationary VIVE/TF jitter can otherwise produce small nonzero Twist commands on
every pose update. `linear_deadband_m` and `angular_deadband_rad` zero those
small target errors before speed limiting. `GEN3_VIVE_LOW_PASS_ALPHA` can also
be set at launch (`1.0` disables smoothing; lower values reduce buzz but add
lag).

## Staged bring-up

### Stage 0: local verification

If the host has no ROS2 installation, use the Docker Compose environment:

```bash
cd <repo>

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

Interactive keyboard deadman helper for physical-topic tests:

```bash
# Publishes std_msgs/Bool on /gen3_vive/physical_deadman from inside Docker.
# Enter toggles false/true, q + Enter exits, Ctrl-C publishes false before exit.
./scripts/gen3_vive_enter_deadman.sh
```

This helper is for bring-up only. It is not a substitute for a real physical
deadman or E-stop during real robot operation.

`example/config/gen3_vive_dry_run_calibration.yaml` is identity calibration for
container smoke tests only. It is not a reviewed real-robot calibration.

### Stage 3: MoveIt Servo simulation / no hardware motion

For a host with no ROS2 install, run the full fake stack with one Compose
command:

```bash
./scripts/gen3_vive_compose_rm.sh full-fake
```

To view the fake Gen3 in RViz from Docker on a Linux desktop, allow local X11
connections for the container, run the RViz profile, then revoke the permission
afterward:

```bash
xhost +local:root
GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib \
GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml \
./scripts/gen3_vive_compose_rm.sh full-fake-rviz
xhost -local:root
```

If RViz cannot open, keep using `full-fake` and inspect `/joint_states`,
`/tf`, `/servo_node/delta_twist_cmds`, and
`/joint_trajectory_controller/joint_trajectory` from the tools shell.

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
