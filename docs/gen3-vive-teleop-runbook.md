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
operator launch path.

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
7. Synthetic deadman is dry-run only.

The ROS2 wrapper runs a watchdog timer (`--watchdog-period-s`, default `0.02`)
that calls the mapper even when no new pose arrives, so sender/tracking loss
reaches the stale-stop path. A physical deadman falling edge publishes an
immediate zero stop before waiting for the next pose.

## Staged bring-up

### Stage 0: local verification

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

### Stage 3: MoveIt Servo simulation / no hardware motion

Run the ROS2 wrapper only against a simulated or disabled Servo target first.
Confirm that disabled mapper output publishes zero twist and status explains the
stop reason.

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

Keep the tracker near neutral and press the physical deadman once to capture
neutral. Release the deadman to stop; re-pressing after release recaptures
neutral rather than resuming from an old offset.

## Stop conditions

Stop immediately on any of:

- role or frame mismatch
- stale `/vive/gen3_tracker/pose`
- unexpected non-zero twist after deadman release
- missing or synthetic deadman in real robot mode
- missing `real_robot_latch`
- calibration mismatch or unreviewed calibration changes
- direct Kortex backend selected outside a diagnostic dry-run

## Sender contract reminder

The sender computer is managed separately. Linux-side code refuses bad packets
locally; do not edit sender-side code from this repository to paper over receiver
diagnostics.
