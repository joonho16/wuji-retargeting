# Gen3 VIVE ROS2 wrapper safety boundary

This repository keeps the Windows/OpenVR/VIVE sender out of scope. The Gen3 VIVE
ROS2 wrapper is a Linux-side boundary around the mapper core: it validates
operator-facing safety configuration, keeps dry-run safe by default, and makes
MoveIt Servo `TwistStamped` the normal command path.

## Defaults

```bash
python3 example/arm_teleop/gen3_vive_ros2_wrapper.py --print-config
```

The default configuration is intentionally non-actuating:

- `dry_run: true`
- `real_robot: false`
- `real_robot_latch: false`
- `backend: moveit_servo_twist`
- fixed pose input topic: `/vive/gen3_tracker/pose`
- MoveIt Servo command topic: `/servo_node/delta_twist_cmds`

With defaults, the CLI validates and prints config without importing ROS2 or
publishing robot commands.

## Physical deadman and real robot latch

Live Gen3 actuation must use an independent physical deadman signal, not a VIVE
pose or synthetic CLI toggle. Real-robot mode fails closed unless all of these
are true:

```text
--no-dry-run
--real-robot
--deadman-source physical_topic
--physical-deadman-topic /gen3_vive/physical_deadman
--real-robot-latch
```

`real_robot_latch` defaults to false and is also declared as a ROS2 parameter by
the optional wrapper node. Synthetic deadman input is accepted only in dry-run
mode for mapper development.

## Command backend policy

The default backend is MoveIt Servo `TwistStamped`. The wrapper exposes a
unit-testable `build_twist_stamped_payload()` helper and the optional ROS2 node
publishes to `/servo_node/delta_twist_cmds` by default.

Direct Kortex twist remains diagnostic-only in the dependency-light mapper core.
The ROS2 wrapper does **not** expose a direct Kortex node branch; selecting
`diagnostic_kortex_twist` through the wrapper is a fail-closed configuration
error. The only runnable wrapper command backend is MoveIt Servo
`TwistStamped`.

## Mapper integration contract

Worker-1 owns the dependency-light mapper core. The wrapper accepts an injected
mapper object implementing:

```python
def handle_pose(
    pose_msg,
    *,
    deadman_pressed: bool,
    real_robot_latch: bool,
    deadman_is_physical: bool = False,
): ...
```

The mapper may return either a `TwistCommand` or a mapping with
`linear_xyz`, `angular_xyz`, `planning_frame`, `command_enabled`, and
`stop_reason`. A disabled or missing command is converted to a zero
`TwistStamped` payload so MoveIt Servo can receive an explicit stop.

## Boundary verification

Focused local checks:

```bash
python3 -m unittest tests.test_gen3_vive_ros2_wrapper -v
python3 -m unittest discover -s tests -v
python3 -m py_compile example/arm_teleop/gen3_vive_ros2_wrapper.py tests/test_gen3_vive_ros2_wrapper.py
```

Boundary paths that must remain untouched by this wrapper task:

- Windows/OpenVR sender code and config
- `get_fingers_data` / input-device code
- `example/teleop_real.py`
- `example/teleop_sim.py`
- `example/tuning_tool.py`
- `example/calibrate_offset.py`
- worker-1-owned mapper core unless a coordinated handoff explicitly requires it
