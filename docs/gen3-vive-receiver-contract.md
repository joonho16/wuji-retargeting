# Gen3 VIVE Linux receiver contract

The Windows OpenVR/VIVE sender is externally managed and out of this repository's
write scope. The Linux side must validate the incoming UDP/ROS2 contract and
refuse mismatches locally.

## Required upstream packet fields

A valid Gen3 tracker packet uses this role/frame pair:

```json
{
  "role": "gen3_tracker",
  "frame": "steamvr_world",
  "serial": "LHR-...",
  "pos": [0.0, 0.0, 0.0],
  "quat": [0.0, 0.0, 0.0, 1.0],
  "t": 1782287700.0
}
```

Linux receiver behavior for Task 2:

- Accept only `role=gen3_tracker` for the Gen3 arm tracker stream.
- Accept only `frame=steamvr_world`; do not reinterpret other frames as the
  SteamVR world frame.
- Publish the raw pose stream on `/vive/gen3_tracker/pose` after validation.
- Refuse role/frame mismatches locally with diagnostics; do not modify, patch, or
  auto-correct sender-side code/configuration.

## Linux validation commands

Mock/unit validation in this repository:

```bash
python3 -m unittest tests.test_gen3_vive_receiver_contract -v
```

Live ROS2 validation on the Linux robot/control host after the externally owned
sender is running:

```bash
ros2 topic hz /vive/gen3_tracker/pose
ros2 topic echo /vive/gen3_tracker/pose --once
```

For the echoed `PoseStamped`, confirm:

- Topic is `/vive/gen3_tracker/pose`.
- `header.frame_id` is `steamvr_world`.
- Receiver diagnostics identify the selected role as `gen3_tracker`.

A packet with any other role or frame is a Linux-side refusal condition, not a
reason to edit sender-side files in this repository.

## Deterministic role/serial selection

Receiver diagnostics should use a deterministic role/serial key so that packet
selection and logs do not depend on JSON object order, Python dictionary order,
or UDP arrival implementation details for a single packet.

For Gen3 VIVE tracker packets, use this ordered diagnostic key shape:

```text
role=gen3_tracker;serial=<sender-serial>
```

Rules:

- `role` is mandatory and appears first in the key.
- `serial` is optional for broad role-based discovery, but when Linux config pins
  an expected tracker serial it becomes mandatory and must match exactly.
- The ROS2 pose topic remains role-based: `/vive/gen3_tracker/pose`.
- Serial is used for deterministic diagnostics and optional exact-device
  filtering; it must not make the role topic change unpredictably.
- Missing or mismatched pinned serial is a local Linux refusal/diagnostic
  condition, not a sender-side edit request.
