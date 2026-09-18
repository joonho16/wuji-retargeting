#!/usr/bin/env python3
"""VIVE pose following through the guarded native Gen3 Twist controller."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import time

from example.arm_teleop.gen3_native_core import (NativeSession, Gen3ViveCalibration, ToolPose, native_vectors, positive,
                  DEFAULT_MAX_LINEAR_MPS, DEFAULT_MAX_ANGULAR_RADPS)
from example.arm_teleop.gen3_vive_mapper import pose_stamped_like_to_tracker_pose


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("preview", "native"), default="native")
    p.add_argument("--calibration", "--calibration-path", type=Path, required=True)
    p.add_argument("--motion", choices=("rotation", "translation", "both"), default="both")
    p.add_argument("--max-linear", type=positive, default=DEFAULT_MAX_LINEAR_MPS,
                   help="m/s (default: 0.5), also capped by calibration")
    angular = p.add_mutually_exclusive_group()
    angular.add_argument("--max-angular", type=positive, default=DEFAULT_MAX_ANGULAR_RADPS,
                         help="rad/s (default: 100 deg/s), also capped by calibration")
    angular.add_argument("--max-angular-deg", type=positive,
                         help="deg/s, alternative to --max-angular; also capped by calibration")
    p.add_argument("--tool-frame", default="end_effector_link",
                   help="Actual native-command TCP frame; set hand offset with T_tool_tracker_mount")
    p.add_argument("--deadman-topic", "--physical-deadman-topic", default="/gen3_vive/physical_deadman")
    p.add_argument("--operator-state", type=Path,
                   help="Shared A/B hold state; replaces Bool deadman input when set")
    p.add_argument("--robot-ip", default=os.environ.get("GEN3_ROBOT_IP"))
    p.add_argument("--home-action", default="ready")
    p.add_argument("--twist-topic", default="/twist_controller/commands")
    p.add_argument("--status-topic", default="/gen3_vive/status")
    p.add_argument("--low-pass-alpha", type=positive, default=1.0)
    p.add_argument("--timeout", type=positive, default=0.1)
    p.add_argument("--rate", type=positive, default=50.0)
    p.add_argument("--log", type=Path)
    p.add_argument("--check-config", action="store_true", help="No ROS, sockets or robot connection")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.operator_state and (args.mode != "native" or not args.robot_ip or args.robot_ip == "0.0.0.0"):
        raise ValueError("A/B operator control requires native mode and GEN3_ROBOT_IP")
    # Existing operator .env files may still name the previous Servo input.
    # Preserve the launch contract while selecting the native controller topic.
    if args.twist_topic == "/servo_node/delta_twist_cmds":
        args.twist_topic = "/twist_controller/commands"
    calibration = Gen3ViveCalibration.from_yaml_file(args.calibration)
    if calibration.planning_frame != "base_link":
        raise ValueError("Native Gen3 expects planning_frame=base_link")
    max_angular = math.radians(args.max_angular_deg) if args.max_angular_deg is not None else args.max_angular
    session = NativeSession(calibration, motion=args.motion,
                                max_linear=args.max_linear, max_angular=max_angular,
                                low_pass_alpha=args.low_pass_alpha)
    summary = dict(mode=args.mode, motion=args.motion, tool_frame=args.tool_frame,
                   twist_topic=args.twist_topic,
                   max_linear_mps=session.calibration.max_linear_mps,
                   max_angular_radps=session.calibration.max_angular_radps,
                   max_angular_degps=math.degrees(session.calibration.max_angular_radps),
                   linear_scale=calibration.linear_scale, angular_scale=calibration.angular_scale,
                   gain_per_second=1.0, native_frame="TOOL", native_angular_units="deg/s",
                   T_tool_tracker_mount=asdict(calibration.T_tool_tracker_mount),
                   control_point_frame=calibration.tracker_mount_frame,
                   twist_origin_frame=args.tool_frame)
    if args.check_config:
        print(json.dumps(summary, indent=2))
        return 0
    if args.mode == "native":
        import yaml
        if yaml.safe_load(args.calibration.read_text()).get("real_robot_calibration_reviewed") is not True:
            raise ValueError("Native output requires real_robot_calibration_reviewed: true")
        from example.arm_teleop.gen3_native_stack import verify_native_installation
        verify_native_installation()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    class Teleop(Node):
        def __init__(self):
            super().__init__("gen3_vive_native")
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.pose = None
            self.pending_pose = None
            self.input_error = None
            self.pressed = False
            self.deadman_time = None
            self.controllers = {}
            self.controller_time = None
            self.controller_request = None
            self.joints = None
            self.last_reason = None
            self.operator = None
            self.home = None
            self.intent = None
            if args.operator_state:
                from example.operator_hold import IntentReader
                from example.arm_teleop.gen3_home_control import HomeControl, COMMAND_TOPIC, STATUS_TOPIC
                self.operator = IntentReader(args.operator_state)
                self.switcher = self.create_client(SwitchController, "/controller_manager/switch_controller")
                self.action_publisher = self.create_publisher(String, COMMAND_TOPIC, 1)
                def publish_action(wire):
                    message = String()
                    message.data = wire
                    self.action_publisher.publish(message)
                self.home = HomeControl(action_name=args.home_action, switch=self.switch_twist,
                                        publish=publish_action, log=lambda s: self.get_logger().warning(s))
                self.create_subscription(String, STATUS_TOPIC, lambda m: self.home.on_status(m.data), 1)
            self.last_log_flush = time.monotonic()
            self.log = args.log.open("x", encoding="utf-8") if args.log else None
            if self.log:
                self.log.write(json.dumps({"configuration": summary}) + "\n")
            self.base_pub = self.create_publisher(TwistStamped, "/gen3_vive/debug/base_twist", 1)
            self.native_pub = self.create_publisher(Twist, "/gen3_vive/debug/native_twist", 1)
            self.status_pub = self.create_publisher(String, args.status_topic, 1)
            self.output = None
            if args.mode == "native":
                self.output = self.create_publisher(Twist, args.twist_topic, 1)
            self.create_subscription(PoseStamped, "/vive/gen3_tracker/pose", self.on_pose, 1)
            if self.operator is None:
                self.create_subscription(Bool, args.deadman_topic, self.on_deadman, 1)
            self.create_subscription(JointState, "/joint_states", self.on_joints,
                                     qos_profile_sensor_data)
            self.manager = self.create_client(ListControllers, "/controller_manager/list_controllers")
            self.create_timer(0.25, self.query_controllers)
            self.create_timer(1.0 / args.rate, self.tick)
            self.get_logger().info(json.dumps(summary))

        def fresh_stamp(self, stamp):
            age = (self.get_clock().now().nanoseconds - stamp.sec*1_000_000_000 - stamp.nanosec)/1e9
            return 0.0 <= age <= args.timeout

        def on_pose(self, msg):
            try:
                if not self.fresh_stamp(msg.header.stamp):
                    raise ValueError("Pose header is stale or in the future")
                self.pose = pose_stamped_like_to_tracker_pose(
                    msg, tracker_role=calibration.tracker_role, received_monotonic_s=time.monotonic()
                )
                self.pending_pose = self.pose
                self.input_error = None
            except (ValueError, TypeError) as exc:
                self.pending_pose = None
                self.input_error = f"INVALID_TRACKER_POSE:{type(exc).__name__}"
                self.tick()

        def on_deadman(self, msg):
            released = self.pressed and not msg.data
            self.pressed = bool(msg.data)
            self.deadman_time = time.monotonic()
            if released:
                self.tick()

        def on_joints(self, msg):
            self.joints = dict(stamp_sec=msg.header.stamp.sec, stamp_nanosec=msg.header.stamp.nanosec,
                               names=list(msg.name), positions=list(msg.position),
                               velocities=list(msg.velocity), efforts=list(msg.effort))

        def switch_twist(self, activate):
            if not self.switcher.service_is_ready():
                return None
            request = SwitchController.Request()
            request.activate_controllers = ["twist_controller"] if activate else []
            request.deactivate_controllers = [] if activate else ["twist_controller"]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 2
            return self.switcher.call_async(request)

        def query_controllers(self):
            if not self.manager.service_is_ready():
                return
            if self.controller_request and not self.controller_request.done():
                return
            self.controller_request = self.manager.call_async(ListControllers.Request())
            self.controller_request.add_done_callback(self.on_controllers)

        def on_controllers(self, future):
            try:
                self.controllers = {c.name: c.state for c in future.result().controller}
                self.controller_time = time.monotonic()
            except Exception:
                self.controller_time = None

        def controller_ready(self, now):
            if args.mode == "preview":
                return True
            if self.controller_time is None or now - self.controller_time > 1.0:
                return False
            desired, other = "twist_controller", "joint_trajectory_controller"
            # Readiness is a controller check, not a claim that hardware has no fault.
            return self.controllers.get(desired) == "active" and self.controllers.get(other) != "active"

        def current_tool(self):
            try:
                tf = self.tf.lookup_transform(calibration.planning_frame, args.tool_frame, rclpy.time.Time())
                if not self.fresh_stamp(tf.header.stamp):
                    return None
                p, q = tf.transform.translation, tf.transform.rotation
                return ToolPose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            except Exception:
                return None

        def tick(self):
            now = time.monotonic()
            if self.operator is not None:
                from example.operator_hold import TELEOP
                previous_intent = self.intent
                self.intent = self.operator.read(now=now)
                self.home.tick(self.intent, controller_ready=self.controller_ready(now), now=now)
                self.pressed = self.intent.mode == TELEOP and self.home.teleop_allowed
                if (previous_intent is not None and previous_intent.mode == TELEOP
                        and self.intent.token != previous_intent.token):
                    # A very short release can fall between ROS timer ticks.
                    # A new hold generation must still recapture the tracker.
                    self.pressed = False
                self.deadman_time = now if self.intent.fresh else None
            tool = self.current_tool()
            control = None
            if tool is not None:
                try:
                    control = session.control_pose(tool)
                except ValueError:
                    pass  # NativeSession.step emits the corresponding stop.
            pose, self.pending_pose = self.pending_pose, None
            command = session.step(
                now=now, pose=pose, tool=tool, pressed=self.pressed,
                deadman_fresh=self.deadman_time is not None and now-self.deadman_time <= args.timeout,
                feedback_fresh=tool is not None, controller_ready=self.controller_ready(now),
                input_error=self.input_error,
            )
            base = TwistStamped()
            base.header.frame_id = calibration.planning_frame
            base.header.stamp = self.get_clock().now().to_msg()
            base.twist.linear.x, base.twist.linear.y, base.twist.linear.z = command.linear_xyz
            base.twist.angular.x, base.twist.angular.y, base.twist.angular.z = command.angular_xyz
            native = Twist()
            linear, angular = (0., 0., 0.), (0., 0., 0.)
            if command.enabled and tool is not None:
                linear, angular = native_vectors(command, tool)
            native.linear.x, native.linear.y, native.linear.z = linear
            native.angular.x, native.angular.y, native.angular.z = angular
            self.base_pub.publish(base)
            self.native_pub.publish(native)
            if self.output:
                self.output.publish(native)
            record = dict(monotonic_s=now, ros_time_ns=self.get_clock().now().nanoseconds,
                          mode=args.mode, enabled=command.enabled, stop_reason=command.stop_reason,
                          deadman=self.pressed, controllers=self.controllers,
                          tracker=asdict(self.pose) if self.pose else None,
                          tcp=asdict(tool) if tool else None,
                          control_point=asdict(control) if control else None,
                          control_point_frame=calibration.tracker_mount_frame,
                          twist_origin_frame=args.tool_frame, joints=self.joints,
                          base_linear_mps=command.linear_xyz, base_angular_radps=command.angular_xyz,
                          native_tool_linear_mps=linear, native_tool_angular_degps=angular)
            if self.home is not None:
                record.update(operator_mode=self.intent.mode, home_state=self.home.state,
                              home_action=args.home_action, home_error=self.home.error)
            status = String()
            status.data = json.dumps(record, allow_nan=False)
            self.status_pub.publish(status)
            if self.log:
                self.log.write(status.data + "\n")
                if now-self.last_log_flush >= 1.0:
                    self.log.flush()
                    self.last_log_flush = now
            if command.stop_reason != self.last_reason:
                self.get_logger().info(f"state={command.stop_reason or 'FOLLOWING'}")
                self.last_reason = command.stop_reason

        def stop_output(self):
            if self.home is not None:
                self.home.close()
            if self.output:
                message = Twist()
                for _ in range(5):
                    self.output.publish(message)
                    time.sleep(0.02)
            if self.log:
                self.log.close()

    # Retain the ROS context while the finally block publishes its zero burst.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Teleop()

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_output()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
