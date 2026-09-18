"""Cartesian IK regression, ONLY in a Docker container with --network none.

Runs Servo alone with synthetic feedback; no hardware driver or robot RPCs.
Reproduces the invalid IK references caused by unwrapped continuous-joint seeds.
"""

import contextlib
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from unittest.mock import patch

import yaml


def main():
    if {path.name for path in Path("/sys/class/net").iterdir()} != {"lo"}:
        raise SystemExit("Refusing motion-command test: run in Docker with --network none")
    for key in list(os.environ):
        if key.startswith("GEN3_"):
            del os.environ[key]
    os.environ.update(ROS_DOMAIN_ID="138", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                      GEN3_USE_FAKE_HARDWARE="true", GEN3_SERVO_AUTOSTART="false", GEN3_DOF="7")
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from std_msgs.msg import Int8
    from trajectory_msgs.msg import JointTrajectory
    from example.arm_teleop import gen3_vive_servo_stack as stack

    # Construct parameters without running the launch actions or any driver.
    with patch.object(stack, "Node", wraps=stack.Node) as node_call, contextlib.redirect_stdout(io.StringIO()):
        stack._generate_launch_description()
    params = {}
    for entry in node_call.call_args.kwargs["parameters"]:
        params.update(entry)
    params["moveit_servo"].update(check_collisions=False, is_primary_planning_scene_monitor=True,
                                 joint_topic="/cartesian_test_states", command_out_topic="/cartesian_test_output",
                                 monitored_planning_scene_topic="/cartesian_test_scene")
    names = [f"joint_{i}" for i in range(1, 8)]
    continuous = (0, 2, 4, 6)
    bounds = params["robot_description_planning"]["joint_limits"]
    # Reproduce the effective model/robot caps without contacting a robot.
    for name in names[:4]:
        bounds[name]["max_acceleration"] = 5.2
    velocity_caps = [bounds[name]["max_velocity"] for name in names]
    acceleration_caps = [bounds[name]["max_acceleration"] for name in names]
    dt = params["moveit_servo"]["publish_period"]
    # Saved reproduction pose: joints 1 and 3 are close to the angle boundary.
    q = [math.remainder(math.radians(value), 2 * math.pi)
         for value in (180.2734, 312.9509, 178.7571, 256.2889, 354.5828, 311.4148, 88.0068)]
    target = q[:]
    angular = [0.0] * 3
    samples = []
    wrapped_feedback = []
    tracking_enabled = True
    statuses = []
    log_path = Path("/tmp/gen3-servo-cartesian-runtime.log")
    with tempfile.TemporaryDirectory() as temporary, log_path.open("w") as log:
        config = Path(temporary) / "params.yaml"
        config.write_text(yaml.safe_dump({"servo_node": {"ros__parameters": params}}))
        process = subprocess.Popen(
            ["ros2", "run", "moveit_servo", "servo_node_main", "--ros-args", "-r", "__node:=servo_node",
             "--params-file", str(config)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node("gen3_cartesian_runtime_test")
        feedback = node.create_publisher(JointState, "/cartesian_test_states", 10)
        command = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        start = node.create_client(Trigger, "/servo_node/start_servo")

        def on_output(message):
            assert len(message.points) == 1
            positions = [message.points[0].positions[message.joint_names.index(name)] for name in names]
            assert all(math.isfinite(value) for value in positions), positions
            samples.append(positions)
            target[:] = positions

        node.create_subscription(JointTrajectory, "/cartesian_test_output", on_output, 1000)
        node.create_subscription(Int8, "/servo_node/status", lambda message: statuses.append(message.data), 1000)

        def tick():
            previous = q[:]
            # Tracking lag plus wrapped sensor feedback exercise both angle
            # representations; ideal unwrapped feedback hides the regression.
            if tracking_enabled:
                for i in range(7):
                    q[i] += 0.5 * (target[i] - q[i])
            state = JointState()
            state.header.stamp = node.get_clock().now().to_msg()
            state.name = names + ["robotiq_85_left_knuckle_joint"]
            state.position = [math.remainder(value, 2 * math.pi) if i in continuous else value
                              for i, value in enumerate(q)] + [0.0]
            state.velocity = [(b - a) / dt for a, b in zip(previous, q)] + [0.0]
            wrapped_feedback.append(state.position[:7])
            feedback.publish(state)
            twist = TwistStamped()
            twist.header.stamp = state.header.stamp
            twist.header.frame_id = "base_link"
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = angular
            command.publish(twist)

        node.create_timer(dt, tick)

        def spin(seconds, until=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise AssertionError(log_path.read_text()[-5000:])
                rclpy.spin_once(node, timeout_sec=dt)
                if until is not None and until():
                    return
            if until is not None:
                raise AssertionError(log_path.read_text()[-5000:])

        try:
            spin(15, lambda: start.service_is_ready() and feedback.get_subscription_count()
                 and command.get_subscription_count())
            spin(0.5)
            response = start.call_async(Trigger.Request())
            spin(5, response.done)
            assert response.result().success
            spin(0.2)
            samples.clear()
            for magnitude in (1.0, 1.5):
                for axis in range(3):
                    for direction in (1.0, -1.0):
                        angular[:] = [direction * magnitude if i == axis else 0.0 for i in range(3)]
                        spin(0.6)
            points = list(samples)
            angular[:] = [0.0] * 3
            spin(0.15)
            assert len(points) > 500, len(points)
            velocities = [[(b - a) / dt for a, b in zip(first, second)]
                          for first, second in zip(points, points[1:])]
            accelerations = [[(b - a) / dt for a, b in zip(first, second)]
                             for first, second in zip(velocities, velocities[1:])]
            max_velocity = [max(abs(row[i]) for row in velocities) for i in range(7)]
            max_acceleration = [max(abs(row[i]) for row in accelerations) for i in range(7)]
            for i in range(7):
                assert max_velocity[i] <= velocity_caps[i] + 1e-7, (i, max_velocity, velocity_caps)
                assert max_acceleration[i] <= acceleration_caps[i] + 1e-5, (i, max_acceleration, acceleration_caps)
            wraps = sum(abs(b[i] - a[i]) > math.pi for a, b in zip(wrapped_feedback, wrapped_feedback[1:])
                        for i in continuous)
            assert wraps > 0, "Test did not cross a wrapped sensor angle boundary"
            assert min(row[0] for row in points) < -math.pi - 0.001, "Unwrapped IK path was not exercised"
            assert max(max_velocity) > 0.1, "Test did not produce meaningful motion"
            tail = samples[-5:]
            assert all(row == tail[-1] for row in tail), "Zero command did not hold the last reference"
            diagnostics = log_path.read_text()
            for error in ("invalid joint reference", "non-finite IK", "joint feedback is stale"):
                assert error not in diagnostics, diagnostics[-5000:]
            # Regress the removed 0.1 rad tracking latch using synthetic lag.
            # Stop this phase as soon as the former threshold is exceeded.
            tracking_enabled = False
            frozen = q[:]
            samples.clear()
            statuses.clear()
            angular[:] = [1.5, 0.0, 0.0]
            spin(1.5, lambda: samples and max(abs(a-b) for a,b in zip(samples[-1], frozen)) > 0.12)
            diagnostics = log_path.read_text()
            assert "joint tracking error exceeded" not in diagnostics, diagnostics[-3000:]
            assert "close to a position limit" not in diagnostics, diagnostics[-3000:]
            assert "joint feedback is stale" not in diagnostics, diagnostics[-3000:]
            assert -1 not in statuses, "Removed tracking guard still set INVALID status"
            # Zero commands must still hold; a later command works without a
            # process restart even after exceeding the old error threshold.
            angular[:] = [0.0] * 3
            spin(0.2)
            held = samples[-1][:]
            assert all(row == held for row in samples[-5:]), "Zero command did not hold after lag"
            assert max(abs(a-b) for a,b in zip(held, frozen)) > 0.1, "Test missed the removed threshold"
            tracking_enabled = True
            angular[:] = [-1.5, 0.0, 0.0]
            spin(0.4)
            assert max(abs(a-b) for a,b in zip(samples[-1], held)) > 0.01, "Motion remained latched"
            assert -1 not in statuses, "Unexpected INVALID status after lag"
            print(json.dumps({"samples": len(points), "wrapped_feedback_crossings": wraps,
                              "max_reference_velocity": max_velocity,
                              "max_reference_acceleration": max_acceleration,
                              "finite_ik_and_output": True, "zero_command_hold": True,
                              "tracking_error_latch_removed": True, "resumes_without_restart": True}, indent=2))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == "__main__":
    main()
