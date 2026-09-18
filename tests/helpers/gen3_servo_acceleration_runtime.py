"""Run ONLY in a --network none container: fake hardware Servo output regression.

Usage: python3 tests/helpers/gen3_servo_acceleration_runtime.py
This publishes JointJog commands. It refuses containers with a non-loopback
network interface and launches its own fake hardware stack in an isolated domain.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import yaml


def main():
    interfaces = {p.name for p in Path("/sys/class/net").iterdir()}
    if interfaces != {"lo"}:
        raise SystemExit("Refusing motion-command test: run in Docker with --network none")
    os.environ["ROS_DOMAIN_ID"] = "137"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    # CycloneDDS may disable multicast on lo; FastDDS handles this isolated test.
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    import rclpy
    from control_msgs.msg import JointJog
    from controller_manager_msgs.srv import SwitchController
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from std_msgs.msg import Float64
    from trajectory_msgs.msg import JointTrajectory

    names = [f"joint_{i}" for i in range(1, 8)]
    velocity_caps = [0.3] * 7
    acceleration_caps = [0.5, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]
    dt = 0.01
    with tempfile.TemporaryDirectory() as temporary:
        temporary = Path(temporary)
        limits_path = temporary / "limits.yaml"
        limits_path.write_text(yaml.safe_dump({"joint_limits": {
            name: {"max_velocity": v, "max_acceleration": a}
            for name, v, a in zip(names, velocity_caps, acceleration_caps)}}))
        servo = yaml.safe_load(Path("example/config/gen3_vive_moveit_servo.yaml").read_text())
        # Only this synthetic JointJog test disables collisions at the fake
        # startup pose. Repository and real-robot collision configuration stays on.
        servo["check_collisions"] = False
        servo_path = temporary / "servo.yaml"
        servo_path.write_text(yaml.safe_dump(servo))
        environment = {key: value for key, value in os.environ.items() if not key.startswith("GEN3_")}
        environment.update(GEN3_USE_FAKE_HARDWARE="true", GEN3_DOF="7", GEN3_ROBOT_IP="0.0.0.0",
                           GEN3_JOINT_LIMITS_CONFIG=str(limits_path), GEN3_SERVO_CONFIG=str(servo_path),
                           GEN3_SERVO_START_DELAY_S="1.0", GEN3_SERVO_AUTOSTART="false")
        log_path = Path("/tmp/gen3-servo-acceleration-runtime.log")
        with log_path.open("w") as log:
            process = subprocess.Popen(["python3", "example/arm_teleop/gen3_vive_servo_stack.py"],
                                       env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node("gen3_acceleration_runtime_test")
            samples = []
            states = []
            node.create_subscription(JointState, "/joint_states", states.append, qos_profile_sensor_data)
            node.create_subscription(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", samples.append, 1000)
            publisher = node.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
            collision = node.create_publisher(Float64, "/servo_node/collision_velocity_scale", 10)
            service = node.create_client(Trigger, "/servo_node/start_servo")
            switch = node.create_client(SwitchController, "/controller_manager/switch_controller")

            def spin_until(predicate, seconds):
                deadline = time.monotonic() + seconds
                while not predicate() and time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise AssertionError(log_path.read_text()[-5000:])
                    rclpy.spin_once(node, timeout_sec=0.01)
                assert predicate(), log_path.read_text()[-5000:]

            def command(velocities, seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    message = JointJog()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.joint_names = names
                    message.velocities = velocities
                    publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.008)

            try:
                spin_until(lambda: bool(states) and service.service_is_ready() and publisher.get_subscription_count(), 25)
                request = service.call_async(Trigger.Request())
                spin_until(request.done, 5)
                assert request.result().success
                command([0.0] * 7, 0.2)
                samples.clear()
                command([1.5, 0.05, -1.5, 0.03, 1.5, 0.02, -1.5], 0.8)
                command([-1.5, 0.05, 1.5, 0.03, -1.5, 0.02, 1.5], 0.8)
                # Keep at least one joint active while others decelerate to zero.
                command([0.0, 0.05, 0.0, 0.03, 0.0, 0.02, 0.0], 0.5)
                active_samples = list(samples)
                assert len(active_samples) > 100, len(active_samples)
                points = []
                for message in active_samples:
                    order = [message.joint_names.index(name) for name in names]
                    assert len(message.points) == 1
                    points.append([message.points[0].positions[i] for i in order])
                velocities = [[(b - a) / dt for a, b in zip(first, second)]
                              for first, second in zip(points, points[1:])]
                accelerations = [[(b - a) / dt for a, b in zip(first, second)]
                                 for first, second in zip(velocities, velocities[1:])]
                for joint in range(7):
                    assert max(abs(row[joint]) for row in velocities) <= velocity_caps[joint] + 1e-7
                    assert max(abs(row[joint]) for row in accelerations) <= acceleration_caps[joint] + 1e-5
                # Zero command must hold immediately, not add a braking tail.
                command([0.0] * 7, 0.15)
                tail = samples[-5:]
                assert len(tail) == 5
                assert all(message.points[0].positions == tail[-1].points[0].positions for message in tail)
                # Simulate Servo's collision-stop input with commands still on.
                command([0.2] * 7, 0.15)
                stop_scale = Float64()
                stop_scale.data = 0.0
                collision.publish(stop_scale)
                command([0.2] * 7, 0.15)
                tail = samples[-5:]
                collision_drift = max(abs(value - final) for message in tail
                                      for value, final in zip(message.points[0].positions, tail[-1].points[0].positions))
                assert collision_drift < 1e-9, (collision_drift, [list(message.points[0].positions) for message in tail])
                stop_scale.data = 1.0
                collision.publish(stop_scale)
                command([0.2] * 7, 0.2)
                # Stop only the fake feedback broadcaster. Fresh input commands
                # must not keep advancing an integrated reference without feedback.
                spin_until(switch.service_is_ready, 5)
                deactivate = SwitchController.Request()
                deactivate.deactivate_controllers = ["joint_state_broadcaster"]
                deactivate.strictness = SwitchController.Request.STRICT
                response = switch.call_async(deactivate)
                spin_until(response.done, 5)
                assert response.result().ok
                command([0.2] * 7, 0.25)
                count_without_feedback = len(samples)
                command([0.2] * 7, 0.2)
                assert len(samples) == count_without_feedback, "Servo advanced after joint feedback expired"
                assert "reference acceleration limiting enabled" in log_path.read_text()
                assert "joint feedback is stale" in log_path.read_text()
                print(json.dumps({"samples": len(points),
                                  "max_reference_velocity": [max(abs(row[i]) for row in velocities) for i in range(7)],
                                  "max_reference_acceleration": [max(abs(row[i]) for row in accelerations) for i in range(7)],
                                  "immediate_zero_hold": True, "collision_hold": True,
                                  "stale_feedback_holds": True}, indent=2))
            finally:
                node.destroy_node()
                rclpy.shutdown()
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == "__main__":
    main()
