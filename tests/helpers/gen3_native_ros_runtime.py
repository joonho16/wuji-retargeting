"""Synthetic ROS integration only. Refuses any non-loopback network interface."""
from pathlib import Path
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import yaml


def main():
    if {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}:
        raise SystemExit('Run only in Docker --network none; this publishes synthetic commands.')
    import rclpy
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
    from std_msgs.msg import Bool, String
    from tf2_ros import TransformBroadcaster
    from example.operator_hold import IntentWriter, TELEOP, STOP, HOME

    os.environ['ROS_DOMAIN_ID'] = '183'
    rclpy.init()
    node = rclpy.create_node('synthetic_native_comparison_test')
    tf = TransformBroadcaster(node)
    poses = node.create_publisher(PoseStamped, '/vive/gen3_tracker/pose', 1)
    deadman = node.create_publisher(Bool, '/gen3_vive/physical_deadman', 1)
    repo = Path(__file__).resolve().parents[2]
    lab = repo / 'example/arm_teleop'
    calibration_dir = tempfile.TemporaryDirectory()
    calibration = Path(calibration_dir.name) / 'calibration.yaml'
    data = yaml.safe_load((repo / 'example/config/gen3_vive_dry_run_calibration.yaml').read_text())
    data['real_robot_calibration_reviewed'] = True
    calibration.write_text(yaml.safe_dump(data))
    mode = 'preview'
    readings, statuses = [], []
    operator = None
    twist_active = False
    switches = []
    action_status = node.create_publisher(String, '/gen3_vive/saved_action/status', 1)
    def saved_action(message):
        fields = message.data.split('\n')
        assert len(fields) == 4
        # Synthetic bridge intentionally rejects actions; no SDK or robot socket.
        response = String()
        response.data = fields[1] + ('\nfailed' if fields[0] == 'H' else '\nstopped')
        action_status.publish(response)
    action_sub = node.create_subscription(String, '/gen3_vive/saved_action/command', saved_action, 1)

    def controllers(_request, response):
        for name in ('twist_controller', 'joint_trajectory_controller'):
            c = ControllerState()
            c.name = name
            c.state = 'active' if (name == 'twist_controller' and twist_active) else 'inactive'
            response.controller.append(c)
        return response

    service = node.create_service(ListControllers, '/controller_manager/list_controllers', controllers)

    def switch(request, response):
        nonlocal twist_active
        assert request.strictness == SwitchController.Request.STRICT
        activate, deactivate = request.activate_controllers, request.deactivate_controllers
        assert (activate, deactivate) in [(['twist_controller'], []), ([], ['twist_controller'])]
        twist_active = bool(activate)
        switches.append(twist_active)
        response.ok = True
        return response
    switch_service = node.create_service(SwitchController, '/controller_manager/switch_controller', switch)

    def publish_for(duration, *, pressed, angle=0., position=(0., 0., 0.),
                    feedback=True, send_deadman=True, operator_mode=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            if feedback:
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'base_link', 'end_effector_link'
                t.transform.rotation.z = math.sqrt(.5)
                t.transform.rotation.w = math.sqrt(.5)
                tf.sendTransform(t)
            p = PoseStamped()
            p.header.stamp, p.header.frame_id = stamp, 'steamvr_world'
            p.pose.position.x, p.pose.position.y, p.pose.position.z = position
            p.pose.orientation.x = math.sin(angle/2)
            p.pose.orientation.w = math.cos(angle/2)
            poses.publish(p)
            if send_deadman:
                if operator is not None:
                    operator.publish(operator_mode or (TELEOP if pressed else STOP))
                d = Bool()
                d.data = pressed
                deadman.publish(d)
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        for mode, use_operator in (('preview', False), ('native', False), ('native', True)):
            twist_active = mode == 'native'
            state_path = Path(calibration_dir.name) / 'operator.json'
            operator = IntentWriter(state_path) if use_operator else None
            topic = {'preview': '/gen3_vive/debug/native_twist', 'native': '/twist_controller/commands',
                     'servo': '/servo_node/delta_twist_cmds'}[mode]
            typ = TwistStamped if mode == 'servo' else Twist
            readings.clear()
            statuses.clear()
            def receive(msg):
                t = msg.twist if mode == 'servo' else msg
                readings.append((t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z))
            subscription = node.create_subscription(typ, topic, receive, 10)
            status_sub = node.create_subscription(String, '/gen3_vive/status',
                                                  lambda m: statuses.append(json.loads(m.data)), 10)
            with tempfile.TemporaryFile(mode='w+') as output:
                process = subprocess.Popen([
                    sys.executable, str(lab / 'gen3_native_teleop.py'), '--mode', mode,
                    '--calibration', str(calibration),
                ] + (['--operator-state', str(state_path), '--robot-ip', '127.0.0.1'] if use_operator else []),
                    stdout=output, stderr=subprocess.STDOUT)
                try:
                    publish_for(1.5, pressed=False)
                    assert process.poll() is None, 'Node exited during initialization'
                    if mode == 'preview':
                        assert not node.get_publishers_info_by_topic('/twist_controller/commands')
                        assert not node.get_publishers_info_by_topic('/servo_node/delta_twist_cmds')
                    publish_for(.5, pressed=True)
                    readings.clear()
                    publish_for(.6, pressed=True, angle=math.radians(10))
                    expected = (0, 0, 0, math.radians(10), 0, 0) if mode == 'servo' else (0, 0, 0, 0, -10, 0)
                    assert readings and any(max(abs(a-b) for a, b in zip(r, expected)) < 1e-5 for r in readings), (mode, readings[-3:])
                    readings.clear()
                    publish_for(.6, pressed=True, angle=math.radians(10), position=(.03, 0., 0.))
                    expected = ((.03, 0, 0, math.radians(10), 0, 0) if mode == 'servo'
                                else (0, -.03, 0, 0, -10, 0))
                    assert readings and any(max(abs(a-b) for a, b in zip(r, expected)) < 1e-5
                                            for r in readings), ('combined motion', mode, readings[-3:])
                    publish_for(.35, pressed=True, angle=math.radians(10), feedback=False)
                    assert max(abs(x) for x in readings[-1]) == 0., 'Stale TF did not stop'
                    publish_for(.3, pressed=True, angle=math.radians(10))
                    assert max(abs(x) for x in readings[-1]) == 0., 'Resumed without new deadman edge'
                    publish_for(.2, pressed=False)
                    publish_for(.3, pressed=True)
                    publish_for(.4, pressed=True, angle=math.radians(10))
                    assert any(abs(x) > .01 for x in readings[-1]), 'Failed to recapture'
                    publish_for(.35, pressed=True, angle=math.radians(10), send_deadman=False)
                    assert max(abs(x) for x in readings[-1]) == 0., 'Deadman timeout did not stop'
                    assert any(s['stop_reason'] == 'DEADMAN_TIMEOUT' for s in statuses)
                    if use_operator:
                        publish_for(.25, pressed=False)
                        publish_for(.8, pressed=False, operator_mode=HOME)
                        assert False in switches, 'B did not deactivate Twist before saved action'
                        assert not twist_active, 'Twist reactivated while B remained held after action failure'
                        # The synthetic bridge rejects the action; recovery
                        # waits for release and a stopped acknowledgement.
                        assert max(abs(x) for x in readings[-1]) == 0.
                        publish_for(1.1, pressed=False)
                        assert switches[-1] is True, 'B release did not restore Twist'
                        publish_for(.3, pressed=True)
                        publish_for(.4, pressed=True, angle=math.radians(10))
                        assert any(abs(x) > .01 for x in readings[-1]), 'A failed after B handover'
                    print(f'{mode} operator={use_operator}: combined motion, axis/units, stale TF, '
                          'deadman timeout, recapture and controller handover passed', flush=True)
                except Exception:
                    output.seek(0)
                    print(output.read())
                    raise
                finally:
                    process.send_signal(signal.SIGTERM) if process.poll() is None else None
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    if operator is not None:
                        operator.close()
                        operator = None
            node.destroy_subscription(subscription)
            node.destroy_subscription(status_sub)
    finally:
        node.destroy_service(service)
        node.destroy_service(switch_service)
        node.destroy_node()
        rclpy.shutdown()
        calibration_dir.cleanup()


if __name__ == '__main__':
    main()
