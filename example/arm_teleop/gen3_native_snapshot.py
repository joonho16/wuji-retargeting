"""Read-only feedback, saved joint actions and encoder offsets for diagnostics.

No fault clearing, motion, power cycling or offset/calibration writes.
"""
import argparse
from datetime import datetime, timezone
import json
import os


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default=os.environ.get('GEN3_ROBOT_IP'), required=False)
    args = parser.parse_args()
    if not args.robot_ip:
        parser.error('--robot-ip or GEN3_ROBOT_IP is required')
    from google.protobuf.json_format import MessageToDict
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
    from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient

    transport = TCPTransport()
    router = RouterClient(transport, lambda _error: None)
    session = SessionManager(router)
    options = RouterClientSendOptions()
    options.timeout_ms = 3000
    authenticated = False
    as_dict = lambda message: MessageToDict(message, preserving_proto_field_name=True)
    try:
        transport.connect(args.robot_ip, 10000)
        session.CreateSession(Session_pb2.CreateSessionInfo(
            username=os.environ.get('GEN3_LIMIT_QUERY_USERNAME', 'admin'),
            password=os.environ.get('GEN3_LIMIT_QUERY_PASSWORD', 'admin'),
            session_inactivity_timeout=10000, connection_inactivity_timeout=2000,
        ), options=options)
        authenticated = True
        base = BaseClient(router)
        feedback = BaseCyclicClient(router).RefreshFeedback(options=options)
        result = dict(utc=datetime.now(timezone.utc).isoformat(), robot_ip=args.robot_ip,
                      servoing_mode=Base_pb2.ServoingMode.Name(base.GetServoingMode(options=options).servoing_mode),
                      active_state=Common_pb2.ArmState.Name(feedback.base.active_state),
                      base=as_dict(feedback.base), actuators=[as_dict(a) for a in feedback.actuators],
                      tool=as_dict(ControlConfigClient(router).GetToolConfiguration(options=options)))
        for name, read in (
            ('joint_actions', lambda: base.ReadAllActions(
                Base_pb2.RequestedActionType(action_type=Base_pb2.REACH_JOINT_ANGLES), options=options)),
            ('measured_joint_angles', lambda: base.GetMeasuredJointAngles(options=options)),
        ):
            try:
                result[name] = as_dict(read())
            except Exception as exc:
                result[name + '_error'] = str(exc)
        config = ActuatorConfigClient(router)
        result['axis_offsets'] = {}
        for device_id in range(1, len(feedback.actuators)+1):
            try:
                result['axis_offsets'][device_id] = as_dict(config.GetAxisOffsets(device_id, options=options))
            except Exception as exc:
                result['axis_offsets'][device_id] = {'error': str(exc)}
        print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    finally:
        try:
            if authenticated:
                session.CloseSession(options=options)
        finally:
            transport.disconnect()


if __name__ == '__main__':
    main()
