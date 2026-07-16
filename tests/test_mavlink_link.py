import unittest

from software.companion.jetson.mavlink_link import (
    MAV_CMD_DO_MOTOR_TEST,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MavlinkLink,
    MavlinkTimeoutError,
)


class MavlinkLinkTest(unittest.TestCase):
    def test_wait_heartbeat_parses_copter_disarmed_status(self):
        connection = FakeConnection(heartbeat=FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR))
        link = MavlinkLink(connection, mavlink_module=FakeMavlink)

        status = link.wait_heartbeat(timeout_s=0.1)

        self.assertEqual(status.target_system, 1)
        self.assertEqual(status.target_component, 1)
        self.assertTrue(status.is_copter)
        self.assertFalse(status.armed)

    def test_wait_heartbeat_detects_armed_status(self):
        connection = FakeConnection(
            heartbeat=FakeHeartbeat(
                vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR,
                base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
            )
        )
        link = MavlinkLink(connection, mavlink_module=FakeMavlink)

        status = link.wait_heartbeat(timeout_s=0.1)

        self.assertTrue(status.armed)

    def test_wait_vehicle_heartbeat_skips_gcs_heartbeat(self):
        connection = FakeConnection(
            recv_messages=[
                FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_GCS, system=255, component=190),
                FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR, system=1, component=1),
            ]
        )
        link = MavlinkLink(connection, mavlink_module=FakeMavlink)

        status = link.wait_vehicle_heartbeat(timeout_s=0.1)

        self.assertEqual(status.target_system, 1)
        self.assertTrue(status.is_copter)

    def test_send_command_long_uses_heartbeat_targets_and_pads_params(self):
        connection = FakeConnection(heartbeat=FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR))
        link = MavlinkLink(connection, mavlink_module=FakeMavlink)
        link.wait_heartbeat(timeout_s=0.1)

        link.send_command_long(MAV_CMD_DO_MOTOR_TEST, [1, 0, 10])

        self.assertEqual(len(connection.mav.sent), 1)
        sent = connection.mav.sent[0]
        self.assertEqual(sent[:4], (1, 1, MAV_CMD_DO_MOTOR_TEST, 0))
        self.assertEqual(sent[4:], (1.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0))

    def test_wait_command_ack_returns_accepted_ack(self):
        connection = FakeConnection(
            heartbeat=FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR),
            recv_messages=[FakeCommandAck(MAV_CMD_DO_MOTOR_TEST, FakeMavlink.MAV_RESULT_ACCEPTED)],
        )
        link = MavlinkLink(connection, mavlink_module=FakeMavlink)

        ack = link.wait_command_ack(MAV_CMD_DO_MOTOR_TEST, timeout_s=0.1)

        self.assertTrue(ack.accepted)
        self.assertEqual(ack.result_name, "MAV_RESULT_ACCEPTED")

    def test_wait_command_ack_times_out(self):
        link = MavlinkLink(FakeConnection(), mavlink_module=FakeMavlink)

        with self.assertRaises(MavlinkTimeoutError):
            link.wait_command_ack(MAV_CMD_DO_MOTOR_TEST, timeout_s=0.01)


class FakeEnumValue:
    def __init__(self, name):
        self.name = name


class FakeMavlink:
    MAV_TYPE_QUADROTOR = 2
    MAV_TYPE_FIXED_WING = 1
    MAV_TYPE_GCS = 6
    MAV_MODE_FLAG_SAFETY_ARMED = MAV_MODE_FLAG_SAFETY_ARMED
    MAV_RESULT_ACCEPTED = 0
    MAV_RESULT_DENIED = 14
    MAV_CMD_SET_MESSAGE_INTERVAL = 511
    MAV_CMD_DO_MOTOR_TEST = MAV_CMD_DO_MOTOR_TEST
    MAV_CMD_ACTUATOR_TEST = 310
    MOTOR_TEST_THROTTLE_PERCENT = 0
    MOTOR_TEST_ORDER_DEFAULT = 0
    ACTUATOR_OUTPUT_FUNCTION_MOTOR1 = 1
    ACTUATOR_OUTPUT_FUNCTION_MOTOR2 = 2
    ACTUATOR_OUTPUT_FUNCTION_MOTOR3 = 3
    ACTUATOR_OUTPUT_FUNCTION_MOTOR4 = 4
    enums = {
        "MAV_RESULT": {
            0: FakeEnumValue("MAV_RESULT_ACCEPTED"),
            14: FakeEnumValue("MAV_RESULT_DENIED"),
        }
    }


class FakeMavSender:
    def __init__(self):
        self.sent = []

    def command_long_send(self, *args):
        self.sent.append(args)


class FakeConnection:
    def __init__(self, heartbeat=None, recv_messages=None):
        self.heartbeat = heartbeat
        self.recv_messages = list(recv_messages or [])
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMavSender()

    def wait_heartbeat(self, timeout=0):
        return self.heartbeat

    def recv_match(self, type=None, blocking=True, timeout=0):
        return self.recv_messages.pop(0) if self.recv_messages else None


class FakeHeartbeat:
    def __init__(self, *, vehicle_type, base_mode=0, custom_mode=0, system=1, component=1):
        self.type = vehicle_type
        self.autopilot = 3
        self.base_mode = base_mode
        self.custom_mode = custom_mode
        self.system = system
        self.component = component

    def get_srcSystem(self):
        return self.system

    def get_srcComponent(self):
        return self.component


class FakeCommandAck:
    def __init__(self, command, result):
        self.command = command
        self.result = result


if __name__ == "__main__":
    unittest.main()
