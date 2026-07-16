import unittest
from contextlib import redirect_stdout
from io import StringIO

from software.companion.jetson.mavlink_link import MAV_CMD_DO_MOTOR_TEST, MAV_MODE_FLAG_SAFETY_ARMED, MavlinkError
from software.companion.jetson.motor_test_cli import (
    CONFIRMATION_TEXT,
    MotorTestRequest,
    main,
    run_motor_test,
    validate_motor_test_request,
)
from tests.test_mavlink_link import FakeCommandAck, FakeConnection, FakeHeartbeat, FakeMavlink


class MotorTestCliTest(unittest.TestCase):
    def test_validation_requires_confirmation_for_live_test(self):
        with self.assertRaises(ValueError):
            validate_motor_test_request(MotorTestRequest(motor=1, confirmation=""))

    def test_validation_allows_dry_run_without_confirmation(self):
        validate_motor_test_request(MotorTestRequest(motor=1, dry_run=True))

    def test_validation_rejects_high_throttle(self):
        with self.assertRaises(ValueError):
            validate_motor_test_request(
                MotorTestRequest(motor=1, throttle_percent=16, confirmation=CONFIRMATION_TEXT)
            )

    def test_validation_rejects_long_duration(self):
        with self.assertRaises(ValueError):
            validate_motor_test_request(MotorTestRequest(motor=1, duration_s=4, confirmation=CONFIRMATION_TEXT))

    def test_validation_rejects_motor_outside_f450_range(self):
        with self.assertRaises(ValueError):
            validate_motor_test_request(MotorTestRequest(motor=5, confirmation=CONFIRMATION_TEXT))

    def test_run_motor_test_sends_guarded_command(self):
        connection = FakeConnection(
            heartbeat=FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR),
            recv_messages=[FakeCommandAck(MAV_CMD_DO_MOTOR_TEST, FakeMavlink.MAV_RESULT_ACCEPTED)],
        )
        link = TestLink(connection)

        ack = run_motor_test(link, MotorTestRequest(motor=2, confirmation=CONFIRMATION_TEXT))

        self.assertTrue(ack.accepted)
        self.assertEqual(len(connection.mav.sent), 1)
        sent = connection.mav.sent[0]
        self.assertEqual(sent[:4], (1, 1, MAV_CMD_DO_MOTOR_TEST, 0))
        self.assertEqual(sent[4:11], (2.0, 0.0, 10.0, 2.0, 1.0, 0.0, 0.0))

    def test_run_motor_test_refuses_already_armed_vehicle(self):
        connection = FakeConnection(
            heartbeat=FakeHeartbeat(
                vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR,
                base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
            )
        )
        link = TestLink(connection)

        with self.assertRaises(MavlinkError):
            run_motor_test(link, MotorTestRequest(motor=1, confirmation=CONFIRMATION_TEXT))

    def test_run_motor_test_refuses_non_copter_vehicle(self):
        connection = FakeConnection(heartbeat=FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_FIXED_WING))
        link = TestLink(connection)

        with self.assertRaises(MavlinkError):
            run_motor_test(link, MotorTestRequest(motor=1, confirmation=CONFIRMATION_TEXT))

    def test_run_motor_test_raises_on_rejected_ack(self):
        connection = FakeConnection(
            heartbeat=FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR),
            recv_messages=[FakeCommandAck(MAV_CMD_DO_MOTOR_TEST, FakeMavlink.MAV_RESULT_DENIED)],
        )
        link = TestLink(connection)

        with self.assertRaises(MavlinkError):
            run_motor_test(link, MotorTestRequest(motor=1, confirmation=CONFIRMATION_TEXT))

    def test_dry_run_cli_does_not_require_confirmation_or_connection(self):
        with redirect_stdout(StringIO()):
            self.assertEqual(main(["--dry-run", "--motor", "1"]), 0)


class TestLink:
    def __init__(self, connection):
        from software.companion.jetson.mavlink_link import MavlinkLink

        self._link = MavlinkLink(connection, mavlink_module=FakeMavlink)
        self.connection = connection
        self.mavlink = FakeMavlink

    def wait_heartbeat(self, timeout_s=5.0):
        return self._link.wait_heartbeat(timeout_s=timeout_s)

    def send_command_long(self, *args, **kwargs):
        return self._link.send_command_long(*args, **kwargs)

    def wait_command_ack(self, *args, **kwargs):
        return self._link.wait_command_ack(*args, **kwargs)


if __name__ == "__main__":
    unittest.main()
