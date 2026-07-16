import unittest
from contextlib import redirect_stdout
from io import StringIO

from software.companion.jetson.mavlink_link import MAV_MODE_FLAG_SAFETY_ARMED, MavlinkError
from software.companion.jetson.px4_actuator_test_cli import (
    CONFIRMATION_TEXT,
    Px4ActuatorTestRequest,
    main,
    run_px4_actuator_test,
    validate_px4_actuator_test_request,
)
from tests.test_mavlink_link import FakeCommandAck, FakeConnection, FakeHeartbeat, FakeMavlink


class Px4ActuatorTestCliTest(unittest.TestCase):
    def test_validation_requires_confirmation_for_live_test(self):
        with self.assertRaises(ValueError):
            validate_px4_actuator_test_request(Px4ActuatorTestRequest(motor=1, confirmation=""))

    def test_validation_allows_dry_run_without_confirmation(self):
        validate_px4_actuator_test_request(Px4ActuatorTestRequest(motor=1, dry_run=True))

    def test_validation_rejects_high_value(self):
        with self.assertRaises(ValueError):
            validate_px4_actuator_test_request(Px4ActuatorTestRequest(motor=1, value=0.20, confirmation=CONFIRMATION_TEXT))

    def test_validation_rejects_long_timeout(self):
        with self.assertRaises(ValueError):
            validate_px4_actuator_test_request(
                Px4ActuatorTestRequest(motor=1, timeout_s=4.0, confirmation=CONFIRMATION_TEXT)
            )

    def test_run_px4_actuator_test_sends_motor_function_command(self):
        connection = FakeConnection(
            recv_messages=[
                FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_GCS, system=255, component=190),
                FakeHeartbeat(vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR),
                FakeCommandAck(FakeMavlink.MAV_CMD_ACTUATOR_TEST, FakeMavlink.MAV_RESULT_ACCEPTED),
            ],
        )
        link = TestLink(connection)

        ack = run_px4_actuator_test(link, Px4ActuatorTestRequest(motor=2, confirmation=CONFIRMATION_TEXT))

        self.assertTrue(ack.accepted)
        self.assertEqual(len(connection.mav.sent), 1)
        sent = connection.mav.sent[0]
        self.assertEqual(sent[:4], (1, 1, FakeMavlink.MAV_CMD_ACTUATOR_TEST, 0))
        self.assertEqual(sent[4:11], (0.10, 2.0, 0.0, 0.0, 2.0, 0.0, 0.0))

    def test_run_px4_actuator_test_refuses_armed_vehicle(self):
        connection = FakeConnection(
            recv_messages=[FakeHeartbeat(
                vehicle_type=FakeMavlink.MAV_TYPE_QUADROTOR,
                base_mode=MAV_MODE_FLAG_SAFETY_ARMED,
            )]
        )
        link = TestLink(connection)

        with self.assertRaises(MavlinkError):
            run_px4_actuator_test(link, Px4ActuatorTestRequest(motor=1, confirmation=CONFIRMATION_TEXT))

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

    def wait_vehicle_heartbeat(self, timeout_s=5.0):
        return self._link.wait_vehicle_heartbeat(timeout_s=timeout_s)

    def send_command_long(self, *args, **kwargs):
        return self._link.send_command_long(*args, **kwargs)

    def wait_command_ack(self, *args, **kwargs):
        return self._link.wait_command_ack(*args, **kwargs)


if __name__ == "__main__":
    unittest.main()
