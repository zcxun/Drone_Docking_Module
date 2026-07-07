import unittest

from software.companion.docking_state_machine import (
    AbortReason,
    DockingConfig,
    DockingState,
    DockingStateMachine,
    SensorSnapshot,
)


class DockingStateMachineTest(unittest.TestCase):
    def test_successful_tabletop_sequence_reaches_complete(self):
        machine = DockingStateMachine()
        sequence = [
            SensorSnapshot(time_s=0.0, auto_enabled=True),
            SensorSnapshot(time_s=1.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.10, target_offset_y_m=0.00, yaw_error_deg=15.0, range_m=0.5),
            SensorSnapshot(time_s=2.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.01, yaw_error_deg=12.0, range_m=0.5),
            SensorSnapshot(time_s=3.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.01, yaw_error_deg=2.0, range_m=0.4),
            SensorSnapshot(time_s=4.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=2.0, range_m=0.2),
            SensorSnapshot(time_s=5.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=2.0, range_m=0.08, contact=True),
            SensorSnapshot(time_s=5.2, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=2.0, contact=True),
            SensorSnapshot(time_s=6.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.00, target_offset_y_m=0.00, yaw_error_deg=0.0, contact=True, seated=True),
            SensorSnapshot(time_s=6.9, auto_enabled=True, contact=True, seated=True),
            SensorSnapshot(time_s=7.1, auto_enabled=True, contact=True, seated=True, lock_switch_closed=True, hall_locked=True),
            SensorSnapshot(time_s=8.8, auto_enabled=True, contact=True, seated=True, lock_switch_closed=True, hall_locked=True),
        ]

        command = None
        for snapshot in sequence:
            command = machine.update(snapshot)

        self.assertIsNotNone(command)
        self.assertEqual(machine.state, DockingState.COMPLETE)
        self.assertEqual(command.state, DockingState.COMPLETE)
        self.assertEqual(command.abort_reason, AbortReason.NONE)

    def test_target_loss_aborts_during_alignment(self):
        machine = DockingStateMachine(DockingConfig(target_loss_timeout_s=0.5))

        machine.update(SensorSnapshot(time_s=0.0, auto_enabled=True))
        machine.update(SensorSnapshot(time_s=0.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.2, target_offset_y_m=0.0, yaw_error_deg=0.0, range_m=0.5))
        command = machine.update(SensorSnapshot(time_s=0.8, auto_enabled=True, target_visible=False))

        self.assertEqual(command.state, DockingState.ABORT)
        self.assertEqual(command.abort_reason, AbortReason.TARGET_LOST)

    def test_single_lock_sensor_does_not_pass_verification(self):
        machine = DockingStateMachine(DockingConfig(lock_verify_timeout_s=0.5))
        self._drive_to_lock_verify(machine)

        command = machine.update(
            SensorSnapshot(
                time_s=7.0,
                auto_enabled=True,
                contact=True,
                seated=True,
                lock_switch_closed=True,
                hall_locked=False,
            )
        )

        self.assertEqual(command.state, DockingState.ABORT)
        self.assertEqual(command.abort_reason, AbortReason.LOCK_SENSOR_DISAGREE)

    def test_pilot_override_aborts(self):
        machine = DockingStateMachine()
        machine.update(SensorSnapshot(time_s=0.0, auto_enabled=True))
        machine.update(SensorSnapshot(time_s=0.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.2, target_offset_y_m=0.0, yaw_error_deg=0.0, range_m=0.5))
        command = machine.update(
            SensorSnapshot(
                time_s=0.2,
                auto_enabled=True,
                target_visible=True,
                target_offset_x_m=0.2,
                target_offset_y_m=0.0,
                yaw_error_deg=0.0,
                range_m=0.5,
                pilot_override=True,
            )
        )

        self.assertEqual(command.state, DockingState.ABORT)
        self.assertEqual(command.abort_reason, AbortReason.PILOT_OVERRIDE)

    def test_unsafe_attitude_aborts(self):
        machine = DockingStateMachine(DockingConfig(max_attitude_deg=8.0))
        machine.update(SensorSnapshot(time_s=0.0, auto_enabled=True))
        machine.update(SensorSnapshot(time_s=0.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.2, target_offset_y_m=0.0, yaw_error_deg=0.0, range_m=0.5))
        command = machine.update(
            SensorSnapshot(
                time_s=0.2,
                auto_enabled=True,
                target_visible=True,
                target_offset_x_m=0.2,
                target_offset_y_m=0.0,
                yaw_error_deg=0.0,
                range_m=0.5,
                roll_deg=9.0,
            )
        )

        self.assertEqual(command.state, DockingState.ABORT)
        self.assertEqual(command.abort_reason, AbortReason.UNSAFE_ATTITUDE)

    def test_contact_with_large_offset_aborts(self):
        machine = DockingStateMachine()
        machine.update(SensorSnapshot(time_s=0.0, auto_enabled=True))
        machine.update(SensorSnapshot(time_s=0.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.0, yaw_error_deg=0.0, range_m=0.5))
        machine.update(SensorSnapshot(time_s=0.2, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.0, yaw_error_deg=0.0, range_m=0.4))
        machine.update(SensorSnapshot(time_s=0.3, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.0, yaw_error_deg=0.0, range_m=0.3))

        command = machine.update(
            SensorSnapshot(
                time_s=0.4,
                auto_enabled=True,
                target_visible=True,
                target_offset_x_m=0.20,
                target_offset_y_m=0.0,
                yaw_error_deg=0.0,
                range_m=0.08,
                contact=True,
            )
        )

        self.assertEqual(command.state, DockingState.ABORT)
        self.assertEqual(command.abort_reason, AbortReason.CONTACT_WITHOUT_ALIGNMENT)

    def test_lost_contact_after_detection_aborts(self):
        machine = DockingStateMachine()
        sequence = [
            SensorSnapshot(time_s=0.0, auto_enabled=True),
            SensorSnapshot(time_s=0.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.5),
            SensorSnapshot(time_s=0.2, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.4),
            SensorSnapshot(time_s=0.3, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.3),
            SensorSnapshot(time_s=1.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.08, contact=True),
        ]
        for snapshot in sequence:
            machine.update(snapshot)

        command = machine.update(SensorSnapshot(time_s=1.1, auto_enabled=True, contact=False))

        self.assertEqual(command.state, DockingState.ABORT)
        self.assertEqual(command.abort_reason, AbortReason.CONTACT_LOST)

    def _drive_to_lock_verify(self, machine):
        sequence = [
            SensorSnapshot(time_s=0.0, auto_enabled=True),
            SensorSnapshot(time_s=0.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.5),
            SensorSnapshot(time_s=0.2, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.4),
            SensorSnapshot(time_s=0.3, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.3),
            SensorSnapshot(time_s=1.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, range_m=0.08, contact=True),
            SensorSnapshot(time_s=1.1, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=0.0, contact=True),
            SensorSnapshot(time_s=1.2, auto_enabled=True, target_visible=True, target_offset_x_m=0.00, target_offset_y_m=0.00, yaw_error_deg=0.0, contact=True, seated=True),
            SensorSnapshot(time_s=2.1, auto_enabled=True, contact=True, seated=True),
        ]
        for snapshot in sequence:
            machine.update(snapshot)
        self.assertEqual(machine.state, DockingState.LOCK_VERIFY)


if __name__ == "__main__":
    unittest.main()
