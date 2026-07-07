"""Dry-run the tabletop docking sequence without flight hardware."""

from docking_state_machine import DockingStateMachine, SensorSnapshot


def main() -> None:
    machine = DockingStateMachine()
    sequence = [
        SensorSnapshot(time_s=0.0, auto_enabled=False),
        SensorSnapshot(time_s=0.2, auto_enabled=True),
        SensorSnapshot(time_s=1.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.12, target_offset_y_m=-0.04, yaw_error_deg=16.0, range_m=0.5),
        SensorSnapshot(time_s=2.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.02, target_offset_y_m=0.01, yaw_error_deg=12.0, range_m=0.45),
        SensorSnapshot(time_s=3.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.02, target_offset_y_m=0.01, yaw_error_deg=3.0, range_m=0.40),
        SensorSnapshot(time_s=4.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=2.0, range_m=0.20),
        SensorSnapshot(time_s=5.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=2.0, range_m=0.08, contact=True),
        SensorSnapshot(time_s=5.2, auto_enabled=True, target_visible=True, target_offset_x_m=0.01, target_offset_y_m=0.00, yaw_error_deg=2.0, contact=True),
        SensorSnapshot(time_s=6.0, auto_enabled=True, target_visible=True, target_offset_x_m=0.00, target_offset_y_m=0.00, yaw_error_deg=0.0, contact=True, seated=True),
        SensorSnapshot(time_s=6.9, auto_enabled=True, contact=True, seated=True),
        SensorSnapshot(time_s=7.2, auto_enabled=True, contact=True, seated=True, lock_switch_closed=True, hall_locked=True),
        SensorSnapshot(time_s=9.0, auto_enabled=True, contact=True, seated=True, lock_switch_closed=True, hall_locked=True),
    ]

    for snapshot in sequence:
        command = machine.update(snapshot)
        print(
            f"t={snapshot.time_s:>4.1f}s "
            f"state={command.state.value:<18} "
            f"mode={command.mode:<7} "
            f"vz={command.vertical_velocity_mps:>5.2f} "
            f"lock={command.lock_command.value:<12} "
            f"reason={command.abort_reason.value}"
        )


if __name__ == "__main__":
    main()
