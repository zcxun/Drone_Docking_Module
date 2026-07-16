"""Map vision observations into the docking state-machine input contract."""

from __future__ import annotations

import math
from typing import Optional

from software.companion.docking_state_machine import SensorSnapshot
from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus


def observation_to_sensor_snapshot(
    observation: VisionObservation,
    *,
    auto_enabled: bool = False,
    allow_uncalibrated_estimate: bool = False,
    contact: bool = False,
    seated: bool = False,
    lock_switch_closed: bool = False,
    hall_locked: bool = False,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    battery_voltage_v: float = 16.0,
    current_a: float = 0.0,
    pilot_override: bool = False,
) -> SensorSnapshot:
    """Convert one vision observation into a conservative SensorSnapshot.

    Uncalibrated observations are useful for logs and direction checks, but the
    default mapping does not mark them as visible/valid for autonomous docking.
    Set allow_uncalibrated_estimate=True only for tabletop dry-runs.
    """

    pose_valid = _pose_can_drive_state_machine(observation, allow_uncalibrated_estimate)
    range_valid = pose_valid and _finite(observation.range_m) and (
        observation.range_valid or allow_uncalibrated_estimate
    )

    return SensorSnapshot(
        time_s=observation.timestamp_s,
        auto_enabled=auto_enabled,
        target_visible=observation.target_visible and pose_valid,
        target_offset_x_m=observation.target_offset_x_m if pose_valid else None,
        target_offset_y_m=observation.target_offset_y_m if pose_valid else None,
        yaw_error_deg=observation.yaw_error_deg if pose_valid else None,
        range_m=observation.range_m if range_valid else None,
        range_valid=range_valid,
        contact=contact,
        seated=seated,
        lock_switch_closed=lock_switch_closed,
        hall_locked=hall_locked,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        battery_voltage_v=battery_voltage_v,
        current_a=current_a,
        pilot_override=pilot_override,
    )


def _pose_can_drive_state_machine(observation: VisionObservation, allow_uncalibrated_estimate: bool) -> bool:
    if not observation.target_visible:
        return False
    if observation.pose_valid and observation.range_valid:
        return _all_finite(
            observation.target_offset_x_m,
            observation.target_offset_y_m,
            observation.yaw_error_deg,
            observation.range_m,
        )
    if not allow_uncalibrated_estimate:
        return False
    if observation.status != VisionStatus.UNCALIBRATED_ESTIMATE:
        return False
    return _all_finite(
        observation.target_offset_x_m,
        observation.target_offset_y_m,
        observation.yaw_error_deg,
        observation.range_m,
    )


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def _all_finite(*values: Optional[float]) -> bool:
    return all(_finite(value) for value in values)
