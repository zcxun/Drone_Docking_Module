"""Safety-first docking state machine for the F450 tabletop demo.

The module is intentionally dependency-free. It models high-level decisions only;
it does not send live MAVLink commands to a vehicle.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import hypot, isfinite
from typing import Optional, Tuple


class DockingState(str, Enum):
    MANUAL_APPROACH = "MANUAL_APPROACH"
    TARGET_SEARCH = "TARGET_SEARCH"
    HORIZONTAL_ALIGN = "HORIZONTAL_ALIGN"
    YAW_ALIGN = "YAW_ALIGN"
    DESCEND = "DESCEND"
    CONTACT_DETECTED = "CONTACT_DETECTED"
    MECHANICAL_GUIDE = "MECHANICAL_GUIDE"
    LOCKING = "LOCKING"
    LOCK_VERIFY = "LOCK_VERIFY"
    LIFT_TEST = "LIFT_TEST"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class AbortReason(str, Enum):
    NONE = "NONE"
    PILOT_OVERRIDE = "PILOT_OVERRIDE"
    UNSAFE_ATTITUDE = "UNSAFE_ATTITUDE"
    LOW_BATTERY = "LOW_BATTERY"
    OVER_CURRENT = "OVER_CURRENT"
    SEARCH_TIMEOUT = "SEARCH_TIMEOUT"
    TARGET_LOST = "TARGET_LOST"
    RANGE_INVALID = "RANGE_INVALID"
    DESCENT_TIMEOUT = "DESCENT_TIMEOUT"
    CONTACT_LOST = "CONTACT_LOST"
    SEATED_LOST = "SEATED_LOST"
    CONTACT_WITHOUT_ALIGNMENT = "CONTACT_WITHOUT_ALIGNMENT"
    GUIDE_TIMEOUT = "GUIDE_TIMEOUT"
    LOCK_TIMEOUT = "LOCK_TIMEOUT"
    LOCK_SENSOR_DISAGREE = "LOCK_SENSOR_DISAGREE"


class LockCommand(str, Enum):
    IDLE = "IDLE"
    OPEN_LOCK = "OPEN_LOCK"
    CLOSE_LOCK = "CLOSE_LOCK"
    HOLD_LOCK = "HOLD_LOCK"
    RELEASE_LOCK = "RELEASE_LOCK"


@dataclass(frozen=True)
class DockingConfig:
    search_timeout_s: float = 8.0
    target_loss_timeout_s: float = 1.0
    max_horizontal_error_m: float = 0.03
    max_contact_horizontal_error_m: float = 0.08
    max_yaw_error_deg: float = 5.0
    max_contact_yaw_error_deg: float = 10.0
    max_attitude_deg: float = 10.0
    min_battery_voltage_v: float = 14.0
    max_current_a: float = 45.0
    max_descent_s: float = 10.0
    guide_timeout_s: float = 5.0
    lock_actuation_s: float = 0.8
    lock_verify_timeout_s: float = 3.0
    lift_test_s: float = 1.5
    max_horizontal_speed_mps: float = 0.20
    max_yaw_rate_dps: float = 15.0
    descent_rate_mps: float = -0.08
    lift_rate_mps: float = 0.05


@dataclass(frozen=True)
class SensorSnapshot:
    time_s: float
    auto_enabled: bool = False
    target_visible: bool = False
    target_offset_x_m: Optional[float] = None
    target_offset_y_m: Optional[float] = None
    yaw_error_deg: Optional[float] = None
    range_m: Optional[float] = None
    range_valid: bool = True
    contact: bool = False
    seated: bool = False
    lock_switch_closed: bool = False
    hall_locked: bool = False
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    battery_voltage_v: float = 16.0
    current_a: float = 0.0
    pilot_override: bool = False


@dataclass(frozen=True)
class ControlCommand:
    state: DockingState
    mode: str
    horizontal_velocity_mps: Tuple[float, float] = (0.0, 0.0)
    yaw_rate_dps: float = 0.0
    vertical_velocity_mps: float = 0.0
    lock_command: LockCommand = LockCommand.IDLE
    abort_reason: AbortReason = AbortReason.NONE
    message: str = ""


class DockingStateMachine:
    """Deterministic state machine for first-stage docking validation."""

    TARGET_REQUIRED_STATES = {
        DockingState.HORIZONTAL_ALIGN,
        DockingState.YAW_ALIGN,
        DockingState.DESCEND,
    }

    def __init__(self, config: DockingConfig | None = None) -> None:
        self.config = config or DockingConfig()
        self.state = DockingState.MANUAL_APPROACH
        self.abort_reason = AbortReason.NONE
        self._state_entered_s = 0.0
        self._last_target_seen_s: Optional[float] = None

    def reset(self, time_s: float = 0.0) -> None:
        self.state = DockingState.MANUAL_APPROACH
        self.abort_reason = AbortReason.NONE
        self._state_entered_s = time_s
        self._last_target_seen_s = None

    def update(self, snapshot: SensorSnapshot) -> ControlCommand:
        if snapshot.target_visible:
            self._last_target_seen_s = snapshot.time_s

        if self.state not in {DockingState.MANUAL_APPROACH, DockingState.COMPLETE, DockingState.ABORT}:
            if snapshot.pilot_override:
                self._abort(AbortReason.PILOT_OVERRIDE, snapshot.time_s)
            elif self._attitude_unsafe(snapshot):
                self._abort(AbortReason.UNSAFE_ATTITUDE, snapshot.time_s)
            elif snapshot.battery_voltage_v < self.config.min_battery_voltage_v:
                self._abort(AbortReason.LOW_BATTERY, snapshot.time_s)
            elif snapshot.current_a > self.config.max_current_a:
                self._abort(AbortReason.OVER_CURRENT, snapshot.time_s)
            elif self.state in self.TARGET_REQUIRED_STATES and self._target_lost(snapshot):
                self._abort(AbortReason.TARGET_LOST, snapshot.time_s)

        if self.state == DockingState.ABORT:
            return self._command_for_current_state(snapshot)

        if self.state == DockingState.MANUAL_APPROACH:
            if snapshot.auto_enabled:
                self._enter(DockingState.TARGET_SEARCH, snapshot.time_s)

        elif self.state == DockingState.TARGET_SEARCH:
            if snapshot.target_visible and snapshot.range_valid:
                self._enter(DockingState.HORIZONTAL_ALIGN, snapshot.time_s)
            elif self._elapsed(snapshot) > self.config.search_timeout_s:
                self._abort(AbortReason.SEARCH_TIMEOUT, snapshot.time_s)

        elif self.state == DockingState.HORIZONTAL_ALIGN:
            if not snapshot.range_valid:
                self._abort(AbortReason.RANGE_INVALID, snapshot.time_s)
            elif self._horizontal_error(snapshot) <= self.config.max_horizontal_error_m:
                self._enter(DockingState.YAW_ALIGN, snapshot.time_s)

        elif self.state == DockingState.YAW_ALIGN:
            if self._yaw_error(snapshot) <= self.config.max_yaw_error_deg:
                self._enter(DockingState.DESCEND, snapshot.time_s)

        elif self.state == DockingState.DESCEND:
            if not snapshot.range_valid:
                self._abort(AbortReason.RANGE_INVALID, snapshot.time_s)
            elif self._elapsed(snapshot) > self.config.max_descent_s:
                self._abort(AbortReason.DESCENT_TIMEOUT, snapshot.time_s)
            elif snapshot.contact:
                if self._contact_alignment_safe(snapshot):
                    self._enter(DockingState.CONTACT_DETECTED, snapshot.time_s)
                else:
                    self._abort(AbortReason.CONTACT_WITHOUT_ALIGNMENT, snapshot.time_s)

        elif self.state == DockingState.CONTACT_DETECTED:
            if snapshot.contact:
                self._enter(DockingState.MECHANICAL_GUIDE, snapshot.time_s)
            else:
                self._abort(AbortReason.CONTACT_LOST, snapshot.time_s)

        elif self.state == DockingState.MECHANICAL_GUIDE:
            if not snapshot.contact:
                self._abort(AbortReason.CONTACT_LOST, snapshot.time_s)
            elif snapshot.seated:
                self._enter(DockingState.LOCKING, snapshot.time_s)
            elif self._elapsed(snapshot) > self.config.guide_timeout_s:
                self._abort(AbortReason.GUIDE_TIMEOUT, snapshot.time_s)

        elif self.state == DockingState.LOCKING:
            if not snapshot.contact:
                self._abort(AbortReason.CONTACT_LOST, snapshot.time_s)
            elif not snapshot.seated:
                self._abort(AbortReason.SEATED_LOST, snapshot.time_s)
            elif self._elapsed(snapshot) >= self.config.lock_actuation_s:
                self._enter(DockingState.LOCK_VERIFY, snapshot.time_s)

        elif self.state == DockingState.LOCK_VERIFY:
            if not snapshot.contact:
                self._abort(AbortReason.CONTACT_LOST, snapshot.time_s)
            elif not snapshot.seated:
                self._abort(AbortReason.SEATED_LOST, snapshot.time_s)
            elif self._lock_confirmed(snapshot):
                self._enter(DockingState.LIFT_TEST, snapshot.time_s)
            elif self._elapsed(snapshot) > self.config.lock_verify_timeout_s:
                if snapshot.lock_switch_closed != snapshot.hall_locked:
                    self._abort(AbortReason.LOCK_SENSOR_DISAGREE, snapshot.time_s)
                else:
                    self._abort(AbortReason.LOCK_TIMEOUT, snapshot.time_s)

        elif self.state == DockingState.LIFT_TEST:
            if not snapshot.contact:
                self._abort(AbortReason.CONTACT_LOST, snapshot.time_s)
            elif not snapshot.seated:
                self._abort(AbortReason.SEATED_LOST, snapshot.time_s)
            elif self._elapsed(snapshot) >= self.config.lift_test_s:
                self._enter(DockingState.COMPLETE, snapshot.time_s)

        return self._command_for_current_state(snapshot)

    def _enter(self, state: DockingState, time_s: float) -> None:
        self.state = state
        self._state_entered_s = time_s

    def _abort(self, reason: AbortReason, time_s: float) -> None:
        self.abort_reason = reason
        self._enter(DockingState.ABORT, time_s)

    def _elapsed(self, snapshot: SensorSnapshot) -> float:
        return max(0.0, snapshot.time_s - self._state_entered_s)

    def _target_lost(self, snapshot: SensorSnapshot) -> bool:
        if snapshot.target_visible:
            return False
        if self._last_target_seen_s is None:
            return True
        return snapshot.time_s - self._last_target_seen_s > self.config.target_loss_timeout_s

    def _attitude_unsafe(self, snapshot: SensorSnapshot) -> bool:
        return (
            abs(snapshot.roll_deg) > self.config.max_attitude_deg
            or abs(snapshot.pitch_deg) > self.config.max_attitude_deg
        )

    def _horizontal_error(self, snapshot: SensorSnapshot) -> float:
        if snapshot.target_offset_x_m is None or snapshot.target_offset_y_m is None:
            return float("inf")
        return hypot(snapshot.target_offset_x_m, snapshot.target_offset_y_m)

    def _yaw_error(self, snapshot: SensorSnapshot) -> float:
        if snapshot.yaw_error_deg is None:
            return float("inf")
        return abs(snapshot.yaw_error_deg)

    def _contact_alignment_safe(self, snapshot: SensorSnapshot) -> bool:
        return (
            self._horizontal_error(snapshot) <= self.config.max_contact_horizontal_error_m
            and self._yaw_error(snapshot) <= self.config.max_contact_yaw_error_deg
        )

    def _lock_confirmed(self, snapshot: SensorSnapshot) -> bool:
        return snapshot.lock_switch_closed and snapshot.hall_locked

    def _command_for_current_state(self, snapshot: SensorSnapshot) -> ControlCommand:
        state = self.state
        message = state.value

        if state == DockingState.MANUAL_APPROACH:
            return ControlCommand(state=state, mode="MANUAL", message="waiting for auto enable")

        if state == DockingState.TARGET_SEARCH:
            return ControlCommand(state=state, mode="LOITER", message="searching for docking target")

        if state == DockingState.HORIZONTAL_ALIGN:
            return ControlCommand(
                state=state,
                mode="GUIDED",
                horizontal_velocity_mps=self._horizontal_velocity(snapshot),
                message="centering over target",
            )

        if state == DockingState.YAW_ALIGN:
            return ControlCommand(
                state=state,
                mode="GUIDED",
                yaw_rate_dps=self._yaw_rate(snapshot),
                message="aligning yaw",
            )

        if state == DockingState.DESCEND:
            return ControlCommand(
                state=state,
                mode="GUIDED",
                vertical_velocity_mps=self.config.descent_rate_mps,
                lock_command=LockCommand.OPEN_LOCK,
                message="descending slowly",
            )

        if state == DockingState.CONTACT_DETECTED:
            return ControlCommand(state=state, mode="GUIDED", lock_command=LockCommand.OPEN_LOCK, message="contact detected")

        if state == DockingState.MECHANICAL_GUIDE:
            return ControlCommand(state=state, mode="GUIDED", lock_command=LockCommand.OPEN_LOCK, message="waiting for seated sensor")

        if state == DockingState.LOCKING:
            return ControlCommand(state=state, mode="GUIDED", lock_command=LockCommand.CLOSE_LOCK, message="closing lock")

        if state == DockingState.LOCK_VERIFY:
            return ControlCommand(state=state, mode="GUIDED", lock_command=LockCommand.HOLD_LOCK, message="verifying two lock sensors")

        if state == DockingState.LIFT_TEST:
            return ControlCommand(
                state=state,
                mode="GUIDED",
                vertical_velocity_mps=self.config.lift_rate_mps,
                lock_command=LockCommand.HOLD_LOCK,
                message="low lift test",
            )

        if state == DockingState.COMPLETE:
            return ControlCommand(state=state, mode="LOITER", lock_command=LockCommand.HOLD_LOCK, message="demo complete")

        if state == DockingState.ABORT:
            lock_command = LockCommand.HOLD_LOCK if self._lock_confirmed(snapshot) else LockCommand.RELEASE_LOCK
            return ControlCommand(
                state=state,
                mode="ABORT",
                lock_command=lock_command,
                abort_reason=self.abort_reason,
                message=f"abort: {self.abort_reason.value}",
            )

        return ControlCommand(state=state, mode="LOITER", message=message)

    def _horizontal_velocity(self, snapshot: SensorSnapshot) -> Tuple[float, float]:
        x = snapshot.target_offset_x_m
        y = snapshot.target_offset_y_m
        if x is None or y is None or not isfinite(x) or not isfinite(y):
            return (0.0, 0.0)
        gain = 0.6
        return (
            self._clamp(-gain * x, self.config.max_horizontal_speed_mps),
            self._clamp(-gain * y, self.config.max_horizontal_speed_mps),
        )

    def _yaw_rate(self, snapshot: SensorSnapshot) -> float:
        yaw = snapshot.yaw_error_deg
        if yaw is None or not isfinite(yaw):
            return 0.0
        gain = 0.5
        return self._clamp(-gain * yaw, self.config.max_yaw_rate_dps)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))
