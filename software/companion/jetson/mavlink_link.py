"""Small pymavlink wrapper for Jetson-Pixhawk bench integration."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence


DEFAULT_MAVLINK_DEVICE = "/dev/ttyTHS1"
DEFAULT_MAVLINK_BAUD = 921600
DEFAULT_SOURCE_SYSTEM = 191
DEFAULT_SOURCE_COMPONENT = 191
DEFAULT_TARGET_COMPONENT = 1

MAV_CMD_DO_MOTOR_TEST = 2093
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MOTOR_TEST_THROTTLE_PERCENT = 0
MOTOR_TEST_ORDER_DEFAULT = 0
MAV_RESULT_ACCEPTED = 0
MAV_MODE_FLAG_SAFETY_ARMED = 128

COPTER_MAV_TYPES = {
    2,  # MAV_TYPE_QUADROTOR
    3,  # MAV_TYPE_COAXIAL
    4,  # MAV_TYPE_HELICOPTER
    13,  # MAV_TYPE_HEXAROTOR
    14,  # MAV_TYPE_OCTOROTOR
    15,  # MAV_TYPE_TRICOPTER
    20,  # MAV_TYPE_VTOL_QUADROTOR
}


class MavlinkError(RuntimeError):
    """Base error for guarded MAVLink bench tools."""


class MavlinkTimeoutError(MavlinkError):
    """Raised when an expected MAVLink message does not arrive in time."""


@dataclass(frozen=True)
class HeartbeatStatus:
    target_system: int
    target_component: int
    vehicle_type: int
    autopilot: int
    base_mode: int
    custom_mode: int
    armed: bool
    is_copter: bool


@dataclass(frozen=True)
class CommandAck:
    command: int
    result: int
    result_name: str
    accepted: bool


class MavlinkLink:
    """Thin testable wrapper around a pymavlink connection."""

    def __init__(
        self,
        connection: Any,
        *,
        mavlink_module: Any | None = None,
        target_system: int | None = None,
        target_component: int | None = None,
    ) -> None:
        self.connection = connection
        self.mavlink = mavlink_module if mavlink_module is not None else getattr(connection, "mavlink", None)
        self.target_system = target_system
        self.target_component = target_component
        self.last_heartbeat: HeartbeatStatus | None = None

    @classmethod
    def connect(
        cls,
        *,
        device: str = DEFAULT_MAVLINK_DEVICE,
        baud: int = DEFAULT_MAVLINK_BAUD,
        source_system: int = DEFAULT_SOURCE_SYSTEM,
        source_component: int = DEFAULT_SOURCE_COMPONENT,
    ) -> "MavlinkLink":
        mavutil = _require_mavutil()
        connection = mavutil.mavlink_connection(
            device,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
        )
        return cls(connection, mavlink_module=mavutil.mavlink)

    def close(self) -> None:
        close = getattr(self.connection, "close", None)
        if callable(close):
            close()

    def wait_heartbeat(self, timeout_s: float = 5.0) -> HeartbeatStatus:
        heartbeat = self.connection.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            raise MavlinkTimeoutError(f"no HEARTBEAT within {timeout_s:.1f}s")

        target_system = _message_source_system(heartbeat, getattr(self.connection, "target_system", None))
        target_component = _message_source_component(
            heartbeat,
            getattr(self.connection, "target_component", DEFAULT_TARGET_COMPONENT),
        )
        self.target_system = int(target_system)
        self.target_component = int(target_component)
        status = HeartbeatStatus(
            target_system=self.target_system,
            target_component=self.target_component,
            vehicle_type=int(getattr(heartbeat, "type", -1)),
            autopilot=int(getattr(heartbeat, "autopilot", -1)),
            base_mode=int(getattr(heartbeat, "base_mode", 0)),
            custom_mode=int(getattr(heartbeat, "custom_mode", 0)),
            armed=is_armed(int(getattr(heartbeat, "base_mode", 0)), self.mavlink),
            is_copter=is_copter_vehicle(int(getattr(heartbeat, "type", -1)), self.mavlink),
        )
        self.last_heartbeat = status
        return status

    def recv_match(
        self,
        *,
        message_type: str | Sequence[str] | None = None,
        timeout_s: float = 1.0,
        blocking: bool = True,
    ) -> Any | None:
        return self.connection.recv_match(type=message_type, blocking=blocking, timeout=timeout_s)

    def send_command_long(
        self,
        command: int,
        params: Iterable[float] = (),
        *,
        confirmation: int = 0,
    ) -> None:
        target_system, target_component = self._targets()
        packed_params = _seven_params(params)
        self.connection.mav.command_long_send(
            target_system,
            target_component,
            int(command),
            int(confirmation),
            *packed_params,
        )

    def wait_command_ack(self, command: int, timeout_s: float = 5.0) -> CommandAck:
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            remaining_s = max(0.0, deadline_s - time.monotonic())
            ack = self.recv_match(message_type="COMMAND_ACK", timeout_s=remaining_s)
            if ack is None:
                break
            if int(getattr(ack, "command", -1)) != int(command):
                continue
            result = int(getattr(ack, "result", -1))
            return CommandAck(
                command=int(command),
                result=result,
                result_name=self.result_name(result),
                accepted=result == _constant(self.mavlink, "MAV_RESULT_ACCEPTED", MAV_RESULT_ACCEPTED),
            )
        raise MavlinkTimeoutError(f"no COMMAND_ACK for command {command} within {timeout_s:.1f}s")

    def request_message_interval(self, message_id: int, rate_hz: float) -> CommandAck:
        interval_us = -1.0 if rate_hz <= 0 else 1_000_000.0 / rate_hz
        self.send_command_long(
            _constant(self.mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", MAV_CMD_SET_MESSAGE_INTERVAL),
            [float(message_id), interval_us, 0, 0, 0, 0, 0],
        )
        return self.wait_command_ack(
            _constant(self.mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", MAV_CMD_SET_MESSAGE_INTERVAL),
            timeout_s=3.0,
        )

    def result_name(self, result: int) -> str:
        enum_map = getattr(self.mavlink, "enums", {}).get("MAV_RESULT") if self.mavlink is not None else None
        enum_value = enum_map.get(result) if enum_map is not None else None
        return getattr(enum_value, "name", f"MAV_RESULT_{result}")

    def _targets(self) -> tuple[int, int]:
        if self.target_system is None:
            self.target_system = int(getattr(self.connection, "target_system", 1))
        if self.target_component is None:
            self.target_component = int(getattr(self.connection, "target_component", DEFAULT_TARGET_COMPONENT))
        return self.target_system, self.target_component


def is_armed(base_mode: int, mavlink_module: Any | None = None) -> bool:
    armed_flag = _constant(mavlink_module, "MAV_MODE_FLAG_SAFETY_ARMED", MAV_MODE_FLAG_SAFETY_ARMED)
    return bool(int(base_mode) & int(armed_flag))


def is_copter_vehicle(vehicle_type: int, mavlink_module: Any | None = None) -> bool:
    copter_types = {
        _constant(mavlink_module, "MAV_TYPE_QUADROTOR", 2),
        _constant(mavlink_module, "MAV_TYPE_COAXIAL", 3),
        _constant(mavlink_module, "MAV_TYPE_HELICOPTER", 4),
        _constant(mavlink_module, "MAV_TYPE_HEXAROTOR", 13),
        _constant(mavlink_module, "MAV_TYPE_OCTOROTOR", 14),
        _constant(mavlink_module, "MAV_TYPE_TRICOPTER", 15),
        _constant(mavlink_module, "MAV_TYPE_VTOL_QUADROTOR", 20),
    }
    return int(vehicle_type) in copter_types or int(vehicle_type) in COPTER_MAV_TYPES


def _require_mavutil() -> Any:
    try:
        from pymavlink import mavutil  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pymavlink is required on Jetson. Install with: python3 -m pip install --user pymavlink") from exc
    return mavutil


def _constant(module: Any | None, name: str, fallback: int) -> int:
    return int(getattr(module, name, fallback)) if module is not None else int(fallback)


def _message_source_system(message: Any, fallback: Optional[int]) -> int:
    getter = getattr(message, "get_srcSystem", None)
    if callable(getter):
        return int(getter())
    return int(fallback if fallback is not None else 1)


def _message_source_component(message: Any, fallback: Optional[int]) -> int:
    getter = getattr(message, "get_srcComponent", None)
    if callable(getter):
        return int(getter())
    return int(fallback if fallback is not None else DEFAULT_TARGET_COMPONENT)


def _seven_params(params: Iterable[float]) -> list[float]:
    values = [float(value) for value in params]
    if len(values) > 7:
        raise ValueError("COMMAND_LONG supports at most 7 params")
    return values + [0.0] * (7 - len(values))

