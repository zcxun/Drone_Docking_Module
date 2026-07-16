"""Read-only MAVLink monitor for Jetson-Pixhawk bench setup."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional, Sequence

from software.companion.jetson.mavlink_link import DEFAULT_MAVLINK_BAUD, DEFAULT_MAVLINK_DEVICE, MavlinkLink


@dataclass
class MonitorState:
    armed: bool = False
    mode: int = 0
    battery_voltage_v: float | None = None
    current_a: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only Pixhawk MAVLink monitor for Jetson.")
    parser.add_argument("--device", default=DEFAULT_MAVLINK_DEVICE, help="MAVLink serial device, e.g. /dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=DEFAULT_MAVLINK_BAUD, help="MAVLink serial baud rate")
    parser.add_argument("--duration-s", type=float, default=60.0, help="How long to monitor")
    parser.add_argument("--heartbeat-timeout-s", type=float, default=5.0, help="Initial heartbeat timeout")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    link = MavlinkLink.connect(device=args.device, baud=args.baud)
    try:
        heartbeat = link.wait_heartbeat(timeout_s=args.heartbeat_timeout_s)
        state = MonitorState(armed=heartbeat.armed, mode=heartbeat.custom_mode)
        print(
            "heartbeat: "
            f"sys={heartbeat.target_system} comp={heartbeat.target_component} "
            f"type={heartbeat.vehicle_type} armed={heartbeat.armed} copter={heartbeat.is_copter}"
        )
        return monitor_loop(link, state, duration_s=args.duration_s)
    finally:
        link.close()


def monitor_loop(link: MavlinkLink, state: MonitorState, *, duration_s: float) -> int:
    started_s = time.monotonic()
    next_print_s = started_s
    while time.monotonic() - started_s < duration_s:
        message = link.recv_match(
            message_type=["HEARTBEAT", "SYS_STATUS", "BATTERY_STATUS", "ATTITUDE"],
            timeout_s=0.5,
        )
        if message is not None:
            _update_state(state, message)
        now_s = time.monotonic()
        if now_s >= next_print_s:
            print(_format_state(state))
            next_print_s = now_s + 1.0
    return 0


def _update_state(state: MonitorState, message) -> None:
    message_type = message.get_type() if hasattr(message, "get_type") else ""
    if message_type == "HEARTBEAT":
        state.armed = bool(int(getattr(message, "base_mode", 0)) & 128)
        state.mode = int(getattr(message, "custom_mode", state.mode))
    elif message_type == "SYS_STATUS":
        voltage_mv = int(getattr(message, "voltage_battery", -1))
        current_ca = int(getattr(message, "current_battery", -1))
        state.battery_voltage_v = voltage_mv / 1000.0 if voltage_mv >= 0 else None
        state.current_a = current_ca / 100.0 if current_ca >= 0 else None
    elif message_type == "BATTERY_STATUS":
        voltages = [int(value) for value in getattr(message, "voltages", []) if 0 < int(value) < 65535]
        current_ca = int(getattr(message, "current_battery", -1))
        state.battery_voltage_v = sum(voltages) / 1000.0 if voltages else state.battery_voltage_v
        state.current_a = current_ca / 100.0 if current_ca >= 0 else state.current_a
    elif message_type == "ATTITUDE":
        state.roll_deg = _rad_to_deg(float(getattr(message, "roll", 0.0)))
        state.pitch_deg = _rad_to_deg(float(getattr(message, "pitch", 0.0)))
        state.yaw_deg = _rad_to_deg(float(getattr(message, "yaw", 0.0)))


def _format_state(state: MonitorState) -> str:
    return (
        f"armed={state.armed} mode={state.mode} "
        f"battery={_fmt(state.battery_voltage_v)}V current={_fmt(state.current_a)}A "
        f"roll={_fmt(state.roll_deg)}deg pitch={_fmt(state.pitch_deg)}deg yaw={_fmt(state.yaw_deg)}deg"
    )


def _rad_to_deg(value: float) -> float:
    return value * 57.29577951308232


def _fmt(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f}"


if __name__ == "__main__":
    raise SystemExit(main())
