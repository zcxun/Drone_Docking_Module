"""Guarded PX4 actuator test CLI for Jetson and Pixhawk.

PX4 may reject MAV_CMD_DO_MOTOR_TEST with MAV_RESULT_UNSUPPORTED. This tool
uses MAV_CMD_ACTUATOR_TEST, which targets actuator output functions such as
Motor1, Motor2, Motor3, and Motor4.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional, Sequence

from software.companion.jetson.mavlink_link import (
    ACTUATOR_OUTPUT_FUNCTION_MOTOR1,
    DEFAULT_MAVLINK_BAUD,
    DEFAULT_MAVLINK_DEVICE,
    MAV_CMD_ACTUATOR_TEST,
    CommandAck,
    MavlinkError,
    MavlinkLink,
    _constant,
)

CONFIRMATION_TEXT = "PROPS_REMOVED"
DEFAULT_VALUE = 0.10
DEFAULT_TIMEOUT_S = 2.0
MIN_VALUE = 0.05
MAX_VALUE = 0.15
MIN_TIMEOUT_S = 1.0
MAX_TIMEOUT_S = 3.0
DEFAULT_MAX_MOTOR = 4


@dataclass(frozen=True)
class Px4ActuatorTestRequest:
    motor: int
    value: float = DEFAULT_VALUE
    timeout_s: float = DEFAULT_TIMEOUT_S
    confirmation: str = ""
    max_motor: int = DEFAULT_MAX_MOTOR
    dry_run: bool = False


def validate_px4_actuator_test_request(request: Px4ActuatorTestRequest) -> None:
    if request.motor < 1 or request.motor > request.max_motor:
        raise ValueError(f"motor must be between 1 and {request.max_motor}")
    if request.value < MIN_VALUE or request.value > MAX_VALUE:
        raise ValueError(f"value must be between {MIN_VALUE:g} and {MAX_VALUE:g}")
    if request.timeout_s < MIN_TIMEOUT_S or request.timeout_s > MAX_TIMEOUT_S:
        raise ValueError(f"timeout_s must be between {MIN_TIMEOUT_S:g} and {MAX_TIMEOUT_S:g}")
    if not request.dry_run and request.confirmation != CONFIRMATION_TEXT:
        raise ValueError(f"confirmation must be exactly {CONFIRMATION_TEXT!r}")


def actuator_function_for_motor(link: MavlinkLink, motor: int) -> int:
    fallback = ACTUATOR_OUTPUT_FUNCTION_MOTOR1 + (motor - 1)
    return _constant(link.mavlink, f"ACTUATOR_OUTPUT_FUNCTION_MOTOR{motor}", fallback)


def build_px4_actuator_test_params(link: MavlinkLink, request: Px4ActuatorTestRequest) -> list[float]:
    return [
        float(request.value),
        float(request.timeout_s),
        0.0,
        0.0,
        float(actuator_function_for_motor(link, request.motor)),
        0.0,
        0.0,
    ]


def run_px4_actuator_test(
    link: MavlinkLink,
    request: Px4ActuatorTestRequest,
    *,
    ack_timeout_s: float = 5.0,
) -> CommandAck | None:
    validate_px4_actuator_test_request(request)
    params = build_px4_actuator_test_params(link, request)
    if request.dry_run:
        return None

    heartbeat = link.wait_heartbeat(timeout_s=5.0)
    if not heartbeat.is_copter:
        raise MavlinkError(f"refusing actuator test: vehicle type {heartbeat.vehicle_type} is not rotorcraft")
    if heartbeat.armed:
        raise MavlinkError("refusing actuator test: vehicle is already armed")

    command = _constant(link.mavlink, "MAV_CMD_ACTUATOR_TEST", MAV_CMD_ACTUATOR_TEST)
    link.send_command_long(command, params)
    ack = link.wait_command_ack(command, timeout_s=ack_timeout_s)
    if not ack.accepted:
        raise MavlinkError(f"actuator test rejected: {ack.result_name} ({ack.result})")
    return ack


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bench-only PX4 actuator test from Jetson. Remove props first.")
    parser.add_argument("--device", default=DEFAULT_MAVLINK_DEVICE, help="MAVLink serial device, e.g. /dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=DEFAULT_MAVLINK_BAUD, help="MAVLink serial baud rate")
    parser.add_argument("--motor", type=int, required=True, help="Motor function to test, default F450 range is 1-4")
    parser.add_argument("--max-motor", type=int, default=DEFAULT_MAX_MOTOR, help="Maximum allowed motor number")
    parser.add_argument("--value", type=float, default=DEFAULT_VALUE, help="Normalized output value, allowed range: 0.05-0.15")
    parser.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S, help="Allowed range: 1-3")
    parser.add_argument("--confirm", default="", help=f"Required exact text: {CONFIRMATION_TEXT}")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print command without connecting to Pixhawk")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    request = Px4ActuatorTestRequest(
        motor=args.motor,
        value=args.value,
        timeout_s=args.timeout_s,
        confirmation=args.confirm,
        max_motor=args.max_motor,
        dry_run=args.dry_run,
    )

    try:
        validate_px4_actuator_test_request(request)
    except ValueError as exc:
        parser.error(str(exc))

    if args.dry_run:
        print(
            "dry-run px4 actuator test: "
            f"motor={request.motor} value={request.value:.2f} timeout={request.timeout_s:.1f}s"
        )
        return 0

    link = MavlinkLink.connect(device=args.device, baud=args.baud)
    try:
        ack = run_px4_actuator_test(link, request)
    finally:
        link.close()

    print(
        "px4 actuator test accepted: "
        f"motor={request.motor} value={request.value:.2f} "
        f"timeout={request.timeout_s:.1f}s ack={ack.result_name if ack else 'DRY_RUN'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

