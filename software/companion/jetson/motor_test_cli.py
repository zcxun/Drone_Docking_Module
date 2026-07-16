"""Guarded bench-only motor test CLI for Jetson and Pixhawk.

This tool only sends MAV_CMD_DO_MOTOR_TEST. It does not arm, force-arm, switch
flight modes, or provide free throttle control.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional, Sequence

from software.companion.jetson.mavlink_link import (
    DEFAULT_MAVLINK_BAUD,
    DEFAULT_MAVLINK_DEVICE,
    MAV_CMD_DO_MOTOR_TEST,
    MOTOR_TEST_ORDER_DEFAULT,
    MOTOR_TEST_THROTTLE_PERCENT,
    CommandAck,
    MavlinkError,
    MavlinkLink,
    _constant,
)

CONFIRMATION_TEXT = "PROPS_REMOVED"
DEFAULT_THROTTLE_PERCENT = 10.0
DEFAULT_DURATION_S = 2.0
MIN_THROTTLE_PERCENT = 5.0
MAX_THROTTLE_PERCENT = 15.0
MIN_DURATION_S = 1.0
MAX_DURATION_S = 3.0
DEFAULT_MAX_MOTOR = 4


@dataclass(frozen=True)
class MotorTestRequest:
    motor: int
    throttle_percent: float = DEFAULT_THROTTLE_PERCENT
    duration_s: float = DEFAULT_DURATION_S
    confirmation: str = ""
    max_motor: int = DEFAULT_MAX_MOTOR
    dry_run: bool = False


def validate_motor_test_request(request: MotorTestRequest) -> None:
    if request.motor < 1 or request.motor > request.max_motor:
        raise ValueError(f"motor must be between 1 and {request.max_motor}")
    if request.throttle_percent < MIN_THROTTLE_PERCENT or request.throttle_percent > MAX_THROTTLE_PERCENT:
        raise ValueError(
            f"throttle_percent must be between {MIN_THROTTLE_PERCENT:g} and {MAX_THROTTLE_PERCENT:g}"
        )
    if request.duration_s < MIN_DURATION_S or request.duration_s > MAX_DURATION_S:
        raise ValueError(f"duration_s must be between {MIN_DURATION_S:g} and {MAX_DURATION_S:g}")
    if not request.dry_run and request.confirmation != CONFIRMATION_TEXT:
        raise ValueError(f"confirmation must be exactly {CONFIRMATION_TEXT!r}")


def build_motor_test_params(link: MavlinkLink, request: MotorTestRequest) -> list[float]:
    throttle_type = _constant(link.mavlink, "MOTOR_TEST_THROTTLE_PERCENT", MOTOR_TEST_THROTTLE_PERCENT)
    test_order = _constant(link.mavlink, "MOTOR_TEST_ORDER_DEFAULT", MOTOR_TEST_ORDER_DEFAULT)
    return [
        float(request.motor),
        float(throttle_type),
        float(request.throttle_percent),
        float(request.duration_s),
        1.0,
        float(test_order),
        0.0,
    ]


def run_motor_test(link: MavlinkLink, request: MotorTestRequest, *, ack_timeout_s: float = 5.0) -> CommandAck | None:
    validate_motor_test_request(request)
    params = build_motor_test_params(link, request)
    if request.dry_run:
        return None

    heartbeat = link.wait_vehicle_heartbeat(timeout_s=5.0)
    if heartbeat.armed:
        raise MavlinkError("refusing motor test: vehicle is already armed")

    command = _constant(link.mavlink, "MAV_CMD_DO_MOTOR_TEST", MAV_CMD_DO_MOTOR_TEST)
    link.send_command_long(command, params)
    ack = link.wait_command_ack(command, timeout_s=ack_timeout_s)
    if not ack.accepted:
        raise MavlinkError(f"motor test rejected: {ack.result_name} ({ack.result})")
    return ack


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Bench-only Pixhawk motor test from Jetson. Remove props first.")
    parser.add_argument("--device", default=DEFAULT_MAVLINK_DEVICE, help="MAVLink serial device, e.g. /dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=DEFAULT_MAVLINK_BAUD, help="MAVLink serial baud rate")
    parser.add_argument("--motor", type=int, required=True, help="Motor number to test, default F450 range is 1-4")
    parser.add_argument("--max-motor", type=int, default=DEFAULT_MAX_MOTOR, help="Maximum allowed motor number")
    parser.add_argument("--throttle-percent", type=float, default=DEFAULT_THROTTLE_PERCENT, help="Allowed range: 5-15")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S, help="Allowed range: 1-3")
    parser.add_argument("--confirm", default="", help=f"Required exact text: {CONFIRMATION_TEXT}")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print command without connecting to Pixhawk")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    request = MotorTestRequest(
        motor=args.motor,
        throttle_percent=args.throttle_percent,
        duration_s=args.duration_s,
        confirmation=args.confirm,
        max_motor=args.max_motor,
        dry_run=args.dry_run,
    )

    try:
        validate_motor_test_request(request)
    except ValueError as exc:
        parser.error(str(exc))

    if args.dry_run:
        print(
            "dry-run motor test: "
            f"motor={request.motor} throttle={request.throttle_percent:.1f}% duration={request.duration_s:.1f}s"
        )
        return 0

    link = MavlinkLink.connect(device=args.device, baud=args.baud)
    try:
        ack = run_motor_test(link, request)
    finally:
        link.close()

    print(
        "motor test accepted: "
        f"motor={request.motor} throttle={request.throttle_percent:.1f}% "
        f"duration={request.duration_s:.1f}s ack={ack.result_name if ack else 'DRY_RUN'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
