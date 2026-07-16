"""ToF/LiDAR range gating for Jetson terminal docking."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite
from typing import Optional

from software.companion.jetson.sensor_node import RangefinderSample
from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus


@dataclass(frozen=True)
class RangeGateConfig:
    """Conservative rules for trusting descent range."""

    require_rangefinder: bool = True
    max_range_age_s: float = 0.25
    min_range_m: float = 0.05
    max_range_m: float = 1.50
    max_range_disagreement_m: float = 0.08


def gate_observation_with_range(
    observation: VisionObservation,
    rangefinder: RangefinderSample | None,
    config: RangeGateConfig | None = None,
) -> VisionObservation:
    """Require a fresh ToF/LiDAR reading before pose can drive docking.

    AprilTag pose supplies x/y/yaw. ToF/LiDAR is the primary descent range. A
    calibrated AprilTag range is kept as a cross-check, not as the final range
    value used by the state machine.
    """

    gate_config = config or RangeGateConfig()
    if not observation.target_visible:
        return observation

    if rangefinder is None:
        if gate_config.require_rangefinder:
            return _range_invalid(observation, VisionStatus.RANGE_INVALID)
        return observation

    range_m = rangefinder.finite_range()
    if not rangefinder.valid or range_m is None:
        return _range_invalid(observation, VisionStatus.RANGE_INVALID, range_m)

    if range_m < gate_config.min_range_m or range_m > gate_config.max_range_m:
        return _range_invalid(observation, VisionStatus.RANGE_INVALID, range_m)

    if abs(observation.timestamp_s - rangefinder.time_s) > gate_config.max_range_age_s:
        return _range_invalid(observation, VisionStatus.RANGE_STALE, range_m)

    vision_range = observation.range_m
    if observation.range_valid and _finite(vision_range):
        if abs(float(vision_range) - range_m) > gate_config.max_range_disagreement_m:
            return _range_invalid(observation, VisionStatus.RANGE_MISMATCH, range_m)

    if not observation.pose_valid:
        return replace(observation, range_m=range_m, range_valid=False)

    return replace(observation, range_m=range_m, range_valid=True)


def _range_invalid(
    observation: VisionObservation,
    status: str,
    range_m: Optional[float] = None,
) -> VisionObservation:
    return replace(
        observation,
        range_m=range_m if _finite(range_m) else observation.range_m,
        range_valid=False,
        pose_valid=False,
        status=status,
    )


def _finite(value: Optional[float]) -> bool:
    return value is not None and isfinite(value)

