"""Low-latency observation filtering for terminal docking."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from math import isfinite
from statistics import median
from typing import Deque, Optional

from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus


@dataclass(frozen=True)
class ObservationFilterConfig:
    """Median + EMA settings for calibrated terminal pose observations."""

    median_window_size: int = 3
    ema_alpha: float = 0.45
    max_offset_jump_m: float = 0.12
    max_range_jump_m: float = 0.20
    max_yaw_jump_deg: float = 20.0


@dataclass(frozen=True)
class _PoseValues:
    x_m: float
    y_m: float
    yaw_deg: float
    range_m: float


class ObservationFilter:
    """Reject large pose jumps and smooth valid observations.

    Invalid or missing target frames are never filled from history; the filter
    resets instead so the state machine can detect target loss.
    """

    def __init__(self, config: ObservationFilterConfig | None = None) -> None:
        self.config = config or ObservationFilterConfig()
        if self.config.median_window_size <= 0:
            raise ValueError("median_window_size must be positive")
        if not 0.0 < self.config.ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        self._history: Deque[_PoseValues] = deque(maxlen=self.config.median_window_size)
        self._filtered: _PoseValues | None = None

    def reset(self) -> None:
        self._history.clear()
        self._filtered = None

    def update(self, observation: VisionObservation) -> VisionObservation:
        values = _pose_values(observation)
        if values is None:
            self.reset()
            return observation

        if self._filtered is not None and self._jump_too_large(values, self._filtered):
            self.reset()
            return replace(
                observation,
                pose_valid=False,
                range_valid=False,
                status=VisionStatus.FILTER_JUMP,
            )

        self._history.append(values)
        median_values = _PoseValues(
            x_m=median(sample.x_m for sample in self._history),
            y_m=median(sample.y_m for sample in self._history),
            yaw_deg=median(sample.yaw_deg for sample in self._history),
            range_m=median(sample.range_m for sample in self._history),
        )

        if self._filtered is None:
            filtered = median_values
        else:
            alpha = self.config.ema_alpha
            filtered = _PoseValues(
                x_m=_ema(median_values.x_m, self._filtered.x_m, alpha),
                y_m=_ema(median_values.y_m, self._filtered.y_m, alpha),
                yaw_deg=_ema(median_values.yaw_deg, self._filtered.yaw_deg, alpha),
                range_m=_ema(median_values.range_m, self._filtered.range_m, alpha),
            )

        self._filtered = filtered
        return replace(
            observation,
            target_offset_x_m=filtered.x_m,
            target_offset_y_m=filtered.y_m,
            yaw_error_deg=filtered.yaw_deg,
            range_m=filtered.range_m,
        )

    def _jump_too_large(self, values: _PoseValues, previous: _PoseValues) -> bool:
        return (
            abs(values.x_m - previous.x_m) > self.config.max_offset_jump_m
            or abs(values.y_m - previous.y_m) > self.config.max_offset_jump_m
            or abs(values.range_m - previous.range_m) > self.config.max_range_jump_m
            or abs(values.yaw_deg - previous.yaw_deg) > self.config.max_yaw_jump_deg
        )


def _pose_values(observation: VisionObservation) -> _PoseValues | None:
    if not observation.target_visible or not observation.pose_valid or not observation.range_valid:
        return None
    values = (
        observation.target_offset_x_m,
        observation.target_offset_y_m,
        observation.yaw_error_deg,
        observation.range_m,
    )
    if not all(_finite(value) for value in values):
        return None
    return _PoseValues(
        x_m=float(observation.target_offset_x_m),
        y_m=float(observation.target_offset_y_m),
        yaw_deg=float(observation.yaw_error_deg),
        range_m=float(observation.range_m),
    )


def _finite(value: Optional[float]) -> bool:
    return value is not None and isfinite(value)


def _ema(current: float, previous: float, alpha: float) -> float:
    return (alpha * current) + ((1.0 - alpha) * previous)

