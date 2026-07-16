"""Typed sensor inputs for the Jetson docking supervisor.

These dataclasses keep hardware adapters separate from the state machine. A
real Jetson deployment can populate them from ToF/LiDAR, ESP32, and Pixhawk
telemetry without changing the supervisor logic.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Optional


@dataclass(frozen=True)
class RangefinderSample:
    """One ToF/LiDAR range reading in meters."""

    time_s: float
    range_m: Optional[float]
    valid: bool = True
    source: str = "tof"

    def finite_range(self) -> Optional[float]:
        if self.range_m is None or not isfinite(self.range_m):
            return None
        return float(self.range_m)


@dataclass(frozen=True)
class DockingSensorSample:
    """Connector-side docking sensor values."""

    contact: bool = False
    seated: bool = False
    lock_switch_closed: bool = False
    hall_locked: bool = False


@dataclass(frozen=True)
class SafetyTelemetry:
    """Safety values normally sourced from Pixhawk and the pilot gate."""

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    battery_voltage_v: float = 16.0
    current_a: float = 0.0
    pilot_override: bool = False


@dataclass(frozen=True)
class JetsonSensorPacket:
    """All non-vision inputs needed for one supervisor update."""

    auto_enabled: bool = False
    rangefinder: RangefinderSample | None = None
    docking: DockingSensorSample = field(default_factory=DockingSensorSample)
    safety: SafetyTelemetry = field(default_factory=SafetyTelemetry)
