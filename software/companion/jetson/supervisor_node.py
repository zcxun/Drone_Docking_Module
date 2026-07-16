"""Log-only Jetson docking supervisor.

The supervisor combines calibrated AprilTag pose, ToF/LiDAR range gating,
docking sensors, and the existing state machine. It returns advisory commands
only; it does not send MAVLink or arm a vehicle.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import TextIO

from software.companion.docking_state_machine import (
    ControlCommand,
    DockingConfig,
    DockingStateMachine,
    SensorSnapshot,
)
from software.companion.jetson.filtering import ObservationFilter, ObservationFilterConfig
from software.companion.jetson.range_gate import RangeGateConfig, gate_observation_with_range
from software.companion.jetson.sensor_node import JetsonSensorPacket
from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus
from software.companion.vision.vision_to_sensor_snapshot import observation_to_sensor_snapshot


@dataclass(frozen=True)
class JetsonSupervisorConfig:
    allow_uncalibrated_estimate: bool = False
    max_observation_latency_ms: float = 300.0
    range_gate: RangeGateConfig = field(default_factory=RangeGateConfig)
    observation_filter: ObservationFilterConfig = field(default_factory=ObservationFilterConfig)
    docking: DockingConfig = field(default_factory=DockingConfig)


@dataclass(frozen=True)
class JetsonSupervisorOutput:
    observation: VisionObservation
    snapshot: SensorSnapshot
    command: ControlCommand


class JetsonDockingSupervisor:
    """Run one safe advisory update of the Jetson terminal docking pipeline."""

    def __init__(self, config: JetsonSupervisorConfig | None = None) -> None:
        self.config = config or JetsonSupervisorConfig()
        self.machine = DockingStateMachine(self.config.docking)
        self.filter = ObservationFilter(self.config.observation_filter)
        self._last_observation_time_s: float | None = None

    def reset(self, time_s: float = 0.0) -> None:
        self.machine.reset(time_s)
        self.filter.reset()
        self._last_observation_time_s = None

    def update(
        self,
        observation: VisionObservation,
        sensors: JetsonSensorPacket | None = None,
    ) -> JetsonSupervisorOutput:
        sensor_packet = sensors or JetsonSensorPacket()
        gated_observation = self._freshness_gate(observation)
        ranged_observation = gate_observation_with_range(
            gated_observation,
            sensor_packet.rangefinder,
            self.config.range_gate,
        )
        filtered_observation = self.filter.update(ranged_observation)
        docking = sensor_packet.docking
        safety = sensor_packet.safety
        snapshot = observation_to_sensor_snapshot(
            filtered_observation,
            auto_enabled=sensor_packet.auto_enabled,
            allow_uncalibrated_estimate=self.config.allow_uncalibrated_estimate,
            contact=docking.contact,
            seated=docking.seated,
            lock_switch_closed=docking.lock_switch_closed,
            hall_locked=docking.hall_locked,
            roll_deg=safety.roll_deg,
            pitch_deg=safety.pitch_deg,
            battery_voltage_v=safety.battery_voltage_v,
            current_a=safety.current_a,
            pilot_override=safety.pilot_override,
        )
        command = self.machine.update(snapshot)
        return JetsonSupervisorOutput(
            observation=filtered_observation,
            snapshot=snapshot,
            command=command,
        )

    def _freshness_gate(self, observation: VisionObservation) -> VisionObservation:
        stale = False
        if observation.latency_ms is not None:
            stale = observation.latency_ms > self.config.max_observation_latency_ms
        if self._last_observation_time_s is not None:
            stale = stale or observation.timestamp_s < self._last_observation_time_s
        self._last_observation_time_s = max(
            observation.timestamp_s,
            self._last_observation_time_s if self._last_observation_time_s is not None else observation.timestamp_s,
        )
        if not stale:
            return observation
        return replace(
            observation,
            target_visible=False,
            pose_valid=False,
            range_valid=False,
            status=VisionStatus.STALE_FRAME,
        )


JETSON_LOG_FIELDS = (
    "timestamp_s",
    "vision_status",
    "target_visible",
    "target_offset_x_m",
    "target_offset_y_m",
    "yaw_error_deg",
    "range_m",
    "range_valid",
    "auto_enabled",
    "contact",
    "seated",
    "lock_switch_closed",
    "hall_locked",
    "state",
    "mode",
    "vx_mps",
    "vy_mps",
    "yaw_rate_dps",
    "vz_mps",
    "lock_command",
    "abort_reason",
    "message",
)


class JetsonCsvLogger:
    """CSV writer for log-only/advisory Jetson test runs."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._handle: TextIO | None = None
        self._writer: csv.DictWriter[str] | None = None

    def __enter__(self) -> "JetsonCsvLogger":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=JETSON_LOG_FIELDS)
        self._writer.writeheader()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self._handle is not None:
            self._handle.close()

    def write(self, output: JetsonSupervisorOutput) -> None:
        if self._writer is None:
            raise RuntimeError("JetsonCsvLogger must be used as a context manager")
        self._writer.writerow(supervisor_output_to_row(output))


def supervisor_output_to_row(output: JetsonSupervisorOutput) -> dict[str, object]:
    observation = output.observation
    snapshot = output.snapshot
    command = output.command
    vx_mps, vy_mps = command.horizontal_velocity_mps
    return {
        "timestamp_s": snapshot.time_s,
        "vision_status": observation.status,
        "target_visible": snapshot.target_visible,
        "target_offset_x_m": snapshot.target_offset_x_m,
        "target_offset_y_m": snapshot.target_offset_y_m,
        "yaw_error_deg": snapshot.yaw_error_deg,
        "range_m": snapshot.range_m,
        "range_valid": snapshot.range_valid,
        "auto_enabled": snapshot.auto_enabled,
        "contact": snapshot.contact,
        "seated": snapshot.seated,
        "lock_switch_closed": snapshot.lock_switch_closed,
        "hall_locked": snapshot.hall_locked,
        "state": command.state.value,
        "mode": command.mode,
        "vx_mps": vx_mps,
        "vy_mps": vy_mps,
        "yaw_rate_dps": command.yaw_rate_dps,
        "vz_mps": command.vertical_velocity_mps,
        "lock_command": command.lock_command.value,
        "abort_reason": command.abort_reason.value,
        "message": command.message,
    }

