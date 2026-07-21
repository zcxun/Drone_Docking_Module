"""Human-readable guidance for Jetson USB camera docking tests."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import isfinite
from typing import Optional

from software.companion.docking_state_machine import ControlCommand, DockingConfig, SensorSnapshot
from software.companion.vision.apriltag_phone_pose import VisionObservation, VisionStatus


STATUS_TEXT = {
    VisionStatus.TAG_NOT_FOUND: "找不到定位標記",
    VisionStatus.WRONG_TAG_ID: "看到標記，但不是指定 ID",
    VisionStatus.UNCALIBRATED_ESTIMATE: "可用於人工導引，尚不可自動控制",
    VisionStatus.OK: "定位可信",
    VisionStatus.RANGE_INVALID: "高度不可信，禁止下降",
    VisionStatus.RANGE_MISMATCH: "高度資料不一致，禁止下降",
    VisionStatus.RANGE_STALE: "高度資料過舊，請等待",
    VisionStatus.FILTER_JUMP: "定位跳動過大，請等待穩定",
    VisionStatus.STALE_FRAME: "影像延遲，請等待",
    VisionStatus.POSE_INVALID: "姿態估計失敗",
}


@dataclass(frozen=True)
class GuidanceConfig:
    camera_forward: str = "up"
    mirror_x: bool = False
    mirror_y: bool = False
    horizontal_deadband_m: float = 0.01
    yaw_deadband_deg: float = 1.0
    height_target_m: float = 0.50
    height_deadband_m: float = 0.03
    attitude_ok_deg: float = 5.0
    attitude_abort_deg: float = 10.0


@dataclass(frozen=True)
class BodyOffset:
    forward_m: Optional[float]
    right_m: Optional[float]


@dataclass(frozen=True)
class GuidancePayload:
    status_text: str
    primary_instruction: str
    horizontal_instruction: str
    height_instruction: str
    yaw_instruction: str
    attitude_instruction: str
    command_preview: str
    control_allowed: bool
    control_note: str
    body_offset: BodyOffset
    range_source: str
    engineering: dict[str, object]

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["body_offset"] = asdict(self.body_offset)
        return payload


def build_guidance_payload(
    observation: VisionObservation,
    snapshot: SensorSnapshot,
    command: ControlCommand,
    config: GuidanceConfig | None = None,
) -> GuidancePayload:
    guidance_config = config or GuidanceConfig()
    body_offset = camera_offsets_to_body(
        observation.target_offset_x_m,
        observation.target_offset_y_m,
        guidance_config,
    )
    horizontal_instruction = _horizontal_instruction(body_offset, guidance_config.horizontal_deadband_m)
    height_instruction, range_source = _height_instruction(snapshot, observation, guidance_config)
    yaw_instruction = _yaw_instruction(snapshot.yaw_error_deg, guidance_config.yaw_deadband_deg)
    attitude_instruction = _attitude_instruction(snapshot, guidance_config)
    command_preview = _command_preview(command)
    status_text = STATUS_TEXT.get(observation.status, observation.status)
    control_allowed, control_note = _control_status(observation, snapshot, guidance_config)
    primary_instruction = _primary_instruction(
        status_text=status_text,
        horizontal_instruction=horizontal_instruction,
        height_instruction=height_instruction,
        yaw_instruction=yaw_instruction,
        attitude_instruction=attitude_instruction,
        snapshot=snapshot,
        control_allowed=control_allowed,
    )

    return GuidancePayload(
        status_text=status_text,
        primary_instruction=primary_instruction,
        horizontal_instruction=horizontal_instruction,
        height_instruction=height_instruction,
        yaw_instruction=yaw_instruction,
        attitude_instruction=attitude_instruction,
        command_preview=command_preview,
        control_allowed=control_allowed,
        control_note=control_note,
        body_offset=body_offset,
        range_source=range_source,
        engineering={
            "state": command.state.value,
            "mode": command.mode,
            "vx_mps": command.horizontal_velocity_mps[0],
            "vy_mps": command.horizontal_velocity_mps[1],
            "vz_mps": command.vertical_velocity_mps,
            "yaw_rate_dps": command.yaw_rate_dps,
            "abort_reason": command.abort_reason.value,
            "lock_command": command.lock_command.value,
            "roll_deg": snapshot.roll_deg,
            "pitch_deg": snapshot.pitch_deg,
            "range_m": snapshot.range_m,
            "range_valid": snapshot.range_valid,
            "calibrated": observation.calibrated,
            "pose_valid": observation.pose_valid,
        },
    )


def camera_offsets_to_body(
    offset_x_m: Optional[float],
    offset_y_m: Optional[float],
    config: GuidanceConfig,
) -> BodyOffset:
    if not _finite(offset_x_m) or not _finite(offset_y_m):
        return BodyOffset(forward_m=None, right_m=None)

    image_right_m = float(offset_x_m)
    image_down_m = float(offset_y_m)
    if config.mirror_x:
        image_right_m = -image_right_m
    if config.mirror_y:
        image_down_m = -image_down_m

    if config.camera_forward == "up":
        forward_m = -image_down_m
        right_m = image_right_m
    elif config.camera_forward == "down":
        forward_m = image_down_m
        right_m = -image_right_m
    elif config.camera_forward == "left":
        forward_m = -image_right_m
        right_m = -image_down_m
    elif config.camera_forward == "right":
        forward_m = image_right_m
        right_m = image_down_m
    else:
        raise ValueError("camera_forward must be one of: up, down, left, right")
    return BodyOffset(forward_m=forward_m, right_m=right_m)


def _horizontal_instruction(offset: BodyOffset, deadband_m: float) -> str:
    if offset.forward_m is None or offset.right_m is None:
        return "水平位置未知"

    parts = []
    if abs(offset.forward_m) > deadband_m:
        parts.append(_direction_text(offset.forward_m, "往前", "往後"))
    if abs(offset.right_m) > deadband_m:
        parts.append(_direction_text(offset.right_m, "往右", "往左"))
    if not parts:
        return "水平位置已對準"
    return "，".join(parts)


def _height_instruction(
    snapshot: SensorSnapshot,
    observation: VisionObservation,
    config: GuidanceConfig,
) -> tuple[str, str]:
    if not _finite(snapshot.range_m):
        return "高度未知，先不要下降", "none"

    range_m = float(snapshot.range_m)
    source = "視覺估計" if observation.range_valid and not observation.calibrated else "已校正視覺"
    delta_m = range_m - config.height_target_m
    base = f"高度 {_cm(range_m)} cm"
    if abs(delta_m) <= config.height_deadband_m:
        return f"{base}，高度接近目標", source
    if delta_m > 0:
        return f"{base}，建議下降 {_cm(delta_m)} cm", source
    return f"{base}，建議上升 {_cm(-delta_m)} cm", source


def _yaw_instruction(yaw_error_deg: Optional[float], deadband_deg: float) -> str:
    if not _finite(yaw_error_deg):
        return "角度未知"
    yaw = float(yaw_error_deg)
    if abs(yaw) <= deadband_deg:
        return "角度已對準"
    # Direction matches the control action used by DockingStateMachine: yaw_rate = -gain * yaw_error.
    direction = "順時針" if yaw > 0 else "逆時針"
    return f"{direction}轉 {abs(yaw):.1f}°"


def _attitude_instruction(snapshot: SensorSnapshot, config: GuidanceConfig) -> str:
    roll = float(snapshot.roll_deg)
    pitch = float(snapshot.pitch_deg)
    if abs(roll) > config.attitude_abort_deg or abs(pitch) > config.attitude_abort_deg:
        return f"傾斜過大，先扶正（roll {roll:.1f}° / pitch {pitch:.1f}°）"
    if abs(roll) <= config.attitude_ok_deg and abs(pitch) <= config.attitude_ok_deg:
        return f"水平 OK（roll {roll:.1f}° / pitch {pitch:.1f}°）"

    parts = []
    if abs(roll) > config.attitude_ok_deg:
        parts.append(("向右傾斜" if roll > 0 else "向左傾斜") + f" {abs(roll):.1f}°")
    if abs(pitch) > config.attitude_ok_deg:
        parts.append(("向後傾斜" if pitch > 0 else "向前傾斜") + f" {abs(pitch):.1f}°")
    return "，".join(parts)


def _command_preview(command: ControlCommand) -> str:
    vx_mps, vy_mps = command.horizontal_velocity_mps
    parts = []
    if abs(vx_mps) > 0.005:
        parts.append(_velocity_text(vx_mps, "往前慢移", "往後慢移"))
    if abs(vy_mps) > 0.005:
        parts.append(_velocity_text(vy_mps, "往右慢移", "往左慢移"))
    if abs(command.vertical_velocity_mps) > 0.005:
        parts.append(_velocity_text(command.vertical_velocity_mps, "上升", "下降"))
    if abs(command.yaw_rate_dps) > 0.1:
        direction = "逆時針修正" if command.yaw_rate_dps > 0 else "順時針修正"
        parts.append(f"{direction} {abs(command.yaw_rate_dps):.1f} deg/s")
    if not parts:
        return "若進入自動控制，Jetson 目前會要求保持"
    return "若進入自動控制，Jetson 會要求" + "，".join(parts)


def _control_status(
    observation: VisionObservation,
    snapshot: SensorSnapshot,
    config: GuidanceConfig,
) -> tuple[bool, str]:
    if not observation.target_visible:
        return False, "不可自動控制：找不到定位標記"
    if not observation.calibrated:
        return False, "不可自動控制：相機尚未校正，僅供人工導引"
    if not snapshot.range_valid:
        return False, "不可自動控制：高度資料不可信"
    if abs(snapshot.roll_deg) > config.attitude_abort_deg or abs(snapshot.pitch_deg) > config.attitude_abort_deg:
        return False, "不可自動控制：傾斜過大"
    return True, "定位資料可作為後續自動控制候選"


def _primary_instruction(
    *,
    status_text: str,
    horizontal_instruction: str,
    height_instruction: str,
    yaw_instruction: str,
    attitude_instruction: str,
    snapshot: SensorSnapshot,
    control_allowed: bool,
) -> str:
    if "傾斜過大" in attitude_instruction:
        return "傾斜過大，先扶正"
    if not snapshot.target_visible:
        return status_text

    parts = [horizontal_instruction]
    if yaw_instruction != "角度已對準":
        parts.append(yaw_instruction)
    if "建議" in height_instruction:
        parts.append(height_instruction.split("，", 1)[1])
    if not control_allowed:
        parts.append("目前僅供人工導引")
    return "；".join(parts)


def _direction_text(value_m: float, positive_text: str, negative_text: str) -> str:
    direction = positive_text if value_m > 0 else negative_text
    return f"{direction} {_cm(abs(value_m))} cm"


def _velocity_text(value: float, positive_text: str, negative_text: str) -> str:
    direction = positive_text if value > 0 else negative_text
    return f"{direction} {abs(value):.2f} m/s"


def _cm(value_m: float) -> int:
    return int(round(value_m * 100.0))


def _finite(value: Optional[float]) -> bool:
    return value is not None and isfinite(value)

