"""Jetson-facing wrapper around the AprilTag pose estimator."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

from software.companion.vision.apriltag_phone_pose import (
    DEFAULT_CAMERA_INDEX,
    DEFAULT_TAG_ID,
    DEFAULT_TAG_SIZE_M,
    AprilTagPhonePoseEstimator,
    CameraCalibration,
    VisionObservation,
)


@dataclass(frozen=True)
class JetsonVisionConfig:
    camera_index: int = DEFAULT_CAMERA_INDEX
    target_tag_id: int = DEFAULT_TAG_ID
    tag_size_m: float = DEFAULT_TAG_SIZE_M
    calibration_json: str | None = None
    max_stale_s: float = 0.30


class JetsonVisionNode:
    """Small adapter so runtime code does not depend on experiment naming."""

    def __init__(self, config: JetsonVisionConfig | None = None) -> None:
        self.config = config or JetsonVisionConfig()
        calibration = (
            CameraCalibration.from_json(Path(self.config.calibration_json))
            if self.config.calibration_json
            else None
        )
        self.estimator = AprilTagPhonePoseEstimator(
            target_tag_id=self.config.target_tag_id,
            tag_size_m=self.config.tag_size_m,
            calibration=calibration,
            max_stale_s=self.config.max_stale_s,
        )

    def estimate(
        self,
        frame: Any,
        *,
        timestamp_s: Optional[float] = None,
        frame_index: Optional[int] = None,
        capture_timestamp_s: Optional[float] = None,
    ) -> VisionObservation:
        return self.estimator.estimate(
            frame,
            timestamp_s=timestamp_s,
            frame_index=frame_index,
            capture_timestamp_s=capture_timestamp_s,
        )

