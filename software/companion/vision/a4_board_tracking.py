"""Phone-camera calibration and A4 four-AprilTag board tracking."""

from __future__ import annotations

import base64
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

from software.companion.vision.a4_apriltag_board import (
    CHECKERBOARD_INNER_CORNERS,
    CHECKERBOARD_SQUARE_SIZE_M,
    DEFAULT_TAG_SIZE_M,
    board_object_points_for_tags,
    default_tag_specs,
    tag_center_distance_m,
)
from software.companion.vision.apriltag_phone_pose import CameraCalibration


DEFAULT_CALIBRATION_PATH = Path("software/companion/vision/calibration/iphone13_rear_checkerboard.json")
MIN_CALIBRATION_SAMPLES = 15
GOOD_REPROJECTION_ERROR_PX = 1.0


@dataclass
class CheckerboardSample:
    corners: np.ndarray
    image_size: tuple[int, int]
    timestamp_s: float


class CalibrationSession:
    def __init__(
        self,
        *,
        pattern_size: tuple[int, int] = CHECKERBOARD_INNER_CORNERS,
        square_size_m: float = CHECKERBOARD_SQUARE_SIZE_M,
        output_path: str | Path = DEFAULT_CALIBRATION_PATH,
    ) -> None:
        self.pattern_size = pattern_size
        self.square_size_m = square_size_m
        self.output_path = Path(output_path)
        self.samples: list[CheckerboardSample] = []

    @property
    def sample_count(self) -> int:
        return len(self.samples)

    def add_frame(self, frame: np.ndarray) -> dict[str, Any]:
        image_size = (int(frame.shape[1]), int(frame.shape[0]))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        found, corners = cv2.findChessboardCorners(
            gray,
            self.pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            return {
                "accepted": False,
                "sample_count": self.sample_count,
                "status": "CHECKERBOARD_NOT_FOUND",
                "image_size": image_size,
            }

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        self.samples.append(CheckerboardSample(corners=refined, image_size=image_size, timestamp_s=time.time()))
        return {
            "accepted": True,
            "sample_count": self.sample_count,
            "status": "SAMPLE_ACCEPTED",
            "image_size": image_size,
            "corners_px": _points_to_list(refined.reshape(-1, 2)),
        }

    def solve(self, *, min_samples: int = MIN_CALIBRATION_SAMPLES) -> dict[str, Any]:
        if len(self.samples) < min_samples:
            return {
                "ok": False,
                "status": "NOT_ENOUGH_SAMPLES",
                "sample_count": self.sample_count,
                "min_samples": min_samples,
            }

        image_size = self.samples[0].image_size
        if any(sample.image_size != image_size for sample in self.samples):
            return {"ok": False, "status": "IMAGE_SIZE_CHANGED", "sample_count": self.sample_count}

        object_points = [_checkerboard_object_points(self.pattern_size, self.square_size_m) for _ in self.samples]
        image_points = [sample.corners for sample in self.samples]
        rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )

        payload = {
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
            "reprojection_error_px": float(rms),
            "quality": "OK" if rms <= GOOD_REPROJECTION_ERROR_PX else "HIGH_ERROR",
            "image_width_px": image_size[0],
            "image_height_px": image_size[1],
            "sample_count": self.sample_count,
            "checkerboard": {
                "inner_corners": list(self.pattern_size),
                "square_size_m": self.square_size_m,
            },
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return {"ok": True, "status": "CALIBRATION_WRITTEN", "path": str(self.output_path), **payload}

    def reset(self) -> dict[str, Any]:
        self.samples.clear()
        return {"ok": True, "status": "RESET", "sample_count": 0}


class A4BoardTracker:
    def __init__(
        self,
        *,
        calibration: CameraCalibration | None = None,
        tag_size_m: float = DEFAULT_TAG_SIZE_M,
    ) -> None:
        self.calibration = calibration
        self.tag_size_m = tag_size_m
        self._dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, cv2.aruco.DetectorParameters())

    @property
    def calibrated(self) -> bool:
        return self.calibration is not None

    def set_calibration(self, calibration: CameraCalibration | None) -> None:
        self.calibration = calibration

    def track(self, frame: np.ndarray) -> dict[str, Any]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        corners, ids, _rejected = self._detector.detectMarkers(gray)
        image_size = {"width_px": int(frame.shape[1]), "height_px": int(frame.shape[0])}
        if ids is None or len(ids) == 0:
            return {
                "status": "SEARCHING",
                "board_visible": False,
                "height_valid": False,
                "calibrated": self.calibrated,
                "image_size": image_size,
                "visible_tags": [],
            }

        wanted = {tag.tag_id: tag for tag in default_tag_specs(tag_size_m=self.tag_size_m)}
        visible_tags = []
        image_points = []
        object_tag_ids = []
        for index, marker_id in enumerate(ids.reshape(-1).tolist()):
            marker_id = int(marker_id)
            if marker_id not in wanted:
                continue
            marker_corners = corners[index][0].astype("float32")
            center = marker_corners.mean(axis=0)
            visible_tags.append(
                {
                    "id": marker_id,
                    "center_px": [float(center[0]), float(center[1])],
                    "corners_px": _points_to_list(marker_corners),
                    "distance_to_center_m": tag_center_distance_m(wanted[marker_id]),
                }
            )
            image_points.extend(marker_corners)
            object_tag_ids.append(marker_id)

        visible_tags.sort(key=lambda item: item["id"])
        visible_ids = [tag["id"] for tag in visible_tags]
        payload: dict[str, Any] = {
            "status": "DEGRADED" if visible_tags else "SEARCHING",
            "board_visible": len(visible_tags) >= 2,
            "height_valid": False,
            "calibrated": self.calibrated,
            "image_size": image_size,
            "visible_tag_ids": visible_ids,
            "visible_tags": visible_tags,
            "board_center_px": _estimated_center_from_visible_tags(visible_tags),
        }

        if len(visible_tags) < 4:
            return payload
        if self.calibration is None:
            payload["status"] = "CALIBRATION_REQUIRED"
            return payload

        object_points = board_object_points_for_tags(object_tag_ids, tag_size_m=self.tag_size_m)
        image_points_array = np.array(image_points, dtype="float32")
        camera_matrix = np.array(self.calibration.camera_matrix, dtype="float32")
        dist_coeffs = np.array(self.calibration.dist_coeffs, dtype="float32")
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points_array, camera_matrix, dist_coeffs)
        if not ok:
            payload["status"] = "POSE_INVALID"
            return payload

        center_px, _jacobian = cv2.projectPoints(
            np.array([[0.0, 0.0, 0.0]], dtype="float32"),
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )
        rotation, _ = cv2.Rodrigues(rvec)
        board_normal_camera = rotation[:, 2]
        tvec_flat = tvec.reshape(3)
        height_m = abs(float(np.dot(board_normal_camera, tvec_flat)))
        board_center_px = center_px.reshape(-1, 2)[0]
        frame_center_x = frame.shape[1] / 2.0
        frame_center_y = frame.shape[0] / 2.0

        payload.update(
            {
                "status": "OK",
                "board_visible": True,
                "pose_valid": True,
                "height_valid": True,
                "height_m": height_m,
                "camera_to_board_center_m": float(np.linalg.norm(tvec_flat)),
                "board_center_px": [float(board_center_px[0]), float(board_center_px[1])],
                "offset_px": {
                    "x": float(board_center_px[0] - frame_center_x),
                    "y": float(board_center_px[1] - frame_center_y),
                },
                "correction_hint": _correction_hint(board_center_px, (frame_center_x, frame_center_y)),
            }
        )
        return payload


def decode_image_bytes(data: bytes) -> np.ndarray:
    image_array = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("could not decode image bytes")
    return frame


def decode_data_url(data_url: str) -> np.ndarray:
    _header, encoded = data_url.split(",", 1)
    return decode_image_bytes(base64.b64decode(encoded))


def load_calibration_if_present(path: str | Path = DEFAULT_CALIBRATION_PATH) -> CameraCalibration | None:
    calibration_path = Path(path)
    if not calibration_path.exists():
        return None
    return CameraCalibration.from_json(calibration_path)


def _checkerboard_object_points(pattern_size: tuple[int, int], square_size_m: float) -> np.ndarray:
    points = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    grid = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
    points[:, :2] = grid * square_size_m
    return points


def _estimated_center_from_visible_tags(visible_tags: list[dict[str, Any]]) -> Optional[list[float]]:
    if len(visible_tags) < 4:
        return None
    xs = [tag["center_px"][0] for tag in visible_tags]
    ys = [tag["center_px"][1] for tag in visible_tags]
    return [float(sum(xs) / len(xs)), float(sum(ys) / len(ys))]


def _correction_hint(board_center_px: np.ndarray, frame_center: tuple[float, float]) -> dict[str, str]:
    dx = float(board_center_px[0] - frame_center[0])
    dy = float(board_center_px[1] - frame_center[1])
    horizontal = "hold"
    vertical = "hold"
    if abs(dx) > 20:
        horizontal = "move right" if dx > 0 else "move left"
    if abs(dy) > 20:
        vertical = "move down" if dy > 0 else "move up"
    return {"x": horizontal, "y": vertical}


def _points_to_list(points: np.ndarray) -> list[list[float]]:
    return [[float(point[0]), float(point[1])] for point in points]
