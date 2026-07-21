"""AprilTag detection for the iPhone 13 + MacBook camera experiment.

The module is intentionally log-first. It can detect the target tag from a
Continuity Camera stream or a video file, but uncalibrated metric estimates are
marked as unsafe for direct state-machine control.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence


TAG_FAMILY = "tag36h11"
DEFAULT_TAG_ID = 0
DEFAULT_TAG_SIZE_M = 0.100
DEFAULT_CAMERA_INDEX = 0
DEFAULT_HORIZONTAL_FOV_DEG = 70.0


class VisionStatus:
    OK = "OK"
    TAG_NOT_FOUND = "TAG_NOT_FOUND"
    WRONG_TAG_ID = "WRONG_TAG_ID"
    POSE_INVALID = "POSE_INVALID"
    RANGE_INVALID = "RANGE_INVALID"
    RANGE_MISMATCH = "RANGE_MISMATCH"
    RANGE_STALE = "RANGE_STALE"
    FILTER_JUMP = "FILTER_JUMP"
    STALE_FRAME = "STALE_FRAME"
    UNCALIBRATED_ESTIMATE = "UNCALIBRATED_ESTIMATE"


@dataclass(frozen=True)
class CameraCalibration:
    """OpenCV camera calibration parameters."""

    camera_matrix: tuple[tuple[float, float, float], ...]
    dist_coeffs: tuple[float, ...]
    source: str

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraCalibration":
        calibration_path = Path(path)
        with calibration_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)

        camera_matrix = tuple(tuple(float(value) for value in row) for row in data["camera_matrix"])
        if len(camera_matrix) != 3 or any(len(row) != 3 for row in camera_matrix):
            raise ValueError("camera_matrix must be a 3x3 array")

        raw_dist_coeffs = data.get("dist_coeffs", data.get("distortion_coefficients", []))
        dist_coeffs = tuple(float(value) for value in raw_dist_coeffs)
        return cls(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, source=str(calibration_path))


@dataclass(frozen=True)
class VisionObservation:
    """One AprilTag observation in the log/adapter contract."""

    timestamp_s: float
    target_visible: bool
    tag_family: str = TAG_FAMILY
    tag_id: Optional[int] = None
    target_offset_x_m: Optional[float] = None
    target_offset_y_m: Optional[float] = None
    yaw_error_deg: Optional[float] = None
    range_m: Optional[float] = None
    range_valid: bool = False
    pose_valid: bool = False
    confidence: Optional[float] = None
    latency_ms: Optional[float] = None
    status: str = VisionStatus.TAG_NOT_FOUND
    frame_index: Optional[int] = None
    frame_width_px: Optional[int] = None
    frame_height_px: Optional[int] = None
    center_x_px: Optional[float] = None
    center_y_px: Optional[float] = None
    calibrated: bool = False

    @classmethod
    def empty(
        cls,
        timestamp_s: float,
        *,
        latency_ms: Optional[float] = None,
        frame_index: Optional[int] = None,
        frame_width_px: Optional[int] = None,
        frame_height_px: Optional[int] = None,
        status: str = VisionStatus.TAG_NOT_FOUND,
    ) -> "VisionObservation":
        return cls(
            timestamp_s=timestamp_s,
            target_visible=False,
            tag_id=None,
            latency_ms=latency_ms,
            status=status,
            frame_index=frame_index,
            frame_width_px=frame_width_px,
            frame_height_px=frame_height_px,
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


CSV_FIELDS = tuple(VisionObservation.__dataclass_fields__.keys())


class AprilTagPhonePoseEstimator:
    """Detect the configured AprilTag and estimate its relative pose."""

    def __init__(
        self,
        *,
        target_tag_id: int = DEFAULT_TAG_ID,
        tag_size_m: float = DEFAULT_TAG_SIZE_M,
        calibration: CameraCalibration | None = None,
        assumed_horizontal_fov_deg: float = DEFAULT_HORIZONTAL_FOV_DEG,
        max_stale_s: float = 0.30,
    ) -> None:
        if tag_size_m <= 0:
            raise ValueError("tag_size_m must be positive")
        if not 1.0 < assumed_horizontal_fov_deg < 179.0:
            raise ValueError("assumed_horizontal_fov_deg must be between 1 and 179 degrees")

        self.target_tag_id = target_tag_id
        self.tag_size_m = tag_size_m
        self.calibration = calibration
        self.assumed_horizontal_fov_deg = assumed_horizontal_fov_deg
        self.max_stale_s = max_stale_s
        self._cv2 = _require_cv2()
        self._detector = self._create_detector()

    @property
    def calibrated(self) -> bool:
        return self.calibration is not None

    def estimate(
        self,
        frame: Any,
        *,
        timestamp_s: Optional[float] = None,
        frame_index: Optional[int] = None,
        capture_timestamp_s: Optional[float] = None,
    ) -> VisionObservation:
        cv2 = self._cv2
        started_s = time.monotonic()
        timestamp = timestamp_s if timestamp_s is not None else started_s
        frame_height, frame_width = frame.shape[:2]

        if capture_timestamp_s is not None and started_s - capture_timestamp_s > self.max_stale_s:
            return VisionObservation.empty(
                timestamp,
                latency_ms=(started_s - capture_timestamp_s) * 1000.0,
                frame_index=frame_index,
                frame_width_px=frame_width,
                frame_height_px=frame_height,
                status=VisionStatus.STALE_FRAME,
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        corners, ids, _rejected = self._detector.detectMarkers(gray)

        latency_ms = (time.monotonic() - started_s) * 1000.0
        if ids is None or len(ids) == 0:
            return VisionObservation.empty(
                timestamp,
                latency_ms=latency_ms,
                frame_index=frame_index,
                frame_width_px=frame_width,
                frame_height_px=frame_height,
            )

        flat_ids = _flatten_marker_ids(ids)
        if self.target_tag_id not in flat_ids:
            return VisionObservation.empty(
                timestamp,
                latency_ms=latency_ms,
                frame_index=frame_index,
                frame_width_px=frame_width,
                frame_height_px=frame_height,
                status=VisionStatus.WRONG_TAG_ID,
            )

        marker_index = flat_ids.index(self.target_tag_id)
        marker_corners = corners[marker_index][0]

        if self.calibration is not None:
            return self._estimate_calibrated(
                marker_corners,
                timestamp_s=timestamp,
                latency_ms=latency_ms,
                frame_index=frame_index,
                frame_width_px=frame_width,
                frame_height_px=frame_height,
            )

        return self._estimate_uncalibrated(
            marker_corners,
            timestamp_s=timestamp,
            latency_ms=latency_ms,
            frame_index=frame_index,
            frame_width_px=frame_width,
            frame_height_px=frame_height,
        )

    def _create_detector(self) -> Any:
        cv2 = self._cv2
        aruco = cv2.aruco
        if hasattr(aruco, "DICT_APRILTAG_36H11"):
            dictionary_id = aruco.DICT_APRILTAG_36H11
        else:
            dictionary_id = aruco.DICT_APRILTAG_36h11
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
        parameters = aruco.DetectorParameters()
        if hasattr(parameters, "cornerRefinementMethod"):
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
        return aruco.ArucoDetector(dictionary, parameters)

    def _estimate_calibrated(
        self,
        corners: Any,
        *,
        timestamp_s: float,
        latency_ms: float,
        frame_index: Optional[int],
        frame_width_px: int,
        frame_height_px: int,
    ) -> VisionObservation:
        cv2 = self._cv2
        calibration = self.calibration
        if calibration is None:
            raise RuntimeError("calibration is required for calibrated pose estimation")

        object_points = self._tag_object_points()
        camera_matrix = _np_array(calibration.camera_matrix)
        dist_coeffs = _np_array(calibration.dist_coeffs)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            _np_array(corners),
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return self._visible_invalid(
                corners,
                timestamp_s=timestamp_s,
                latency_ms=latency_ms,
                frame_index=frame_index,
                frame_width_px=frame_width_px,
                frame_height_px=frame_height_px,
                status=VisionStatus.POSE_INVALID,
                calibrated=True,
            )

        translation = [float(value) for value in tvec.reshape(3)]
        yaw_error_deg = self._yaw_from_rvec(rvec)
        range_m = translation[2]

        if not all(_finite(value) for value in [translation[0], translation[1], yaw_error_deg, range_m]) or range_m <= 0:
            return self._visible_invalid(
                corners,
                timestamp_s=timestamp_s,
                latency_ms=latency_ms,
                frame_index=frame_index,
                frame_width_px=frame_width_px,
                frame_height_px=frame_height_px,
                status=VisionStatus.RANGE_INVALID,
                calibrated=True,
            )

        center_x, center_y = _corner_center(corners)
        return VisionObservation(
            timestamp_s=timestamp_s,
            target_visible=True,
            tag_id=self.target_tag_id,
            target_offset_x_m=translation[0],
            target_offset_y_m=translation[1],
            yaw_error_deg=yaw_error_deg,
            range_m=range_m,
            range_valid=True,
            pose_valid=True,
            latency_ms=latency_ms,
            status=VisionStatus.OK,
            frame_index=frame_index,
            frame_width_px=frame_width_px,
            frame_height_px=frame_height_px,
            center_x_px=center_x,
            center_y_px=center_y,
            calibrated=True,
        )

    def _estimate_uncalibrated(
        self,
        corners: Any,
        *,
        timestamp_s: float,
        latency_ms: float,
        frame_index: Optional[int],
        frame_width_px: int,
        frame_height_px: int,
    ) -> VisionObservation:
        center_x, center_y = _corner_center(corners)
        focal_px = (frame_width_px / 2.0) / math.tan(math.radians(self.assumed_horizontal_fov_deg) / 2.0)
        side_px = _average_side_px(corners)
        if side_px <= 0 or not _finite(side_px):
            return self._visible_invalid(
                corners,
                timestamp_s=timestamp_s,
                latency_ms=latency_ms,
                frame_index=frame_index,
                frame_width_px=frame_width_px,
                frame_height_px=frame_height_px,
                status=VisionStatus.POSE_INVALID,
                calibrated=False,
            )

        range_m = self.tag_size_m * focal_px / side_px
        offset_x_m = (center_x - (frame_width_px / 2.0)) * range_m / focal_px
        offset_y_m = (center_y - (frame_height_px / 2.0)) * range_m / focal_px
        yaw_error_deg = _image_plane_yaw_deg(corners)

        return VisionObservation(
            timestamp_s=timestamp_s,
            target_visible=True,
            tag_id=self.target_tag_id,
            target_offset_x_m=offset_x_m,
            target_offset_y_m=offset_y_m,
            yaw_error_deg=yaw_error_deg,
            range_m=range_m,
            range_valid=False,
            pose_valid=False,
            latency_ms=latency_ms,
            status=VisionStatus.UNCALIBRATED_ESTIMATE,
            frame_index=frame_index,
            frame_width_px=frame_width_px,
            frame_height_px=frame_height_px,
            center_x_px=center_x,
            center_y_px=center_y,
            calibrated=False,
        )

    def _visible_invalid(
        self,
        corners: Any,
        *,
        timestamp_s: float,
        latency_ms: float,
        frame_index: Optional[int],
        frame_width_px: int,
        frame_height_px: int,
        status: str,
        calibrated: bool,
    ) -> VisionObservation:
        center_x, center_y = _corner_center(corners)
        return VisionObservation(
            timestamp_s=timestamp_s,
            target_visible=True,
            tag_id=self.target_tag_id,
            range_valid=False,
            pose_valid=False,
            latency_ms=latency_ms,
            status=status,
            frame_index=frame_index,
            frame_width_px=frame_width_px,
            frame_height_px=frame_height_px,
            center_x_px=center_x,
            center_y_px=center_y,
            calibrated=calibrated,
        )

    def _tag_object_points(self) -> Any:
        half = self.tag_size_m / 2.0
        return _np_array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ]
        )

    def _yaw_from_rvec(self, rvec: Any) -> float:
        rotation_matrix, _jacobian = self._cv2.Rodrigues(rvec)
        return math.degrees(math.atan2(float(rotation_matrix[1][0]), float(rotation_matrix[0][0])))


def run_stream(args: argparse.Namespace) -> int:
    cv2 = _require_cv2()
    estimator = _estimator_from_args(args)
    source: int | str = args.video if args.video else args.camera_index
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"could not open video source: {source!r}")

    csv_file = None
    writer = None
    try:
        if args.output:
            output_path = Path(args.output)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            csv_file = output_path.open("w", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

        frame_index = 0
        while True:
            capture_timestamp_s = time.monotonic()
            ok, frame = capture.read()
            if not ok:
                break

            observation = estimator.estimate(
                frame,
                timestamp_s=time.monotonic(),
                frame_index=frame_index,
                capture_timestamp_s=capture_timestamp_s,
            )
            _print_observation(observation)
            if writer is not None:
                writer.writerow(observation.to_dict())

            if args.display:
                _draw_overlay(cv2, frame, observation)
                cv2.imshow("AprilTag phone pose", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            frame_index += 1
            if args.max_frames is not None and frame_index >= args.max_frames:
                break
    finally:
        capture.release()
        if args.display:
            cv2.destroyAllWindows()
        if csv_file is not None:
            csv_file.close()

    return 0


def scan_cameras(max_index: int = 8) -> int:
    cv2 = _require_cv2()
    for index in range(max_index + 1):
        capture = cv2.VideoCapture(index)
        ok, frame = capture.read() if capture.isOpened() else (False, None)
        if ok and frame is not None:
            height, width = frame.shape[:2]
            print(f"camera_index={index} opened=true width={width} height={height}")
        else:
            print(f"camera_index={index} opened=false")
        capture.release()
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect tag36h11 id=0 from iPhone 13 Continuity Camera or video.")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--video", help="Path to a video file for replay.")
    source.add_argument("--camera-index", type=int, default=DEFAULT_CAMERA_INDEX, help="OpenCV camera index for live capture.")
    parser.add_argument("--scan-cameras", action="store_true", help="Probe camera indices and exit.")
    parser.add_argument("--output", help="Optional CSV log path.")
    parser.add_argument("--display", action="store_true", help="Show an OpenCV preview window; press q to quit.")
    parser.add_argument("--max-frames", type=int, help="Stop after N frames.")
    parser.add_argument("--tag-id", type=int, default=DEFAULT_TAG_ID, help="Target AprilTag id.")
    parser.add_argument("--tag-size-m", type=float, default=DEFAULT_TAG_SIZE_M, help="Measured black-border tag size in meters.")
    parser.add_argument("--calibration-json", help="Optional JSON file with camera_matrix and dist_coeffs.")
    parser.add_argument(
        "--fov-deg",
        type=float,
        default=DEFAULT_HORIZONTAL_FOV_DEG,
        help="Assumed horizontal FOV for uncalibrated logging only.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.scan_cameras:
        return scan_cameras()
    return run_stream(args)


def _estimator_from_args(args: argparse.Namespace) -> AprilTagPhonePoseEstimator:
    calibration = CameraCalibration.from_json(args.calibration_json) if args.calibration_json else None
    return AprilTagPhonePoseEstimator(
        target_tag_id=args.tag_id,
        tag_size_m=args.tag_size_m,
        calibration=calibration,
        assumed_horizontal_fov_deg=args.fov_deg,
    )


def _require_cv2() -> Any:
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError("OpenCV is required. Install opencv-contrib-python to use AprilTag detection.") from exc

    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise RuntimeError("OpenCV aruco support is required. Install opencv-contrib-python, not opencv-python.")
    return cv2


def _flatten_marker_ids(ids: Any) -> list[int]:
    """Normalize OpenCV ArUco id arrays across OpenCV builds."""

    if hasattr(ids, "reshape"):
        return [int(marker_id) for marker_id in ids.reshape(-1).tolist()]

    flat_ids: list[int] = []
    for marker_id in ids:
        try:
            flat_ids.append(int(marker_id[0]))
        except (IndexError, TypeError):
            flat_ids.append(int(marker_id))
    return flat_ids


def _np_array(values: Iterable[Any]) -> Any:
    try:
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise RuntimeError("NumPy is required for pose estimation.") from exc
    return np.array(values, dtype="float32")


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def _corner_center(corners: Any) -> tuple[float, float]:
    return (
        sum(float(point[0]) for point in corners) / 4.0,
        sum(float(point[1]) for point in corners) / 4.0,
    )


def _average_side_px(corners: Any) -> float:
    lengths = []
    for start, end in [(0, 1), (1, 2), (2, 3), (3, 0)]:
        dx = float(corners[end][0]) - float(corners[start][0])
        dy = float(corners[end][1]) - float(corners[start][1])
        lengths.append(math.hypot(dx, dy))
    return sum(lengths) / len(lengths)


def _image_plane_yaw_deg(corners: Any) -> float:
    dx = float(corners[1][0]) - float(corners[0][0])
    dy = float(corners[1][1]) - float(corners[0][1])
    return math.degrees(math.atan2(dy, dx))


def _print_observation(observation: VisionObservation) -> None:
    print(
        f"frame={observation.frame_index} "
        f"visible={observation.target_visible} "
        f"status={observation.status} "
        f"x={_fmt(observation.target_offset_x_m)}m "
        f"y={_fmt(observation.target_offset_y_m)}m "
        f"yaw={_fmt(observation.yaw_error_deg)}deg "
        f"range={_fmt(observation.range_m)}m "
        f"range_valid={observation.range_valid} "
        f"latency={_fmt(observation.latency_ms)}ms"
    )


def _fmt(value: Optional[float]) -> str:
    return "None" if value is None else f"{value:.3f}"


def _draw_overlay(cv2: Any, frame: Any, observation: VisionObservation) -> None:
    color = (0, 180, 0) if observation.pose_valid else (0, 165, 255)
    if observation.center_x_px is not None and observation.center_y_px is not None:
        center = (int(observation.center_x_px), int(observation.center_y_px))
        cv2.circle(frame, center, 6, color, 2)
    cv2.putText(
        frame,
        f"{observation.status} range={_fmt(observation.range_m)}m",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


if __name__ == "__main__":
    raise SystemExit(main())
