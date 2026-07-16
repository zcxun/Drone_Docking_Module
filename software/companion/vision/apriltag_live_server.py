"""Local live dashboard server for the iPhone 13 AprilTag experiment."""

from __future__ import annotations

import argparse
import csv
import json
import queue
import signal
import socket
import threading
import time
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional, Sequence
from urllib.parse import urlparse

from software.companion.vision.apriltag_phone_pose import (
    CSV_FIELDS,
    AprilTagPhonePoseEstimator,
    CameraCalibration,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_HORIZONTAL_FOV_DEG,
    DEFAULT_TAG_ID,
    DEFAULT_TAG_SIZE_M,
    VisionObservation,
    _require_cv2,
)


DASHBOARD_PATH = Path(__file__).with_name("apriltag_alignment_dashboard.html")


class ObservationHub:
    """Fan out observations from the camera thread to browser clients."""

    def __init__(self) -> None:
        self._subscribers: list[queue.Queue[VisionObservation]] = []
        self._lock = threading.Lock()
        self.latest: Optional[VisionObservation] = None

    def publish(self, observation: VisionObservation) -> None:
        with self._lock:
            self.latest = observation
            subscribers = list(self._subscribers)
        for subscriber in subscribers:
            try:
                subscriber.put_nowait(observation)
            except queue.Full:
                try:
                    subscriber.get_nowait()
                except queue.Empty:
                    pass
                subscriber.put_nowait(observation)

    def subscribe(self) -> queue.Queue[VisionObservation]:
        subscriber: queue.Queue[VisionObservation] = queue.Queue(maxsize=2)
        with self._lock:
            self._subscribers.append(subscriber)
            latest = self.latest
        if latest is not None:
            subscriber.put_nowait(latest)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[VisionObservation]) -> None:
        with self._lock:
            if subscriber in self._subscribers:
                self._subscribers.remove(subscriber)


class CameraWorker(threading.Thread):
    """Read frames from OpenCV, estimate AprilTag pose, and publish observations."""

    def __init__(
        self,
        *,
        camera_index: int,
        estimator: AprilTagPhonePoseEstimator,
        hub: ObservationHub,
        stop_event: threading.Event,
        output: Optional[str],
        max_fps: float,
    ) -> None:
        super().__init__(daemon=True)
        self.camera_index = camera_index
        self.estimator = estimator
        self.hub = hub
        self.stop_event = stop_event
        self.output = output
        self.max_fps = max_fps
        self.error: Optional[str] = None

    def run(self) -> None:
        cv2 = _require_cv2()
        capture = None
        while not self.stop_event.is_set():
            capture = cv2.VideoCapture(self.camera_index)
            if capture.isOpened():
                self.error = None
                break
            self.error = f"could not open camera index {self.camera_index}"
            capture.release()
            time.sleep(1.0)

        if capture is None or not capture.isOpened():
            return

        csv_file = None
        writer = None
        try:
            if self.output:
                output_path = Path(self.output)
                output_path.parent.mkdir(parents=True, exist_ok=True)
                csv_file = output_path.open("w", newline="", encoding="utf-8")
                writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
                writer.writeheader()

            frame_index = 0
            min_period_s = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
            while not self.stop_event.is_set():
                loop_started_s = time.monotonic()
                capture_timestamp_s = time.monotonic()
                ok, frame = capture.read()
                if not ok:
                    self.error = "camera frame read failed"
                    time.sleep(0.1)
                    continue

                observation = self.estimator.estimate(
                    frame,
                    timestamp_s=time.monotonic(),
                    frame_index=frame_index,
                    capture_timestamp_s=capture_timestamp_s,
                )
                self.hub.publish(observation)
                if writer is not None:
                    writer.writerow(observation.to_dict())
                    if frame_index % 15 == 0:
                        csv_file.flush()

                frame_index += 1
                elapsed_s = time.monotonic() - loop_started_s
                if elapsed_s < min_period_s:
                    time.sleep(min_period_s - elapsed_s)
        finally:
            capture.release()
            if csv_file is not None:
                csv_file.close()


class DashboardHandler(BaseHTTPRequestHandler):
    hub: ObservationHub
    camera_worker: CameraWorker

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/dashboard", "/dashboard.html"}:
            self._send_dashboard()
        elif path == "/events":
            self._send_events()
        elif path == "/api/latest":
            self._send_json(self._latest_payload())
        elif path == "/health":
            self._send_json({"ok": self.camera_worker.error is None, "error": self.camera_worker.error})
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_dashboard(self) -> None:
        content = DASHBOARD_PATH.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_json(self, payload: dict[str, Any]) -> None:
        content = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        subscriber = self.hub.subscribe()
        try:
            self._write_event("hello", self._latest_payload())
            while True:
                try:
                    observation = subscriber.get(timeout=10.0)
                    self._write_event("observation", _observation_payload(observation))
                except queue.Empty:
                    self._write_event("heartbeat", self._latest_payload())
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.hub.unsubscribe(subscriber)

    def _write_event(self, event: str, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, allow_nan=False)
        self.wfile.write(f"event: {event}\n".encode("utf-8"))
        self.wfile.write(f"data: {encoded}\n\n".encode("utf-8"))
        self.wfile.flush()

    def _latest_payload(self) -> dict[str, Any]:
        latest = self.hub.latest
        return {
            "camera_error": self.camera_worker.error,
            "observation": _observation_payload(latest) if latest is not None else None,
        }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve the AprilTag alignment dashboard with live camera data.")
    parser.add_argument("--host", default="127.0.0.1", help="Dashboard bind host.")
    parser.add_argument("--port", type=int, default=8765, help="Dashboard HTTP port.")
    parser.add_argument("--camera-index", type=int, default=DEFAULT_CAMERA_INDEX, help="OpenCV camera index.")
    parser.add_argument("--output", default="experiments/apriltag_phone_camera/live_dashboard_log.csv", help="Optional CSV log path.")
    parser.add_argument("--no-output", action="store_true", help="Disable CSV logging.")
    parser.add_argument("--max-fps", type=float, default=12.0, help="Maximum camera processing rate.")
    parser.add_argument("--tag-id", type=int, default=DEFAULT_TAG_ID, help="Target AprilTag id.")
    parser.add_argument("--tag-size-m", type=float, default=DEFAULT_TAG_SIZE_M, help="Measured black-border tag size in meters.")
    parser.add_argument("--calibration-json", help="Optional JSON file with camera_matrix and dist_coeffs.")
    parser.add_argument("--fov-deg", type=float, default=DEFAULT_HORIZONTAL_FOV_DEG, help="Assumed FOV for uncalibrated logging.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    calibration = CameraCalibration.from_json(args.calibration_json) if args.calibration_json else None
    estimator = AprilTagPhonePoseEstimator(
        target_tag_id=args.tag_id,
        tag_size_m=args.tag_size_m,
        calibration=calibration,
        assumed_horizontal_fov_deg=args.fov_deg,
    )

    hub = ObservationHub()
    stop_event = threading.Event()
    worker = CameraWorker(
        camera_index=args.camera_index,
        estimator=estimator,
        hub=hub,
        stop_event=stop_event,
        output=None if args.no_output else args.output,
        max_fps=args.max_fps,
    )

    handler_class = type(
        "LiveDashboardHandler",
        (DashboardHandler,),
        {"hub": hub, "camera_worker": worker},
    )
    server = ThreadingHTTPServer((args.host, args.port), handler_class)

    def stop_server(_signum: int, _frame: Any) -> None:
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)

    worker.start()
    local_url = f"http://127.0.0.1:{args.port}/dashboard"
    bound_url = f"http://{args.host}:{args.port}/dashboard"
    lan_ip = _local_lan_ip()
    lan_url = f"http://{lan_ip}:{args.port}/dashboard" if lan_ip else None
    print(f"AprilTag live dashboard: {local_url}")
    if args.host not in {"127.0.0.1", "localhost"}:
        print(f"Bound address: {bound_url}")
        if lan_url is not None:
            print(f"Phone/LAN URL: {lan_url}")
    print(f"Camera index: {args.camera_index}")
    if args.no_output:
        print("CSV logging: disabled")
    else:
        print(f"CSV logging: {args.output}")

    try:
        server.serve_forever()
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        server.server_close()
    return 0


def _observation_payload(observation: VisionObservation) -> dict[str, Any]:
    return asdict(observation)


def _local_lan_ip() -> Optional[str]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
