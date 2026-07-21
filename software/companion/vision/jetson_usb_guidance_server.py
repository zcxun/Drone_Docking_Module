"""Jetson USB camera AprilTag guidance dashboard.

This server is log-only. It reads a USB camera, estimates AprilTag pose, turns
the result into human-readable guidance, and serves a browser dashboard. It
does not send MAVLink control commands.
"""

from __future__ import annotations

import argparse
import csv
import json
import queue
import signal
import socket
import threading
import time
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional, Sequence
from urllib.parse import urlparse

from software.companion.jetson.range_gate import RangeGateConfig
from software.companion.jetson.sensor_node import JetsonSensorPacket, SafetyTelemetry
from software.companion.jetson.supervisor_node import JetsonDockingSupervisor, JetsonSupervisorConfig
from software.companion.vision.apriltag_phone_pose import (
    AprilTagPhonePoseEstimator,
    CameraCalibration,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_HORIZONTAL_FOV_DEG,
    DEFAULT_TAG_ID,
    DEFAULT_TAG_SIZE_M,
    VisionObservation,
    _require_cv2,
)
from software.companion.vision.guidance_adapter import GuidanceConfig, GuidancePayload, build_guidance_payload


DEFAULT_GUIDANCE_OUTPUT = "experiments/jetson_usb_camera/guidance_log.csv"


@dataclass(frozen=True)
class GuidanceFrame:
    observation: VisionObservation
    snapshot: Any
    command: Any
    guidance: GuidancePayload
    jpeg: bytes
    frame_index: int
    camera_error: Optional[str] = None


class GuidanceHub:
    def __init__(self) -> None:
        self._subscribers: list[queue.Queue[GuidanceFrame]] = []
        self._lock = threading.Lock()
        self.latest: GuidanceFrame | None = None
        self.error: str | None = None

    def publish(self, frame: GuidanceFrame) -> None:
        with self._lock:
            self.latest = frame
            self.error = frame.camera_error
            subscribers = list(self._subscribers)
        for subscriber in subscribers:
            try:
                subscriber.put_nowait(frame)
            except queue.Full:
                try:
                    subscriber.get_nowait()
                except queue.Empty:
                    pass
                subscriber.put_nowait(frame)

    def set_error(self, error: str) -> None:
        with self._lock:
            self.error = error

    def subscribe(self) -> queue.Queue[GuidanceFrame]:
        subscriber: queue.Queue[GuidanceFrame] = queue.Queue(maxsize=2)
        with self._lock:
            self._subscribers.append(subscriber)
            latest = self.latest
        if latest is not None:
            subscriber.put_nowait(latest)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[GuidanceFrame]) -> None:
        with self._lock:
            if subscriber in self._subscribers:
                self._subscribers.remove(subscriber)


class TelemetryWorker(threading.Thread):
    """Optional read-only Pixhawk telemetry for roll/pitch/battery display."""

    def __init__(self, *, device: str, baud: int, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.device = device
        self.baud = baud
        self.stop_event = stop_event
        self.error: str | None = None
        self.safety = SafetyTelemetry()

    def run(self) -> None:
        try:
            from software.companion.jetson.mavlink_link import MavlinkLink
        except Exception as exc:  # pragma: no cover - depends on Jetson environment
            self.error = str(exc)
            return

        link = None
        try:
            link = MavlinkLink.connect(device=self.device, baud=self.baud)
            link.wait_vehicle_heartbeat(timeout_s=5.0)
            while not self.stop_event.is_set():
                message = link.recv_match(
                    message_type=["ATTITUDE", "SYS_STATUS", "BATTERY_STATUS"],
                    timeout_s=0.5,
                )
                if message is not None:
                    self._update(message)
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.error = str(exc)
        finally:
            if link is not None:
                link.close()

    def _update(self, message: Any) -> None:
        message_type = message.get_type() if hasattr(message, "get_type") else ""
        roll = self.safety.roll_deg
        pitch = self.safety.pitch_deg
        voltage = self.safety.battery_voltage_v
        current = self.safety.current_a
        if message_type == "ATTITUDE":
            roll = _rad_to_deg(float(getattr(message, "roll", 0.0)))
            pitch = _rad_to_deg(float(getattr(message, "pitch", 0.0)))
        elif message_type == "SYS_STATUS":
            voltage_mv = int(getattr(message, "voltage_battery", -1))
            current_ca = int(getattr(message, "current_battery", -1))
            if 0 < voltage_mv < 65535:
                voltage = voltage_mv / 1000.0
            if current_ca >= 0:
                current = current_ca / 100.0
        elif message_type == "BATTERY_STATUS":
            voltages = [int(value) for value in getattr(message, "voltages", []) if 0 < int(value) < 65535]
            current_ca = int(getattr(message, "current_battery", -1))
            if voltages:
                voltage = sum(voltages) / 1000.0
            if current_ca >= 0:
                current = current_ca / 100.0
        self.safety = SafetyTelemetry(
            roll_deg=roll,
            pitch_deg=pitch,
            battery_voltage_v=voltage,
            current_a=current,
        )


class CameraGuidanceWorker(threading.Thread):
    def __init__(
        self,
        *,
        camera_index: int,
        estimator: AprilTagPhonePoseEstimator,
        guidance_config: GuidanceConfig,
        supervisor: JetsonDockingSupervisor,
        hub: GuidanceHub,
        stop_event: threading.Event,
        output: Optional[str],
        max_fps: float,
        telemetry_worker: TelemetryWorker | None = None,
    ) -> None:
        super().__init__(daemon=True)
        self.camera_index = camera_index
        self.estimator = estimator
        self.guidance_config = guidance_config
        self.supervisor = supervisor
        self.hub = hub
        self.stop_event = stop_event
        self.output = output
        self.max_fps = max_fps
        self.telemetry_worker = telemetry_worker

    def run(self) -> None:
        try:
            cv2 = _require_cv2()
        except Exception as exc:
            self.hub.set_error(str(exc))
            return

        capture = cv2.VideoCapture(self.camera_index)
        if not capture.isOpened():
            self.hub.set_error(f"could not open camera index {self.camera_index}")
            capture.release()
            return

        csv_file = None
        writer = None
        try:
            if self.output:
                output_path = Path(self.output)
                output_path.parent.mkdir(parents=True, exist_ok=True)
                csv_file = output_path.open("w", newline="", encoding="utf-8")
                writer = csv.DictWriter(csv_file, fieldnames=_csv_fields())
                writer.writeheader()

            frame_index = 0
            min_period_s = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
            while not self.stop_event.is_set():
                started_s = time.monotonic()
                capture_timestamp_s = time.monotonic()
                ok, frame = capture.read()
                if not ok:
                    self.hub.set_error("camera frame read failed")
                    time.sleep(0.1)
                    continue

                observation = self.estimator.estimate(
                    frame,
                    timestamp_s=time.monotonic(),
                    frame_index=frame_index,
                    capture_timestamp_s=capture_timestamp_s,
                )
                safety = self.telemetry_worker.safety if self.telemetry_worker is not None else SafetyTelemetry()
                output = self.supervisor.update(
                    observation,
                    JetsonSensorPacket(auto_enabled=True, safety=safety),
                )
                guidance = build_guidance_payload(
                    output.observation,
                    output.snapshot,
                    output.command,
                    self.guidance_config,
                )
                overlay = _draw_guidance_overlay(cv2, frame, output.observation, guidance)
                ok, encoded = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
                if not ok:
                    self.hub.set_error("could not encode camera frame")
                    continue

                guidance_frame = GuidanceFrame(
                    observation=output.observation,
                    snapshot=output.snapshot,
                    command=output.command,
                    guidance=guidance,
                    jpeg=bytes(encoded),
                    frame_index=frame_index,
                    camera_error=None,
                )
                self.hub.publish(guidance_frame)
                if writer is not None:
                    writer.writerow(_csv_row(guidance_frame))
                    if frame_index % 15 == 0:
                        csv_file.flush()

                frame_index += 1
                elapsed_s = time.monotonic() - started_s
                if elapsed_s < min_period_s:
                    time.sleep(min_period_s - elapsed_s)
        finally:
            capture.release()
            if csv_file is not None:
                csv_file.close()


class GuidanceDashboardHandler(BaseHTTPRequestHandler):
    hub: GuidanceHub

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/dashboard", "/dashboard.html"}:
            self._send_html(DASHBOARD_HTML)
        elif path == "/events":
            self._send_events()
        elif path == "/stream.mjpg":
            self._send_mjpeg()
        elif path == "/frame.jpg":
            self._send_frame()
        elif path == "/api/latest":
            self._send_json(_latest_payload(self.hub))
        elif path == "/health":
            self._send_json({"ok": self.hub.error is None and self.hub.latest is not None, "error": self.hub.error})
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_html(self, content: str) -> None:
        encoded = content.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_json(self, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_frame(self) -> None:
        latest = self.hub.latest
        if latest is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "no frame yet")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(latest.jpeg)))
        self.end_headers()
        self.wfile.write(latest.jpeg)

    def _send_mjpeg(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_index = -1
        try:
            while True:
                latest = self.hub.latest
                if latest is None or latest.frame_index == last_index:
                    time.sleep(0.05)
                    continue
                last_index = latest.frame_index
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(latest.jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(latest.jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        subscriber = self.hub.subscribe()
        try:
            self._write_event("hello", _latest_payload(self.hub))
            while True:
                try:
                    guidance_frame = subscriber.get(timeout=10.0)
                    self._write_event("guidance", _frame_payload(guidance_frame))
                except queue.Empty:
                    self._write_event("heartbeat", _latest_payload(self.hub))
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.hub.unsubscribe(subscriber)

    def _write_event(self, event: str, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, allow_nan=False)
        self.wfile.write(f"event: {event}\n".encode("utf-8"))
        self.wfile.write(f"data: {encoded}\n\n".encode("utf-8"))
        self.wfile.flush()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Jetson USB AprilTag guidance dashboard.")
    parser.add_argument("--host", default="0.0.0.0", help="Dashboard bind host.")
    parser.add_argument("--port", type=int, default=8765, help="Dashboard HTTP port.")
    parser.add_argument("--camera-index", type=int, default=DEFAULT_CAMERA_INDEX, help="OpenCV USB camera index.")
    parser.add_argument("--scan-cameras", action="store_true", help="Probe camera indices and exit.")
    parser.add_argument("--output", default=DEFAULT_GUIDANCE_OUTPUT, help="Optional CSV log path.")
    parser.add_argument("--no-output", action="store_true", help="Disable CSV logging.")
    parser.add_argument("--max-fps", type=float, default=12.0, help="Maximum camera processing rate.")
    parser.add_argument("--tag-id", type=int, default=DEFAULT_TAG_ID, help="Target AprilTag id.")
    parser.add_argument("--tag-size-m", type=float, default=DEFAULT_TAG_SIZE_M, help="Measured black-border tag size in meters.")
    parser.add_argument("--calibration-json", help="Optional JSON camera calibration.")
    parser.add_argument("--fov-deg", type=float, default=DEFAULT_HORIZONTAL_FOV_DEG, help="Assumed FOV for uncalibrated guidance.")
    parser.add_argument("--camera-forward", choices=("up", "down", "left", "right"), default="up", help="Which image direction points to the aircraft nose.")
    parser.add_argument("--mirror-x", action="store_true", help="Mirror image x before guidance conversion.")
    parser.add_argument("--mirror-y", action="store_true", help="Mirror image y before guidance conversion.")
    parser.add_argument("--height-target-m", type=float, default=0.50, help="Human-guidance target height.")
    parser.add_argument("--mavlink-device", help="Optional read-only Pixhawk telemetry device.")
    parser.add_argument("--mavlink-baud", type=int, default=57600, help="Optional read-only Pixhawk telemetry baud.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.scan_cameras:
        return _scan_cameras()

    calibration = CameraCalibration.from_json(args.calibration_json) if args.calibration_json else None
    estimator = AprilTagPhonePoseEstimator(
        target_tag_id=args.tag_id,
        tag_size_m=args.tag_size_m,
        calibration=calibration,
        assumed_horizontal_fov_deg=args.fov_deg,
    )
    guidance_config = GuidanceConfig(
        camera_forward=args.camera_forward,
        mirror_x=args.mirror_x,
        mirror_y=args.mirror_y,
        height_target_m=args.height_target_m,
    )
    supervisor = JetsonDockingSupervisor(
        JetsonSupervisorConfig(
            allow_uncalibrated_estimate=True,
            range_gate=RangeGateConfig(require_rangefinder=False),
        )
    )
    hub = GuidanceHub()
    stop_event = threading.Event()
    telemetry_worker = (
        TelemetryWorker(device=args.mavlink_device, baud=args.mavlink_baud, stop_event=stop_event)
        if args.mavlink_device
        else None
    )
    camera_worker = CameraGuidanceWorker(
        camera_index=args.camera_index,
        estimator=estimator,
        guidance_config=guidance_config,
        supervisor=supervisor,
        hub=hub,
        stop_event=stop_event,
        output=None if args.no_output else args.output,
        max_fps=args.max_fps,
        telemetry_worker=telemetry_worker,
    )

    handler_class = type("JetsonGuidanceDashboardHandler", (GuidanceDashboardHandler,), {"hub": hub})
    server = ThreadingHTTPServer((args.host, args.port), handler_class)

    def stop_server(_signum: int, _frame: Any) -> None:
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)

    if telemetry_worker is not None:
        telemetry_worker.start()
    camera_worker.start()

    local_url = f"http://127.0.0.1:{args.port}/dashboard"
    lan_ip = _local_lan_ip()
    print(f"Jetson guidance dashboard: {local_url}")
    if args.host not in {"127.0.0.1", "localhost"}:
        print(f"Bound address: http://{args.host}:{args.port}/dashboard")
        if lan_ip is not None:
            print(f"Computer/LAN URL: http://{lan_ip}:{args.port}/dashboard")
    print(f"Camera index: {args.camera_index}")
    print(f"Camera forward: {args.camera_forward} mirror_x={args.mirror_x} mirror_y={args.mirror_y}")
    print("MAVLink control output: disabled")

    try:
        server.serve_forever()
    finally:
        stop_event.set()
        camera_worker.join(timeout=2.0)
        if telemetry_worker is not None:
            telemetry_worker.join(timeout=2.0)
        server.server_close()
    return 0


def _draw_guidance_overlay(cv2: Any, frame: Any, observation: VisionObservation, guidance: GuidancePayload) -> Any:
    if len(frame.shape) == 2:
        display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:
        display = frame.copy()

    height, width = display.shape[:2]
    frame_center = (width // 2, height // 2)
    cv2.drawMarker(display, frame_center, (255, 255, 255), cv2.MARKER_CROSS, 22, 2)
    if observation.center_x_px is not None and observation.center_y_px is not None:
        tag_center = (int(observation.center_x_px), int(observation.center_y_px))
        color = (0, 200, 0) if guidance.control_allowed else (0, 190, 255)
        cv2.circle(display, tag_center, 8, color, 2)
        cv2.arrowedLine(display, frame_center, tag_center, color, 2, tipLength=0.15)

    status_color = (0, 200, 0) if guidance.control_allowed else (0, 190, 255)
    if not observation.target_visible:
        status_color = (0, 0, 220)
    cv2.rectangle(display, (0, 0), (width, 76), (0, 0, 0), -1)
    cv2.putText(display, guidance.status_text, (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, status_color, 2, cv2.LINE_AA)
    cv2.putText(display, guidance.horizontal_instruction, (18, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2, cv2.LINE_AA)
    return display


def _scan_cameras(max_index: int = 8) -> int:
    cv2 = _require_cv2()
    for index in range(max_index + 1):
        capture = cv2.VideoCapture(index)
        ok, frame = capture.read() if capture.isOpened() else (False, None)
        if ok and frame is not None:
            height, width = frame.shape[:2]
            channels = 1 if len(frame.shape) == 2 else frame.shape[2]
            print(f"camera_index={index} opened=true width={width} height={height} channels={channels}")
        else:
            print(f"camera_index={index} opened=false")
        capture.release()
    return 0


def _latest_payload(hub: GuidanceHub) -> dict[str, Any]:
    latest = hub.latest
    return {
        "camera_error": hub.error,
        "frame": _frame_payload(latest) if latest is not None else None,
    }


def _frame_payload(guidance_frame: GuidanceFrame | None) -> dict[str, Any] | None:
    if guidance_frame is None:
        return None
    observation = asdict(guidance_frame.observation)
    return {
        "frame_index": guidance_frame.frame_index,
        "observation": observation,
        "snapshot": asdict(guidance_frame.snapshot),
        "command": {
            "state": guidance_frame.command.state.value,
            "mode": guidance_frame.command.mode,
            "horizontal_velocity_mps": list(guidance_frame.command.horizontal_velocity_mps),
            "yaw_rate_dps": guidance_frame.command.yaw_rate_dps,
            "vertical_velocity_mps": guidance_frame.command.vertical_velocity_mps,
            "lock_command": guidance_frame.command.lock_command.value,
            "abort_reason": guidance_frame.command.abort_reason.value,
            "message": guidance_frame.command.message,
        },
        "guidance": guidance_frame.guidance.to_dict(),
    }


def _csv_fields() -> list[str]:
    return [
        "timestamp_s",
        "frame_index",
        "status",
        "primary_instruction",
        "horizontal_instruction",
        "height_instruction",
        "yaw_instruction",
        "attitude_instruction",
        "command_preview",
        "target_visible",
        "x_m",
        "y_m",
        "yaw_deg",
        "range_m",
        "roll_deg",
        "pitch_deg",
        "vx_mps",
        "vy_mps",
        "vz_mps",
        "yaw_rate_dps",
        "control_allowed",
        "control_note",
    ]


def _csv_row(guidance_frame: GuidanceFrame) -> dict[str, object]:
    obs = guidance_frame.observation
    snap = guidance_frame.snapshot
    command = guidance_frame.command
    guidance = guidance_frame.guidance
    return {
        "timestamp_s": obs.timestamp_s,
        "frame_index": guidance_frame.frame_index,
        "status": obs.status,
        "primary_instruction": guidance.primary_instruction,
        "horizontal_instruction": guidance.horizontal_instruction,
        "height_instruction": guidance.height_instruction,
        "yaw_instruction": guidance.yaw_instruction,
        "attitude_instruction": guidance.attitude_instruction,
        "command_preview": guidance.command_preview,
        "target_visible": obs.target_visible,
        "x_m": obs.target_offset_x_m,
        "y_m": obs.target_offset_y_m,
        "yaw_deg": obs.yaw_error_deg,
        "range_m": snap.range_m,
        "roll_deg": snap.roll_deg,
        "pitch_deg": snap.pitch_deg,
        "vx_mps": command.horizontal_velocity_mps[0],
        "vy_mps": command.horizontal_velocity_mps[1],
        "vz_mps": command.vertical_velocity_mps,
        "yaw_rate_dps": command.yaw_rate_dps,
        "control_allowed": guidance.control_allowed,
        "control_note": guidance.control_note,
    }


def _local_lan_ip() -> Optional[str]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def _rad_to_deg(value: float) -> float:
    return value * 57.29577951308232


DASHBOARD_HTML = """<!doctype html>
<html lang="zh-Hant">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Jetson 定位導引</title>
  <style>
    :root {
      --bg: #f5f6f2;
      --ink: #151515;
      --muted: #686b70;
      --panel: #ffffff;
      --line: #d9dcd3;
      --good: #087f5b;
      --warn: #b7791f;
      --bad: #b42318;
      --blue: #1f5f99;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: var(--ink);
      background: var(--bg);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 16px;
      padding: 14px 20px;
      border-bottom: 1px solid var(--line);
      background: rgba(255,255,255,0.92);
      position: sticky;
      top: 0;
      z-index: 2;
    }
    h1 { margin: 0; font-size: 22px; }
    .status { font-weight: 800; color: var(--warn); }
    main {
      display: grid;
      grid-template-columns: minmax(360px, 1.4fr) minmax(320px, 0.8fr);
      gap: 14px;
      padding: 14px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }
    .video-panel { min-height: 420px; }
    .video-panel img {
      width: 100%;
      display: block;
      background: #111;
      aspect-ratio: 16 / 10;
      object-fit: contain;
    }
    .instruction {
      padding: 18px;
      border-top: 1px solid var(--line);
    }
    .big {
      font-size: clamp(26px, 5vw, 52px);
      line-height: 1.08;
      font-weight: 900;
      margin: 0 0 10px;
    }
    .note { color: var(--muted); font-size: 15px; }
    .side { display: grid; gap: 14px; align-content: start; }
    .section { padding: 16px; }
    .section h2 { margin: 0 0 12px; font-size: 16px; }
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 11px;
      min-height: 72px;
      background: #fbfcf8;
    }
    .label { color: var(--muted); font-size: 12px; font-weight: 700; }
    .value { margin-top: 5px; font-size: 18px; font-weight: 850; }
    .good { color: var(--good); }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }
    details {
      border-top: 1px solid var(--line);
      padding: 12px 16px;
    }
    pre {
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      font-size: 12px;
      color: #222;
      background: #f1f2ee;
      padding: 10px;
      border-radius: 6px;
    }
    @media (max-width: 880px) {
      main { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Jetson 定位導引</h1>
    <div id="connection" class="status">等待資料</div>
  </header>
  <main>
    <section class="panel video-panel">
      <img src="/stream.mjpg" alt="Jetson USB camera stream">
      <div class="instruction">
        <p id="primary" class="big">等待相機畫面</p>
        <div id="controlNote" class="note">目前不送 Pixhawk/PX4 控制命令</div>
      </div>
    </section>
    <aside class="side">
      <section class="panel section">
        <h2>人工操作提示</h2>
        <div class="metric-grid">
          <div class="metric"><div class="label">水平</div><div id="horizontal" class="value">--</div></div>
          <div class="metric"><div class="label">角度</div><div id="yaw" class="value">--</div></div>
          <div class="metric"><div class="label">高度</div><div id="height" class="value">--</div></div>
          <div class="metric"><div class="label">傾斜</div><div id="attitude" class="value">--</div></div>
        </div>
      </section>
      <section class="panel section">
        <h2>定位狀態</h2>
        <div class="metric-grid">
          <div class="metric"><div class="label">狀態</div><div id="visionStatus" class="value">--</div></div>
          <div class="metric"><div class="label">控制資格</div><div id="controlAllowed" class="value">--</div></div>
          <div class="metric"><div class="label">Range 來源</div><div id="rangeSource" class="value">--</div></div>
          <div class="metric"><div class="label">Latency</div><div id="latency" class="value">--</div></div>
        </div>
      </section>
      <section class="panel section">
        <h2>Pixhawk/PX4 命令預覽</h2>
        <div id="commandPreview" class="value">目前保持</div>
      </section>
      <details class="panel">
        <summary>工程資料</summary>
        <pre id="engineering">{}</pre>
      </details>
    </aside>
  </main>
  <script>
    const fields = {
      connection: document.getElementById("connection"),
      primary: document.getElementById("primary"),
      controlNote: document.getElementById("controlNote"),
      horizontal: document.getElementById("horizontal"),
      yaw: document.getElementById("yaw"),
      height: document.getElementById("height"),
      attitude: document.getElementById("attitude"),
      visionStatus: document.getElementById("visionStatus"),
      controlAllowed: document.getElementById("controlAllowed"),
      rangeSource: document.getElementById("rangeSource"),
      latency: document.getElementById("latency"),
      commandPreview: document.getElementById("commandPreview"),
      engineering: document.getElementById("engineering")
    };
    function setClass(el, ok) {
      el.classList.remove("good", "warn", "bad");
      el.classList.add(ok ? "good" : "warn");
    }
    function render(payload) {
      if (!payload || !payload.frame) {
        fields.connection.textContent = payload && payload.camera_error ? payload.camera_error : "等待資料";
        fields.connection.className = "status bad";
        return;
      }
      const frame = payload.frame;
      const guidance = frame.guidance;
      const obs = frame.observation;
      fields.connection.textContent = "即時連線";
      fields.connection.className = "status good";
      fields.primary.textContent = guidance.primary_instruction;
      fields.controlNote.textContent = guidance.control_note;
      fields.horizontal.textContent = guidance.horizontal_instruction;
      fields.yaw.textContent = guidance.yaw_instruction;
      fields.height.textContent = guidance.height_instruction;
      fields.attitude.textContent = guidance.attitude_instruction;
      fields.visionStatus.textContent = guidance.status_text;
      fields.controlAllowed.textContent = guidance.control_allowed ? "可作為候選" : "僅人工導引";
      setClass(fields.controlAllowed, guidance.control_allowed);
      fields.rangeSource.textContent = guidance.range_source;
      fields.latency.textContent = obs.latency_ms == null ? "--" : obs.latency_ms.toFixed(1) + " ms";
      fields.commandPreview.textContent = guidance.command_preview;
      fields.engineering.textContent = JSON.stringify(guidance.engineering, null, 2);
    }
    const events = new EventSource("/events");
    events.addEventListener("hello", event => render(JSON.parse(event.data)));
    events.addEventListener("guidance", event => render({frame: JSON.parse(event.data)}));
    events.addEventListener("heartbeat", event => render(JSON.parse(event.data)));
    events.onerror = () => {
      fields.connection.textContent = "連線中斷，嘗試重連";
      fields.connection.className = "status bad";
    };
  </script>
</body>
</html>
"""


if __name__ == "__main__":
    raise SystemExit(main())

