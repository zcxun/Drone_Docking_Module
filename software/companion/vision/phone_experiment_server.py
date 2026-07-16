"""HTTPS phone workflow for camera calibration and A4 AprilTag tracking."""

from __future__ import annotations

import argparse
import json
import socket
import ssl
import subprocess
import tempfile
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional, Sequence
from urllib.parse import urlparse

from software.companion.vision.a4_board_tracking import (
    DEFAULT_CALIBRATION_PATH,
    A4BoardTracker,
    CalibrationSession,
    decode_image_bytes,
    load_calibration_if_present,
)
from software.companion.vision.apriltag_phone_pose import CameraCalibration


CALIBRATION_PAGE = Path(__file__).with_name("phone_calibration.html")
TRACKING_PAGE = Path(__file__).with_name("phone_tracking.html")
CERT_DIR = Path("software/companion/vision/certs")


class PhoneExperimentState:
    def __init__(self, *, calibration_path: str | Path) -> None:
        self.calibration_path = Path(calibration_path)
        self.session = CalibrationSession(output_path=self.calibration_path)
        self.calibration = load_calibration_if_present(self.calibration_path)
        self.tracker = A4BoardTracker(calibration=self.calibration)

    def reload_calibration(self) -> None:
        self.calibration = CameraCalibration.from_json(self.calibration_path)
        self.tracker.set_calibration(self.calibration)


class PhoneExperimentHandler(BaseHTTPRequestHandler):
    state: PhoneExperimentState

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/calibration"}:
            self._send_file(CALIBRATION_PAGE)
        elif path == "/phone":
            self._send_file(TRACKING_PAGE)
        elif path == "/api/status":
            self._send_json(
                {
                    "calibration_path": str(self.state.calibration_path),
                    "calibrated": self.state.calibration is not None,
                    "sample_count": self.state.session.sample_count,
                }
            )
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            if path == "/api/calibration/sample":
                self._handle_calibration_sample()
            elif path == "/api/calibration/solve":
                result = self.state.session.solve()
                if result.get("ok"):
                    self.state.reload_calibration()
                self._send_json(result)
            elif path == "/api/calibration/reset":
                self._send_json(self.state.session.reset())
            elif path == "/api/board/frame":
                self._handle_board_frame()
            else:
                self.send_error(HTTPStatus.NOT_FOUND, "not found")
        except Exception as exc:  # noqa: BLE001 - return actionable errors to phone UI
            self._send_json({"ok": False, "status": "SERVER_ERROR", "error": str(exc)}, status=500)

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _handle_calibration_sample(self) -> None:
        frame = decode_image_bytes(self._read_body())
        self._send_json(self.state.session.add_frame(frame))

    def _handle_board_frame(self) -> None:
        frame = decode_image_bytes(self._read_body())
        self._send_json(self.state.tracker.track(frame))

    def _read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", "0"))
        return self.rfile.read(length)

    def _send_file(self, path: Path) -> None:
        content = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_json(self, payload: dict[str, Any], *, status: int = 200) -> None:
        content = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve the phone calibration/tracking workflow over HTTPS.")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host. Use 0.0.0.0 for phone access.")
    parser.add_argument("--port", type=int, default=9443, help="HTTPS port.")
    parser.add_argument("--calibration-json", default=str(DEFAULT_CALIBRATION_PATH), help="Calibration JSON output/input path.")
    parser.add_argument("--cert", default=str(CERT_DIR / "phone_server.crt"), help="TLS certificate path.")
    parser.add_argument("--key", default=str(CERT_DIR / "phone_server.key"), help="TLS private key path.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    lan_ip = _local_lan_ip() or "127.0.0.1"
    cert_path, key_path = ensure_self_signed_cert(args.cert, args.key, lan_ip=lan_ip)

    state = PhoneExperimentState(calibration_path=args.calibration_json)
    handler = type("BoundPhoneExperimentHandler", (PhoneExperimentHandler,), {"state": state})
    server = ThreadingHTTPServer((args.host, args.port), handler)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=cert_path, keyfile=key_path)
    server.socket = context.wrap_socket(server.socket, server_side=True)

    print(f"Calibration page: https://{lan_ip}:{args.port}/calibration")
    print(f"Tracking page:    https://{lan_ip}:{args.port}/phone")
    print(f"Certificate:      {cert_path}")
    print(f"Calibration JSON: {args.calibration_json}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def ensure_self_signed_cert(cert_path: str | Path, key_path: str | Path, *, lan_ip: str) -> tuple[str, str]:
    cert = Path(cert_path)
    key = Path(key_path)
    if cert.exists() and key.exists():
        return str(cert), str(key)

    cert.parent.mkdir(parents=True, exist_ok=True)
    openssl = _openssl_path()
    with tempfile.NamedTemporaryFile("w", suffix=".cnf", delete=False) as config:
        config.write(
            "\n".join(
                [
                    "[req]",
                    "distinguished_name=req_distinguished_name",
                    "x509_extensions=v3_req",
                    "prompt=no",
                    "[req_distinguished_name]",
                    "CN=apriltag-phone.local",
                    "[v3_req]",
                    "subjectAltName=@alt_names",
                    "[alt_names]",
                    "DNS.1=localhost",
                    "IP.1=127.0.0.1",
                    f"IP.2={lan_ip}",
                    "",
                ]
            )
        )
        config_path = config.name

    subprocess.run(
        [
            openssl,
            "req",
            "-x509",
            "-nodes",
            "-newkey",
            "rsa:2048",
            "-days",
            "365",
            "-keyout",
            str(key),
            "-out",
            str(cert),
            "-config",
            config_path,
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    Path(config_path).unlink(missing_ok=True)
    return str(cert), str(key)


def _openssl_path() -> str:
    for candidate in ("/opt/anaconda3/bin/openssl", "/usr/bin/openssl", "openssl"):
        try:
            subprocess.run([candidate, "version"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return candidate
        except (OSError, subprocess.CalledProcessError):
            continue
    raise RuntimeError("openssl is required to generate a local HTTPS certificate")


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
