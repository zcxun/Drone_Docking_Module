#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/../../.."

echo "Starting AprilTag live dashboard..."
echo "If macOS asks for Camera permission, allow Terminal."
echo

/opt/anaconda3/bin/python3 -m software.companion.vision.apriltag_live_server \
  --host "${APRILTAG_DASHBOARD_HOST:-0.0.0.0}" \
  --camera-index "${APRILTAG_CAMERA_INDEX:-0}" \
  --port "${APRILTAG_DASHBOARD_PORT:-8765}"
