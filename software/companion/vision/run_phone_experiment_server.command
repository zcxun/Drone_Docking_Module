#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")/../../.."

echo "Starting phone calibration and A4 AprilTag tracking server..."
echo "Open the printed URL on iPhone Safari."
echo

/opt/anaconda3/bin/python3 -m software.companion.vision.phone_experiment_server \
  --host "${APRILTAG_PHONE_HOST:-0.0.0.0}" \
  --port "${APRILTAG_PHONE_PORT:-9443}"
