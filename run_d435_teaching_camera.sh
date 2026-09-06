#!/usr/bin/env bash
set -euo pipefail

TASK_STUDIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset PYTHONNOUSERSITE
cd "$TASK_STUDIO_DIR"
exec /usr/bin/python3 d435_rgbd_sender.py \
  --config "$TASK_STUDIO_DIR/config/openarm_d435_teleop_visual.yaml" \
  --udp-host 127.0.0.1 \
  --udp-port 5010 \
  --no-preview \
  --preview-file /tmp/openarm_task_studio_d435_preview.jpg \
  --preview-every 1 \
  "$@"
