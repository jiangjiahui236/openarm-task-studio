#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
if rg --pcre2 -n --hidden --glob '!.git/**' --glob '!scripts/privacy_check.sh' --glob '!.env.example' \
  '(/home/[^/[:space:]]+|/Users/[^/[:space:]]+|BEGIN (RSA|OPENSSH|EC) PRIVATE KEY|WIFI_PASSWORD[[:space:]]*=[[:space:]]*"(?!(YOUR_WIFI_PASSWORD)))' .; then
  echo "[FAIL] Potential private path or credential found." >&2
  exit 1
fi
if find logs tasks -type f ! -name '.gitkeep' ! -name 'example_task.json' -print -quit | grep -q .; then
  echo "[FAIL] Runtime logs or private task files found." >&2
  exit 1
fi
echo "[PASS] No known private paths, credentials, logs, or task history found."
