#!/usr/bin/env bash
set -euo pipefail

TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTENSIONS_DIR="$(dirname "$TASK_DIR")"
ISAACSIM_VENV="${ISAACSIM_VENV:-}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-}"
OPENARM_ISAAC_REPO="${OPENARM_ISAAC_REPO:-}"

for variable_name in ISAACSIM_VENV ISAACLAB_ROOT OPENARM_ISAAC_REPO; do
  if [[ -z "${!variable_name}" ]]; then
    echo "[ERROR] Set $variable_name before starting Task Studio." >&2
    exit 2
  fi
done

if [[ ! -f "$ISAACSIM_VENV/bin/activate" ]]; then
  echo "[ERROR] Isaac Sim virtual environment not found: $ISAACSIM_VENV" >&2
  exit 2
fi
if [[ ! -x "$ISAACLAB_ROOT/isaaclab.sh" ]]; then
  echo "[ERROR] Isaac Lab launcher not found: $ISAACLAB_ROOT/isaaclab.sh" >&2
  exit 2
fi
if [[ ! -d "$OPENARM_ISAAC_REPO/openarm" ]]; then
  echo "[ERROR] OpenArm Isaac Lab repository not found: $OPENARM_ISAAC_REPO" >&2
  exit 2
fi

unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PROMPT_MODIFIER CONDA_PYTHON_EXE CONDA_SHLVL _CE_CONDA _CE_M
export TERM=xterm
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1
export OMNI_KIT_ACCEPT_EULA=YES

KIT_ARGS="--ext-folder $EXTENSIONS_DIR --enable openarm_task_studio"
if [[ "${OPENARM_TASK_STUDIO_SELF_TEST:-0}" == "1" ]]; then
  KIT_ARGS+=" --/exts/openarm_task_studio/runSelfTest=1"
fi

# shellcheck disable=SC1090
source "$ISAACSIM_VENV/bin/activate"
cd "$OPENARM_ISAAC_REPO"

echo "[INFO] Starting OpenArm Task Studio"
exec "$ISAACLAB_ROOT/isaaclab.sh" -p "$TASK_DIR/launch_task_studio.py" \
  --kit_args="$KIT_ARGS" \
  "$@"
