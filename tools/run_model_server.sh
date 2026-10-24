#!/bin/bash
# Start the human-detection model service that thermal_detector_node talks to.
#
# The model is a separate repository (https://github.com/andrew2719/thermal_rescue_ai_model),
# cloned to external/thermal_rescue_ai_model with its own venv in external/venv (CPU PyTorch,
# versions in external/requirements-lock.txt). external/ is git-ignored and COLCON_IGNOREd.
# Setup on a fresh clone:
#   git clone https://github.com/andrew2719/thermal_rescue_ai_model external/thermal_rescue_ai_model
#   touch external/COLCON_IGNORE
#   python3 -m venv external/venv
#   external/venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
#   external/venv/bin/pip install -r external/thermal_rescue_ai_model/requirements.txt
#
# Usage: tools/run_model_server.sh [PORT]     (default 8766, = server_url in thermal_detector.yaml;
#        8765 is the model repo's own default but tools/xr_viewer/bridge.py uses it)
# The service has no authentication and binds to 127.0.0.1.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="$ROOT/external/thermal_rescue_ai_model"
[ -d "$MODEL" ] || { echo "model repo not found at $MODEL (see the setup notes in this script)" >&2; exit 1; }
unset PYTHONPATH            # keep the sourced ROS python packages out of the model's venv
cd "$MODEL"
exec "$ROOT/external/venv/bin/python" -m sphere_scout serve --host 127.0.0.1 --port "${1:-8766}"
