#!/usr/bin/env bash
# Offline only: independent P3 evaluation of one finalized run directory.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: verify_p3_evidence.sh ABSOLUTE_P3_RUN_DIRECTORY'
  exit 0
fi
[[ $# == 1 && -f $1/summary.json ]] || fail 'One existing P3 run required'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec "$SIM_ROOT/venv-px4/bin/python" "$SIM_DIR/validation/p3_evaluate.py" "$1"
