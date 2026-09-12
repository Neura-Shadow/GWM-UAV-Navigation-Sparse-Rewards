#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: verify_p2_evidence.sh ABSOLUTE_P2_RUN_DIRECTORY'
  exit 0
fi
[[ $# -ge 1 && -f $1/summary.json ]] || fail 'One existing P2 run required'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec "$SIM_ROOT/venv-px4/bin/python" "$SIM_DIR/validation/collect_p2_evidence.py" "$@"
