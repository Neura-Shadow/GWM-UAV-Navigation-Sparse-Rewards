#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: existing run plus optional historical report arguments'; exit 0
fi
[[ $# -ge 1 && -f $1/summary.json ]] || fail 'One existing run required'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 "$SIM_DIR/validation/collect_p3_coexistence.py" "$@"
