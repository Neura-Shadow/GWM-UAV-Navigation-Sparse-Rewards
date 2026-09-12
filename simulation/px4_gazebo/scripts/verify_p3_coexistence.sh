#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
[[ $# == 1 && -f $1/summary.json ]] || { echo 'Offline only: existing run required; nothing launched'; exit 0; }
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 "$SIM_DIR/validation/collect_p3_coexistence.py" "$1"
