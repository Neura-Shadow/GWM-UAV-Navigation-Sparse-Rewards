#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
[[ $# == 1 && $1 == --build ]] || { echo 'Usage: build_p3.sh --build; no runtime launched'; exit 0; }
clean_linux_env
start_log p3-build
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
/usr/bin/python3 "$SIM_DIR/scripts/p3_build.py" --build
