#!/usr/bin/env bash
# Build/test the P4 avoidance package mirror. Never launches a simulator or flight.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
[[ $# == 1 && $1 == --build ]] || { echo 'Usage: build_p4.sh --build; no runtime launched'; exit 0; }
clean_linux_env
start_log p4-build
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1
/usr/bin/python3 "$(dirname "${BASH_SOURCE[0]}")/p4_build.py" --build
