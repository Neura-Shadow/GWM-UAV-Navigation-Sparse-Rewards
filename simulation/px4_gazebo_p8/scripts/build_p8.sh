#!/usr/bin/env bash
# Build the P8 recovery package mirror (mirrors build_p4.sh). Never launches a simulator or flight.
# The build is a P8-M3 step: it needs its own explicit operator authorization (contract §13).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
[[ $# == 1 && $1 == --build ]] || { echo 'Usage: build_p8.sh --build; no runtime launched'; exit 0; }
clean_linux_env
start_log p8-build
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json p4-built.json p5-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1
/usr/bin/python3 "$(dirname "${BASH_SOURCE[0]}")/p8_build.py" --build
