#!/usr/bin/env bash
# Runs a benchmark script under the P5 controller interpreter and environment (as the ROS harness tests do):
# /usr/bin/python3, NumPy 1.26.4, OPENBLAS_NUM_THREADS=1. No simulator, no ROS graph.
# Usage: run_controller_python.sh SCRIPT.py [args...]
R=/mnt/d/GWM-UAV-Navigation-Sparse-Rewards
source $R/simulation/px4_gazebo/scripts/common.sh
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash; source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
here=$(cd "$(dirname "$0")" && pwd)
script=$1; shift
cd "$here" && exec /usr/bin/python3 "$here/$script" "$@"
