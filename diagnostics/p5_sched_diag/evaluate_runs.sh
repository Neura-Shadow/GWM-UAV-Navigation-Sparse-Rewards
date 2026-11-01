#!/usr/bin/env bash
# Read-only: decision-window guest load and the standard P5 evaluator (in its normal environment, as
# verify_p5_evidence.sh sets it up) for every scheduling-diagnostic run. Writes nothing into gate state.
R=/mnt/d/GWM-UAV-Navigation-Sparse-Rewards
HERE=$R/diagnostics/p5_sched_diag
source $R/simulation/px4_gazebo/scripts/common.sh
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json p4-built.json p5-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONDONTWRITEBYTECODE=1
cd /tmp
for run in "$SIM_ROOT"/runs/*-p5diag-flight-*; do
  echo "== $(basename "$run")"
  timeout 900 "$SIM_ROOT/venv-px4/bin/python" "$HERE/evaluate_one.py" "$run" 2>&1 | tail -4
done
echo "== evaluation done"
