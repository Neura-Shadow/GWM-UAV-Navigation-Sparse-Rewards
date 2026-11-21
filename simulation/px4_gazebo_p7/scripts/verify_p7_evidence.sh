#!/usr/bin/env bash
# Offline only: the P7 classification of one finalized P7 run, after the unchanged P4 or P5 evaluation.
# It runs in the accepted evaluators' environment (venv-px4 holds pyulog for the ULog arming check).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
P7_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: verify_p7_evidence.sh ABSOLUTE_P7_RUN_DIRECTORY'
  exit 0
fi
[[ $# == 1 && -f $1/summary.json ]] || fail 'One existing run required'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json p4-built.json p5-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1
exec "$SIM_ROOT/venv-px4/bin/python" "$P7_DIR/validation/p7_classify.py" "$1"
