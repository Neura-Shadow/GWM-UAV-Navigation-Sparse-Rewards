#!/usr/bin/env bash
# Offline only: independent P4 evaluation of one finalized run directory.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
P4_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: verify_p4_evidence.sh ABSOLUTE_P4_RUN_DIRECTORY'
  exit 0
fi
[[ $# == 1 && -f $1/summary.json ]] || fail 'One existing P4 run required'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json p4-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1
exec "$SIM_ROOT/venv-px4/bin/python" "$P4_DIR/validation/p4_evaluate.py" "$1"
