#!/usr/bin/env bash
# Offline only: independent P5 evaluation of one finalized run directory. The policy-ledger replay
# runs inside it under the controller runtime interpreter (see p5_evaluate.py).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
P5_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: verify_p5_evidence.sh ABSOLUTE_P5_RUN_DIRECTORY'
  exit 0
fi
[[ $# == 1 && -f $1/summary.json ]] || fail 'One existing P5 run required'
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
exec "$SIM_ROOT/venv-px4/bin/python" "$P5_DIR/validation/p5_evaluate.py" "$1"
