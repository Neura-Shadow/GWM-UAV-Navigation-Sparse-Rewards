#!/usr/bin/env bash
# Offline only: the unchanged P5 evaluation of one finalized P6 run (verify_p5_evidence.sh), then the
# independent P6 evaluation, which composes its result (contract §8, C8). Never launches anything.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
P6_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'Offline only: verify_p6_evidence.sh ABSOLUTE_P6_RUN_DIRECTORY'
  exit 0
fi
[[ $# == 1 && -f $1/summary.json ]] || fail 'One existing P6 run required'
if [[ ! -f $1/p5-evaluation.json ]]; then
  # The P5 evaluator exits 1 for a failed run; the P6 evaluator decides which of its failures a row allows.
  bash "$P6_DIR/../px4_gazebo_p5/scripts/verify_p5_evidence.sh" "$1" || true
fi
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json p4-built.json p5-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
exec /usr/bin/python3 "$P6_DIR/validation/p6_evaluate.py" "$1"
