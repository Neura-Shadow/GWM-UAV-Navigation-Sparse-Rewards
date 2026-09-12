#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'No runtime started. Use --run --case CASE --readiness-run ABSOLUTE_FINAL_READINESS.'
  exit 0
fi
[[ $# == 5 && $1 == --run && $2 == --case && $4 == --readiness-run && -f $5/runtime-finalized.json ]] || fail 'Explicit --run --case and final readiness required'
case "$3" in plane2|plane4|plane6|oblique|asymmetric|out_of_range|interruption) ;; *) fail 'Unknown ground case';; esac
for gate in GWM_ALLOW_OPTIONAL_RUNTIME GWM_RUN_GAZEBO_PX4_TESTS GWM_ALLOW_PX4_LAUNCH; do
  [[ ${!gate:-} == 1 ]] || fail "Missing $gate=1"
done
check_root
verify_checkout "$PX4_DIR" px4
[[ -f $SIM_ROOT/state/p3-built.json ]] || fail 'Run build_p3.sh --build first'
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'Simulation workspace already owned'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
p2_install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/p2-built.json")
source "$p2_install/setup.bash"
p3_install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/p3-built.json")
source "$p3_install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; ip link set lo multicast on; exec /usr/bin/python3 "$@"' bash \
  "$SIM_DIR/scripts/p3_runner.py" "$@"
