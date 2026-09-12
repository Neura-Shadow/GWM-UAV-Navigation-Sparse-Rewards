#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'No runtime started. Readiness: --run --observe. Flight: --run --allow-simulated-flight --ground-matrix ABSOLUTE_PASSED_MATRIX.'
  exit 0
fi
[[ ${1:-} == --run ]] || fail 'Explicit P3 --run required'
case "${2:-}" in
  --observe) [[ $# == 2 ]] || fail 'Readiness precedes ground qualification';;
  --allow-simulated-flight)
    [[ $# == 4 && $3 == --ground-matrix && -f $4/summary.json ]] || fail 'Exact P3 flight arguments required'
    [[ ${GWM_ALLOW_SITL_COMMANDS:-} == 1 ]] || fail 'Missing flight gate';;
  *) fail 'Unknown P3 purpose';;
esac
for gate in GWM_ALLOW_OPTIONAL_RUNTIME GWM_RUN_GAZEBO_PX4_TESTS GWM_ALLOW_PX4_LAUNCH; do
  [[ ${!gate:-} == 1 ]] || fail "Missing $gate=1"
done
check_root
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
verify_checkout "$AGENT_DIR" dds_agent
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'Simulation workspace already owned'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
p2_install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/p2-built.json")
p3_install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/p3-built.json")
source "$p2_install/setup.bash"
source "$p3_install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; ip link set lo multicast on; exec /usr/bin/python3 "$@"' bash \
  "$SIM_DIR/scripts/p3_coexistence.py" "$@"
