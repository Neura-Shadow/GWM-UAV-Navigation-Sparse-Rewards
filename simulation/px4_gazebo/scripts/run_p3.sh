#!/usr/bin/env bash
# One P3 runtime scope inside a fresh private namespace. Invoked by run_p3_gates.py.
# Without --run nothing starts. Flight additionally needs GWM_ALLOW_SITL_COMMANDS=1
# and --allow-simulated-flight. The gate sequencer consumes the authorization first.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'No runtime started. Use run_p3_gates.py; it invokes: --run --scope readiness|fixture|flight ...'
  exit 0
fi
[[ $1 == --run ]] || fail 'Explicit --run required'
for gate in GWM_ALLOW_OPTIONAL_RUNTIME GWM_RUN_GAZEBO_PX4_TESTS GWM_ALLOW_PX4_LAUNCH; do
  [[ ${!gate:-} == 1 ]] || fail "Missing $gate=1"
done
check_root
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
verify_checkout "$AGENT_DIR" dds_agent
for receipt in p2-built.json p3-built.json px4-mc-trace-built.json qgc-unattended-built.json; do
  [[ -f $SIM_ROOT/state/$receipt ]] || fail "Missing build receipt: $receipt"
done
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'P1, P2 or P3 validation already owns this workspace'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; ip link set lo multicast on; exec /usr/bin/python3 "$@"' bash \
  "$SIM_DIR/scripts/p3_namespace_owner.py" "$@"
