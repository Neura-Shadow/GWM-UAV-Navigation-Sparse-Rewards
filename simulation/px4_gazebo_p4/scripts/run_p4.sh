#!/usr/bin/env bash
# One P4 runtime scope inside a fresh private namespace. Invoked by run_p4_gates.py only.
# Without --run nothing starts. Flight scopes additionally need GWM_ALLOW_SITL_COMMANDS=1 and
# --allow-simulated-flight. The gate sequencer consumes the authorization first.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../px4_gazebo/scripts/common.sh"
P4_SCRIPTS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'No runtime started. Use run_p4_gates.py; it invokes: --run --scope ... --gate A|B|S|C ...'
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
for receipt in p2-built.json p3-built.json p4-built.json px4-mc-trace-built.json qgc-unattended-built.json p3-dds-agent-built.json; do
  [[ -f $SIM_ROOT/state/$receipt ]] || fail "Missing build receipt: $receipt"
done
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'P1, P2, P3 or P4 validation already owns this workspace'
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
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; ip link set lo multicast on; exec /usr/bin/python3 "$@"' bash \
  "$P4_SCRIPTS/p4_namespace_owner.py" "$@"
