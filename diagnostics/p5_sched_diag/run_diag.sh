#!/usr/bin/env bash
# One loaded-SITL scheduling-diagnostic flight inside a fresh private namespace (operator approval 2).
# Mirrors simulation/px4_gazebo_p5/scripts/run_p5.sh exactly, except that it execs the byte-identical owner
# copy in this directory (which dispatches to diag_runner.py). Never an acceptance run; no gate files.
set -euo pipefail
R=/mnt/d/GWM-UAV-Navigation-Sparse-Rewards
source "$R/simulation/px4_gazebo/scripts/common.sh"
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
[[ ${1:-} == --run ]] || fail 'Explicit --run required'
shift
for gate in GWM_ALLOW_OPTIONAL_RUNTIME GWM_RUN_GAZEBO_PX4_TESTS GWM_ALLOW_PX4_LAUNCH GWM_ALLOW_SITL_COMMANDS; do
  [[ ${!gate:-} == 1 ]] || fail "Missing $gate=1"
done
check_root
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
verify_checkout "$AGENT_DIR" dds_agent
for receipt in p2-built.json p3-built.json p4-built.json p5-built.json px4-mc-trace-built.json qgc-unattended-built.json p3-dds-agent-built.json; do
  [[ -f $SIM_ROOT/state/$receipt ]] || fail "Missing build receipt: $receipt"
done
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'P1, P2, P3, P4 or P5 validation already owns this workspace'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
for receipt in p2-built.json p3-built.json p4-built.json p5-built.json; do
  install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/$receipt")
  source "$install/setup.bash"
done
set -u
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; ip link set lo multicast on; exec /usr/bin/python3 "$@"' bash \
  "$HERE/p5_namespace_owner.py" "$@"
