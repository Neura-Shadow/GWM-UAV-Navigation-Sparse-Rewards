#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'No runtime launched. Usage: run_p1_baseline.sh --run [--allow-simulated-flight] [--headless] [--qgc-monitor]'
  echo 'Requires GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 GWM_ALLOW_PX4_LAUNCH=1'
  echo 'Flight also requires GWM_ALLOW_SITL_COMMANDS=1. Default trial is GUI boot-only.'
  exit 0
fi
[[ $1 == --run ]] || fail 'Explicit --run required'
flight=0
for arg in "${@:2}"; do
  case "$arg" in --allow-simulated-flight) flight=1;; --headless|--qgc-monitor) ;; *) fail "Unknown argument: $arg";; esac
done
for gate in GWM_ALLOW_OPTIONAL_RUNTIME GWM_RUN_GAZEBO_PX4_TESTS GWM_ALLOW_PX4_LAUNCH; do
  [[ ${!gate:-} == 1 ]] || fail "Missing $gate=1"
done
[[ $flight == 0 || ${GWM_ALLOW_SITL_COMMANDS:-} == 1 ]] || fail 'Missing GWM_ALLOW_SITL_COMMANDS=1'
check_root
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
verify_checkout "$AGENT_DIR" dds_agent
[[ -f $SIM_ROOT/state/p0-built.json ]] || fail 'Complete build_baseline.sh --build first'
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'Another validation session owns this simulation workspace'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
set -u
# No external interfaces exist in this namespace. Its PID 1 owns all trial
# descendants; exiting PID 1 also reaps orphaned simulator processes.
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; exec /usr/bin/python3 "$@"' bash \
  "$SIM_DIR/scripts/p1_runner.py" "$@"
