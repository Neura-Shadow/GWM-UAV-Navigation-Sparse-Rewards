#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ $# == 0 || ${1:-} == --help ]]; then
  echo 'No runtime started. Use --run --observe or --run --allow-simulated-flight, optionally --headless.'
  exit 0
fi
[[ $1 == --run ]] || fail 'Explicit --run required'
flight=0
observe=0
for arg in "${@:2}"; do
  case "$arg" in --allow-simulated-flight) flight=1;; --observe) observe=1;; --headless|--diagnostic) ;; *) fail "Unknown argument: $arg";; esac
done
[[ $((flight+observe)) == 1 ]] || fail 'Choose exactly one of observation and flight'
for gate in GWM_ALLOW_OPTIONAL_RUNTIME GWM_RUN_GAZEBO_PX4_TESTS GWM_ALLOW_PX4_LAUNCH; do
  [[ ${!gate:-} == 1 ]] || fail "Missing $gate=1"
done
[[ $flight == 0 || ${GWM_ALLOW_SITL_COMMANDS:-} == 1 ]] || fail 'Missing GWM_ALLOW_SITL_COMMANDS=1'
check_root
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
verify_checkout "$AGENT_DIR" dds_agent
[[ -f $SIM_ROOT/state/p2-built.json ]] || fail 'Run build_p2.sh --build first'
exec 9> "$SIM_ROOT/state/p1.lock"
flock -n 9 || fail 'P1 or P2 validation already owns this workspace'
clean_linux_env
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
p2_install=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["install"])' "$SIM_ROOT/state/p2-built.json")
source "$p2_install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
exec unshare --user --map-current-user --keep-caps --net --mount --pid --fork --mount-proc \
  bash -c 'ip link set lo up; ip link set lo multicast on; exec /usr/bin/python3 "$@"' bash \
  "$SIM_DIR/scripts/p2_runner.py" "$@"
