#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
[[ ${1:-} == --build && $# == 1 ]] || { echo 'Usage: build_baseline.sh --build (compiles; never launches SITL)'; exit 0; }
start_log build
verify_checkout "$PX4_DIR" px4
verify_checkout "$ROS_WS/src/px4_msgs" px4_msgs
verify_checkout "$AGENT_DIR" dds_agent
clean_linux_env
[[ -f /opt/ros/jazzy/setup.bash ]] || fail 'ROS Jazzy is not installed'
set +u
source /opt/ros/jazzy/setup.bash
set -u
export GZ_DISTRO=harmonic
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
gz sim --versions
[[ $(gz sim --versions) == 8.* ]] || fail 'Expected Gazebo Sim major 8 (Harmonic)'
cd "$ROS_WS"
/usr/bin/colcon build --packages-select px4_msgs --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
printf 'px4_msgs=0\n' >> "$STEP_RUN/steps.txt"
agent_build=$AGENT_DIR/build-jazzy-upstream-logger
cmake -S "$AGENT_DIR" -B "$agent_build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SIM_ROOT/dds-install" \
  -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=OFF
cmake --build "$agent_build" --parallel 2
cmake --install "$agent_build"
agent_help_exit=0
LD_LIBRARY_PATH="$SIM_ROOT/dds-install/lib:${LD_LIBRARY_PATH:-}" "$SIM_ROOT/dds-install/bin/MicroXRCEAgent" --help > "$STEP_RUN/dds-help.txt" 2>&1 || agent_help_exit=$?
printf 'dds_help_exit=%s\n' "$agent_help_exit" >> "$STEP_RUN/steps.txt"
# v2.4.3 AgentInstance::create(HELP) returns false; main returns 1 without
# running a transport. Check this documented CLI behavior, not a server start.
if [[ $agent_help_exit != 1 ]] || ! grep -aq "Usage: 'MicroXRCEAgent" "$STEP_RUN/dds-help.txt"; then
  fail 'DDS Agent help probe failed'
fi
printf 'dds_agent=0\n' >> "$STEP_RUN/steps.txt"
source "$SIM_ROOT/venv-px4/bin/activate"
cd "$PX4_DIR"
make -j2 px4_sitl BUILD_DIR_SUFFIX=_linux
# Build the Gazebo plugins, without executing the launch target.
cmake --build "$PX4_BUILD" --target px4_gz_plugins --parallel 2
cmake --build "$PX4_BUILD" --target help > "$STEP_RUN/build-targets.txt"
grep -q 'gz_x500:' "$STEP_RUN/build-targets.txt" || fail 'gz_x500 target missing'
printf 'px4_sitl=0\ngazebo_plugins=0\n' >> "$STEP_RUN/steps.txt"
/usr/bin/python3 "$SIM_DIR/scripts/record_versions.py" --output "$STEP_RUN/versions.resolved.json"
mkdir -p "$SIM_ROOT/state"
cp "$STEP_RUN/versions.resolved.json" "$SIM_ROOT/state/p0-built.json"
echo 'P0 build checks passed; no simulator or flight was executed.'
