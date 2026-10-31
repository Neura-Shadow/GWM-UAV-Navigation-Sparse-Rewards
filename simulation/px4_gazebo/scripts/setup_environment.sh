#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ ${1:-} != --apply ]]; then
  [[ $# == 0 || ${1:-} == --check ]] || fail 'Usage: setup_environment.sh [--check|--apply]'
  echo 'Plan: pinned PX4 v1.17.0, px4_msgs release/1.17, DDS Agent v2.4.3; Jazzy/Harmonic vendor packages.'
  echo 'Apply uses sudo package management, a Linux build venv, and official Git sources. No runtime launch.'
  exit 0
fi
[[ $# == 1 ]] || fail 'Unexpected arguments'
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 24.04 ]] || fail 'Requires Ubuntu 24.04'
clean_linux_env
start_log setup
bash "$SIM_DIR/scripts/check_environment.sh"
sudo -n true || fail 'Authenticate sudo in your own WSL terminal, then rerun; never provide a password in chat'
# Reject alternative Gazebo providers rather than removing their packages.
if grep -Rqs 'packages.osrfoundation.org' /etc/apt/sources.list.d; then
  fail 'Existing OSRF source requires explicit provider reconciliation'
fi
checkout_pin px4 "$PX4_DIR"
checkout_pin px4_msgs "$ROS_WS/src/px4_msgs"
checkout_pin dds_agent "$AGENT_DIR"
# This dedicated generated build directory is owned by this lane. Preserve
# upstream tracked ignore rules and never mask pre-existing dirty source work.
if ! grep -qxF '/build-jazzy-upstream-logger/' "$AGENT_DIR/.git/info/exclude"; then
  printf '\n/build-jazzy-upstream-logger/\n' >> "$AGENT_DIR/.git/info/exclude"
fi
mkdir -p "$SIM_ROOT/setup" "$SIM_ROOT/state"
sudo apt-get update
sudo apt-get install -y --no-remove --no-install-recommends python3-venv curl ca-certificates gnupg
key_commit=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["ros_key_source_commit"])' "$SIM_DIR/configs/versions.lock.yaml")
curl --fail --location --max-time 60 "https://raw.githubusercontent.com/ros/rosdistro/$key_commit/ros.key" -o "$SIM_ROOT/setup/ros.key"
gpg --show-keys --with-fingerprint "$SIM_ROOT/setup/ros.key"
key_dest=/usr/share/keyrings/gwm-ros-archive-keyring.gpg
if [[ -e $key_dest ]]; then
  cmp "$SIM_ROOT/setup/ros.key" "$key_dest" || fail 'Existing ROS key differs'
else
  sudo install -m 644 "$SIM_ROOT/setup/ros.key" "$key_dest"
fi
printf 'deb [arch=%s signed-by=%s] http://packages.ros.org/ros2/ubuntu noble main\n' "$(dpkg --print-architecture)" "$key_dest" > "$SIM_ROOT/setup/gwm-ros2.list"
source_dest=/etc/apt/sources.list.d/gwm-ros2.list
if [[ -e $source_dest ]]; then
  cmp "$SIM_ROOT/setup/gwm-ros2.list" "$source_dest" || fail 'Existing source differs'
else
  if grep -Rqs 'packages.ros.org/ros2/ubuntu' /etc/apt/sources.list.d; then fail 'Existing ROS source needs reconciliation'; fi
  sudo install -m 644 "$SIM_ROOT/setup/gwm-ros2.list" "$source_dest"
fi
sudo apt-get update
sudo apt-get install -y --no-remove --no-install-recommends ros-jazzy-ros-base ros-jazzy-ros-gz python3-colcon-common-extensions libeigen3-dev libunwind-dev cppzmq-dev protobuf-compiler pkg-config libspdlog-dev bc
/usr/bin/python3 -m venv "$SIM_ROOT/venv-px4"
# Inspectable minimal adaptation: remove the protection override, preserve upstream.
/usr/bin/python3 - "$PX4_DIR" "$SIM_ROOT/setup" <<'PY'
from pathlib import Path
import sys
src = Path(sys.argv[1]) / 'Tools/setup'
dst = Path(sys.argv[2])
script = (src / 'ubuntu.sh').read_text()
assert script.count('--break-system-packages') == 1
script = script.replace('--break-system-packages ', '')
script = script.replace('apt-get -y --quiet --no-install-recommends install', 'apt-get -y --quiet --no-remove --no-install-recommends install')
(dst / 'ubuntu-venv.sh').write_text(script)
(dst / 'requirements.txt').write_bytes((src / 'requirements.txt').read_bytes())
PY
diff -u "$PX4_DIR/Tools/setup/ubuntu.sh" "$SIM_ROOT/setup/ubuntu-venv.sh" > "$STEP_RUN/installer-adaptation.diff" || [[ $? == 1 ]]
source "$SIM_ROOT/venv-px4/bin/activate"
bash "$SIM_ROOT/setup/ubuntu-venv.sh" --no-nuttx --no-sim-tools
python -m pip freeze > "$STEP_RUN/px4-python-freeze.txt"
sha256sum "$PX4_DIR/Tools/setup/ubuntu.sh" "$SIM_ROOT/setup/ubuntu-venv.sh" > "$STEP_RUN/installer-hashes.txt"
printf '%s\n' "$STEP_RUN" >> "$SIM_ROOT/state/setup-completed.txt"
echo 'Setup completed. Build verification is still required for P0.'
