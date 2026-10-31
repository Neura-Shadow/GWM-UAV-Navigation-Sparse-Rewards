#!/usr/bin/env bash
# Read-only, allowlisted host inventory. No runtime launch or installation.
set -euo pipefail
source /etc/os-release
printf 'OS=%s VERSION_ID=%s\n' "$ID" "$VERSION_ID"
id
printf 'HOME=%s\nDISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "$HOME" "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}"
df -h "$HOME"
free -h
for tool in gcc g++ cmake ninja git python3 colcon gz; do
  command -v "$tool" || true
done
/usr/bin/python3 --version
ls /opt/ros 2>/dev/null || true
dpkg-query -W -f='${binary:Package}\t${Version}\t${db:Status-Status}\n' 'ros-*-ros-base' 'ros-*-ros-gz*' 'gz-*' 2>/dev/null || true
grep -RhsE '^(deb |URIs:|Suites:|Components:)' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true
find "$HOME" -maxdepth 5 -type d \( -name PX4-Autopilot -o -name px4_msgs -o -name Micro-XRCE-DDS-Agent \) -print 2>/dev/null || true
# shellcheck disable=SC2009 # Include ownership columns in this read-only inventory.
ps -eo pid,ppid,user,comm | grep -Ei 'px4|gazebo|MicroXRCE|mavsdk|QGround|ros2|gz sim' || true
ss -lunp
if sudo -n true 2>/dev/null; then
  echo 'sudo_noninteractive=available'
else
  echo 'sudo_noninteractive=requires_operator_authentication'
fi
