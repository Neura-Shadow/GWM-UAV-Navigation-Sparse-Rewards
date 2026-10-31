#!/usr/bin/env bash
# Sourced helpers only; no installation or runtime action.
# shellcheck disable=SC2034 # Public variables consumed by the entrypoint scripts.
SIM_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_ROOT=${GWM_SIM_ROOT:-$HOME/uav_autonomy}
PX4_DIR=$SIM_ROOT/upstream/PX4-Autopilot
ROS_WS=$SIM_ROOT/ros_ws
AGENT_DIR=$SIM_ROOT/upstream/Micro-XRCE-DDS-Agent
PX4_BUILD=$PX4_DIR/build/px4_sitl_default_linux
fail() { printf 'BLOCKED: %s\n' "$*" >&2; exit 2; }
clean_linux_env() {
  [[ -z ${CONDA_PREFIX:-} && -z ${VIRTUAL_ENV:-} ]] || fail 'Use a clean Linux shell'
  # WSL inherits Windows PATH even without CONDA_PREFIX. CMake can otherwise
  # discover Windows Anaconda headers via that PATH, despite Linux Python.
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  unset CMAKE_PREFIX_PATH CMAKE_LIBRARY_PATH CMAKE_INCLUDE_PATH CMAKE_FRAMEWORK_PATH
  unset CPATH CPLUS_INCLUDE_PATH C_INCLUDE_PATH LIBRARY_PATH PKG_CONFIG_PATH
  unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
}
check_root() {
  [[ $SIM_ROOT = /* ]] || fail 'GWM_SIM_ROOT must be absolute'
  local resolved
  resolved=$(realpath -m -- "$SIM_ROOT")
  [[ $resolved == "$HOME/"* && $resolved != /mnt/* ]] || fail 'Use a dedicated workspace beneath Linux HOME'
  [[ $(stat -f -c %T "$(dirname "$resolved")") != 9p ]] || fail 'Build workspace must use Linux storage'
}
pin_value() {
  /usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sources"][sys.argv[2]][sys.argv[3]])' "$SIM_DIR/configs/versions.lock.yaml" "$1" "$2"
}
verify_checkout() {
  local path=$1 key=$2
  [[ -d $path/.git ]] || fail "Missing checkout: $path"
  [[ $(git -C "$path" remote get-url origin) == "$(pin_value "$key" url)" ]] || fail "Origin mismatch: $path"
  [[ $(git -C "$path" rev-parse HEAD) == "$(pin_value "$key" commit)" ]] || fail "Commit mismatch: $path"
  [[ -z $(git -C "$path" status --porcelain --untracked-files=normal) ]] || fail "Dirty checkout: $path"
  ! git -C "$path" submodule status --recursive | grep -qE '^[+U]' || fail "Submodule mismatch: $path"
}
checkout_pin() {
  local key=$1 path=$2
  if [[ ! -e $path ]]; then
    mkdir -p -- "$(dirname "$path")"
    git clone --no-checkout "$(pin_value "$key" url)" "$path"
    git -C "$path" checkout --detach "$(pin_value "$key" commit)"
  fi
  verify_checkout "$path" "$key"
  git -C "$path" submodule update --init --recursive --jobs 2
}
start_log() {
  check_root
  mkdir -p "$SIM_ROOT/runs"
  STEP_RUN=$(mktemp -d "$SIM_ROOT/runs/$(date -u +%Y%m%dT%H%M%SZ)-$1-XXXXXX")
  export STEP_RUN
  exec > >(tee "$STEP_RUN/console.log") 2>&1
  trap 'printf "%s\n" "$?" > "$STEP_RUN/exit-code.txt"' EXIT
  printf 'Evidence: %s\n' "$STEP_RUN"
}
