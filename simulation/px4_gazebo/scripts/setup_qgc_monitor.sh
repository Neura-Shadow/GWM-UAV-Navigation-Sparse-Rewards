#!/usr/bin/env bash
# Optional real GCS monitor needed by the unchanged PX4 x500 data-link check.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
if [[ ${1:-} != --apply ]]; then
  echo 'Plan: install official QGroundControl v5.1.4 AppImage beneath Linux simulation root; no launch.'
  [[ $# == 0 || ${1:-} == --check ]] || fail 'Usage: setup_qgc_monitor.sh [--check|--apply]'
  exit 0
fi
[[ $# == 1 ]] || fail 'Unexpected arguments'
clean_linux_env
start_log qgc-setup
qgc_dir=$SIM_ROOT/qgc-v5.1.4
mkdir -p "$qgc_dir"
app=$qgc_dir/QGroundControl-x86_64.AppImage
expected=1c4ac089abfaac6c6fcd75c7b477ea18da1bc3592cddca5ab1a19c1a13410e65
if [[ ! -f $app ]]; then
  curl --fail --location --max-time 600 https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.4/QGroundControl-x86_64.AppImage -o "$app.partial"
  printf '%s  %s\n' "$expected" "$app.partial" | sha256sum --check
  mv -- "$app.partial" "$app"
fi
printf '%s  %s\n' "$expected" "$app" | sha256sum --check
chmod u+x "$app"
if [[ ! -f $qgc_dir/extraction-completed.txt ]]; then
  [[ ! -e $qgc_dir/squashfs-root ]] || fail 'Interrupted extraction retained; inspect before resuming'
  cd "$qgc_dir"
  "$app" --appimage-extract > "$STEP_RUN/extraction.txt"
  printf '%s\n' "$expected" > "$qgc_dir/extraction-completed.txt"
fi
sha256sum "$qgc_dir/squashfs-root/AppRun" > "$STEP_RUN/apprun-sha256.txt"
echo 'QGC extracted; no GCS/runtime launched.'
