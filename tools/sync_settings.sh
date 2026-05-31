#!/usr/bin/env bash
set -euo pipefail

# Sync AirSim settings.json from this project to Windows user profile.

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_JSON="$PROJECT_DIR/settings.json"

# Allow override via env var.
if [[ -n "${AIRSIM_SETTINGS_WIN:-}" ]]; then
  WIN_JSON="$AIRSIM_SETTINGS_WIN"
else
  # Try common AirSim settings locations (OneDrive CN/EN, then local Documents).
  CANDIDATES=(
    "/mnt/c/Users/zongx/OneDrive/文件/AirSim/settings.json"
    "/mnt/c/Users/zongx/OneDrive/Documents/AirSim/settings.json"
    "/mnt/c/Users/zongx/Documents/AirSim/settings.json"
  )
  WIN_JSON="${CANDIDATES[0]}"
  for path in "${CANDIDATES[@]}"; do
    if [[ -f "$path" || -d "$(dirname "$path")" ]]; then
      WIN_JSON="$path"
      break
    fi
  done
fi

if [[ ! -f "$SRC_JSON" ]]; then
  echo "Missing $SRC_JSON"
  echo "Create it in the project folder first, then rerun this script."
  exit 1
fi

mkdir -p "$(dirname "$WIN_JSON")"

cp "$SRC_JSON" "$WIN_JSON"

echo "Synced to: $WIN_JSON"
