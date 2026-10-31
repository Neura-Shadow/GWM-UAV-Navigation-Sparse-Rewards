#!/usr/bin/env bash
# Runs a benchmark script in the existing WSL conda env `thesis` (torch + CUDA), isolated from the controller
# environment. Nothing is installed. Usage: run_gpu_python.sh SCRIPT.py [args...]
set -u
here=$(cd "$(dirname "$0")" && pwd)
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
script=$1; shift
cd "$here" && exec "$HOME/miniconda3/envs/thesis/bin/python" "$here/$script" "$@"
