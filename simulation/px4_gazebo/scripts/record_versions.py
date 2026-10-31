"""Collect local reproducibility inputs; does not execute a simulator."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def command(args, cwd=None):
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True, check=False)
    return {"exit_code": result.returncode, "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip()}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collect(root, sim):
    lock = json.loads((sim / "configs/versions.lock.yaml").read_text())
    paths = {"px4": root / "upstream/PX4-Autopilot",
             "px4_msgs": root / "ros_ws/src/px4_msgs",
             "dds_agent": root / "upstream/Micro-XRCE-DDS-Agent"}
    measured = {"sources": {}, "asset_hashes": {}, "workspace": str(root),
                "project_commit": command(["git", "rev-parse", "HEAD"], sim),
                "lock_sha256": sha256(sim / "configs/versions.lock.yaml")}
    measured["px4_binary_sha256"] = sha256(paths["px4"] / "build/px4_sitl_default_linux/bin/px4")
    for name, path in paths.items():
        measured["sources"][name] = {
            "head": command(["git", "rev-parse", "HEAD"], path),
            "submodules": command(["git", "submodule", "status", "--recursive"], path),
            "status": command(["git", "status", "--porcelain"], path)}
    measured["dds_build_dependencies"] = {}
    for git_dir in sorted((paths["dds_agent"] / "build-jazzy-upstream-logger").rglob(".git")):
        checkout = git_dir.parent
        measured["dds_build_dependencies"][str(checkout.relative_to(paths["dds_agent"]))] = {
            "head": command(["git", "rev-parse", "HEAD"], checkout),
            "origin": command(["git", "remote", "get-url", "origin"], checkout)}
    px4 = paths["px4"]
    measured["px4_build_dependencies"] = {}
    for git_dir in sorted((px4 / "build/px4_sitl_default_linux/OpticalFlow").rglob(".git")):
        checkout = git_dir.parent
        measured["px4_build_dependencies"][str(checkout.relative_to(px4))] = {
            "head": command(["git", "rev-parse", "HEAD"], checkout),
            "origin": command(["git", "remote", "get-url", "origin"], checkout),
            "status": command(["git", "status", "--porcelain"], checkout),
            "submodules": command(["git", "submodule", "status", "--recursive"], checkout)}
    lock["reproducibility_limitations"] = [
        "PX4's pinned optical_flow.cmake requests external OpticalFlow master; actual fetched commits are recorded, not a clean-rebuild guarantee.",
        "Apt and pip package versions are recorded; setup does not use a frozen package mirror."]
    for directory in ("Tools/simulation/gz/models/x500", "Tools/simulation/gz/models/x500_base"):
        for path in sorted((px4 / directory).rglob("*")):
            if path.is_file():
                measured["asset_hashes"][str(path.relative_to(px4))] = sha256(path)
    world = px4 / "Tools/simulation/gz/worlds/default.sdf"
    if world.is_file():
        measured["asset_hashes"][str(world.relative_to(px4))] = sha256(world)
    measured["packages"] = command(["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\t${db:Status-Status}\n",
                                      "ros-jazzy-*", "gz-*", "cmake", "ninja-build", "gcc", "g++", "python3"])
    measured["tools"] = {name: command(args) for name, args in {
        "cmake": ["cmake", "--version"], "ninja": ["ninja", "--version"],
        "gcc": ["gcc", "--version"], "git": ["git", "--version"],
        "linux_python": ["/usr/bin/python3", "--version"],
        "gazebo": ["gz", "sim", "--versions"],
        "px4_python_freeze": [str(root / "venv-px4/bin/python"), "-m", "pip", "freeze"]}.items()}
    lock["measured"] = measured
    lock["unresolved_reason"] = None
    lock["clean_rebuild_proven"] = False
    return lock


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    args.output.write_text(json.dumps(collect(root, Path(__file__).resolve().parents[1]), indent=2) + "\n")
