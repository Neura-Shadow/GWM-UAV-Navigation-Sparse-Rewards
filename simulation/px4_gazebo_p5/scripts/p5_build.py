"""One-way content-addressed build of the P5 package; never launches runtime.

Mirrors `gwm_policy_control` into ~/uav_autonomy/p5_ws/<package-hash>, runs colcon build and its
installed tests (P2/P3/P4 installs sourced by build_p5.sh), and writes state/p5-built.json. The
P4 build pattern, unchanged.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

P5 = Path(__file__).resolve().parents[1]
PACKAGE = P5 / "ros2_ws/src/gwm_policy_control"
CONTRACT = "p5-policy-build-v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def package_manifest():
    return {p.relative_to(PACKAGE).as_posix(): sha(p) for p in sorted(PACKAGE.rglob("*"))
            if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}


def package_hash(manifest):
    return hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()


def build():
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy"))).resolve()
    if Path.home().resolve() not in root.parents:
        raise ValueError("Linux HOME workspace required")
    manifest = package_manifest()
    identity = package_hash(manifest)
    workspace = root / "p5_ws" / identity
    source = workspace / "src/gwm_policy_control"
    if source.exists():
        for name, value in manifest.items():
            if sha(source / name) != value:
                raise ValueError("Existing build mirror modified: " + name)
    else:
        for name in manifest:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PACKAGE / name, destination)
    for command in (["colcon", "build", "--packages-select", "gwm_policy_control", "--executor", "sequential"],
                    ["colcon", "test", "--packages-select", "gwm_policy_control", "--event-handlers", "console_direct+"],
                    ["colcon", "test-result", "--verbose"]):
        subprocess.run(command, cwd=workspace, check=True)
    installed = next((workspace / "install").rglob("site-packages/gwm_policy_control/__init__.py")).parent
    receipt = {"contract": CONTRACT, "package_hash": identity, "source_files": manifest,
               "workspace": str(workspace), "install": str(workspace / "install"),
               "installed_source_files": {p.relative_to(installed).as_posix(): sha(p)
                                          for p in sorted(installed.rglob("*.py"))}}
    (root / "state/p5-built.json").write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print("P5 policy package built/tested: " + identity, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    if parser.parse_args().build:
        build()
    else:
        print("No build/runtime action. Use build_p5.sh --build.")
