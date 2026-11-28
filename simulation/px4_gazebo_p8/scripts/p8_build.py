"""One-way content-addressed build of the P8 recovery package; never launches runtime (mirrors `p4_build.py`).

* Mirrors `gwm_navigation_recovery` into ~/uav_autonomy/p8_ws/<package-hash>.
* Runs colcon build (the P2-P5 installs are sourced by `build_p8.sh`).
* Writes state/p8-built.json, whose `package_hash` the P8 manifest compares with the package source.

The build is a P8-M3 step and needs its own operator authorization (contract §13).
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

P8 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(P8 / "scripts"))
import p8_manifest  # noqa: E402


def build():
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy"))).resolve()
    if Path.home().resolve() not in root.parents:
        raise ValueError("Linux HOME workspace required")
    manifest = p8_manifest.package_manifest()
    identity = p8_manifest.package_hash(manifest)
    workspace = root / "p8_ws" / identity
    source = workspace / "src/gwm_navigation_recovery"
    if source.exists():
        for name, value in manifest.items():
            if p8_manifest.digest(source / name) != value:
                raise ValueError("Existing build mirror modified: " + name)
    else:
        for name in manifest:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p8_manifest.PACKAGE / name, destination)
    subprocess.run(["colcon", "build", "--packages-select", "gwm_navigation_recovery", "--executor", "sequential"],
                   cwd=workspace, check=True)
    installed = next((workspace / "install").rglob("site-packages/gwm_navigation_recovery/__init__.py")).parent
    receipt = {"contract": "p8-recovery-build-v1", "package_hash": identity, "source_files": manifest,
               "workspace": str(workspace), "install": str(workspace / "install"),
               "installed_source_files": {p.relative_to(installed).as_posix(): p8_manifest.digest(p)
                                          for p in sorted(installed.rglob("*.py"))}}
    (root / p8_manifest.BUILD_RECEIPT).write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print("P8 recovery package built: " + identity, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    if parser.parse_args().build:
        build()
    else:
        print("No build/runtime action. Use build_p8.sh --build (a P8-M3 step, separately authorized).")
