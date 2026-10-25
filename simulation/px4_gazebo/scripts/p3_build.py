"""One-way content-addressed build of the P3 sensor package; never launches runtime.

Also re-derives the optical-to-base extrinsic from the pinned x500_depth SDF
include chain and refuses to build when it differs from p3_sensors.yaml.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

SIM = Path(__file__).resolve().parents[1]
PACKAGE = SIM / "ros2_ws/src/gwm_sensor_adapter"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def package_manifest():
    return {p.relative_to(PACKAGE).as_posix(): sha(p) for p in sorted(PACKAGE.rglob("*"))
            if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}


def package_hash(manifest):
    return hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()


def model_loader(models):
    def load(uri):
        name = uri.replace("model://", "").strip().strip("/")
        root = ET.parse(Path(models) / name / "model.sdf").getroot()
        model = root.find("model") if root.tag == "sdf" else root
        if model is None:
            raise ValueError("No model element in " + name)
        return model
    return load


def verify_extrinsic(px4):
    sys.path.insert(0, str(PACKAGE))
    from gwm_sensor_adapter.geometry import sdf_chain
    config = json.loads((SIM / "configs/p3_sensors.yaml").read_text())
    chain = sdf_chain(model_loader(px4 / "Tools/simulation/gz/models"), "x500_depth", "base_link",
                      config["depth_sensor_name"])
    chain["matches_config"] = (all(abs(a - b) < 1e-9 for a, b in zip(chain["optical_origin_base_flu_m"],
                                                                     config["optical_origin_base_flu_m"]))
                               and all(abs(a - b) < 1e-9 for a, b in zip(chain["sensor_origin_model_m"],
                                                                         config["optical_origin_model_m"]))
                               and chain["all_rotations_identity"])
    if not chain["matches_config"]:
        raise ValueError("Pinned SDF extrinsic differs from p3_sensors.yaml: " + json.dumps(chain))
    return chain


def build():
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy"))).resolve()
    if Path.home().resolve() not in root.parents:
        raise ValueError("Linux HOME workspace required")
    px4 = root / "upstream/PX4-Autopilot"
    chain = verify_extrinsic(px4)
    manifest = package_manifest()
    identity = package_hash(manifest)
    workspace = root / "p3_ws" / identity
    source = workspace / "src/gwm_sensor_adapter"
    if source.exists():
        for name, value in manifest.items():
            if sha(source / name) != value:
                raise ValueError("Existing build mirror modified: " + name)
    else:
        for name in manifest:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PACKAGE / name, destination)
    for command in (["colcon", "build", "--packages-select", "gwm_sensor_adapter", "--executor", "sequential"],
                    ["colcon", "test", "--packages-select", "gwm_sensor_adapter", "--event-handlers", "console_direct+"],
                    ["colcon", "test-result", "--verbose"]):
        subprocess.run(command, cwd=workspace, check=True)
    installed = next((workspace / "install").rglob("site-packages/gwm_sensor_adapter/__init__.py")).parent
    receipt = {"contract": "p3-sensor-build-v1", "package_hash": identity, "source_files": manifest,
               "workspace": str(workspace), "install": str(workspace / "install"),
               "installed_source_files": {p.relative_to(installed).as_posix(): sha(p) for p in sorted(installed.rglob("*.py"))},
               "extrinsic_sdf_chain": chain}
    (root / "state/p3-built.json").write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print("P3 sensor package built/tested: " + identity, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    if parser.parse_args().build:
        build()
    else:
        print("No build/runtime action. Use build_p3.sh --build.")
