"""One-way content-addressed Linux package mirror and actual colcon build/tests."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def package_manifest(sim):
    source = sim / "ros2_ws/src/gwm_px4_control"
    return {str(p.relative_to(source)).replace("\\", "/"): sha(p) for p in sorted(source.rglob("*"))
            if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}


def package_hash(manifest):
    return hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()


def source_contract(root, sim):
    import yaml
    import px4_msgs.msg as messages
    sys.path.insert(0, str(sim / "ros2_ws/src/gwm_px4_control"))
    from gwm_px4_control.contracts import TOPICS, topic_name
    px4 = root / "upstream/PX4-Autopilot"
    dds_path = px4 / "src/modules/uxrce_dds_client/dds_topics.yaml"
    dds = yaml.safe_load(dds_path.read_text())
    configured = {entry["topic"]: entry["type"].split("::")[-1]
                  for group in dds.values() if group for entry in group}
    result = {"dds_topics_sha256": sha(dds_path), "topics": {}}
    for key, (direction, name) in TOPICS.items():
        expected = f"/fmu/{direction}/{key}"
        if configured.get(expected) != name:
            raise ValueError("Pinned DDS topic mapping differs: " + expected)
        ros_file = root / "ros_ws/src/px4_msgs/msg" / (name+".msg")
        candidates = [p for p in (px4 / "msg").rglob(name+".msg") if "old" not in p.parts]
        def schema(path):
            return [re.sub(r"\s+", " ", line.split("#", 1)[0].strip())
                    for line in path.read_text().splitlines() if line.split("#", 1)[0].strip()]
        matches = [p for p in candidates if schema(p) == schema(ros_file)]
        if not matches:
            raise ValueError("PX4/px4_msgs definition mismatch: " + name)
        cls = getattr(messages, name)
        version = getattr(cls, "MESSAGE_VERSION", 0)
        result["topics"][key] = {"type": "px4_msgs/msg/"+name, "version": version,
            "topic": topic_name("/px4_71", key, direction, version),
            "px4_definition": str(matches[0].relative_to(px4)), "px4_sha256": sha(matches[0]),
            "ros_sha256": sha(ros_file)}
    return result


def build():
    sim = Path(__file__).resolve().parents[1]
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home()/"uav_autonomy"))).resolve()
    if Path.home().resolve() not in root.parents:
        raise ValueError("Linux HOME workspace required")
    manifest = package_manifest(sim)
    identity = package_hash(manifest)
    workspace = root / "p2_ws" / identity
    source = workspace / "src/gwm_px4_control"
    if source.exists():
        actual={p.relative_to(source).as_posix():sha(p) for p in sorted(source.rglob('*'))
                if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
        if actual!=manifest: raise ValueError('Existing build mirror inventory modified')
    else:
        source.mkdir(parents=True)
        for name in manifest:
            destination = source / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(sim / "ros2_ws/src/gwm_px4_control" / name, destination)
    contract = source_contract(root, sim)
    for command in (["colcon", "build", "--packages-select", "gwm_px4_control", "--executor", "sequential"],
                    ["colcon", "test", "--packages-select", "gwm_px4_control", "--event-handlers", "console_direct+"],
                    ["colcon", "test-result", "--verbose"]):
        subprocess.run(command, cwd=workspace, check=True)
    receipt = {"package_hash": identity, "source_files": manifest, "workspace": str(workspace),
               "install": str(workspace / "install"), "topic_contract": contract}
    installed=next((workspace/'install').rglob('site-packages/gwm_px4_control/__init__.py')).parent
    expected={p.relative_to(source/'gwm_px4_control').as_posix():sha(p)
              for p in (source/'gwm_px4_control').rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    actual={p.relative_to(installed).as_posix():sha(p) for p in installed.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    if actual!=expected: raise ValueError('Installed controller sources differ from mirror')
    receipt['installed_source_files']=expected
    (root / "state/p2-built.json").write_text(json.dumps(receipt, indent=2, allow_nan=False)+"\n")
    print("P2 package built/tested: " + identity, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    if parser.parse_args().build:
        build()
    else:
        print("No build/runtime action. Use build_p2.sh --build.")
