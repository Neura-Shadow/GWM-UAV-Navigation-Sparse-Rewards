"""P5 source identity and baseline binding (contract §10). M0 scope: hashing and baseline checks only;
freeze and authorization tooling follow in M7. Never starts runtime."""
import argparse
import json
from pathlib import Path
import sys

P5 = Path(__file__).resolve().parents[1]
REPO = P5.parents[1]
P4 = REPO / "simulation/px4_gazebo_p4"
SIM = REPO / "simulation/px4_gazebo"
for path in (SIM / "scripts", SIM / "validation", P4 / "scripts", P5 / "ros2_ws/src/gwm_policy_control"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from p3_provenance import digest, strict_json  # noqa: E402

SOURCE_FOLDERS = ("configs", "models", "training", "scripts", "validation", "ros2_ws/src/gwm_policy_control")
P4_SOURCE_FOLDERS = ("configs", "scripts", "validation", "ros2_ws/src/gwm_obstacle_avoidance")


def validation():
    return strict_json(P5 / "configs/p5_validation.yaml")


def p5_source_files():
    files = {}
    for folder in SOURCE_FOLDERS:
        base = P5 / folder
        if not base.exists():
            continue
        for path in sorted(base.rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(P5).as_posix()] = digest(path)
    return files


def schema_identity():
    from gwm_policy_control.contracts import load_schema, schema_sha256
    return schema_sha256(load_schema(P5 / "configs/p5_model_schema.json"))


def baselines(root):
    """Contract C4: the accepted P3 and P4 candidates must still verify byte for byte."""
    import p3_manifest
    import p4_manifest
    v = validation()
    _, p3 = p3_manifest.verify_current(root, Path(root) / v["p3_baseline_manifest"])
    if p3 != v["p3_baseline_manifest_sha256"]:
        raise ValueError("Accepted P3 baseline does not verify to " + v["p3_baseline_manifest_sha256"])
    _, p4 = p4_manifest.verify_current(root, Path(root) / v["p4_baseline_manifest"])
    if p4 != v["p4_baseline_manifest_sha256"]:
        raise ValueError("Accepted P4 baseline does not verify to " + v["p4_baseline_manifest_sha256"])
    return {"p3_sha256": p3, "p4_sha256": p4}


def main(argv=None):
    parser = argparse.ArgumentParser(description="P5 source identity and baseline checks; never starts runtime.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("sources")
    b = sub.add_parser("baselines")
    b.add_argument("--root", type=Path, default=Path.home() / "uav_autonomy")
    args = parser.parse_args(argv)
    if args.command == "sources":
        print(json.dumps({"schema_sha256": schema_identity(), "files": p5_source_files()}, indent=2))
    else:
        print(json.dumps(baselines(args.root), indent=2))


if __name__ == "__main__":
    main()
