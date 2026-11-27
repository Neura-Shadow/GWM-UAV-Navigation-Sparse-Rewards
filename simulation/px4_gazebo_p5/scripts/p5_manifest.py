"""P5 candidate identity, baseline binding, operator authorization and gate bookkeeping (contract §9, §10).

The P5 candidate binds the P5 tree (configs, models, training, scripts, validation and the
gwm_policy_control package), the frozen schema hash, the model artifact identity (file SHA-256,
architecture, schema hash, `trained`, metadata and the three hash-bound reports), the controller
runtime's NumPy, the P3 runtime identity (receipts, builds, assets, renderer profile and the
per-candidate Policy A approval) and the accepted P3 and P4 baselines, which must still verify byte
for byte, and the P5 package build receipt. Freezing and authorizing are operator steps
(`freeze`, `authorize`); every runner and gate re-verifies with `verify_current`. Never starts runtime.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

P5 = Path(__file__).resolve().parents[1]
REPO = P5.parents[1]
P4 = REPO / "simulation/px4_gazebo_p4"
SIM = REPO / "simulation/px4_gazebo"
for path in (SIM / "scripts", SIM / "validation", P4 / "scripts", P5 / "ros2_ws/src/gwm_policy_control"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p5-candidate-manifest-v1"
SOURCE_FOLDERS = ("configs", "models", "training", "scripts", "validation", "ros2_ws/src/gwm_policy_control")
P4_SOURCE_FOLDERS = ("configs", "scripts", "validation", "ros2_ws/src/gwm_obstacle_avoidance")
GATES = {"A": "p5_ground_readiness_load_fixtures_policy_faults", "B": "p5_policy_smoke",
         "S": "p5_safety_flights", "C": "p5_qualification"}
FLIGHT_GATES = ("B", "S", "C")
CONTROLLER_PYTHON = "/usr/bin/python3"  # the interpreter the installed p5_control runs under


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


def model_path(sha256):
    return P5 / "models" / ("p5-ranker-%s.npz" % sha256)


def model_identity(sha256):
    """The bound checkpoint must load as a trained model, and its reports must match their embedded hashes."""
    from gwm_policy_control.policy_runtime import FrozenPolicy
    schema = schema_identity()
    path = model_path(sha256)
    policy = FrozenPolicy.load(path, sha256, schema, require_trained=True)
    reports = {}
    for kind in ("dataset_manifest", "training_report", "readiness_report"):
        report = path.with_name("p5-ranker-%s.%s.json" % (sha256, kind))
        if digest(report) != policy.metadata[kind + "_sha256"]:
            raise ValueError("P5 model report does not match its embedded hash: " + kind)
        reports[kind + "_sha256"] = digest(report)
    return {"path": path.relative_to(P5).as_posix(), "sha256": sha256, "architecture": policy.metadata["architecture"],
            "schema_sha256": schema, "trained": True, "metadata_sha256": canonical_sha256(policy.metadata), **reports}


def controller_runtime(python=CONTROLLER_PYTHON):
    """The NumPy that p5_control and the evaluator's score replay run under (contract §10)."""
    version = subprocess.check_output([python, "-I", "-c", "import numpy; print(numpy.__version__)"], text=True).strip()
    return {"python": python, "numpy": version}


def p5_build(root):
    """The P5 package build receipt must match the current package source."""
    receipt = strict_json(Path(root) / "state/p5-built.json")
    sys.path.insert(0, str(P5 / "scripts"))
    import p5_build as build
    if receipt.get("contract") != build.CONTRACT or receipt.get("package_hash") != build.package_hash(build.package_manifest()):
        raise ValueError("P5 package build receipt does not match the current source")
    return {"receipt_sha256": canonical_sha256(receipt), "package_hash": receipt["package_hash"]}


def candidate(root, model_sha256, policy_name="A", policy_approval=None, mc_benchmark_run=None):
    import p3_manifest
    v = validation()
    return {"contract": CONTRACT, "p5_contract": v["contract"], "p5_contract_revision": v["contract_revision"],
            "baselines": baselines(root), "p5_source_files": p5_source_files(), "schema_sha256": schema_identity(),
            "model": model_identity(model_sha256), "controller_runtime": controller_runtime(), "p5_build": p5_build(root),
            "runtime": p3_manifest.runtime_identity(root, policy_name=policy_name, policy_approval=policy_approval,
                                                    mc_benchmark_run=mc_benchmark_run),
            "gates": GATES, "qualification_flights": v["qualification_flights"],
            "execution_allowed_without_authorization": False}


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def freeze(root, output, model_sha256, **kwargs):
    value = candidate(root, model_sha256, **kwargs)
    value["frozen_unix_ns"] = time.time_ns()
    atomic_json(output, value, exclusive=True)
    return manifest_sha(value)


def verify_current(root, manifest_path):
    """Recompute everything; any byte difference blocks the runtime."""
    frozen = strict_json(manifest_path)
    runtime = frozen["runtime"]
    policy = runtime["native_command_policy"]
    current = candidate(root, frozen["model"]["sha256"], policy_name=policy["name"],
                        policy_approval=policy.get("approval_path"), mc_benchmark_run=runtime["mc_benchmark"]["run"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P5 candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def authorization_template(gate, sha, operator):
    return {"contract": "p5-runtime-authorization-v1", "gate": gate, "scope": GATES[gate], "manifest_sha256": sha,
            "max_attempts": 1, "authorized": True, "authorized_by": operator,
            "excludes": [] if gate in FLIGHT_GATES else ["flight"],
            "statement": f"I explicitly authorize exactly one P5 Gate {gate} ({GATES[gate]}) execution of manifest {sha}."}


def require_authorization(path, gate, sha):
    value = strict_json(path)
    expected = {"contract": "p5-runtime-authorization-v1", "gate": gate, "manifest_sha256": sha,
                "scope": GATES[gate], "max_attempts": 1}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this scope")
    return value


def marker_path(root, gate, sha):
    return Path(root) / "state" / f"p5-gate-{gate}-{sha}.consumed.json"


def consume(root, gate, sha, authorization_path):
    """Exclusive, permanent consumption marker: no automatic retry is possible."""
    marker = marker_path(root, gate, sha)
    atomic_json(marker, {"gate": gate, "manifest_sha256": sha, "authorization_sha256": digest(authorization_path),
                         "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


def main(argv=None):
    parser = argparse.ArgumentParser(description="Identify, freeze, verify or authorize the P5 candidate; never starts runtime.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("sources")
    sub.add_parser("baselines")
    f = sub.add_parser("freeze")
    f.add_argument("--output", type=Path, required=True)
    f.add_argument("--model-sha256", required=True)
    f.add_argument("--policy-approval", type=Path, required=True)
    f.add_argument("--mc-benchmark-run", type=Path, required=True)
    v = sub.add_parser("verify")
    v.add_argument("manifest", type=Path)
    a = sub.add_parser("authorize")
    a.add_argument("manifest", type=Path)
    a.add_argument("--gate", choices=sorted(GATES), required=True)
    a.add_argument("--operator", required=True)
    a.add_argument("--confirm", required=True, help='Exactly: "authorize P5 Gate <G> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.command == "sources":
        print(json.dumps({"schema_sha256": schema_identity(), "files": p5_source_files()}, indent=2))
    elif args.command == "baselines":
        print(json.dumps(baselines(root), indent=2))
    elif args.command == "freeze":
        print(freeze(root, args.output, args.model_sha256, policy_name="A", policy_approval=args.policy_approval,
                     mc_benchmark_run=args.mc_benchmark_run))
    elif args.command == "verify":
        print(verify_current(root, args.manifest)[1])
    else:
        _, sha = verify_current(root, args.manifest)
        if args.confirm != f"authorize P5 Gate {args.gate} {sha}":
            raise SystemExit("Confirmation phrase must be exactly: authorize P5 Gate %s %s" % (args.gate, sha))
        atomic_json(args.output, authorization_template(args.gate, sha, args.operator), exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    main()
