"""P4 frozen candidate identity, operator authorization and gate bookkeeping (contract §8).

The P4 manifest content-addresses the P4 tree (configs, scripts, validation, the
gwm_obstacle_avoidance package), the P4 build receipt, every P3 input P4 imports
(the P3 source set and MC sources, recomputed with the unchanged P3 functions),
the P3 runtime identity (receipts, builds, assets, libraries, renderer profile,
native-command policy with its per-candidate approval), and the accepted P3
baseline: the P3 candidate must still verify to its frozen hash.
"""
import json
import os
from pathlib import Path
import sys
import time

P4 = Path(__file__).resolve().parents[1]
REPO = P4.parents[1]
SIM = REPO / "simulation/px4_gazebo"
for path in (SIM / "scripts", SIM / "validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p3_manifest  # noqa: E402
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p4-candidate-manifest-v1"
GATES = {"A": "p4_ground_readiness_load_fixtures_faults", "B": "p4_avoidance_smoke",
         "S": "p4_safety_flights", "C": "p4_qualification"}
FLIGHT_GATES = ("B", "S", "C")
SOURCE_FOLDERS = ("configs", "scripts", "validation", "ros2_ws/src/gwm_obstacle_avoidance")


def validation():
    return strict_json(P4 / "configs/p4_validation.yaml")


def p4_source_files():
    files = {}
    for folder in SOURCE_FOLDERS:
        for path in sorted((P4 / folder).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(P4).as_posix()] = digest(path)
    return files


def p3_baseline(root):
    """The accepted P3 candidate must still verify, byte for byte, to the hash P4 was approved against."""
    v = validation()
    manifest = Path(root) / v["p3_baseline_manifest"]
    _, sha = p3_manifest.verify_current(root, manifest)
    if sha != v["p3_baseline_manifest_sha256"]:
        raise ValueError("Accepted P3 baseline does not verify to " + v["p3_baseline_manifest_sha256"])
    return {"manifest": str(manifest), "sha256": sha}


def p4_build(root):
    receipt = strict_json(Path(root) / "state/p4-built.json")
    sys.path.insert(0, str(P4 / "scripts"))
    import p4_build as build
    if receipt.get("package_hash") != build.package_hash(build.package_manifest()):
        raise ValueError("P4 package build receipt does not match the current source")
    return {"receipt_sha256": canonical_sha256(receipt), "package_hash": receipt["package_hash"]}


def candidate(root, policy_name="A", policy_approval=None, mc_benchmark_run=None):
    return {"contract": CONTRACT, "p4_contract": validation()["contract"],
            "p4_contract_revision": validation()["contract_revision"], "p3_baseline": p3_baseline(root),
            "p4_source_files": p4_source_files(), "p3_source_files": p3_manifest.source_files(),
            "mc_sources": p3_manifest.required_mc_sources(), "p4_build": p4_build(root),
            "runtime": p3_manifest.runtime_identity(root, policy_name=policy_name, policy_approval=policy_approval,
                                                    mc_benchmark_run=mc_benchmark_run),
            "gates": GATES, "qualification_flights": validation()["qualification_flights"],
            "execution_allowed_without_authorization": False}


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def freeze(root, output, **kwargs):
    value = candidate(root, **kwargs)
    value["frozen_unix_ns"] = time.time_ns()
    atomic_json(output, value, exclusive=True)
    return manifest_sha(value)


def verify_current(root, manifest_path):
    """Recompute everything; any byte difference blocks the runtime."""
    frozen = strict_json(manifest_path)
    runtime = frozen["runtime"]
    policy = runtime["native_command_policy"]
    current = candidate(root, policy_name=policy["name"], policy_approval=policy.get("approval_path"),
                        mc_benchmark_run=runtime["mc_benchmark"]["run"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P4 candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def authorization_template(gate, sha, operator):
    return {"contract": "p4-runtime-authorization-v1", "gate": gate, "scope": GATES[gate], "manifest_sha256": sha,
            "max_attempts": 1, "authorized": True, "authorized_by": operator,
            "excludes": [] if gate in FLIGHT_GATES else ["flight"],
            "statement": f"I explicitly authorize exactly one P4 Gate {gate} ({GATES[gate]}) execution of manifest {sha}."}


def require_authorization(path, gate, sha):
    value = strict_json(path)
    expected = {"contract": "p4-runtime-authorization-v1", "gate": gate, "manifest_sha256": sha,
                "scope": GATES[gate], "max_attempts": 1}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this scope")
    return value


def marker_path(root, gate, sha):
    return Path(root) / "state" / f"p4-gate-{gate}-{sha}.consumed.json"


def consume(root, gate, sha, authorization_path):
    """Exclusive, permanent consumption marker: no automatic retry is possible."""
    marker = marker_path(root, gate, sha)
    atomic_json(marker, {"gate": gate, "manifest_sha256": sha, "authorization_sha256": digest(authorization_path),
                         "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description="Freeze, verify or authorize the P4 candidate; never starts runtime.")
    sub = parser.add_subparsers(dest="action", required=True)
    f = sub.add_parser("freeze")
    f.add_argument("--output", type=Path, required=True)
    f.add_argument("--policy-approval", type=Path, required=True)
    f.add_argument("--mc-benchmark-run", type=Path, required=True)
    v = sub.add_parser("verify")
    v.add_argument("manifest", type=Path)
    a = sub.add_parser("authorize")
    a.add_argument("manifest", type=Path)
    a.add_argument("--gate", choices=sorted(GATES), required=True)
    a.add_argument("--operator", required=True)
    a.add_argument("--confirm", required=True, help='Exactly: "authorize P4 Gate <G> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.action == "freeze":
        print(freeze(root, args.output, policy_name="A", policy_approval=args.policy_approval,
                     mc_benchmark_run=args.mc_benchmark_run))
    elif args.action == "verify":
        print(verify_current(root, args.manifest)[1])
    else:
        _, sha = verify_current(root, args.manifest)
        if args.confirm != f"authorize P4 Gate {args.gate} {sha}":
            raise SystemExit("Confirmation phrase must be exactly: authorize P4 Gate %s %s" % (args.gate, sha))
        atomic_json(args.output, authorization_template(args.gate, sha, args.operator), exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
