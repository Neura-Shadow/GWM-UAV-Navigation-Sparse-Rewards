"""P6 candidate identity, baseline binding, operator authorization and gate bookkeeping (contract §8, §9).

The P6 candidate binds:

* the P6 tree (configs, injection, validation, scripts) and the P6 test files, file by file;
* the accepted P3, P4 and P5 manifests, which must still verify to the SHA-256s of plan §0 (the P5 check
  recomputes the P5 candidate, including its own P3 and P4 baseline checks, contract C1);
* the P5 model, schema, controller runtime and build receipt, as the verified P5 manifest binds them (C2);
* the injector and witness hashes and the frozen fault and scenario specifications (the gate definitions);
* the runtime identity with a per-candidate Policy A approval that names P6 (contract §8);
* the namespace-owner copy, which must be byte-identical to the P3 owner the unattended QGC build pins.

A **development** candidate (`development: true`, plan §7 M5) has the same content. It is used only by dry
runs under a dry-run approval, never by a gate, and carries no acceptance credit. Freezing, approving and
authorizing are operator steps; every runner and gate re-verifies with `verify_current`. Never starts runtime.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

P6 = Path(__file__).resolve().parents[1]
REPO = P6.parents[1]
SIM = REPO / "simulation/px4_gazebo"
P5 = REPO / "simulation/px4_gazebo_p5"
for path in (SIM / "scripts", SIM / "validation", P6):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

_spec = importlib.util.spec_from_file_location("p5_manifest_accepted", P5 / "scripts/p5_manifest.py")
p5_manifest = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p5_manifest)

CONTRACT = "p6-candidate-manifest-v1"
SOURCE_FOLDERS = ("configs", "injection", "validation", "scripts")
TEST_GLOB = "test_px4_gazebo_p6_*.py"
P5_MANIFEST = "state/p5-candidate-40a109b.json"  # the accepted P5 candidate (plan §0)
GATES = {"A": "p6_ground_readiness_fault_fixtures", "B": "p6_smoke", "S": "p6_safety_critical_faults",
         "C": "p6_fault_tolerance_qualification"}
FLIGHT_GATES = ("B", "S", "C")
DRY_RUN = "DRY"
DRY_RUN_CONTRACT = "p6-dry-run-approval-v1"
OWNER = P6 / "scripts/p6_namespace_owner.py"
P3_OWNER = SIM / "scripts/p3_namespace_owner.py"


def scenarios():
    return strict_json(P6 / "configs/p6_scenarios.yaml")


def p6_source_files():
    """Every P6 file the runtime, the evaluators and the offline suite use, and the P6 tests."""
    files = {}
    for folder in SOURCE_FOLDERS:
        for path in sorted((P6 / folder).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(REPO).as_posix()] = digest(path)
    for path in sorted((REPO / "tests").glob(TEST_GLOB)):
        files[path.relative_to(REPO).as_posix()] = digest(path)
    return files


def injector_identity():
    """Contract C7 and §9: each injector, the witness and the controller shim, by hash (also inside the sources)."""
    return {path.name: digest(path) for path in sorted((P6 / "injection").glob("*.py"))}


def specs_identity():
    return {"faults_sha256": digest(P6 / "configs/p6_faults.yaml"), "scenarios_sha256": digest(P6 / "configs/p6_scenarios.yaml")}


def baselines(root):
    """Contract C1: the accepted P5 manifest verifies (with its own P3 and P4 checks) to the plan §0 values."""
    expected = scenarios()["baselines"]
    p5_frozen, p5_sha = p5_manifest.verify_current(root, Path(root) / P5_MANIFEST)
    if p5_sha != expected["p5_manifest_sha256"]:
        raise ValueError("Accepted P5 baseline does not verify to " + expected["p5_manifest_sha256"])
    if p5_frozen["baselines"] != {"p3_sha256": expected["p3_manifest_sha256"], "p4_sha256": expected["p4_manifest_sha256"]}:
        raise ValueError("The P5 manifest does not bind the accepted P3 and P4 baselines")
    return {"p3_sha256": expected["p3_manifest_sha256"], "p4_sha256": expected["p4_manifest_sha256"], "p5_sha256": p5_sha}, p5_frozen


def p5_identity(p5_frozen):
    """What the P5 runner needs from the accepted candidate, copied from its verified manifest (contract C2)."""
    return {"manifest": P5_MANIFEST, "model": p5_frozen["model"], "schema_sha256": p5_frozen["schema_sha256"],
            "controller_runtime": p5_frozen["controller_runtime"], "p5_build": p5_frozen["p5_build"],
            "baselines": p5_frozen["baselines"], "mc_benchmark_run": p5_frozen["runtime"]["mc_benchmark"]["run"]}


def check_policy_approval(path, development):
    """A Policy A approval for a P6 candidate (contract §8); a P5 approval never transfers."""
    approval = strict_json(path)
    if not str(approval.get("candidate_phase", "")).startswith("P6"):
        raise PermissionError("The Policy A approval does not name a P6 candidate")
    if bool(approval.get("development_only")) != bool(development):
        raise PermissionError("A development-only Policy A approval binds development candidates only, and vice versa")
    return approval


def owner_identity():
    if digest(OWNER) != digest(P3_OWNER):
        raise ValueError("The P6 namespace owner is not byte-identical to the P3 owner the QGC build pins")
    return {"path": OWNER.relative_to(REPO).as_posix(), "sha256": digest(OWNER)}


def candidate(root, policy_approval, development=False):
    import p3_manifest
    check_policy_approval(policy_approval, development)
    bound, p5_frozen = baselines(root)
    p5 = p5_identity(p5_frozen)
    return {"contract": CONTRACT, "p6_contract": scenarios()["acceptance_contract"], "development": bool(development),
            "acceptance_credit_possible": not development, "baselines": bound, "p5": p5,
            "p6_source_files": p6_source_files(), "injectors": injector_identity(), "specs": specs_identity(),
            "namespace_owner": owner_identity(),
            "runtime": p3_manifest.runtime_identity(root, policy_name="A", policy_approval=policy_approval,
                                                    mc_benchmark_run=p5["mc_benchmark_run"]),
            "gates": GATES, "execution_allowed_without_authorization": False}


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def freeze(root, output, policy_approval, development=False):
    value = candidate(root, policy_approval, development)
    value["frozen_unix_ns"] = time.time_ns()
    value["frozen_git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    atomic_json(output, value, exclusive=True)
    return manifest_sha(value)


def verify_current(root, manifest_path):
    """Recompute everything; any byte difference blocks the runtime."""
    frozen = strict_json(manifest_path)
    if frozen.get("contract") != CONTRACT:
        raise ValueError("Not a P6 candidate manifest")
    policy = frozen["runtime"]["native_command_policy"]
    current = candidate(root, policy["approval_path"], frozen["development"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    current["frozen_git_head"] = frozen.get("frozen_git_head")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P6 candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def runner_view(frozen):
    """The `frozen` mapping the accepted P5/P4/P3 runner code reads, with P6's own runtime identity."""
    p5 = frozen["p5"]
    return {"model": p5["model"], "schema_sha256": p5["schema_sha256"], "controller_runtime": p5["controller_runtime"],
            "baselines": p5["baselines"], "runtime": frozen["runtime"]}


# ---- gate authorization (real candidates) -----------------------------------------------------------------
def authorization_template(gate, sha, operator):
    return {"contract": "p6-runtime-authorization-v1", "gate": gate, "scope": GATES[gate], "manifest_sha256": sha,
            "max_attempts": 1, "authorized": True, "authorized_by": operator,
            "excludes": [] if gate in FLIGHT_GATES else ["flight"],
            "statement": f"I explicitly authorize exactly one P6 Gate {gate} ({GATES[gate]}) execution of manifest {sha}."}


def require_authorization(path, gate, sha):
    value = strict_json(path)
    expected = {"contract": "p6-runtime-authorization-v1", "gate": gate, "manifest_sha256": sha,
                "scope": GATES[gate], "max_attempts": 1}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this scope")
    return value


def marker_path(root, gate, sha):
    return Path(root) / "state" / f"p6-gate-{gate}-{sha}.consumed.json"


def consume(root, gate, sha, authorization_path):
    """Exclusive, permanent consumption marker: no automatic retry is possible."""
    marker = marker_path(root, gate, sha)
    atomic_json(marker, {"gate": gate, "manifest_sha256": sha, "authorization_sha256": digest(authorization_path),
                         "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


# ---- dry runs (development candidates, plan §7 M5) ----------------------------------------------------------
def dry_run_template(sha, operator, statement, gate_content="A", live_runs_max=40):
    return {"contract": DRY_RUN_CONTRACT, "manifest_sha256": sha, "gate_content": gate_content, "authorized": True,
            "authorized_by": operator, "operator_statement": statement, "acceptance_credit": False,
            "flight": False, "vehicle_armed": False, "live_runs_max": live_runs_max,
            "statement": f"Dry run of P6 Gate {gate_content} content on development candidate {sha}: ground only, "
                         "the vehicle is never armed, no gate result, no acceptance credit."}


def require_dry_run_approval(path, sha, frozen):
    value = strict_json(path)
    if value.get("contract") != DRY_RUN_CONTRACT or value.get("manifest_sha256") != sha:
        raise PermissionError("Dry-run approval does not bind this development candidate")
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Dry-run approval is not an explicit operator approval")
    if value.get("acceptance_credit") is not False or value.get("flight") is not False:
        raise PermissionError("A dry run never flies and never earns credit")
    if frozen.get("development") is not True:
        raise PermissionError("Dry runs use a development candidate only")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="Identify, freeze, verify or authorize the P6 candidate; never starts runtime.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("sources")
    sub.add_parser("baselines")
    f = sub.add_parser("freeze")
    f.add_argument("--output", type=Path, required=True)
    f.add_argument("--policy-approval", type=Path, required=True)
    f.add_argument("--development", action="store_true")
    v = sub.add_parser("verify")
    v.add_argument("manifest", type=Path)
    a = sub.add_parser("authorize")
    a.add_argument("manifest", type=Path)
    a.add_argument("--gate", choices=sorted(GATES), required=True)
    a.add_argument("--operator", required=True)
    a.add_argument("--confirm", required=True, help='Exactly: "authorize P6 Gate <G> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    d = sub.add_parser("approve-dry-run")
    d.add_argument("manifest", type=Path)
    d.add_argument("--operator", required=True)
    d.add_argument("--statement", required=True, help="the operator's chat statement that authorizes the dry run")
    d.add_argument("--confirm", required=True, help='Exactly: "dry run P6 Gate A content on <manifest-sha>"')
    d.add_argument("--live-runs-max", type=int, default=40)
    d.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.command == "sources":
        print(json.dumps({"files": p6_source_files(), "injectors": injector_identity(), "specs": specs_identity()}, indent=2))
    elif args.command == "baselines":
        print(json.dumps(baselines(root)[0], indent=2))
    elif args.command == "freeze":
        print(freeze(root, args.output, args.policy_approval, args.development))
    elif args.command == "verify":
        print(verify_current(root, args.manifest)[1])
    elif args.command == "authorize":
        frozen, sha = verify_current(root, args.manifest)
        if frozen["development"]:
            raise SystemExit("A development candidate is never authorized for a gate")
        if args.confirm != f"authorize P6 Gate {args.gate} {sha}":
            raise SystemExit("Confirmation phrase must be exactly: authorize P6 Gate %s %s" % (args.gate, sha))
        atomic_json(args.output, authorization_template(args.gate, sha, args.operator), exclusive=True)
        print(args.output)
    else:
        frozen, sha = verify_current(root, args.manifest)
        if not frozen["development"]:
            raise SystemExit("Dry runs use a development candidate only")
        if args.confirm != f"dry run P6 Gate A content on {sha}":
            raise SystemExit("Confirmation phrase must be exactly: dry run P6 Gate A content on %s" % sha)
        atomic_json(args.output, dry_run_template(sha, args.operator, args.statement, live_runs_max=args.live_runs_max),
                    exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
