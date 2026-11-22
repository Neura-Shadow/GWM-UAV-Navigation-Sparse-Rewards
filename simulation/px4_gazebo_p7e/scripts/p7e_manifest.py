"""P7-E candidate identity, the binding of the frozen P7 candidate, the approved E1 specifications, operator
authorization and block bookkeeping (contract `p7-engineering-validation-v1` E-C1, E-C2, §8, §13).

**What the P7-E candidate binds:**
* **The frozen P7 candidate `0799c37e…`,** verified through the accepted `p7_manifest.verify_current`. That
  recomputes the P6 candidate, the P5 candidate inside it, and the P3 and P4 candidates inside P5. P7-E therefore
  binds P3–P6 as well, and the P7 code it imports (E-C1).
* **The two approved E1 specifications, by canonical SHA-256:** the schedule (`9b3deacb…`) and the report plan
  (`15e7f7b4…`). It also binds the reused frozen P7 layouts (`d742b7af…`) and host check (`9f78af75…`).
* **The P7-E tree** (configs, scripts, validation, analysis) and the P7-E tests, file by file.
* **The runtime identity,** with a Policy A approval that names P7-E. An earlier phase's approval never transfers.
* **The namespace-owner copy,** byte-identical to the P3 owner.

**Development candidates** run dry runs only. A real candidate runs the 3 blocks of scope `engineering-30`, each under
its own authorization (`max_attempts` 1, a permanent marker). The live lock refuses a freeze or a second sequence
while either a P7-E or a P7 sequence runs. Nothing here starts runtime.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
SIM, P7 = REPO / "simulation/px4_gazebo", REPO / "simulation/px4_gazebo_p7"
for _path in (SIM / "scripts", SIM / "validation", P7 / "scripts", P7E / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7_manifest  # noqa: E402  (frozen P7 code, used unchanged)
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p7e-candidate-manifest-v1"
SOURCE_FOLDERS = ("configs", "scripts", "validation", "analysis")
TEST_GLOB = "test_px4_gazebo_p7e_*.py"
P7_MANIFEST = "state/p7-candidate-a8bd50e.json"
P7_SHA256 = "0799c37e800c06c278acbc45fdec90b84159ee9d41eb7aaf868cb65a0b950aa5"
SPEC_FILES = {"schedule": "configs/p7e_schedule.json", "report_plan": "configs/p7e_report_plan.yaml"}
APPROVED_SPECS = {"schedule": "9b3deacb3ee77a0baa38c78fa9197c22e569150ff6d4c76b489c494b6aea377b",
                  "report_plan": "15e7f7b460277e650823342bbf33a63426c226730192f3bbc2c536ab70c31d94"}
REUSED_P7 = {"layouts": ("configs/p7_layouts.json", "d742b7af47c6339fb6d8478870d0cf767b6f09cf41d0ef23d6661bc0b1d2258d"),
             "host_check": ("configs/p7_host_check.yaml", "9f78af75faebcfe366399e709bec1bb200e74f5af84101c72e5e8a184396f335")}
SCOPE, BLOCKS = "engineering-30", 3
DRY_RUN = "DRY"
AUTHORIZATION_CONTRACT = "p7e-runtime-authorization-v1"
DRY_RUN_CONTRACT = "p7e-dry-run-approval-v1"
CONTROLLERS = ("p4", "p5")
OWNER = P7E / "scripts/p7e_namespace_owner.py"
P3_OWNER = SIM / "scripts/p3_namespace_owner.py"
LIVE_LOCK = "state/p7e-live-run.lock"


# ---- identity -----------------------------------------------------------------------------------------------
def p7e_source_files():
    files = {}
    for folder in SOURCE_FOLDERS:
        for path in sorted((P7E / folder).rglob("*")) if (P7E / folder).exists() else ():
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(REPO).as_posix()] = digest(path)
    for path in sorted((REPO / "tests").glob(TEST_GLOB)):
        files[path.relative_to(REPO).as_posix()] = digest(path)
    return files


def specs_identity():
    """The approved E1 specifications and the reused frozen P7 specifications, by canonical SHA-256."""
    current = {name: canonical_sha256(strict_json(P7E / path)) for name, path in SPEC_FILES.items()}
    if current != APPROVED_SPECS:
        raise ValueError("P7-E specifications differ from the approved hashes: "
                         + ", ".join(sorted(k for k in current if current[k] != APPROVED_SPECS[k])))
    for name, (path, approved) in REUSED_P7.items():
        if canonical_sha256(strict_json(P7 / path)) != approved:
            raise ValueError("the reused frozen P7 %s differs from %s…" % (name, approved[:8]))
        current["p7_" + name] = approved
    return current


def owner_identity():
    if digest(OWNER) != digest(P3_OWNER):
        raise ValueError("The P7-E namespace owner is not byte-identical to the P3 owner the QGC build pins")
    return {"path": OWNER.relative_to(REPO).as_posix(), "sha256": digest(OWNER)}


def p7_binding(root):
    """E-C1/E-C2: the frozen P7 candidate verifies (and with it P3-P6) to 0799c37e."""
    frozen, sha = p7_manifest.verify_current(root, Path(root) / P7_MANIFEST)
    if sha != P7_SHA256:
        raise ValueError("The frozen P7 candidate does not verify to " + P7_SHA256)
    return frozen, sha


def check_policy_approval(path, development):
    approval = strict_json(path)
    if not str(approval.get("candidate_phase", "")).startswith("P7-E"):
        raise PermissionError("The Policy A approval does not name a P7-E candidate")
    if bool(approval.get("development_only")) != bool(development):
        raise PermissionError("A development-only Policy A approval binds development candidates only, and vice versa")
    return approval


def candidate(root, policy_approval, development=False):
    import p3_manifest
    check_policy_approval(policy_approval, development)
    p7_frozen, p7_sha = p7_binding(root)
    return {"contract": CONTRACT, "p7e_contract": "p7-engineering-validation-v1", "development": bool(development),
            "engineering_credit_possible": not development,
            "baselines": dict(p7_frozen["baselines"], p7_sha256=p7_sha),
            "p7": {"manifest": P7_MANIFEST, "sha256": p7_sha,
                   "source_files_canonical_sha256": canonical_sha256(p7_frozen["p7_source_files"])},
            "p4": p7_frozen["p4"], "p5": p7_frozen["p5"],
            "p7e_source_files": p7e_source_files(), "specs": specs_identity(), "namespace_owner": owner_identity(),
            "runtime": p3_manifest.runtime_identity(root, policy_name="A", policy_approval=policy_approval,
                                                    mc_benchmark_run=p7_frozen["p5"]["mc_benchmark_run"]),
            "scope": SCOPE, "blocks": BLOCKS, "execution_allowed_without_authorization": False}


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def freeze(root, output, policy_approval, development=False):
    if live_locks(root):
        raise SystemExit("A live P7 or P7-E sequence holds %s; no candidate may be frozen" % live_locks(root))
    value = candidate(root, policy_approval, development)
    value["frozen_unix_ns"] = time.time_ns()
    value["frozen_git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    atomic_json(output, value, exclusive=True)
    return manifest_sha(value)


def verify_current(root, manifest_path):
    frozen = strict_json(manifest_path)
    if frozen.get("contract") != CONTRACT:
        raise ValueError("Not a P7-E candidate manifest")
    current = candidate(root, frozen["runtime"]["native_command_policy"]["approval_path"], frozen["development"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    current["frozen_git_head"] = frozen.get("frozen_git_head")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P7-E candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def source_drift(frozen, root=None):
    """Hash-bound P7-E and P7 inputs that differ now (cheap; no P3-P6 recomputation)."""
    bound, current = frozen["p7e_source_files"], p7e_source_files()
    drift = sorted(k for k in set(bound) | set(current) if bound.get(k) != current.get(k))
    try:
        if specs_identity() != frozen["specs"]:
            drift.append("specs")
    except ValueError:
        drift.append("specs")
    if frozen["namespace_owner"]["sha256"] != digest(OWNER):
        drift.append(frozen["namespace_owner"]["path"])
    p7_frozen = strict_json(Path(root or os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy"))) / P7_MANIFEST)
    drift += ["p7:" + d for d in p7_manifest.source_drift(p7_frozen)]  # the P7 code P7-E imports
    return drift


def runner_view(frozen, controller):
    """The `frozen` mapping the accepted runners read: P7's view of P7-E's own runtime identity."""
    return p7_manifest.runner_view(frozen, controller)


# ---- live locks (P7-E and P7 exclude each other) ----------------------------------------------------------------
def live_lock(root):
    return Path(root) / LIVE_LOCK


def live_locks(root):
    return [str(p) for p in (live_lock(root), p7_manifest.live_lock(root)) if p.exists()]


def acquire_live_lock(root, sha):
    if p7_manifest.live_lock(root).exists():
        raise SystemExit("A live P7 sequence holds %s" % p7_manifest.live_lock(root))
    try:
        atomic_json(live_lock(root), {"manifest_sha256": sha, "pid": os.getpid(), "started_unix_ns": time.time_ns()},
                    exclusive=True)
    except FileExistsError:
        raise SystemExit("Another live P7-E sequence holds %s" % live_lock(root))


def release_live_lock(root, sha):
    lock = live_lock(root)
    if lock.exists() and strict_json(lock).get("manifest_sha256") == sha and strict_json(lock).get("pid") == os.getpid():
        lock.unlink()


# ---- block authorizations ---------------------------------------------------------------------------------------
def authorization_template(block, sha, operator):
    if not 1 <= int(block) <= BLOCKS:
        raise ValueError("No block %s of scope %s" % (block, SCOPE))
    return {"contract": AUTHORIZATION_CONTRACT, "scope": SCOPE, "block": int(block), "manifest_sha256": sha,
            "schedule_canonical_sha256": APPROVED_SPECS["schedule"], "max_attempts": 1, "authorized": True,
            "authorized_by": operator, "simulated_flight_only": True,
            "statement": "I explicitly authorize exactly one execution of P7-E %s block %d of manifest %s, in simulated "
                         "flight only (SITL, never real hardware)." % (SCOPE, int(block), sha)}


def require_authorization(path, block, sha):
    value = strict_json(path)
    expected = {"contract": AUTHORIZATION_CONTRACT, "scope": SCOPE, "block": int(block), "manifest_sha256": sha,
                "schedule_canonical_sha256": APPROVED_SPECS["schedule"], "max_attempts": 1, "simulated_flight_only": True}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this block")
    return value


def marker_path(root, block, sha):
    return Path(root) / "state" / ("p7e-%s-block-%02d-%s.consumed.json" % (SCOPE, int(block), sha))


def consume(root, block, sha, authorization_path):
    marker = marker_path(root, block, sha)
    atomic_json(marker, {"scope": SCOPE, "block": int(block), "manifest_sha256": sha,
                         "authorization_sha256": digest(authorization_path), "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


def result_path(root, block, sha):
    return Path(root) / "state" / ("p7e-%s-block-%02d-%s.result.json" % (SCOPE, int(block), sha))


# ---- dry runs (development candidates) ---------------------------------------------------------------------------
def dry_run_template(sha, operator, statement, items, live_runs_max):
    return {"contract": DRY_RUN_CONTRACT, "manifest_sha256": sha, "authorized": True, "authorized_by": operator,
            "operator_statement": statement, "items": [list(i) for i in items], "live_runs_max": int(live_runs_max),
            "flight": True, "simulated_flight_only": True, "vehicle_armed": True, "acceptance_credit": False,
            "statement": "Dry run of P7-E flights %s on development candidate %s: simulated flight only (SITL, never "
                         "real hardware), no block result, no engineering credit."
                         % (", ".join("%s/%s" % tuple(i) for i in items), sha)}


def require_dry_run_approval(path, sha, frozen):
    value = strict_json(path)
    if value.get("contract") != DRY_RUN_CONTRACT or value.get("manifest_sha256") != sha:
        raise PermissionError("Dry-run approval does not bind this development candidate")
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Dry-run approval is not an explicit operator approval")
    if value.get("acceptance_credit") is not False:
        raise PermissionError("A dry run never earns credit")
    if value.get("flight") is not True or value.get("simulated_flight_only") is not True:
        raise PermissionError("A P7-E dry run needs an approval that names simulated flight explicitly")
    if not value.get("items") or any(len(i) != 2 or i[1] not in CONTROLLERS for i in value["items"]):
        raise PermissionError("A dry-run approval names its flights as [item, controller]")
    if frozen.get("development") is not True:
        raise PermissionError("Dry runs use a development candidate only")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="Identify, freeze, verify or authorize the P7-E candidate; never starts runtime.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("sources")
    f = sub.add_parser("freeze")
    f.add_argument("--output", type=Path, required=True)
    f.add_argument("--policy-approval", type=Path, required=True)
    f.add_argument("--development", action="store_true")
    v = sub.add_parser("verify")
    v.add_argument("manifest", type=Path)
    a = sub.add_parser("authorize")
    a.add_argument("manifest", type=Path)
    a.add_argument("--block", type=int, required=True)
    a.add_argument("--operator", required=True)
    a.add_argument("--confirm", required=True, help='Exactly: "authorize P7-E engineering-30 block <n> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    d = sub.add_parser("approve-dry-run")
    d.add_argument("manifest", type=Path)
    d.add_argument("--operator", required=True)
    d.add_argument("--statement", required=True)
    d.add_argument("--item", nargs=2, action="append", metavar=("ITEM", "CONTROLLER"), required=True)
    d.add_argument("--live-runs-max", type=int, required=True)
    d.add_argument("--confirm", required=True, help='Exactly: "dry run P7-E on <manifest-sha> in simulated flight"')
    d.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.command == "sources":
        print(json.dumps({"files": p7e_source_files(), "specs": specs_identity()}, indent=2))
    elif args.command == "freeze":
        print(freeze(root, args.output, args.policy_approval, args.development))
    elif args.command == "verify":
        print(verify_current(root, args.manifest)[1])
    elif args.command == "authorize":
        frozen, sha = verify_current(root, args.manifest)
        if frozen["development"]:
            raise SystemExit("A development candidate is never authorized for a block")
        phrase = "authorize P7-E %s block %d %s" % (SCOPE, args.block, sha)
        if args.confirm != phrase:
            raise SystemExit("Confirmation phrase must be exactly: " + phrase)
        atomic_json(args.output, authorization_template(args.block, sha, args.operator), exclusive=True)
        print(args.output)
    else:
        frozen, sha = verify_current(root, args.manifest)
        if not frozen["development"]:
            raise SystemExit("Dry runs use a development candidate only")
        phrase = "dry run P7-E on %s in simulated flight" % sha
        if args.confirm != phrase:
            raise SystemExit("Confirmation phrase must be exactly: " + phrase)
        atomic_json(args.output, dry_run_template(sha, args.operator, args.statement, args.item, args.live_runs_max),
                    exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
