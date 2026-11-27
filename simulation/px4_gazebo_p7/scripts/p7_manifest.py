"""P7 candidate identity, baseline binding, the approved specifications, operator authorization and block
bookkeeping (contract §2 C1, C7, §9).

The P7 candidate binds:
* **the P7 tree** (configs, scripts, and later validation and analysis) and the P7 test files, file by file;
* **the accepted P3-P6 manifests.** The accepted P6 manifest must still verify to plan §0. Its verification
  recomputes the P5 candidate and, through it, the P3 and P4 candidates (C1).
* **what each accepted runner reads,** copied from the verified manifests (C2):
  * P4 side: its P3 baseline;
  * P5 side: model, schema, controller runtime, build and baselines.
* **the three M1 specifications by canonical SHA-256.** They must equal the hashes the operator approved on
  2026-11-21 ("Approve specs"), or nothing is frozen (C7).
* **the host-check definition** (`configs/p7_host_check.yaml`);
* **the runtime identity**, with a per-candidate Policy A approval that names P7 (D12);
* **the namespace-owner copy**, byte-identical to the P3 owner that the unattended QGC build pins.

**A development candidate** (`development: true`) runs only dry runs (M4) under a dry-run approval, and never a
pilot or confirmatory block.

**Authorizations.** Every block is one authorization, with `max_attempts` 1 and a permanent consumption marker
(D12). The pilot is the one block of scope `pilot`. A confirmatory block also needs the schedule selection the pilot
rule produced. Freezing, approving and authorizing are operator steps. Nothing here starts runtime.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

P7 = Path(__file__).resolve().parents[1]
REPO = P7.parents[1]
SIM, P6 = REPO / "simulation/px4_gazebo", REPO / "simulation/px4_gazebo_p6"
for _path in (SIM / "scripts", SIM / "validation", P7 / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

_spec = importlib.util.spec_from_file_location("p6_manifest_accepted", P6 / "scripts/p6_manifest.py")
p6_manifest = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(p6_manifest)

CONTRACT = "p7-candidate-manifest-v1"
SOURCE_FOLDERS = ("configs", "scripts", "validation", "analysis")
TEST_GLOB = "test_px4_gazebo_p7_*.py"
P4_MANIFEST = "state/p4-candidate-50d5002.json"
P6_MANIFEST = "state/p6-candidate-8e29f3f.json"
BASELINES = {"p3_sha256": "1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c",
             "p4_sha256": "c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b",
             "p5_sha256": "2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632",
             "p6_sha256": "222927752a507fdb2fb0ef13f44d6ce8163d50b7d22f405ebbf27df6c21bfa41"}
SPEC_FILES = {"layouts": "configs/p7_layouts.json", "schedule": "configs/p7_schedule.json",
              "analysis_plan": "configs/p7_analysis_plan.yaml"}
APPROVED_SPECS = {"layouts": "d742b7af47c6339fb6d8478870d0cf767b6f09cf41d0ef23d6661bc0b1d2258d",
                  "schedule": "ef74c71d4242409793c6de759454a498481564c75a3dd13e3402294277ba7c88",
                  "analysis_plan": "c9c7a6f3781d3d90cd858de4f2ea9ef8e4bdebcd1b0a8e6ab84918a4f46d9548"}
HOST_CHECK = "configs/p7_host_check.yaml"
SCOPES = ("pilot", "confirmatory-200", "confirmatory-250")
DRY_RUN = "DRY"
DRY_RUN_CONTRACT = "p7-dry-run-approval-v1"
AUTHORIZATION_CONTRACT = "p7-runtime-authorization-v1"
SELECTION_CONTRACT = "p7-schedule-selection-v1"
CONTROLLERS = ("p4", "p5")
OWNER = P7 / "scripts/p7_namespace_owner.py"
P3_OWNER = SIM / "scripts/p3_namespace_owner.py"
LIVE_LOCK = "state/p7-live-run.lock"


# ---- identity -----------------------------------------------------------------------------------------------
def p7_source_files():
    """Every P7 file the runner, the sequencer, the classifier and the analysis use, and the P7 tests."""
    files = {}
    for folder in SOURCE_FOLDERS:
        for path in sorted((P7 / folder).rglob("*")) if (P7 / folder).exists() else ():
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(REPO).as_posix()] = digest(path)
    for path in sorted((REPO / "tests").glob(TEST_GLOB)):
        files[path.relative_to(REPO).as_posix()] = digest(path)
    return files


def specs_identity():
    """Canonical SHA-256 of each M1 specification (independent of line endings); they must be the approved ones."""
    current = {name: canonical_sha256(strict_json(P7 / path)) for name, path in SPEC_FILES.items()}
    if current != APPROVED_SPECS:
        changed = sorted(k for k in current if current[k] != APPROVED_SPECS[k])
        raise ValueError("P7 specifications differ from the operator-approved hashes: " + ", ".join(changed))
    return dict(current, host_check=canonical_sha256(strict_json(P7 / HOST_CHECK)))


def owner_identity():
    if digest(OWNER) != digest(P3_OWNER):
        raise ValueError("The P7 namespace owner is not byte-identical to the P3 owner the QGC build pins")
    return {"path": OWNER.relative_to(REPO).as_posix(), "sha256": digest(OWNER)}


def baselines(root):
    """C1: the accepted P6 manifest verifies (recomputing P5, which recomputes P3 and P4) to plan §0."""
    p6_frozen, p6_sha = p6_manifest.verify_current(root, Path(root) / P6_MANIFEST)
    if p6_sha != BASELINES["p6_sha256"]:
        raise ValueError("Accepted P6 baseline does not verify to " + BASELINES["p6_sha256"])
    if p6_frozen["baselines"] != {k: BASELINES[k] for k in ("p3_sha256", "p4_sha256", "p5_sha256")}:
        raise ValueError("The P6 manifest does not bind the accepted P3, P4 and P5 baselines")
    p4_frozen = strict_json(Path(root) / P4_MANIFEST)  # recomputed to c110bd30 inside the P5 verification above
    if canonical_sha256({k: v for k, v in p4_frozen.items() if k != "frozen_unix_ns"}) != BASELINES["p4_sha256"]:
        raise ValueError("The accepted P4 manifest file is not the verified c110bd30 candidate")
    return dict(BASELINES), p6_frozen, p4_frozen


def check_policy_approval(path, development):
    """A Policy A approval for a P7 candidate (D12); an approval of an earlier phase never transfers."""
    approval = strict_json(path)
    if not str(approval.get("candidate_phase", "")).startswith("P7"):
        raise PermissionError("The Policy A approval does not name a P7 candidate")
    if bool(approval.get("development_only")) != bool(development):
        raise PermissionError("A development-only Policy A approval binds development candidates only, and vice versa")
    return approval


def candidate(root, policy_approval, development=False):
    import p3_manifest
    check_policy_approval(policy_approval, development)
    bound, p6_frozen, p4_frozen = baselines(root)
    p5 = p6_frozen["p5"]
    return {"contract": CONTRACT, "p7_contract": "p7-evaluation-v1", "development": bool(development),
            "confirmatory_credit_possible": not development, "baselines": bound,
            "p4": {"manifest": P4_MANIFEST, "p3_baseline": p4_frozen["p3_baseline"]}, "p5": p5,
            "p7_source_files": p7_source_files(), "specs": specs_identity(), "namespace_owner": owner_identity(),
            "runtime": p3_manifest.runtime_identity(root, policy_name="A", policy_approval=policy_approval,
                                                    mc_benchmark_run=p5["mc_benchmark_run"]),
            "scopes": list(SCOPES), "execution_allowed_without_authorization": False}


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def freeze(root, output, policy_approval, development=False):
    if live_lock(root).exists():
        raise SystemExit("A live P7 sequence holds %s; no candidate may be frozen until it ends" % live_lock(root))
    value = candidate(root, policy_approval, development)
    value["frozen_unix_ns"] = time.time_ns()
    import subprocess
    value["frozen_git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    atomic_json(output, value, exclusive=True)
    return manifest_sha(value)


def verify_current(root, manifest_path):
    """Recompute everything; any difference blocks the runtime."""
    frozen = strict_json(manifest_path)
    if frozen.get("contract") != CONTRACT:
        raise ValueError("Not a P7 candidate manifest")
    current = candidate(root, frozen["runtime"]["native_command_policy"]["approval_path"], frozen["development"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    current["frozen_git_head"] = frozen.get("frozen_git_head")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P7 candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def source_drift(frozen):
    """Hash-bound P7 inputs that differ from the frozen candidate now (cheap; no P3-P6 recomputation)."""
    bound, current = frozen["p7_source_files"], p7_source_files()
    drift = sorted(k for k in set(bound) | set(current) if bound.get(k) != current.get(k))
    try:
        if specs_identity() != frozen["specs"]:
            drift.append("specs")
    except ValueError:
        drift.append("specs")
    if frozen["namespace_owner"]["sha256"] != digest(OWNER):
        drift.append(frozen["namespace_owner"]["path"])
    return drift


def runner_view(frozen, controller):
    """The `frozen` mapping the accepted P4 or P5 runner and its evaluator read, with P7's own runtime identity."""
    if controller == "p4":
        return {"p3_baseline": frozen["p4"]["p3_baseline"], "runtime": frozen["runtime"]}
    if controller == "p5":
        p5 = frozen["p5"]
        return {"model": p5["model"], "schema_sha256": p5["schema_sha256"], "controller_runtime": p5["controller_runtime"],
                "baselines": p5["baselines"], "runtime": frozen["runtime"]}
    raise ValueError("Unknown controller: " + str(controller))


# ---- the live-run lifecycle ----------------------------------------------------------------------------------
def live_lock(root):
    return Path(root) / LIVE_LOCK


def acquire_live_lock(root, sha):
    try:
        atomic_json(live_lock(root), {"manifest_sha256": sha, "pid": os.getpid(), "started_unix_ns": time.time_ns()},
                    exclusive=True)
    except FileExistsError:
        raise SystemExit("Another live P7 sequence holds %s" % live_lock(root))


def release_live_lock(root, sha):
    lock = live_lock(root)
    if lock.exists() and strict_json(lock).get("manifest_sha256") == sha and strict_json(lock).get("pid") == os.getpid():
        lock.unlink()


# ---- block authorizations (D12) ------------------------------------------------------------------------------
def block_count(scope):
    return 1 if scope == "pilot" else -(-int(scope.split("-")[1]) // 20)


def authorization_template(scope, block, sha, operator):
    if scope not in SCOPES or not 1 <= int(block) <= block_count(scope):
        raise ValueError("No block %s of scope %s" % (block, scope))
    return {"contract": AUTHORIZATION_CONTRACT, "scope": scope, "block": int(block), "manifest_sha256": sha,
            "schedule_canonical_sha256": APPROVED_SPECS["schedule"], "max_attempts": 1, "authorized": True,
            "authorized_by": operator, "simulated_flight_only": True,
            "statement": "I explicitly authorize exactly one execution of P7 %s block %d of manifest %s, in simulated "
                         "flight only (SITL, never real hardware)." % (scope, int(block), sha)}


def require_authorization(path, scope, block, sha):
    value = strict_json(path)
    expected = {"contract": AUTHORIZATION_CONTRACT, "scope": scope, "block": int(block), "manifest_sha256": sha,
                "schedule_canonical_sha256": APPROVED_SPECS["schedule"], "max_attempts": 1, "simulated_flight_only": True}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this block")
    return value


def marker_path(root, scope, block, sha):
    return Path(root) / "state" / ("p7-%s-block-%02d-%s.consumed.json" % (scope, int(block), sha))


def consume(root, scope, block, sha, authorization_path):
    """Exclusive, permanent consumption marker: a block can never run twice, and nothing retries automatically."""
    marker = marker_path(root, scope, block, sha)
    atomic_json(marker, {"scope": scope, "block": int(block), "manifest_sha256": sha,
                         "authorization_sha256": digest(authorization_path), "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


def result_path(root, scope, block, sha):
    return Path(root) / "state" / ("p7-%s-block-%02d-%s.result.json" % (scope, int(block), sha))


# ---- the schedule selection (analysis plan pilot rule) -------------------------------------------------------
def selection_path(root, sha):
    return Path(root) / "state" / ("p7-schedule-selection-%s.json" % sha)


def require_selection(root, sha, scope):
    """A confirmatory block runs only on the schedule the pilot rule selected for this manifest."""
    path = selection_path(root, sha)
    if not path.exists():
        raise PermissionError("No schedule selection recorded for manifest %s; the pilot rule decides it" % sha)
    value = strict_json(path)
    if value.get("contract") != SELECTION_CONTRACT or value.get("manifest_sha256") != sha:
        raise PermissionError("The schedule selection does not bind this manifest")
    if "confirmatory-%d" % int(value.get("pairs", 0)) != scope:
        raise PermissionError("The pilot rule selected %s pairs, not %s" % (value.get("pairs"), scope))
    return value


# ---- dry runs (development candidates, M4) -------------------------------------------------------------------
def dry_run_template(sha, operator, statement, items, live_runs_max):
    """`items`: [item, controller] flights the dry run may make, e.g. [["S2", "p4"], ["S2", "p5"]]."""
    return {"contract": DRY_RUN_CONTRACT, "manifest_sha256": sha, "authorized": True, "authorized_by": operator,
            "operator_statement": statement, "items": [list(i) for i in items], "live_runs_max": int(live_runs_max),
            "flight": True, "simulated_flight_only": True, "vehicle_armed": True, "acceptance_credit": False,
            "statement": "Dry run of P7 flights %s on development candidate %s: simulated flight only (SITL, the vehicle "
                         "is armed in simulation, never real hardware), no block result, no confirmatory credit."
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
        raise PermissionError("A P7 dry run needs an approval that names simulated flight explicitly")
    if any(len(i) != 2 or i[1] not in CONTROLLERS for i in value.get("items", [])) or not value.get("items"):
        raise PermissionError("A dry-run approval names its flights as [item, controller]")
    if frozen.get("development") is not True:
        raise PermissionError("Dry runs use a development candidate only")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="Identify, freeze, verify or authorize the P7 candidate; never starts runtime.")
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
    a.add_argument("--scope", choices=SCOPES, required=True)
    a.add_argument("--block", type=int, required=True)
    a.add_argument("--operator", required=True)
    a.add_argument("--confirm", required=True, help='Exactly: "authorize P7 <scope> block <n> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    d = sub.add_parser("approve-dry-run")
    d.add_argument("manifest", type=Path)
    d.add_argument("--operator", required=True)
    d.add_argument("--statement", required=True, help="the operator's chat statement that authorizes the dry run")
    d.add_argument("--item", nargs=2, action="append", metavar=("ITEM", "CONTROLLER"), required=True)
    d.add_argument("--live-runs-max", type=int, required=True)
    d.add_argument("--confirm", required=True, help='Exactly: "dry run P7 on <manifest-sha> in simulated flight"')
    d.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.command == "sources":
        print(json.dumps({"files": p7_source_files(), "specs": specs_identity()}, indent=2))
    elif args.command == "freeze":
        print(freeze(root, args.output, args.policy_approval, args.development))
    elif args.command == "verify":
        print(verify_current(root, args.manifest)[1])
    elif args.command == "authorize":
        frozen, sha = verify_current(root, args.manifest)
        if frozen["development"]:
            raise SystemExit("A development candidate is never authorized for a pilot or confirmatory block")
        if args.confirm != "authorize P7 %s block %d %s" % (args.scope, args.block, sha):
            raise SystemExit("Confirmation phrase must be exactly: authorize P7 %s block %d %s" % (args.scope, args.block, sha))
        atomic_json(args.output, authorization_template(args.scope, args.block, sha, args.operator), exclusive=True)
        print(args.output)
    else:
        frozen, sha = verify_current(root, args.manifest)
        if not frozen["development"]:
            raise SystemExit("Dry runs use a development candidate only")
        if args.confirm != "dry run P7 on %s in simulated flight" % sha:
            raise SystemExit("Confirmation phrase must be exactly: dry run P7 on %s in simulated flight" % sha)
        atomic_json(args.output, dry_run_template(sha, args.operator, args.statement, args.item, args.live_runs_max),
                    exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
