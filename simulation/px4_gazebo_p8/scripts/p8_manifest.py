"""P8 candidate identity, bindings, operator authorization and block bookkeeping (contract `p8-navigation-recovery-v1`
§2, §8, §13). Nothing here starts runtime.

**What the P8 candidate binds:**
* **The accepted P7-E candidate `e9561f05…`,** verified through the accepted `p7e_manifest.verify_current`. That
  re-verifies the frozen P7 candidate `0799c37e`, and P3–P6 inside it (B1).
* **The four approved P8-M1 specifications,** by canonical SHA-256 (B6): layouts `62ae950c…`, schedule `df90cf7b…`,
  recovery `dee90d51…` and report plan `a313a034…`.
* **The offline-stage records** V1 and V2 (H11), by file SHA-256.
* **The P8 tree** (configs, scripts, validation, replay, analysis, the ROS package) and the P8 tests, file by file.
  The tests include V0.
* **The P8 build receipt:** the built package must equal the package source.
* **The runtime identity,** with a Policy A approval that names P8. An earlier phase's approval never transfers.
* **The namespace-owner copy,** byte-identical to the P3 owner.

**Candidates.** A development candidate runs dry runs only. A real candidate runs the 3 blocks (V3, V4, V5) of scope
`p8-validation-60`, each under its own authorization (`max_attempts` 1, a permanent marker). The live lock refuses a
freeze or a second sequence while a P8, P7-E or P7 sequence runs.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
SIM, P7, P7E = REPO / "simulation/px4_gazebo", REPO / "simulation/px4_gazebo_p7", REPO / "simulation/px4_gazebo_p7e"
for _path in (SIM / "scripts", SIM / "validation", P7 / "scripts", P7E / "scripts", P8 / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7_manifest  # noqa: E402  (frozen P7 code, used unchanged)
import p7e_manifest  # noqa: E402  (accepted P7-E code, used unchanged)
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p8-candidate-manifest-v1"
SOURCE_FOLDERS = ("configs", "scripts", "validation", "replay", "analysis", "ros2_ws/src/gwm_navigation_recovery")
TEST_GLOB = "test_px4_gazebo_p8_*.py"
P7E_MANIFEST = "state/p7e-candidate-2b6ee5d-r2.json"
P7E_SHA256 = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
SPEC_FILES = {"layouts": "configs/p8_layouts.json", "schedule": "configs/p8_schedule.json",
              "recovery": "configs/p8_recovery.json", "report_plan": "configs/p8_report_plan.yaml"}
APPROVED_SPECS = {"layouts": "62ae950cbcce9981b013992ed7a8d22e1bcbc4436852745e249172de25b40e85",
                  "schedule": "df90cf7ba1a172a100cad1d5f72cf864088190331ecf11f2af4263f5f8f3da93",
                  "recovery": "dee90d51098210daf714fe0b875b0e98040195362095800dc48836f78371333b",
                  "report_plan": "a313a034b2c8e481c0a1de651dc9ebc530bdf86736c4577216a281db429c2b14"}
OFFLINE_RECORDS = {"V1": "docs/evidence/v3_sim_p8_v1_replay.json", "V2": "docs/evidence/v3_sim_p8_v2_perception.json"}
SCOPE, BLOCKS = "p8-validation-60", 3
STAGES = {1: "V3", 2: "V4", 3: "V5"}
DRY_RUN = "DRY"
AUTHORIZATION_CONTRACT = "p8-runtime-authorization-v1"
DRY_RUN_CONTRACT = "p8-dry-run-approval-v1"
VARIANTS = {"p8_p4r": "p4", "p8_p5r": "p5"}  # variant -> the accepted stack it wraps
OWNER = P8 / "scripts/p8_namespace_owner.py"
P3_OWNER = SIM / "scripts/p3_namespace_owner.py"
LIVE_LOCK = "state/p8-live-run.lock"
BUILD_RECEIPT = "state/p8-built.json"
PACKAGE = P8 / "ros2_ws/src/gwm_navigation_recovery"


# ---- identity -----------------------------------------------------------------------------------------------
def p8_source_files():
    files = {}
    for folder in SOURCE_FOLDERS:
        for path in sorted((P8 / folder).rglob("*")) if (P8 / folder).exists() else ():
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                files[path.relative_to(REPO).as_posix()] = digest(path)
    for path in sorted((REPO / "tests").glob(TEST_GLOB)):
        files[path.relative_to(REPO).as_posix()] = digest(path)
    return files


def specs_identity():
    current = {name: canonical_sha256(strict_json(P8 / path)) for name, path in SPEC_FILES.items()}
    if current != APPROVED_SPECS:
        raise ValueError("P8 specifications differ from the approved hashes: "
                         + ", ".join(sorted(k for k in current if current[k] != APPROVED_SPECS[k])))
    return current


def offline_records():
    """H11: the V1 and V2 records, which must exist and record a pass."""
    out = {}
    for stage, path in OFFLINE_RECORDS.items():
        value = strict_json(REPO / path)
        if (value.get("summary") or {}).get("passed") is not True:
            raise ValueError("The %s record does not record a pass" % stage)
        out[stage] = {"path": path, "sha256": digest(REPO / path)}
    return out


def owner_identity():
    if digest(OWNER) != digest(P3_OWNER):
        raise ValueError("The P8 namespace owner is not byte-identical to the P3 owner the QGC build pins")
    return {"path": OWNER.relative_to(REPO).as_posix(), "sha256": digest(OWNER)}


def package_manifest():
    return {p.relative_to(PACKAGE).as_posix(): digest(p) for p in sorted(PACKAGE.rglob("*"))
            if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"}


def package_hash(manifest=None):
    return hashlib.sha256(json.dumps(manifest or package_manifest(), sort_keys=True).encode()).hexdigest()


def build_identity(root):
    path = Path(root) / BUILD_RECEIPT
    if not path.exists():
        raise ValueError("The P8 package is not built (%s missing)" % BUILD_RECEIPT)
    receipt = strict_json(path)
    if receipt.get("package_hash") != package_hash():
        raise ValueError("The P8 build differs from the package source")
    return {"receipt": BUILD_RECEIPT, "receipt_sha256": digest(path), "package_hash": receipt["package_hash"],
            "install": receipt["install"]}


def p7e_binding(root):
    frozen, sha = p7e_manifest.verify_current(root, Path(root) / P7E_MANIFEST)  # P7 0799c37e and P3-P6 inside it
    if sha != P7E_SHA256:
        raise ValueError("The accepted P7-E candidate does not verify to " + P7E_SHA256)
    return frozen, sha


def check_policy_approval(path, development):
    approval = strict_json(path)
    if not str(approval.get("candidate_phase", "")).startswith("P8"):
        raise PermissionError("The Policy A approval does not name a P8 candidate")
    if bool(approval.get("development_only")) != bool(development):
        raise PermissionError("A development-only Policy A approval binds development candidates only, and vice versa")
    return approval


def candidate(root, policy_approval, development=False):
    import p3_manifest
    check_policy_approval(policy_approval, development)
    p7e_frozen, p7e_sha = p7e_binding(root)
    return {"contract": CONTRACT, "p8_contract": "p8-navigation-recovery-v1", "development": bool(development),
            "engineering_credit_possible": not development,
            "baselines": dict(p7e_frozen["baselines"], p7e_sha256=p7e_sha),
            "p7e": {"manifest": P7E_MANIFEST, "sha256": p7e_sha,
                    "source_files_canonical_sha256": canonical_sha256(p7e_frozen["p7e_source_files"])},
            "p4": p7e_frozen["p4"], "p5": p7e_frozen["p5"],
            "p8_source_files": p8_source_files(), "specs": specs_identity(), "offline_records": offline_records(),
            "build": build_identity(root), "namespace_owner": owner_identity(),
            "runtime": p3_manifest.runtime_identity(root, policy_name="A", policy_approval=policy_approval,
                                                    mc_benchmark_run=p7e_frozen["p5"]["mc_benchmark_run"]),
            "scope": SCOPE, "blocks": BLOCKS, "execution_allowed_without_authorization": False}


def manifest_sha(value):
    return canonical_sha256({k: v for k, v in value.items() if k != "frozen_unix_ns"})


def freeze(root, output, policy_approval, development=False):
    if live_locks(root):
        raise SystemExit("A live sequence holds %s; no candidate may be frozen" % live_locks(root))
    value = candidate(root, policy_approval, development)
    value["frozen_unix_ns"] = time.time_ns()
    value["frozen_git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    atomic_json(output, value, exclusive=True)
    return manifest_sha(value)


def verify_current(root, manifest_path):
    frozen = strict_json(manifest_path)
    if frozen.get("contract") != CONTRACT:
        raise ValueError("Not a P8 candidate manifest")
    current = candidate(root, frozen["runtime"]["native_command_policy"]["approval_path"], frozen["development"])
    current["frozen_unix_ns"] = frozen.get("frozen_unix_ns")
    current["frozen_git_head"] = frozen.get("frozen_git_head")
    if current != frozen:
        differing = sorted(k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k))
        raise ValueError("Current inputs differ from the frozen P8 candidate: " + ",".join(differing))
    return frozen, manifest_sha(frozen)


def source_drift(frozen, root=None):
    """Hash-bound P8 inputs, and the P7-E and P7 inputs P8 imports, that differ now (FC-7)."""
    root = Path(root or os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    bound, current = frozen["p8_source_files"], p8_source_files()
    drift = sorted(k for k in set(bound) | set(current) if bound.get(k) != current.get(k))
    try:
        if specs_identity() != frozen["specs"]:
            drift.append("specs")
    except ValueError:
        drift.append("specs")
    try:
        if offline_records() != frozen["offline_records"]:
            drift.append("offline_records")
    except ValueError:
        drift.append("offline_records")
    if frozen["namespace_owner"]["sha256"] != digest(OWNER):
        drift.append(frozen["namespace_owner"]["path"])
    p7e_frozen = strict_json(root / P7E_MANIFEST)
    drift += ["p7e:" + d for d in p7e_manifest.source_drift(p7e_frozen, root)]
    return drift


def runner_view(frozen, base_controller):
    return p7_manifest.runner_view(frozen, base_controller)


# ---- live locks (P8, P7-E and P7 exclude each other) -------------------------------------------------------------
def live_lock(root):
    return Path(root) / LIVE_LOCK


def live_locks(root):
    return [str(live_lock(root))] * live_lock(root).exists() + p7e_manifest.live_locks(root)


def acquire_live_lock(root, sha):
    if p7e_manifest.live_locks(root):
        raise SystemExit("A live P7 or P7-E sequence holds %s" % p7e_manifest.live_locks(root))
    try:
        atomic_json(live_lock(root), {"manifest_sha256": sha, "pid": os.getpid(), "started_unix_ns": time.time_ns()},
                    exclusive=True)
    except FileExistsError:
        raise SystemExit("Another live P8 sequence holds %s" % live_lock(root))


def release_live_lock(root, sha):
    lock = live_lock(root)
    if lock.exists() and strict_json(lock).get("manifest_sha256") == sha and strict_json(lock).get("pid") == os.getpid():
        lock.unlink()


# ---- block authorizations ---------------------------------------------------------------------------------------
def authorization_template(block, sha, operator):
    if int(block) not in STAGES:
        raise ValueError("No block %s of scope %s" % (block, SCOPE))
    return {"contract": AUTHORIZATION_CONTRACT, "scope": SCOPE, "block": int(block), "stage": STAGES[int(block)],
            "manifest_sha256": sha, "schedule_canonical_sha256": APPROVED_SPECS["schedule"], "max_attempts": 1,
            "authorized": True, "authorized_by": operator, "simulated_flight_only": True,
            "statement": "I explicitly authorize exactly one execution of P8 %s block %d (%s) of manifest %s, in "
                         "simulated flight only (SITL, never real hardware)." % (SCOPE, int(block), STAGES[int(block)], sha)}


def require_authorization(path, block, sha):
    value = strict_json(path)
    expected = {"contract": AUTHORIZATION_CONTRACT, "scope": SCOPE, "block": int(block), "stage": STAGES.get(int(block)),
                "manifest_sha256": sha, "schedule_canonical_sha256": APPROVED_SPECS["schedule"], "max_attempts": 1,
                "simulated_flight_only": True}
    for key, item in expected.items():
        if value.get(key) != item:
            raise PermissionError("Authorization does not bind " + key)
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Authorization is not an explicit operator approval for this block")
    return value


def marker_path(root, block, sha):
    return Path(root) / "state" / ("p8-%s-block-%02d-%s.consumed.json" % (SCOPE, int(block), sha))


def consume(root, block, sha, authorization_path):
    marker = marker_path(root, block, sha)
    atomic_json(marker, {"scope": SCOPE, "block": int(block), "manifest_sha256": sha,
                         "authorization_sha256": digest(authorization_path), "consumed_unix_ns": time.time_ns()}, exclusive=True)
    return marker


def result_path(root, block, sha):
    return Path(root) / "state" / ("p8-%s-block-%02d-%s.result.json" % (SCOPE, int(block), sha))


# ---- dry runs (development candidates) ---------------------------------------------------------------------------
def dry_run_template(sha, operator, statement, items, live_runs_max):
    return {"contract": DRY_RUN_CONTRACT, "manifest_sha256": sha, "authorized": True, "authorized_by": operator,
            "operator_statement": statement, "items": [list(i) for i in items], "live_runs_max": int(live_runs_max),
            "flight": True, "simulated_flight_only": True, "vehicle_armed": True, "acceptance_credit": False,
            "statement": "Dry run of P8 flights %s on development candidate %s: simulated flight only (SITL, never real "
                         "hardware), no block result, no engineering credit." % (", ".join("%s/%s" % tuple(i) for i in items), sha)}


def require_dry_run_approval(path, sha, frozen):
    value = strict_json(path)
    if value.get("contract") != DRY_RUN_CONTRACT or value.get("manifest_sha256") != sha:
        raise PermissionError("Dry-run approval does not bind this development candidate")
    if value.get("authorized") is not True or not value.get("authorized_by"):
        raise PermissionError("Dry-run approval is not an explicit operator approval")
    if value.get("acceptance_credit") is not False:
        raise PermissionError("A dry run never earns credit")
    if value.get("flight") is not True or value.get("simulated_flight_only") is not True:
        raise PermissionError("A P8 dry run needs an approval that names simulated flight explicitly")
    if not value.get("items") or any(len(i) != 2 or i[1] not in VARIANTS for i in value["items"]):
        raise PermissionError("A dry-run approval names its flights as [item, variant]")
    if frozen.get("development") is not True:
        raise PermissionError("Dry runs use a development candidate only")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description="Identify, freeze, verify or authorize the P8 candidate; never starts runtime.")
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
    a.add_argument("--confirm", required=True, help='Exactly: "authorize P8 p8-validation-60 block <n> <manifest-sha>"')
    a.add_argument("--output", type=Path, required=True)
    d = sub.add_parser("approve-dry-run")
    d.add_argument("manifest", type=Path)
    d.add_argument("--operator", required=True)
    d.add_argument("--statement", required=True)
    d.add_argument("--item", nargs=2, action="append", metavar=("ITEM", "VARIANT"), required=True)
    d.add_argument("--live-runs-max", type=int, required=True)
    d.add_argument("--confirm", required=True, help='Exactly: "dry run P8 on <manifest-sha> in simulated flight"')
    d.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.command == "sources":
        print(json.dumps({"files": p8_source_files(), "specs": specs_identity()}, indent=2))
    elif args.command == "freeze":
        print(freeze(root, args.output, args.policy_approval, args.development))
    elif args.command == "verify":
        print(verify_current(root, args.manifest)[1])
    elif args.command == "authorize":
        frozen, sha = verify_current(root, args.manifest)
        if frozen["development"]:
            raise SystemExit("A development candidate is never authorized for a block")
        phrase = "authorize P8 %s block %d %s" % (SCOPE, args.block, sha)
        if args.confirm != phrase:
            raise SystemExit("Confirmation phrase must be exactly: " + phrase)
        atomic_json(args.output, authorization_template(args.block, sha, args.operator), exclusive=True)
        print(args.output)
    else:
        frozen, sha = verify_current(root, args.manifest)
        if not frozen["development"]:
            raise SystemExit("Dry runs use a development candidate only")
        phrase = "dry run P8 on %s in simulated flight" % sha
        if args.confirm != phrase:
            raise SystemExit("Confirmation phrase must be exactly: " + phrase)
        atomic_json(args.output, dry_run_template(sha, args.operator, args.statement, args.item, args.live_runs_max),
                    exclusive=True)
        print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
