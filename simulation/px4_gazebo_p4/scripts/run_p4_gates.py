"""Explicit, stop-on-first-failure P4 gate sequencer (contract §8). Runs exactly one gate per call.

Gate A: P3 readiness unchanged, P4 readiness under load, ground fixtures G2/G4/G6, fault fixtures.
Gate B: one S2 smoke flight (requires an accepted Gate A).
Gate S: S1, S5, S6 safety flights in order (requires an accepted Gate B).
Gate C: S2, S3, S4 qualification flights in order (requires an accepted Gate S).

Each gate needs its own operator authorization bound to the frozen P4 manifest; it is
consumed before the first runtime starts, so nothing is ever retried automatically.
Every runtime is evaluated by the independent P4 evaluator before the next starts.
No P3 gate result grants P4 credit.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import time

P4 = Path(__file__).resolve().parents[1]
SIM = P4.parents[0] / "px4_gazebo"
for path in (P4 / "scripts", SIM / "scripts", SIM / "validation"):
    if str(path) not in sys.path:
        sys.path.append(str(path))
import p4_manifest  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

FLIGHT_GATES = p4_manifest.FLIGHT_GATES


def plan(gate):
    return [tuple(row) for row in p4_manifest.validation()["gates"][gate]]


def result_path(root, gate, sha):
    return Path(root) / "state" / f"p4-gate-{gate}-{sha}.result.json"


def require_predecessor(root, gate, sha):
    predecessor = p4_manifest.validation()["predecessor"].get(gate)
    if predecessor is None:
        return None
    previous = result_path(root, predecessor, sha)
    if not previous.exists():
        raise PermissionError(f"P4 Gate {gate} requires an accepted Gate {predecessor} for this manifest")
    value = strict_json(previous)
    if value.get("accepted") is not True or value.get("manifest_sha256") != sha:
        raise PermissionError(f"P4 Gate {predecessor} is not accepted for this exact manifest")
    for row in value["runs"]:
        if digest(Path(row["run"]) / "p4-evaluation.json") != row["evaluation_sha256"]:
            raise PermissionError("Predecessor evaluation changed after acceptance")
    return {"path": str(previous), "sha256": digest(previous)}


def runtime_command(scope, item, role, gate, manifest, authorization):
    command = ["bash", str(P4 / "scripts/run_p4.sh"), "--run", "--scope", scope, "--gate", gate, "--role", role,
               "--manifest", str(manifest), "--authorization", str(authorization)]
    if scope in ("fixture", "fault"):
        command += ["--fixture", item]
    if scope == "flight":
        command += ["--scenario", item, "--allow-simulated-flight"]
    return command


def execute_scope(command):
    completed = subprocess.run(command, text=True, capture_output=True)
    found = re.search(r"^P4 evidence: (.+)$", completed.stdout, re.M)
    return completed, Path(found.group(1)) if found else None


def evaluate(run):
    completed = subprocess.run(["bash", str(P4 / "scripts/verify_p4_evidence.sh"), str(run)], text=True,
                               capture_output=True)
    path = run / "p4-evaluation.json"
    return completed, (strict_json(path) if path.exists() else None)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate", choices=sorted(p4_manifest.GATES), required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    args = parser.parse_args(argv)
    flight = args.gate in FLIGHT_GATES
    required = ["GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"]
    if flight:
        required.append("GWM_ALLOW_SITL_COMMANDS")
        if not args.allow_simulated_flight:
            raise SystemExit("P4 Gate %s requires --allow-simulated-flight" % args.gate)
    missing = [key for key in required if os.environ.get(key) != "1"]
    if missing:
        raise SystemExit("Missing explicit gates: " + ", ".join(missing))
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = p4_manifest.verify_current(root, args.manifest)
    import mc_trace_runtime
    if mc_trace_runtime.selected_build(root, SIM) != frozen["runtime"]["mc_trace_build"]:
        raise SystemExit("GWM_PX4_MC_TRACE=%s must select the frozen MC trace build" % mc_trace_runtime.CONTRACT)
    authorization = p4_manifest.require_authorization(args.authorization, args.gate, sha)
    predecessor = require_predecessor(root, args.gate, sha)
    output = result_path(root, args.gate, sha)
    if output.exists():
        raise SystemExit("This P4 gate already has a result for this manifest; results are never overwritten")
    marker = p4_manifest.consume(root, args.gate, sha, args.authorization)
    rows = plan(args.gate)
    result = {"contract": "p4-gate-result-v1", "gate": args.gate, "scope": p4_manifest.GATES[args.gate],
              "manifest_sha256": sha, "p3_baseline": frozen["p3_baseline"],
              "authorization": {"path": str(args.authorization), "sha256": digest(args.authorization),
                                "authorized_by": authorization["authorized_by"]},
              "consumption_marker": str(marker), "predecessor": predecessor, "started_unix_ns": time.time_ns(),
              "planned": [role for _, _, role in rows], "runs": [], "first_failure": None, "accepted": False,
              "acceptance_credit": True, "automatic_retry": False, "next_gate_started": False}
    try:
        for scope, item, role in rows:
            completed, run = execute_scope(runtime_command(scope, item, role, args.gate, args.manifest, args.authorization))
            row = {"role": role, "scope": scope, "item": item, "run": str(run) if run else None,
                   "runtime_exit_code": completed.returncode, "runtime_stdout_tail": completed.stdout[-4000:],
                   "runtime_stderr_tail": completed.stderr[-4000:]}
            result["runs"].append(row)
            if run is None:
                result["first_failure"] = role + ":runtime_did_not_create_evidence"
                break
            evaluated, evaluation = evaluate(run)
            row.update(evaluator_exit_code=evaluated.returncode,
                       evaluation_status=evaluation.get("status") if evaluation else None,
                       evaluation_sha256=digest(run / "p4-evaluation.json") if evaluation else None,
                       evaluation_failures=(evaluation or {}).get("failures", ["evaluation_missing"])[:50],
                       outcome=(evaluation or {}).get("outcome"))
            if completed.returncode != 0 or not evaluation or evaluation["status"] != "passed":
                result["first_failure"] = role + ":" + ((evaluation or {}).get("failures") or ["runtime_failed"])[0]
                break
        else:
            result["accepted"] = True
    finally:
        result["completed_unix_ns"] = time.time_ns()
        atomic_json(output, result, exclusive=True)
    print(("ACCEPTED" if result["accepted"] else "FAILED") + f" P4 Gate {args.gate}: {output}", flush=True)
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
