"""Explicit, stop-on-first-failure P3 gate sequencer. Runs exactly one gate per call.

Gate A: one ground readiness evaluation, then the seven ground fixtures in order.
Gate B: one sensor-equipped nominal flight smoke (requires an accepted Gate A).
Gate C: three consecutive qualification flights (requires an accepted Gate B).

Each gate needs its own operator authorization bound to the frozen manifest.
The authorization is consumed before the first runtime starts, so a failed or
interrupted gate can never be retried automatically. Every runtime is followed
by the independent offline evaluator before the next one may start. The next
gate is never entered automatically.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import time

SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM / "scripts"), str(SIM / "validation")]
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402
import p3_manifest  # noqa: E402
from p3_fixtures import FIXTURES  # noqa: E402

PLAN = {"A": [("readiness", None, "readiness")] + [("fixture", name, "fixture-" + name) for name in FIXTURES],
        "B": [("flight", None, "smoke")],
        "C": [("flight", None, f"qualification-{n}") for n in (1, 2, 3)]}
PLAN["V"] = list(PLAN["A"])  # same ground plan; llvmpipe; no acceptance credit, never a predecessor
# Stall attribution (docs/v3_sim_p3_stall_attribution.md): fixed order, one start each, never credit.
PLAN["D"] = [("diagnostic", variant, "diag-" + role) for variant, role in (
    ("baseline", "baseline"), ("no_rgb_observer", "no_rgb_observer"), ("recorder_none", "recorder_none"),
    ("no_probe", "no_probe"), ("baseline", "baseline_repeat"))]
PREDECESSOR = {"B": "A", "C": "B"}


def result_path(root, gate, sha):
    return Path(root) / "state" / f"p3-gate-{gate}-{sha}.result.json"


def require_predecessor(root, gate, sha):
    if gate not in PREDECESSOR:
        return None
    previous = result_path(root, PREDECESSOR[gate], sha)
    if not previous.exists():
        raise PermissionError(f"Gate {gate} requires an accepted Gate {PREDECESSOR[gate]} for this manifest")
    value = strict_json(previous)
    if value.get("accepted") is not True or value.get("manifest_sha256") != sha:
        raise PermissionError(f"Gate {PREDECESSOR[gate]} is not accepted for this exact manifest")
    for row in value["runs"]:
        evaluation = Path(row["run"]) / "p3-evaluation.json"
        if digest(evaluation) != row["evaluation_sha256"]:
            raise PermissionError("Predecessor evaluation changed after acceptance")
    return {"path": str(previous), "sha256": digest(previous)}


def execute_scope(scope, fixture, role, gate, manifest, authorization, flight):
    command = ["bash", str(SIM / "scripts/run_p3.sh"), "--run", "--scope", scope, "--gate", gate, "--role", role,
               "--manifest", str(manifest), "--authorization", str(authorization)]
    if fixture:
        command += ["--variant" if scope == "diagnostic" else "--fixture", fixture]
    if flight:
        command.append("--allow-simulated-flight")
    completed = subprocess.run(command, text=True, capture_output=True)
    found = re.search(r"^P3 evidence: (.+)$", completed.stdout, re.M)
    return completed, Path(found.group(1)) if found else None


def evaluate(run):
    completed = subprocess.run(["bash", str(SIM / "scripts/verify_p3_evidence.sh"), str(run)], text=True,
                               capture_output=True)
    path = run / "p3-evaluation.json"
    return completed, (strict_json(path) if path.exists() else None)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate", choices=sorted(PLAN), required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--allow-simulated-flight", action="store_true")
    args = parser.parse_args(argv)
    flight = args.gate in ("B", "C")
    required = ["GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"]
    if flight:
        required.append("GWM_ALLOW_SITL_COMMANDS")
        if not args.allow_simulated_flight:
            raise SystemExit("Gate %s requires --allow-simulated-flight" % args.gate)
    missing = [key for key in required if os.environ.get(key) != "1"]
    if missing:
        raise SystemExit("Missing explicit gates: " + ", ".join(missing))
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = p3_manifest.verify_current(root, args.manifest)
    # The runner refuses an MC build other than the frozen one; check the same explicit
    # GWM_PX4_MC_TRACE selection here, before the authorization is consumed.
    import mc_trace_runtime
    if mc_trace_runtime.selected_build(root, SIM) != frozen["runtime"]["mc_trace_build"]:
        raise SystemExit("GWM_PX4_MC_TRACE=%s must select the frozen MC trace build" % mc_trace_runtime.CONTRACT)
    authorization = p3_manifest.require_authorization(args.authorization, args.gate, sha)
    predecessor = require_predecessor(root, args.gate, sha)
    output = result_path(root, args.gate, sha)
    if output.exists():
        raise SystemExit("This gate already has a result for this manifest; results are never overwritten")
    marker = p3_manifest.consume(root, args.gate, sha, args.authorization)
    result = {"contract": "p3-gate-result-v1", "gate": args.gate, "scope": p3_manifest.GATES[args.gate],
              "manifest_sha256": sha, "authorization": {"path": str(args.authorization),
                                                         "sha256": digest(args.authorization),
                                                         "authorized_by": authorization["authorized_by"]},
              "consumption_marker": str(marker), "predecessor": predecessor, "started_unix_ns": time.time_ns(),
              "planned": [role for _, _, role in PLAN[args.gate]], "runs": [], "first_failure": None,
              "accepted": False, "acceptance_credit": args.gate in ("A", "B", "C"),
              "automatic_retry": False, "next_gate_started": False}
    try:
        for scope, fixture, role in PLAN[args.gate]:
            completed, run = execute_scope(scope, fixture, role, args.gate, args.manifest, args.authorization, flight)
            row = {"role": role, "run": str(run) if run else None, "runtime_exit_code": completed.returncode,
                   "runtime_stdout_tail": completed.stdout[-4000:], "runtime_stderr_tail": completed.stderr[-4000:]}
            result["runs"].append(row)
            if run is None:
                result["first_failure"] = role + ":runtime_did_not_create_evidence"
                break
            if args.gate == "D":
                # Diagnostics are measured by p3_stall_diagnostic.py, not acceptance-evaluated.
                if completed.returncode != 0:
                    result["first_failure"] = role + ":runtime_failed"
                    break
                continue
            evaluated, evaluation = evaluate(run)
            row.update(evaluator_exit_code=evaluated.returncode,
                       evaluation_status=evaluation.get("status") if evaluation else None,
                       evaluation_sha256=digest(run / "p3-evaluation.json") if evaluation else None,
                       evaluation_failures=(evaluation or {}).get("failures", ["evaluation_missing"])[:50])
            if completed.returncode != 0 or not evaluation or evaluation["status"] != "passed":
                result["first_failure"] = role + ":" + ((evaluation or {}).get("failures") or ["runtime_failed"])[0]
                break
        else:
            result["accepted"] = True
    finally:
        result["completed_unix_ns"] = time.time_ns()
        atomic_json(output, result, exclusive=True)
    print(("ACCEPTED" if result["accepted"] else "FAILED") + f" Gate {args.gate}: {output}", flush=True)
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
