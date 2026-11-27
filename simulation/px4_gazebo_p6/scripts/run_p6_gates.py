"""Explicit P6 gate sequencer (contract §8), and the M5 dry run of Gate A content (plan §7).

Gate A (contract §8.1), in the frozen order of `p6_scenarios.yaml`:
* A-1 P3 readiness unchanged (live);
* A-2 the P3, P4 and P5 manifests verify and the P6 manifest binds them (offline, from the verification);
* A-3 P6 readiness with every injector in pass-through (live);
* A-4 the M3 offline suite, run here and bound by hash to the manifest's sources (offline);
* A-5 the 12 ground fault fixtures (live; the vehicle is never armed);
* A-6 injector activation and cleanup, and A-7 ownership, Policy A and cleanup, evaluated over the runs.
Gates B, S and C (contract §8.2-8.4, M6): every row is a simulated flight on the row's world, run by the same
runner with `--allow-simulated-flight` and GWM_ALLOW_SITL_COMMANDS=1 (the accepted P5 flight gating). A flight gate
needs the accepted result of its predecessor gate for the same manifest (A -> B -> S -> C). A flight-content dry run
(development candidate) needs an approval that names simulated flight explicitly.

A gate needs its own operator authorization bound to a frozen, non-development P6 manifest. It is consumed
before the first runtime starts; the first failure stops the gate and nothing is retried. A dry run
(`--dry-run`) runs the same rows on a development candidate under a dry-run approval: every row is run and
recorded (a failure does not stop it), no gate result or marker is written, and nothing earns credit.
Every live row is evaluated (the unchanged P5 evaluator, then the P6 evaluator) before the next one starts.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

P6 = Path(__file__).resolve().parents[1]
REPO = P6.parents[1]
SIM = REPO / "simulation/px4_gazebo"
for path in (P6 / "scripts", SIM / "scripts", SIM / "validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p6_manifest  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

LIVE = ("p3_readiness", "ground_readiness", "ground_fixture", "startup_refusal", "flight")
FLIGHT = "flight"
PYTHON = "/usr/bin/python3"
NEVER_EXCEPTED = ("native_commands:", "cleanup:", "residual_namespace_processes", "artifact_hash_changed:")
# The one designed fail-closed exit the evaluator allows (p6_evaluate.FAIL_CLOSED_EXIT, operator decision 2026-11-07).
FAIL_CLOSED_EXIT = ("cleanup:ros-controller:nonzero_exit_1", ("A-5.F4.07", "A-5.F4.13"))


def rows(gate):
    return p6_manifest.scenarios()["gates"][gate]


def result_path(root, gate, sha):
    return Path(root) / "state" / f"p6-gate-{gate}-{sha}.result.json"


def dry_result_path(root, gate, sha):
    return Path(root) / "state" / f"p6-dryrun-{gate}-{sha[:12]}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}.result.json"


def runtime_command(row, role, gate, manifest, authorization, flight=False):
    command = ["bash", str(P6 / "scripts/run_p6.sh"), "--run", "--row", row, "--gate", gate, "--role", role,
               "--manifest", str(manifest), "--authorization", str(authorization)]
    return command + (["--allow-simulated-flight"] if flight else [])


def runtime_env(flight):
    """The accepted P5 flight gating: the SITL command gate is set for flight rows only, never for ground rows."""
    env = {k: v for k, v in os.environ.items() if k != "GWM_ALLOW_SITL_COMMANDS"}
    if flight:
        env["GWM_ALLOW_SITL_COMMANDS"] = "1"
    return env


def predecessor_accepted(root, gate, sha):
    """Contract §8: each gate after A needs the accepted predecessor for the same manifest."""
    before = p6_manifest.scenarios()["predecessor"].get(gate)
    if before is None:
        return None, None
    path = result_path(root, before, sha)
    result = strict_json(path) if path.exists() else None
    return before, bool(result and result.get("accepted") is True and result.get("manifest_sha256") == sha)


def execute_row(command, env=None):
    completed = subprocess.run(command, text=True, capture_output=True, env=env)
    found = re.search(r"^P6 evidence: (.+)$", completed.stdout, re.M)
    return completed, Path(found.group(1)) if found else None


def evaluate(run):
    completed = subprocess.run(["bash", str(P6 / "scripts/verify_p6_evidence.sh"), str(run)], text=True, capture_output=True)
    path = run / "p6/p6-evaluation.json"
    return completed, (strict_json(path) if path.exists() else None)


def offline_run_dir(root, label, dry):
    run = Path(root) / "runs" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + ("-p6dry-" if dry else "-p6-") + label
                                 + "-" + uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    return run


def offline_suite(root, frozen, dry):
    """A-4: the M3 suite under the controller interpreter, and every file it binds equal to the manifest's."""
    run = offline_run_dir(root, "A-4", dry)
    output = run / "p6-offline-suite.json"
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", PYTHONDONTWRITEBYTECODE="1")
    completed = subprocess.run([PYTHON, str(P6 / "validation/p6_offline_suite.py"), "--output", str(output)],
                               cwd=REPO, env=env, text=True, capture_output=True)
    report = strict_json(output) if output.exists() else {}
    bound = report.get("bound_files") or {}
    sources = frozen["p6_source_files"]
    differing = sorted(path for path, sha in bound.items() if sources.get(path) != sha)
    failures = []
    if completed.returncode != 0 or report.get("passed") is not True:
        failures.append("offline_suite_failed")
    if not bound or differing:
        failures.append("offline_suite_sources_differ_from_manifest")
    if report.get("interpreter") != PYTHON:
        failures.append("offline_suite_not_under_the_controller_interpreter")
    return {"run": str(run), "result_sha256": digest(output) if output.exists() else None,
            "required_offline": len(report.get("required_offline") or {}), "counts": report.get("counts"),
            "differing_sources": differing, "stdout_tail": completed.stdout[-2000:], "failures": failures}


def aggregate(row_id, records):
    """A-6 (injectors) and A-7 (ownership, Policy A, cleanup) over every evaluated live row."""
    live = [r for r in records if r.get("evaluation") is not None]
    failures = []
    for r in live:
        evaluation = r["evaluation"]
        if row_id == "A-6":
            check = (evaluation.get("response") or {}).get("INJECTORS") or {}
            if not check.get("passed"):
                failures.append("injectors:" + r["row"])
        else:
            refused = evaluation.get("p5_failures_refused") or []
            allowed = [x["item"] for x in evaluation.get("p5_failures_allowed") or []]
            designed = [x for x in allowed if r["row"] in FAIL_CLOSED_EXIT[1] and x == FAIL_CLOSED_EXIT[0]]
            bad = [x for x in refused + allowed if x.startswith(NEVER_EXCEPTED) and x not in designed]
            failures += [r["row"] + ":" + x for x in bad]
    if not live:
        failures.append("no_evaluated_live_rows")
    return {"rows_evaluated": [r["row"] for r in live], "failures": failures}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate", choices=sorted(p6_manifest.GATES), required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true", help="M5: a development candidate under a dry-run approval")
    parser.add_argument("--rows", nargs="*", help="dry run only: a subset of the gate's rows")
    args = parser.parse_args(argv)
    if args.rows and not args.dry_run:
        raise SystemExit("A gate always runs every row")
    missing = [key for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
               if os.environ.get(key) != "1"]
    if missing:
        raise SystemExit("Missing explicit gates: " + ", ".join(missing))
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = p6_manifest.verify_current(root, args.manifest)  # P3, P4 and P5 re-verified (C1)
    import mc_trace_runtime
    if mc_trace_runtime.selected_build(root, SIM) != frozen["runtime"]["mc_trace_build"]:
        raise SystemExit("GWM_PX4_MC_TRACE=%s must select the frozen MC trace build" % mc_trace_runtime.CONTRACT)
    plan = rows(args.gate)
    if args.dry_run:
        approval = p6_manifest.require_dry_run_approval(args.authorization, sha, frozen, args.gate)
        if args.rows:
            unknown = sorted(set(args.rows) - {r["id"] for r in plan})
            if unknown:
                raise SystemExit("Not rows of Gate %s: %s" % (args.gate, ", ".join(unknown)))
            plan = [r for r in plan if r["id"] in args.rows]
        output, marker, gate_flag, operator = dry_result_path(root, args.gate, sha), None, p6_manifest.DRY_RUN, approval["authorized_by"]
    else:
        if frozen["development"]:
            raise SystemExit("A development candidate never runs a gate")
        authorization = p6_manifest.require_authorization(args.authorization, args.gate, sha)
        output = result_path(root, args.gate, sha)
        if output.exists():
            raise SystemExit("This P6 gate already has a result for this manifest; results are never overwritten")
        before, accepted = predecessor_accepted(root, args.gate, sha)
        if before is not None and not accepted:
            raise SystemExit("Gate %s needs the accepted Gate %s result for manifest %s" % (args.gate, before, sha))
        marker, gate_flag, operator = p6_manifest.consume(root, args.gate, sha, args.authorization), args.gate, authorization["authorized_by"]
    p6_manifest.acquire_live_lock(root, sha)  # no freeze and no second sequence while this one runs
    result = {"contract": "p6-dry-run-result-v1" if args.dry_run else "p6-gate-result-v1", "gate": args.gate,
              "scope": p6_manifest.GATES[args.gate], "manifest_sha256": sha, "development": frozen["development"],
              "baselines": frozen["baselines"], "authorization": {"path": str(args.authorization), "sha256": digest(args.authorization),
                                                                  "authorized_by": operator},
              "consumption_marker": str(marker) if marker else None, "started_unix_ns": time.time_ns(),
              "planned": [r["id"] for r in plan], "runs": [], "first_failure": None, "accepted": False,
              "acceptance_credit": not args.dry_run, "automatic_retry": False, "dry_run": args.dry_run}
    try:
        for row in plan:
            record = {"row": row["id"], "role": row["role"], "scope": row["scope"]}
            result["runs"].append(record)
            drift = p6_manifest.source_drift(frozen)
            if drift:  # a changed candidate ends the sequence: its later rows are not run, never counted as passed
                record.update(failures=["candidate_changed_during_run:" + ",".join(drift[:8])], passed=False)
                result["first_failure"] = result["first_failure"] or row["id"] + ":" + record["failures"][0]
                result["candidate_changed_during_run"] = drift
                break
            if row["scope"] in LIVE:
                flight = row["scope"] == FLIGHT
                completed, run = execute_row(runtime_command(row["id"], row["role"], gate_flag, args.manifest, args.authorization, flight),
                                             runtime_env(flight))
                record.update(run=str(run) if run else None, runtime_exit_code=completed.returncode,
                              runtime_stdout_tail=completed.stdout[-4000:], runtime_stderr_tail=completed.stderr[-4000:])
                if run is None:
                    record["failures"] = ["runtime_did_not_create_evidence"]
                else:
                    evaluated, evaluation = evaluate(run)
                    record.update(evaluator_exit_code=evaluated.returncode, evaluator_stderr_tail=evaluated.stderr[-2000:],
                                  evaluation=evaluation, evaluation_status=(evaluation or {}).get("status"),
                                  evaluation_sha256=digest(run / "p6/p6-evaluation.json") if evaluation else None,
                                  p5_evaluation_sha256=digest(run / "p5-evaluation.json") if (run / "p5-evaluation.json").exists() else None,
                                  failures=(evaluation or {}).get("failures", ["evaluation_missing"]))
                    if evaluation and evaluation.get("status") != "passed" and not record["failures"]:
                        record["failures"] = ["evaluation_" + str(evaluation.get("status"))]
            elif row["id"] == "A-2":
                record.update(baselines=frozen["baselines"], p5=frozen["p5"]["manifest"], failures=[])
            elif row["id"] == "A-4":
                record.update(offline_suite(root, frozen, args.dry_run))
            elif row["scope"] == "evaluation":
                record.update(aggregate(row["id"], result["runs"][:-1]))
            else:
                record["failures"] = ["row_scope_not_supported:" + row["scope"]]
            record["passed"] = not record["failures"]
            if not record["passed"] and result["first_failure"] is None:
                result["first_failure"] = row["id"] + ":" + record["failures"][0]
                if not args.dry_run:
                    break
        else:
            result["accepted"] = result["first_failure"] is None and not args.dry_run
        result["all_rows_passed"] = len(result["runs"]) == len(plan) and all(r.get("passed") for r in result["runs"])
        result["not_run"] = [r["id"] for r in plan][len(result["runs"]):]
    finally:
        result["completed_unix_ns"] = time.time_ns()
        result["candidate_unchanged_at_end"] = not p6_manifest.source_drift(frozen)
        atomic_json(output, result, exclusive=True)
        p6_manifest.release_live_lock(root, sha)
    label = "DRY RUN" if args.dry_run else ("ACCEPTED" if result["accepted"] else "FAILED")
    print(f"{label} P6 Gate {args.gate}: {output}", flush=True)
    return 0 if (result["accepted"] or (args.dry_run and result["all_rows_passed"])) else 1


if __name__ == "__main__":
    raise SystemExit(main())
