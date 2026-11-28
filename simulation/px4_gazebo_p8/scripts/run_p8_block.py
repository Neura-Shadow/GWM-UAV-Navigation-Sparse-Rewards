"""Explicit P8 block sequencer, with fail-closed rules FC-1..FC-7 (contract `p8-navigation-recovery-v1` §6, §8, §13).
Nothing runs without --run inputs.

**One invocation, one block:** one of the 3 blocks of scope `p8-validation-60`, or, on a development candidate, a dry
run.
* Block 1 is V3, block 2 V4, block 3 V5: 10 pairs and 20 flights each.

**Reused unchanged, by import:**
* from the frozen P7 sequencer, through the accepted P7-E one:
  * the replacement and stop rules (`BlockRules`);
  * the sequence loop (`run_sequence`) and the fsynced `Ledger`;
  * the accepted P4/P5 evidence scripts;
* from the accepted P7-E sequencer:
  * FC-1 (`Tracker`, `sequence`, `stop_reason`, `unflown`, `accounting`);
  * the FC-2 host and process checks (`p7e_host`);
  * FC-3 (`p7e_assess.assess`).

**Each flight:**
1. It runs through `run_p8.sh`.
2. It is evaluated by the unchanged P4 or P5 evidence script, for the stack the variant wraps.
3. It is classified by the frozen P7 classifier, then assessed by FC-3.
4. **P8 adds the H10 check** (`validation/p8_h10_check.py`):
   * a missing or unparseable recovery ledger, or an action whose frozen check does not reproduce, makes the flight
     safety-unverified (FC-5);
   * an action on a stale tick or beyond its budget is a safety event (FC-6).
   * Either one stops the batch.
5. The assessment is written to `p8/flight-assessment.json`, and the H10 record to `p8/h10-record.json`.

**FC-7:** drift of any P8 input, or of the P7-E and P7 inputs P8 imports, stops the block.

Valid flights are never re-flown, nothing is retried automatically, and the result is written once (exclusive).
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
P7E = REPO / "simulation/px4_gazebo_p7e"
for _path in (P7E / "scripts", P7E / "validation", P8 / "scripts", P8 / "validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import run_p7e_block as p7eb  # noqa: E402  (accepted P7-E code, used unchanged)
import p7e_assess  # noqa: E402
import p7e_host  # noqa: E402
import p8_h10_check  # noqa: E402
import p8_manifest  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

RUN_P8 = P8 / "scripts/run_p8.sh"
EVIDENCE_LINE = re.compile(r"^P8 evidence: (.+)$", re.M)
ASSESSMENT, H10_RECORD = "p8/flight-assessment.json", "p8/h10-record.json"
FC5 = "safety_unverified:fc5_recovery_ledger"
FC6 = "safety_event:fc6_recovery_invariant"
FC6_MARKERS = ("action on a stale tick", "budget exceeded", "outside its episode window")


def schedule():
    return strict_json(P8 / p8_manifest.SPEC_FILES["schedule"])


def plan_flights(sched, block):
    rows = [r for r in sched["slots"] if r["block"] == int(block)]
    if not rows:
        raise SystemExit("No block %s in the approved P8 schedule" % block)
    flights = [{"slot": r["slot"], "block": r["block"], "stage": r["stage"], "pair_in_block": r["pair_in_block"],
                "layout": r["layout"], "variant": v, "controller": p8_manifest.VARIANTS[v], "position": i, "attempt": 1}
               for r in rows for i, v in enumerate(r["order"])]
    return [dict(f, plan_index=n) for n, f in enumerate(flights)]


def dry_flights(approval):
    return [{"slot": 0, "block": 0, "stage": "DRY", "pair_in_block": 0, "layout": item, "variant": variant,
             "controller": p8_manifest.VARIANTS[variant], "position": 0, "attempt": 1, "plan_index": n}
            for n, (item, variant) in enumerate(approval["items"])]


def assess(run, params=None, classifier=None):
    """The frozen classification, FC-3, then H10 (FC-5, FC-6). Writes the H10 record and the P8 assessment."""
    run = Path(run)
    params = params or strict_json(P8 / p8_manifest.SPEC_FILES["recovery"])
    record = (classifier or p7e_assess.frozen_classifier()).classify(run)
    value = p7e_assess.assess(run, record)
    h10 = p8_h10_check.check_run(run, strict_json(run / "p4_avoidance.yaml"), params)
    (run / "p8").mkdir(exist_ok=True)
    atomic_json(run / H10_RECORD, h10, exclusive=True)
    stops = list(value["stop_reasons"])
    if not h10["ok"]:
        fc6 = any(any(m in v for m in FC6_MARKERS) for v in h10["violations"])
        stops.append(FC6 if fc6 else FC5)
    out = dict(value, contract="p8-flight-assessment-v1", stop_reasons=stops,  # validity stays FC-3's; a stop reason stops
               h10={"ok": h10["ok"], "actions": h10["actions"], "refusals": h10["refusals"],
                    "violations": h10["violations"][:20], "record_sha256": digest(run / H10_RECORD)})
    atomic_json(run / ASSESSMENT, out, exclusive=True)
    return out


def runtime_command(flight, scope, manifest, authorization):
    command = ["bash", str(RUN_P8), "--run", "--variant", flight["variant"], "--item", flight["layout"], "--scope", scope]
    if scope != p8_manifest.DRY_RUN:
        command += ["--block", str(flight["block"]), "--slot", str(flight["slot"])]
    return command + ["--manifest", str(manifest), "--authorization", str(authorization), "--allow-simulated-flight"]


def fly_one(flight, scope, manifest, authorization, classify=assess, run=subprocess.run):
    """One flight, its accepted evaluation and the P8 assessment. After the runtime returns, nothing here raises:
    every failure is recorded, and an unclassified flight stops the block (BlockRules)."""
    completed = run(runtime_command(flight, scope, manifest, authorization), text=True, capture_output=True,
                    env=p7eb.frozen_block.runtime_env())
    found = EVIDENCE_LINE.search(completed.stdout or "")
    path = Path(found.group(1)) if found else None
    record = dict(flight, run=str(path) if path else None, runtime_exit_code=completed.returncode,
                  runtime_stdout_tail=(completed.stdout or "")[-4000:], runtime_stderr_tail=(completed.stderr or "")[-4000:],
                  finished_unix_ns=time.time_ns())
    if path is None:
        return record
    try:
        evaluated = run(["bash", str(p7eb.EVIDENCE[flight["controller"]]), str(path)], text=True, capture_output=True)
        evaluation = path / p7eb.EVALUATION[flight["controller"]]
        record.update(evaluator_exit_code=evaluated.returncode, evaluator_stderr_tail=(evaluated.stderr or "")[-2000:],
                      evaluation_status=strict_json(evaluation).get("status") if evaluation.exists() else None,
                      evaluation_sha256=digest(evaluation) if evaluation.exists() else None)
    except Exception as exc:
        record["evaluator_error"] = type(exc).__name__ + ":" + str(exc)[:400]
    try:
        record["classification"] = classify(path)
    except Exception as exc:
        record["classification"] = {"validity": None, "stop_reasons": [], "error": type(exc).__name__ + ":" + str(exc)[:400]}
    for key, name in (("summary_sha256", "summary.json"), ("flight_record_sha256", p7e_assess.RECORD),
                      ("assessment_sha256", ASSESSMENT), ("h10_record_sha256", H10_RECORD),
                      ("recovery_ledger_sha256", "p8/recovery-ledger.jsonl")):
        record[key] = digest(path / name) if (path / name).exists() else None
    return record


def previous_block_ok(root, block, sha, sched=None):
    """FC-1: block n needs block n-1's result, unstopped, for the approved plan, every planned flight accounted for
    (recomputed), and its ledger intact."""
    if int(block) == 1:
        return True, None
    path = p8_manifest.result_path(root, int(block) - 1, sha)
    if not path.exists():
        return False, "no result for block %d" % (int(block) - 1)
    prev = strict_json(path)
    planned = plan_flights(sched or schedule(), int(block) - 1)
    if prev.get("manifest_sha256") != sha or prev.get("block") != int(block) - 1 or prev.get("planned") != planned:
        return False, "the result of block %d is not for this manifest and plan" % (int(block) - 1)
    if prev.get("stopped") is not False or prev.get("not_flown") or prev.get("interrupted") is not None:
        return False, "block %d stopped or left flights unflown" % (int(block) - 1)
    check = p7eb.accounting(planned, prev.get("flights") or [], prev.get("not_flown") or [], prev.get("missing_pairs") or [],
                            prev.get("interrupted"), False)
    if not (check["complete"] and check["all_flown_or_missing"]):
        return False, "block %d does not account for every planned flight" % (int(block) - 1)
    ledger = Path(prev["ledger"]["path"])
    if not ledger.exists() or digest(ledger) != prev["ledger"]["sha256"] or len(p7eb.ledger_records(ledger)) != len(prev["flights"]):
        return False, "the ledger of block %d does not match its result" % (int(block) - 1)
    return True, None


def dry_result_path(root, sha):
    return Path(root) / "state" / ("p8-dryrun-%s-%s.result.json" % (sha[:12], time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scope", choices=(p8_manifest.SCOPE, p8_manifest.DRY_RUN), required=True)
    parser.add_argument("--block", type=int, default=0)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    args = parser.parse_args(argv)
    missing = [key for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
               if os.environ.get(key) != "1"]
    if missing:
        raise SystemExit("Missing explicit gates: " + ", ".join(missing))
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = p8_manifest.verify_current(root, args.manifest)  # P7-E e9561f05, P7 and P3-P6 inside it
    import mc_trace_runtime
    if mc_trace_runtime.selected_build(root, p7eb.SIM) != frozen["runtime"]["mc_trace_build"]:
        raise SystemExit("GWM_PX4_MC_TRACE=%s must select the frozen MC trace build" % mc_trace_runtime.CONTRACT)
    dry = args.scope == p8_manifest.DRY_RUN
    if dry:
        if args.block:
            raise SystemExit("A dry run has no block")
        approval = p8_manifest.require_dry_run_approval(args.authorization, sha, frozen)
        flights, operator, output = dry_flights(approval), approval["authorized_by"], dry_result_path(root, sha)
    else:
        if frozen["development"]:
            raise SystemExit("A development candidate never flies a P8 block")
        authorization = p8_manifest.require_authorization(args.authorization, args.block, sha)
        output = p8_manifest.result_path(root, args.block, sha)
        if output.exists():
            raise SystemExit("This block already has a result; results are never overwritten")
        ok, why = previous_block_ok(root, args.block, sha)
        if not ok:
            raise SystemExit("Block %d cannot start: %s (FC-1)" % (args.block, why))
        flights, operator = plan_flights(schedule(), args.block), authorization["authorized_by"]
    ledger_path = output.with_name(output.name.replace(".result.json", ".ledger.jsonl"))
    if ledger_path.exists():
        raise SystemExit("Ledger exists; a block never runs twice: %s" % ledger_path)
    if p8_manifest.live_locks(root):
        raise SystemExit("A live P8, P7-E or P7 sequence holds %s" % p8_manifest.live_locks(root))
    host = p7e_host.check(root, len(flights))
    if not host["passed"]:
        record = output.with_name(output.name.replace(".result.json", ".not-flown-%d.json" % time.time_ns()))
        atomic_json(record, {"contract": "p8-not-flown-v1", "scope": args.scope, "block": args.block, "manifest_sha256": sha,
                             "host_check": host, "authorization_consumed": False}, exclusive=True)
        print("NOT FLOWN (host check failed): %s" % record, flush=True)
        return 1
    marker = None if dry else p8_manifest.consume(root, args.block, sha, args.authorization)
    p8_manifest.acquire_live_lock(root, sha)
    try:
        rules = p7eb.BlockRules(len(flights), replacements_allowed=not dry)
        ledger = p7eb.Ledger(ledger_path)
        result = {"contract": "p8-dry-run-result-v1" if dry else "p8-block-result-v1", "scope": args.scope,
                  "block": args.block, "stage": p8_manifest.STAGES.get(args.block, "DRY"), "manifest_sha256": sha,
                  "p7e_manifest_sha256": p8_manifest.P7E_SHA256, "development": frozen["development"],
                  "baselines": frozen["baselines"],
                  "authorization": {"path": str(args.authorization), "sha256": digest(args.authorization), "authorized_by": operator},
                  "consumption_marker": str(marker) if marker else None, "host_check": host, "started_unix_ns": time.time_ns(),
                  "planned": flights, "replacement_cap": rules.cap, "automatic_retry": False, "dry_run": dry,
                  "engineering_credit": not dry, "fail_closed_rules": ["FC-1", "FC-2", "FC-3", "FC-4", "FC-5", "FC-6", "FC-7"]}
        try:
            p7eb.sequence(result, flights, rules, lambda f: fly_one(f, args.scope, args.manifest, args.authorization),
                          lambda: p8_manifest.source_drift(frozen, root), p7e_host.residual, ledger)
        finally:
            try:
                drift_end = p8_manifest.source_drift(frozen, root)
            except Exception as exc:
                drift_end = ["drift_check_failed:" + type(exc).__name__]
            for key in ("flights", "not_flown"):
                result.setdefault(key, [])
            result.setdefault("interrupted", None)
            result.update(replacements=rules.replacements, missing_pairs=rules.missing, stopped=rules.stop is not None,
                          stop_reason=rules.stop, completed_unix_ns=time.time_ns(), candidate_unchanged_at_end=not drift_end,
                          drift_at_end=drift_end[:8],
                          ledger={"path": str(ledger.path), "sha256": digest(ledger.path) if ledger.path.exists() else None})
            result["accounting"] = p7eb.accounting(flights, result["flights"], result["not_flown"], rules.missing,
                                                   result["interrupted"], result["stopped"])
            atomic_json(output, result, exclusive=True)
    finally:
        p8_manifest.release_live_lock(root, sha)
    print("%s P8 %s block %s: %s" % ("STOPPED" if rules.stop else "COMPLETED", args.scope, args.block, output), flush=True)
    return 1 if rules.stop else 0


if __name__ == "__main__":
    raise SystemExit(main())
