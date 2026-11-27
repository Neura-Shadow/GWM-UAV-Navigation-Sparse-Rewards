"""Explicit P7-E block sequencer, with fail-closed rules FC-1..FC-3 (contract `p7-engineering-validation-v1` §4,
§8, §13). Nothing runs without --run inputs.

**One invocation, one block:** one of the 3 blocks of scope `engineering-30` (10 pairs, 20 flights), or, on a
development candidate, a dry run.

**Reused unchanged from the frozen P7 sequencer (`run_p7_block.py`):**
* the replacement and stop rules (`BlockRules`): an infrastructure-invalid flight is re-flown once at the end of the
  block; at most 10 % of the block's flights (2 of 20) are replaced, and at most once each;
* the sequence loop (`run_sequence`), the fsynced append-only `Ledger`, and the accepted P4/P5 evidence scripts.

**Before anything is consumed:**
1. The gates are set.
2. The P7-E manifest verifies: the frozen P7 candidate 0799c37e, and P3-P6 inside it. The frozen MC-trace build is
   selected.
3. The authorization binds this block and manifest.
4. Block 2 or 3 needs the previous block's result: unstopped, and accounting for every planned flight (FC-1).
5. The host check passes (FC-2: a failed `pgrep` fails it). Otherwise nothing is flown, and the authorization is
   not consumed.

**Then:**
* The marker is consumed, and the live lock is taken. It refuses while a P7 sequence runs.
* Each flight runs through `run_p7e.sh`, is evaluated by the unchanged P4 or P5 evidence script, and is classified
  by the frozen P7 classifier and then the P7-E assessment (FC-3).
* Each flight is appended to the ledger before the next one starts.

**Fail-closed rules:**
* **FC-1, exceptions.** Any exception in the sequence is a stop (`sequencer_exception:<type>`). The flown flights
  are recovered from the ledger. The flight in progress, if any, is listed as interrupted, and every other unflown
  flight is listed as not flown.
* **FC-1, accounting.** Every result carries the accounting of every planned flight. The next block requires the
  previous result to be unstopped and complete.
* **FC-2, process check.** A failed `pgrep` after a flight stops the batch (`process_check_failed:…`).
* **FC-3, arm state.** An unknown arm state is safety-unverified and stops the batch (`p7e_assess.py`).

Valid flights are never re-flown, nothing is retried automatically, and the result is written once (exclusive).
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
SIM, P7 = REPO / "simulation/px4_gazebo", REPO / "simulation/px4_gazebo_p7"
for _path in (SIM / "validation", SIM / "scripts", P7 / "scripts", P7E / "validation", P7E / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import run_p7_block as frozen_block  # noqa: E402  (frozen P7 code, used unchanged)
import p7e_assess  # noqa: E402
import p7e_host  # noqa: E402
import p7e_manifest  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

RUN_P7E = P7E / "scripts/run_p7e.sh"
EVIDENCE, EVALUATION = frozen_block.EVIDENCE, frozen_block.EVALUATION
BlockRules, Ledger, run_sequence = frozen_block.BlockRules, frozen_block.Ledger, frozen_block.run_sequence
EVIDENCE_LINE = re.compile(r"^P7-E evidence: (.+)$", re.M)
BLOCK_FLIGHTS = 2 * 10


# ---- the plan -------------------------------------------------------------------------------------------------
def schedule():
    return strict_json(P7E / p7e_manifest.SPEC_FILES["schedule"])


def plan_flights(sched, block):
    """The block's flights in schedule order: each pair's two flights back to back, in the pair's order."""
    rows = [r for r in sched["slots"] if r["block"] == int(block)]
    if not rows:
        raise SystemExit("No block %s in the approved P7-E schedule" % block)
    flights = [{"slot": r["slot"], "block": r["block"], "pair_in_block": r["pair_in_block"], "layout": r["layout"],
                "controller": c, "position": i, "attempt": 1} for r in rows for i, c in enumerate(r["order"])]
    return [dict(f, plan_index=n) for n, f in enumerate(flights)]


def dry_flights(approval):
    return [dict(f, plan_index=n) for n, f in enumerate(frozen_block.dry_flights(approval))]


# ---- one flight ------------------------------------------------------------------------------------------------
def runtime_command(flight, scope, manifest, authorization):
    command = ["bash", str(RUN_P7E), "--run", "--controller", flight["controller"], "--item", flight["layout"], "--scope", scope]
    if scope != p7e_manifest.DRY_RUN:
        command += ["--block", str(flight["block"]), "--slot", str(flight["slot"])]
    return command + ["--manifest", str(manifest), "--authorization", str(authorization), "--allow-simulated-flight"]


def fly_one(flight, scope, manifest, authorization, classify=p7e_assess.classify, run=subprocess.run):
    """One flight, its accepted evaluation, the frozen classification and the FC-3 assessment. After the runtime has
    returned, nothing here raises: every failure is recorded, and an unclassified flight stops the block."""
    completed = run(runtime_command(flight, scope, manifest, authorization), text=True, capture_output=True,
                    env=frozen_block.runtime_env())
    found = EVIDENCE_LINE.search(completed.stdout or "")
    path = Path(found.group(1)) if found else None
    record = dict(flight, run=str(path) if path else None, runtime_exit_code=completed.returncode,
                  runtime_stdout_tail=(completed.stdout or "")[-4000:], runtime_stderr_tail=(completed.stderr or "")[-4000:],
                  finished_unix_ns=time.time_ns())
    if path is None:
        return record
    try:
        evaluated = run(["bash", str(EVIDENCE[flight["controller"]]), str(path)], text=True, capture_output=True)
        evaluation = path / EVALUATION[flight["controller"]]
        record.update(evaluator_exit_code=evaluated.returncode, evaluator_stderr_tail=(evaluated.stderr or "")[-2000:],
                      evaluation_status=strict_json(evaluation).get("status") if evaluation.exists() else None,
                      evaluation_sha256=digest(evaluation) if evaluation.exists() else None)
    except Exception as exc:
        record["evaluator_error"] = type(exc).__name__ + ":" + str(exc)[:400]
    try:
        record["classification"] = classify(path)
    except Exception as exc:  # BlockRules: unclassified_flight (a stop)
        record["classification"] = {"validity": None, "stop_reasons": [], "error": type(exc).__name__ + ":" + str(exc)[:400]}
    for key, name in (("summary_sha256", "summary.json"), ("flight_record_sha256", p7e_assess.RECORD),
                      ("assessment_sha256", p7e_assess.OUTPUT)):
        record[key] = digest(path / name) if (path / name).exists() else None
    return record


# ---- FC-1 ----------------------------------------------------------------------------------------------------
class Tracker:
    """Knows which flight is in progress, so an exception can name it (FC-1)."""

    def __init__(self, fly, ledger):
        self._fly, self._ledger, self.current = fly, ledger, None

    def fly(self, flight):
        self.current = flight
        return self._fly(flight)

    def record(self, record):
        self._ledger(record)
        self.current = None


def stop_reason(exc):
    if isinstance(exc, p7e_host.ProcessCheckFailed):
        return "process_check_failed:" + str(exc)[:200]
    return "sequencer_exception:" + type(exc).__name__


def ledger_records(path):
    """The flights the ledger holds, in order; an unreadable line is kept as such (never silently dropped)."""
    path = Path(path)
    if not path.exists():
        return []
    out = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if line.strip():
            try:
                out.append(json.loads(line))
            except ValueError:
                out.append({"unreadable_ledger_line": number})
    return out


def unflown(planned, replacements, flown, interrupted):
    """Planned flights and queued replacements that neither the ledger nor the interrupted flight accounts for."""
    done = Counter((r.get("plan_index"), r.get("attempt")) for r in flown)
    if interrupted is not None:
        done[(interrupted["plan_index"], interrupted["attempt"])] += 1
    out = []
    for f in list(planned) + list(replacements):
        k = (f["plan_index"], f["attempt"])
        if done[k]:
            done[k] -= 1
        else:
            out.append(f)
    return out


def stopping_flight(record):
    """A ledger record that itself triggers a stop rule."""
    c = record.get("classification") or {}
    return bool(c.get("stop_reasons") or record.get("run") is None or record.get("residual_processes")
                or c.get("validity") not in ("valid", "infrastructure_invalid"))


def accounting(planned, flights, not_flown, missing, interrupted, stopped):
    """FC-1: the status of every planned flight. `complete` means none is unaccounted for.

    flown_valid: a valid attempt. missing: still invalid after its replacement (or in a dry run). interrupted: in
    progress at an exception. flown_stopped: its own record stopped the block. not_flown: it (or its queued
    replacement) was never flown. flown_before_stop: flown and not valid, with the block stopped before the rules
    resolved it. flown_unresolved (an unstopped block) and unaccounted make the accounting incomplete."""
    attempts = {}
    for r in flights:
        attempts.setdefault(r.get("plan_index"), []).append(r)
    left = {f["plan_index"] for f in not_flown}
    lost = {(m["slot"], m["layout"], m["controller"]) for m in missing}
    rows = []
    for f in planned:
        tried = attempts.get(f["plan_index"], [])
        if any((r.get("classification") or {}).get("validity") == "valid" for r in tried):
            status = "flown_valid"
        elif (f["slot"], f["layout"], f["controller"]) in lost:
            status = "missing"
        elif interrupted is not None and interrupted["plan_index"] == f["plan_index"]:
            status = "interrupted"
        elif stopped and any(stopping_flight(r) for r in tried):
            status = "flown_stopped"
        elif f["plan_index"] in left:
            status = "not_flown"
        elif tried:
            status = "flown_before_stop" if stopped else "flown_unresolved"
        else:
            status = "unaccounted"
        rows.append({"plan_index": f["plan_index"], "slot": f["slot"], "controller": f["controller"],
                     "attempts": len(tried), "status": status})
    counts = Counter(r["status"] for r in rows)
    bad = counts["unaccounted"] + counts["flown_unresolved"]
    return {"planned": len(planned), "ledger_flights": len(flights), "status_counts": dict(sorted(counts.items())),
            "complete": bad == 0, "all_flown_or_missing": counts["flown_valid"] + counts["missing"] == len(planned),
            "flights": rows}


def previous_block_ok(root, block, sha, sched=None):
    """FC-1: block n needs the result of block n-1, unstopped, for the approved plan, with every planned flight
    accounted for (recomputed here, not trusted), and its ledger intact."""
    if int(block) == 1:
        return True, None
    path = p7e_manifest.result_path(root, int(block) - 1, sha)
    if not path.exists():
        return False, "no result for block %d" % (int(block) - 1)
    prev = strict_json(path)
    planned = plan_flights(sched or schedule(), int(block) - 1)
    if prev.get("manifest_sha256") != sha or prev.get("block") != int(block) - 1 or prev.get("planned") != planned:
        return False, "the result of block %d is not for this manifest and plan" % (int(block) - 1)
    if prev.get("stopped") is not False or prev.get("not_flown") or prev.get("interrupted") is not None:
        return False, "block %d stopped or left flights unflown" % (int(block) - 1)
    check = accounting(planned, prev.get("flights") or [], prev.get("not_flown") or [], prev.get("missing_pairs") or [],
                       prev.get("interrupted"), False)
    if not (check["complete"] and check["all_flown_or_missing"]):
        return False, "block %d does not account for every planned flight" % (int(block) - 1)
    ledger = Path(prev["ledger"]["path"])
    if not ledger.exists() or digest(ledger) != prev["ledger"]["sha256"] or len(ledger_records(ledger)) != len(prev["flights"]):
        return False, "the ledger of block %d does not match its result" % (int(block) - 1)
    return True, None


def dry_result_path(root, sha):
    return Path(root) / "state" / ("p7e-dryrun-%s-%s.result.json" % (sha[:12], time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())))


def sequence(result, flights, rules, fly, drift, residual, ledger):
    """The frozen run_sequence under FC-1: an exception becomes a stop, and the state is rebuilt from the ledger."""
    tracker = Tracker(fly, ledger)
    try:
        records, not_flown = run_sequence(flights, rules, tracker.fly, drift, residual, tracker.record)
        result.update(flights=records, not_flown=not_flown, interrupted=None, exception=None, recovered_from_ledger=False)
    except BaseException as exc:  # FC-1: every exception, including an interrupt, is a stop
        reason = stop_reason(exc)
        rules.stop = rules.stop + ";" + reason if rules.stop else reason
        flown = ledger_records(ledger.path)
        result.update(flights=flown, interrupted=tracker.current, exception=type(exc).__name__ + ":" + str(exc)[:400],
                      recovered_from_ledger=True, not_flown=unflown(flights, rules.replacements, flown, tracker.current))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scope", choices=(p7e_manifest.SCOPE, p7e_manifest.DRY_RUN), required=True)
    parser.add_argument("--block", type=int, default=0)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    args = parser.parse_args(argv)
    missing = [key for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
               if os.environ.get(key) != "1"]
    if missing:
        raise SystemExit("Missing explicit gates: " + ", ".join(missing))
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = p7e_manifest.verify_current(root, args.manifest)  # P7 0799c37e, and P3-P6 inside it
    import mc_trace_runtime
    if mc_trace_runtime.selected_build(root, SIM) != frozen["runtime"]["mc_trace_build"]:
        raise SystemExit("GWM_PX4_MC_TRACE=%s must select the frozen MC trace build" % mc_trace_runtime.CONTRACT)
    dry = args.scope == p7e_manifest.DRY_RUN
    if dry:
        if args.block:
            raise SystemExit("A dry run has no block")
        approval = p7e_manifest.require_dry_run_approval(args.authorization, sha, frozen)
        flights, operator, output = dry_flights(approval), approval["authorized_by"], dry_result_path(root, sha)
    else:
        if frozen["development"]:
            raise SystemExit("A development candidate never flies a P7-E block")
        authorization = p7e_manifest.require_authorization(args.authorization, args.block, sha)
        output = p7e_manifest.result_path(root, args.block, sha)
        if output.exists():
            raise SystemExit("This block already has a result; results are never overwritten")
        ok, why = previous_block_ok(root, args.block, sha)
        if not ok:
            raise SystemExit("Block %d cannot start: %s (FC-1)" % (args.block, why))
        flights, operator = plan_flights(schedule(), args.block), authorization["authorized_by"]
    ledger_path = output.with_name(output.name.replace(".result.json", ".ledger.jsonl"))
    if ledger_path.exists():  # refused before anything is consumed
        raise SystemExit("Ledger exists; a block never runs twice: %s" % ledger_path)
    if p7e_manifest.live_locks(root):
        raise SystemExit("A live P7 or P7-E sequence holds %s" % p7e_manifest.live_locks(root))
    host = p7e_host.check(root, len(flights))
    if not host["passed"]:
        record = output.with_name(output.name.replace(".result.json", ".not-flown-%d.json" % time.time_ns()))
        atomic_json(record, {"contract": "p7e-not-flown-v1", "scope": args.scope, "block": args.block, "manifest_sha256": sha,
                             "host_check": host, "authorization_consumed": False}, exclusive=True)
        print("NOT FLOWN (host check failed): %s" % record, flush=True)
        return 1
    marker = None if dry else p7e_manifest.consume(root, args.block, sha, args.authorization)
    p7e_manifest.acquire_live_lock(root, sha)
    try:
        rules = BlockRules(len(flights), replacements_allowed=not dry)
        ledger = Ledger(ledger_path)
        result = {"contract": "p7e-dry-run-result-v1" if dry else "p7e-block-result-v1", "scope": args.scope,
                  "block": args.block, "manifest_sha256": sha, "p7_manifest_sha256": frozen["p7"]["sha256"],
                  "development": frozen["development"], "baselines": frozen["baselines"],
                  "authorization": {"path": str(args.authorization), "sha256": digest(args.authorization), "authorized_by": operator},
                  "consumption_marker": str(marker) if marker else None, "host_check": host, "started_unix_ns": time.time_ns(),
                  "planned": flights, "replacement_cap": rules.cap, "automatic_retry": False, "dry_run": dry,
                  "engineering_credit": not dry, "fail_closed_rules": ["FC-1", "FC-2", "FC-3"]}
        try:
            sequence(result, flights, rules,
                     lambda f: fly_one(f, args.scope, args.manifest, args.authorization),
                     lambda: p7e_manifest.source_drift(frozen, root), p7e_host.residual, ledger)
        finally:
            try:
                drift_end = p7e_manifest.source_drift(frozen, root)
            except Exception as exc:
                drift_end = ["drift_check_failed:" + type(exc).__name__]
            for key in ("flights", "not_flown"):
                result.setdefault(key, [])
            result.setdefault("interrupted", None)
            result.update(replacements=rules.replacements, missing_pairs=rules.missing, stopped=rules.stop is not None,
                          stop_reason=rules.stop, completed_unix_ns=time.time_ns(), candidate_unchanged_at_end=not drift_end,
                          drift_at_end=drift_end[:8],
                          ledger={"path": str(ledger.path), "sha256": digest(ledger.path) if ledger.path.exists() else None})
            result["accounting"] = accounting(flights, result["flights"], result["not_flown"], rules.missing,
                                              result["interrupted"], result["stopped"])
            atomic_json(output, result, exclusive=True)
    finally:
        p7e_manifest.release_live_lock(root, sha)
    print("%s P7-E %s block %s: %s" % ("STOPPED" if rules.stop else "COMPLETED", args.scope, args.block, output), flush=True)
    return 1 if rules.stop else 0


if __name__ == "__main__":
    raise SystemExit(main())
