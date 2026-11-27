"""Explicit P7 block sequencer (contract §4.2, §7, §8; decisions D6, D12). Nothing runs without --run inputs.

**One invocation, one block.** A block is the pilot, or one block of the selected confirmatory schedule, or (on a
development candidate) a dry run.

**Before anything is consumed:**
1. The gates are set.
2. The P7 manifest verifies (P3-P6 inside it, C1), and the frozen MC-trace build is selected.
3. The authorization binds this scope, block and manifest.
4. A confirmatory block needs the pilot rule's schedule selection and the previous block's result, unstopped.
5. The M3 classifier is present.
6. The host check passes (overlay, AC, CPU, no simulator, lock free, disk floor). Otherwise nothing is flown
   ("not flown"), and the authorization is not consumed.

**Then:**
* The marker is consumed, and the live lock is taken (no freeze, no second sequence).
* Every scheduled flight is flown in order, through `run_p7.sh` inside a fresh namespace.
* Each flight is evaluated by the unchanged P4 or P5 evidence script, then classified by the M3 classifier. The
  classifier writes `p7/flight-record.json` and returns its `validity` and `stop_reasons`.
* Each flight is appended to the block ledger (fsynced) before the next one starts.

**The rules (contract §7):**
* **Infrastructure-invalid flight** (no armed evidence): re-flown once, at the end of the block. The pair's valid
  flight is kept.
* **Replacement limits:** at most one replacement per flight, and at most 10 % of the block's flights. Exceeding
  either stops the batch. A pair still invalid after its replacement is listed as missing.
* **Stops:**
  * any classifier stop reason: a safety event or a safety-unverified flight (D14);
  * manifest drift before a flight;
  * a simulator process outliving its flight;
  * a runtime that created no evidence.

Valid flights are never re-flown, nothing is retried automatically, and the result is written once (exclusive).
A dry run flies the approval's items once each, with the same stop rules, no replacement and no credit.
"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

P7 = Path(__file__).resolve().parents[1]
REPO = P7.parents[1]
SIM, P4, P5 = REPO / "simulation/px4_gazebo", REPO / "simulation/px4_gazebo_p4", REPO / "simulation/px4_gazebo_p5"
for _path in (P7 / "scripts", SIM / "scripts", SIM / "validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7_host  # noqa: E402
import p7_manifest  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

RUN_P7 = P7 / "scripts/run_p7.sh"
EVIDENCE = {"p4": P4 / "scripts/verify_p4_evidence.sh", "p5": P5 / "scripts/verify_p5_evidence.sh"}
EVALUATION = {"p4": "p4-evaluation.json", "p5": "p5-evaluation.json"}
CLASSIFIER = P7 / "validation/p7_classify.py"
REPLACEMENT_FRACTION = 0.10


# ---- the plan -------------------------------------------------------------------------------------------------
def schedule_rows(sched, scope, block):
    rows = sched["pilot"]["slots"] if scope == "pilot" else sched["confirmatory"][scope.split("-")[1]]["slots"]
    rows = [r for r in rows if r["block"] == int(block)]
    if not rows:
        raise SystemExit("No block %s in the frozen %s schedule" % (block, scope))
    return rows


def plan_flights(sched, scope, block):
    """The block's flights in schedule order: each pair's two flights back to back, in the pair's order."""
    return [{"slot": r["slot"], "block": r["block"], "pair_in_block": r["pair_in_block"], "layout": r["layout"],
             "controller": c, "position": i, "attempt": 1, **({"repeat": r["repeat"]} if "repeat" in r else {})}
            for r in schedule_rows(sched, scope, block) for i, c in enumerate(r["order"])]


def dry_flights(approval):
    return [{"slot": 0, "block": 0, "pair_in_block": 0, "layout": item, "controller": c, "position": 0, "attempt": 1}
            for item, c in approval["items"]]


# ---- replacement and stop rules (contract §7.2, §7.3) -----------------------------------------------------------
class BlockRules:
    def __init__(self, planned_flights, replacements_allowed=True):
        self.cap = math.floor(REPLACEMENT_FRACTION * planned_flights) if replacements_allowed else 0
        self.replace = replacements_allowed
        self.replacements, self.missing, self.stop = [], [], None

    def after(self, flight, record):
        """Apply the rules to one flown flight; sets `stop` or queues a replacement."""
        if record.get("run") is None:
            self.stop = "runtime_did_not_create_evidence"
            return
        c = record.get("classification") or {}
        if c.get("stop_reasons"):
            self.stop = "stop_rule:" + ",".join(c["stop_reasons"])
            return
        if c.get("validity") == "valid":
            return
        if c.get("validity") != "infrastructure_invalid":
            self.stop = "unclassified_flight"
            return
        pair = {"slot": flight["slot"], "layout": flight["layout"], "controller": flight["controller"]}
        if not self.replace:
            self.missing.append(pair)
        elif flight["attempt"] >= 2:
            self.missing.append(pair)
            self.stop = "replacement_limit_per_flight"
        elif len(self.replacements) + 1 > self.cap:
            self.missing.append(pair)
            self.stop = "replacement_cap"
        else:
            self.replacements.append(dict(flight, attempt=2))


def run_sequence(flights, rules, fly, drift, residual, ledger):
    """Fly `flights`, then the replacements they queue, applying the rules; returns (records, not_flown)."""
    records, queue = [], list(flights)
    while queue:
        flight = queue.pop(0)
        changed = drift()
        if changed:
            rules.stop = "manifest_drift:" + ",".join(changed[:8])
            queue.insert(0, flight)
            break
        record = fly(flight)
        records.append(record)
        ledger(record)
        left = residual()
        if left:
            record["residual_processes"] = left[:10]
            rules.stop = "simulator_process_outlived_flight"
            break
        before = len(rules.replacements)
        rules.after(flight, record)
        if rules.stop:
            break
        queue += rules.replacements[before:]  # re-flown at the end of the block (D6)
    return records, queue


# ---- one flight ------------------------------------------------------------------------------------------------
def runtime_command(flight, scope, manifest, authorization):
    command = ["bash", str(RUN_P7), "--run", "--controller", flight["controller"], "--item", flight["layout"], "--scope", scope]
    if scope != p7_manifest.DRY_RUN:
        command += ["--block", str(flight["block"]), "--slot", str(flight["slot"])]
    return command + ["--manifest", str(manifest), "--authorization", str(authorization), "--allow-simulated-flight"]


def runtime_env():
    return dict(os.environ, GWM_ALLOW_SITL_COMMANDS="1")  # the accepted flight gating (every P7 run is a flight)


def classifier():
    """The M3 classifier: `classify(run) -> {"validity", "stop_reasons", ...}`, writing p7/flight-record.json."""
    if not CLASSIFIER.exists():
        raise SystemExit("The M3 classifier (%s) is not present; no block can run before M3" % CLASSIFIER.relative_to(REPO))
    spec = importlib.util.spec_from_file_location("p7_classify", CLASSIFIER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.classify


def fly_one(flight, scope, manifest, authorization, classify):
    completed = subprocess.run(runtime_command(flight, scope, manifest, authorization), text=True, capture_output=True,
                               env=runtime_env())
    found = re.search(r"^P7 evidence: (.+)$", completed.stdout, re.M)
    run = Path(found.group(1)) if found else None
    record = dict(flight, run=str(run) if run else None, runtime_exit_code=completed.returncode,
                  runtime_stdout_tail=completed.stdout[-4000:], runtime_stderr_tail=completed.stderr[-4000:],
                  finished_unix_ns=time.time_ns())
    if run is None:
        return record
    evaluated = subprocess.run(["bash", str(EVIDENCE[flight["controller"]]), str(run)], text=True, capture_output=True)
    path = run / EVALUATION[flight["controller"]]
    evaluation = strict_json(path) if path.exists() else None
    record.update(evaluator_exit_code=evaluated.returncode, evaluator_stderr_tail=evaluated.stderr[-2000:],
                  evaluation_status=(evaluation or {}).get("status"), evaluation_sha256=digest(path) if evaluation else None)
    try:
        record["classification"] = classify(run)
    except Exception as exc:  # an unclassifiable flight stops the block (BlockRules: unclassified_flight)
        record["classification"] = {"validity": None, "stop_reasons": [], "error": type(exc).__name__ + ":" + str(exc)}
    flight_record = run / "p7/flight-record.json"
    record["flight_record_sha256"] = digest(flight_record) if flight_record.exists() else None
    return record


class Ledger:
    """Append-only, one fsynced JSON line per flight, before the next flight starts."""

    def __init__(self, path):
        self.path = Path(path)
        if self.path.exists():
            raise SystemExit("Ledger exists; a block never runs twice: %s" % self.path)

    def __call__(self, record):
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())


def previous_block_ok(root, scope, block, sha):
    if int(block) == 1:
        return True
    path = p7_manifest.result_path(root, scope, int(block) - 1, sha)
    return path.exists() and strict_json(path).get("stopped") is False


def dry_result_path(root, sha):
    return Path(root) / "state" / ("p7-dryrun-%s-%s.result.json" % (sha[:12], time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scope", choices=p7_manifest.SCOPES + (p7_manifest.DRY_RUN,), required=True)
    parser.add_argument("--block", type=int, default=0)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    args = parser.parse_args(argv)
    missing = [key for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
               if os.environ.get(key) != "1"]
    if missing:
        raise SystemExit("Missing explicit gates: " + ", ".join(missing))
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = p7_manifest.verify_current(root, args.manifest)  # P3-P6 re-verified inside (C1)
    import mc_trace_runtime
    if mc_trace_runtime.selected_build(root, SIM) != frozen["runtime"]["mc_trace_build"]:
        raise SystemExit("GWM_PX4_MC_TRACE=%s must select the frozen MC trace build" % mc_trace_runtime.CONTRACT)
    dry = args.scope == p7_manifest.DRY_RUN
    if dry:
        approval = p7_manifest.require_dry_run_approval(args.authorization, sha, frozen)
        flights, operator, output = dry_flights(approval), approval["authorized_by"], dry_result_path(root, sha)
    else:
        if frozen["development"]:
            raise SystemExit("A development candidate never flies a pilot or confirmatory block")
        authorization = p7_manifest.require_authorization(args.authorization, args.scope, args.block, sha)
        output = p7_manifest.result_path(root, args.scope, args.block, sha)
        if output.exists():
            raise SystemExit("This block already has a result; results are never overwritten")
        if args.scope != "pilot":
            p7_manifest.require_selection(root, sha, args.scope)
        if not previous_block_ok(root, args.scope, args.block, sha):
            raise SystemExit("Block %d needs the unstopped result of block %d" % (args.block, args.block - 1))
        flights, operator = plan_flights(strict_json(P7 / p7_manifest.SPEC_FILES["schedule"]), args.scope, args.block), authorization["authorized_by"]
    classify = classifier()
    host = p7_host.check(root, len(flights))
    if not host["passed"]:
        record = output.with_name(output.name.replace(".result.json", ".not-flown-%d.json" % time.time_ns()))
        atomic_json(record, {"contract": "p7-not-flown-v1", "scope": args.scope, "block": args.block, "manifest_sha256": sha,
                             "host_check": host, "authorization_consumed": False}, exclusive=True)
        print("NOT FLOWN (host check failed): %s" % record, flush=True)
        return 1
    marker = None if dry else p7_manifest.consume(root, args.scope, args.block, sha, args.authorization)
    p7_manifest.acquire_live_lock(root, sha)
    rules = BlockRules(len(flights), replacements_allowed=not dry)
    ledger = Ledger(output.with_name(output.name.replace(".result.json", ".ledger.jsonl")))
    result = {"contract": "p7-dry-run-result-v1" if dry else "p7-block-result-v1", "scope": args.scope, "block": args.block,
              "manifest_sha256": sha, "development": frozen["development"], "baselines": frozen["baselines"],
              "authorization": {"path": str(args.authorization), "sha256": digest(args.authorization), "authorized_by": operator},
              "consumption_marker": str(marker) if marker else None, "host_check": host, "started_unix_ns": time.time_ns(),
              "planned": flights, "replacement_cap": rules.cap, "automatic_retry": False, "dry_run": dry,
              "confirmatory_credit": not dry and args.scope != "pilot"}
    pattern = p7_host.config()["wsl"]["simulator_process_pattern"]
    try:
        records, not_flown = run_sequence(
            flights, rules, lambda f: fly_one(f, args.scope, args.manifest, args.authorization, classify),
            lambda: p7_manifest.source_drift(frozen), lambda: p7_host.simulator_processes(pattern), ledger)
        result.update(flights=records, not_flown=not_flown)
    finally:
        result.update(replacements=rules.replacements, missing_pairs=rules.missing, stopped=rules.stop is not None,
                      stop_reason=rules.stop, completed_unix_ns=time.time_ns(),
                      candidate_unchanged_at_end=not p7_manifest.source_drift(frozen),
                      ledger={"path": str(ledger.path), "sha256": digest(ledger.path) if ledger.path.exists() else None})
        result.setdefault("flights", [])
        result.setdefault("not_flown", [])
        atomic_json(output, result, exclusive=True)
        p7_manifest.release_live_lock(root, sha)
    print("%s P7 %s block %s: %s" % ("STOPPED" if rules.stop else "COMPLETED", args.scope, args.block, output), flush=True)
    return 1 if rules.stop else 0


if __name__ == "__main__":
    raise SystemExit(main())
