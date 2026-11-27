"""P7-E block recovery after the sequencer was terminated without an exception (operator "Authorize recovery",
2026-11-27).

FC-1 turns every exception into a stop. A sequencer killed by a signal writes nothing: no result, and its live lock
stays. This tool writes what FC-1 would have written. It is not part of the P7-E candidate (it lives in
`diagnostics/`), so the frozen candidate does not drift.

**What it does,** in order, stopping at the first failure. Nothing is retried.
1. **Checks:**
   * no result exists for the block, and its ledger and consumption marker do;
   * the marker matches an authorization file by hash;
   * the lock names this manifest, and its PID is not a live sequencer;
   * no simulator process is running (FC-2: a failed `pgrep` refuses);
   * the ledger is the plan's prefix;
   * at most one run of this block is missing from the ledger, it is the next planned flight, and it has no
     evaluation yet.
2. **The orphaned flight** is verified as the sequencer would have done it: the unchanged evaluator, then the frozen
   classifier and FC-3 (`p7e_assess.classify`).
3. **The block result** is written (exclusive). It is rebuilt from the ledger as FC-1 would have:
   * stopped, with stop reason `sequencer_terminated_externally`;
   * flights from the ledger, and the orphan as `interrupted`, with its record;
   * every other planned flight as not flown;
   * the accepted accounting, and `reconstructed` details.
4. **A receipt** is written (exclusive).
5. **The stale lock is removed.** Its content and hash are kept in the receipt.

The result marks the block stopped, so FC-1 refuses the next block on this candidate.

Usage (WSL, from /tmp, PYTHONDONTWRITEBYTECODE=1, GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1):
  /usr/bin/python3 p7e_block_recovery.py --manifest STATE/p7e-candidate-….json --block 1 --statement "…"
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
P7E = REPO / "simulation/px4_gazebo_p7e"
for _path in (REPO / "simulation/px4_gazebo/validation", REPO / "simulation/px4_gazebo/scripts",
              REPO / "simulation/px4_gazebo_p7/scripts", P7E / "validation", P7E / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7e_assess  # noqa: E402
import p7e_host  # noqa: E402
import p7e_manifest as m  # noqa: E402
import run_p7e_block as seq  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

CONTRACT = "p7e-block-recovery-v1"
STOP_REASON = "sequencer_terminated_externally"


class Refused(RuntimeError):
    """A recovery check failed; nothing further is done."""


def pid_is_live_sequencer(pid, proc=Path("/proc")):
    path = proc / str(pid) / "cmdline"
    try:
        command = path.read_bytes().replace(b"\0", b" ").decode(errors="replace")
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return False
    return "run_p7e_block" in command


def block_runs(root, sha, block):
    """Every run on disk of this candidate and block, by its summary's p7e block."""
    out = []
    for run in sorted((Path(root) / "runs").glob("*-p7e-*")):
        path = run / "summary.json"
        if path.exists():
            p = strict_json(path).get("p7e") or {}
            if p.get("p7e_manifest_sha256") == sha and p.get("block") == int(block) and p.get("dry_run") is False:
                out.append(run)
    return out


def facts(root, sha, block):
    root = Path(root)
    result = m.result_path(root, block, sha)
    ledger = result.with_name(result.name.replace(".result.json", ".ledger.jsonl"))
    marker = m.marker_path(root, block, sha)
    lock = m.live_lock(root)
    planned = seq.plan_flights(seq.schedule(), block)
    records = seq.ledger_records(ledger) if ledger.exists() else None
    in_ledger = {Path(r["run"]).name for r in records or [] if r.get("run")}
    orphans = [r for r in block_runs(root, sha, block) if r.name not in in_ledger]
    authorization = None
    if marker.exists():
        want = strict_json(marker).get("authorization_sha256")
        found = [p for p in sorted((root / "state").glob("p7e-authorization-*.json")) if digest(p) == want]
        authorization = found[0] if len(found) == 1 else None
    return {"root": root, "result": result, "ledger": ledger, "marker": marker, "lock": lock, "planned": planned,
            "records": records, "orphans": orphans, "authorization": authorization}


def precheck(f, sha, residual=p7e_host.residual, live=pid_is_live_sequencer):
    """Every condition, or Refused with the first that fails."""
    if f["result"].exists():
        raise Refused("the block already has a result: " + str(f["result"]))
    if f["records"] is None or not f["marker"].exists():
        raise Refused("no ledger or no consumption marker: the block never started")
    if any("unreadable_ledger_line" in r for r in f["records"]):
        raise Refused("the ledger has an unreadable line")
    if f["authorization"] is None:
        raise Refused("no single authorization file matches the consumption marker")
    if not f["lock"].exists():
        raise Refused("no live lock: the sequencer did not leave one")
    lock = strict_json(f["lock"])
    if lock.get("manifest_sha256") != sha:
        raise Refused("the live lock names another manifest")
    if live(lock.get("pid")):
        raise Refused("the lock's PID %s is a live sequencer" % lock.get("pid"))
    left = residual()  # raises ProcessCheckFailed (FC-2) if it cannot be known
    if left:
        raise Refused("a simulator process is running: %s" % left[:3])
    keys = [(r.get("plan_index"), r.get("attempt")) for r in f["records"]]
    if keys != [(p["plan_index"], 1) for p in f["planned"][:len(keys)]]:
        raise Refused("the ledger is not the plan's prefix (replacements are not handled by this tool)")
    if len(f["orphans"]) > 1:
        raise Refused("more than one run of this block is missing from the ledger: %s" % [r.name for r in f["orphans"]])
    if f["orphans"]:
        run = f["orphans"][0]
        nxt = f["planned"][len(keys)] if len(keys) < len(f["planned"]) else None
        s = strict_json(run / "summary.json")
        if nxt is None or (s["p7"]["item"], s["p7"]["controller"], s["p7"]["slot"]) != (nxt["layout"], nxt["controller"], nxt["slot"]):
            raise Refused("the orphaned run is not the next planned flight")
        done = [n for n in (seq.EVALUATION[nxt["controller"]], p7e_assess.RECORD, p7e_assess.OUTPUT) if (run / n).exists()]
        if done:
            raise Refused("the orphaned run already has %s" % done)
    return lock


def verify_orphan(flight, run, evaluate=None, classify=p7e_assess.classify):
    """The post-runtime half of `run_p7e_block.fly_one`: the unchanged evaluator, then the frozen classifier and FC-3."""
    run = Path(run)
    s = strict_json(run / "summary.json")
    record = dict(flight, run=str(run), runtime_exit_code=None, runtime_observed=False,
                  runtime_status=s.get("status"), runtime_failure=s.get("failure"), runtime_finalized=s.get("finalized"),
                  recovered=True, finished_unix_ns=time.time_ns())
    try:
        done = (evaluate or (lambda path: subprocess.run(["bash", str(seq.EVIDENCE[flight["controller"]]), str(path)],
                                                         text=True, capture_output=True)))(run)
        evaluation = run / seq.EVALUATION[flight["controller"]]
        record.update(evaluator_exit_code=done.returncode, evaluator_stderr_tail=(done.stderr or "")[-2000:],
                      evaluation_status=strict_json(evaluation).get("status") if evaluation.exists() else None,
                      evaluation_sha256=digest(evaluation) if evaluation.exists() else None)
    except Exception as exc:
        record["evaluator_error"] = type(exc).__name__ + ":" + str(exc)[:400]
    try:
        record["classification"] = classify(run)
    except Exception as exc:
        record["classification"] = {"validity": None, "stop_reasons": [], "error": type(exc).__name__ + ":" + str(exc)[:400]}
    for key, name in (("summary_sha256", "summary.json"), ("flight_record_sha256", p7e_assess.RECORD),
                      ("assessment_sha256", p7e_assess.OUTPUT)):
        record[key] = digest(run / name) if (run / name).exists() else None
    return record


def rebuild(f, frozen, sha, lock, orphan_record, statement, drift):
    """The block result FC-1 would have written."""
    planned, flights = f["planned"], f["records"]
    interrupted = planned[len(flights)] if orphan_record is not None else None
    not_flown = planned[len(flights) + (1 if interrupted else 0):]
    stops = [STOP_REASON]
    if orphan_record is not None:
        c = orphan_record.get("classification") or {}
        if c.get("stop_reasons"):
            stops.append("stop_rule:" + ",".join(c["stop_reasons"]))
        elif c.get("validity") not in ("valid", "infrastructure_invalid"):
            stops.append("unclassified_flight")
    auth = strict_json(f["authorization"])
    result = {"contract": "p7e-block-result-v1", "scope": m.SCOPE, "block": planned[0]["block"], "manifest_sha256": sha,
              "p7_manifest_sha256": frozen["p7"]["sha256"], "development": frozen["development"], "baselines": frozen["baselines"],
              "authorization": {"path": str(f["authorization"]), "sha256": digest(f["authorization"]),
                                "authorized_by": auth.get("authorized_by")},
              "consumption_marker": str(f["marker"]),
              "host_check": {"passed": True, "recorded": False,
                             "note": "the sequencer consumes the marker only after the pre-block host check passes; "
                                     "the check's record was in the lost log"},
              "started_unix_ns": lock.get("started_unix_ns"), "planned": planned, "replacement_cap": seq.BlockRules(len(planned)).cap,
              "automatic_retry": False, "dry_run": False, "engineering_credit": True, "fail_closed_rules": ["FC-1", "FC-2", "FC-3"],
              "flights": flights, "not_flown": not_flown, "interrupted": interrupted, "interrupted_flight_record": orphan_record,
              "exception": None, "recovered_from_ledger": True, "replacements": [], "missing_pairs": [],
              "stopped": True, "stop_reason": ";".join(stops), "completed_unix_ns": time.time_ns(),
              "candidate_unchanged_at_end": not drift, "drift_at_end": drift[:8],
              "ledger": {"path": str(f["ledger"]), "sha256": digest(f["ledger"])},
              "reconstructed": {"contract": CONTRACT, "tool": str(Path(__file__).relative_to(REPO)), "tool_sha256": digest(Path(__file__)),
                                "operator_statement": statement, "reason": "the sequencer (PID %s) ended without an exception, "
                                "so FC-1 wrote no result; most likely a hangup signal at the end of the controlling session"
                                % lock.get("pid"), "lock": lock}}
    result["accounting"] = seq.accounting(planned, flights, not_flown, [], interrupted, True)
    return result


def recover(root, frozen, sha, block, statement, residual=p7e_host.residual, live=pid_is_live_sequencer,
            evaluate=None, classify=p7e_assess.classify, drift=lambda: []):
    f = facts(root, sha, block)
    lock = precheck(f, sha, residual, live)
    lock_text = f["lock"].read_text()
    steps = {"precheck": "passed", "lock_sha256": digest(f["lock"])}
    orphan = None
    if f["orphans"]:
        flight = f["planned"][len(f["records"])]
        orphan = verify_orphan(flight, f["orphans"][0], evaluate, classify)
        steps["orphan"] = {k: orphan.get(k) for k in ("run", "evaluator_exit_code", "evaluation_status", "evaluation_sha256",
                                                      "flight_record_sha256", "assessment_sha256")}
        steps["orphan"]["classification"] = {k: (orphan.get("classification") or {}).get(k) for k in ("validity", "outcome", "stop_reasons")}
    result = rebuild(f, frozen, sha, lock, orphan, statement, drift())
    atomic_json(f["result"], result, exclusive=True)
    steps["result"] = {"path": str(f["result"]), "sha256": digest(f["result"]), "stop_reason": result["stop_reason"],
                       "accounting": result["accounting"]["status_counts"]}
    receipt = f["result"].with_name(f["result"].name.replace(".result.json", ".recovery.json"))
    value = {"contract": CONTRACT, "block": int(block), "manifest_sha256": sha, "operator_statement": statement,
             "tool_sha256": digest(Path(__file__)), "lock_content": lock_text, "steps": steps, "unix_ns": time.time_ns()}
    atomic_json(receipt, value, exclusive=True)
    f["lock"].unlink()  # last: the stale lock, whose content and hash the receipt keeps
    return result, receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--block", type=int, required=True)
    parser.add_argument("--statement", required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    frozen, sha = m.verify_current(root, args.manifest)  # P7 0799c37e and P3-P6 inside it
    if frozen["development"]:
        raise SystemExit("A development candidate has no blocks")
    result, receipt = recover(root, frozen, sha, args.block, args.statement, drift=lambda: m.source_drift(frozen, root))
    print(json.dumps({"result": str(m.result_path(root, args.block, sha)), "receipt": str(receipt),
                      "stop_reason": result["stop_reason"], "accounting": result["accounting"]["status_counts"],
                      "interrupted": (result["interrupted_flight_record"] or {}).get("classification")}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
