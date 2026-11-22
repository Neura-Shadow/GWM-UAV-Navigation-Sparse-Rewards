"""P7-E engineering report: the approved report plan `p7e-report-plan-v1` (canonical 15e7f7b4…) as code (contract
`p7-engineering-validation-v1` §5, §6, §7, §10). Offline only. It runs once, after the last block.

**Inputs, each checked by hash.**
* The 3 block results of the P7-E candidate and their fsynced ledgers.
* For every flown flight, its summary, the accepted P4 or P5 evaluation, the frozen P7 flight record, and the P7-E
  assessment (FC-3).
* The approved schedule (`p7e_schedule.json`) and report plan.

**Hard criteria H1-H9 decide engineering completion.** Each limit is the accepted limit the plan cites, read from
the plan and checked against its source file.
* **H1-H6** apply to every flight, and H2 and H5 to every valid flight. A flight whose evidence cannot be read or
  matched counts against H6 (fail-closed).
* **H7** checks the evidence of every flown flight, and **H8** the execution and FC-1 accounting of every block.
* **H9** re-runs the frozen classifier, through its own wrapper, on a symlink view of every retained flight. The
  view holds the run's files without `p7/flight-record.json` and `p7e/`. The re-run must reproduce the recorded
  record exactly, the FC-3 assessment must reproduce, and no file of the run may change. The manifest (with P7 and
  P3-P6 inside it), the specs and every world hash must re-verify.

**Reported criteria R1-R10** are descriptive. A breach of an accepted limit is the accepted evaluator's own failure
item; it is listed as a new risk-register entry, and is not by itself an engineering failure.

**Statistics.** Only the frozen interval functions of `p7_analysis.py` (bd036a2b…) are imported: Clopper-Pearson,
Tango and Hodges-Lehmann, plus the zero-event safety bounds (`safety`). The report contains no p-value and no test;
`p7_analysis.primary` is not used because it computes one. Every paired difference is P5 minus P4.

**Engineering validation passed** only if every H criterion is met, the declared 60 flights completed (3 blocks,
none stopped, every planned flight accounted for; contract §7.1), and R1-R10 were all computed. The report always
carries the contract's required statement verbatim. Runtime: WSL /usr/bin/python3; H9 runs the frozen classifier
in its own environment. The output is written once (exclusive create).
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

P7E = Path(__file__).resolve().parents[1]
REPO = P7E.parents[1]
SIM, P4, P5, P7 = (REPO / "simulation" / n for n in ("px4_gazebo", "px4_gazebo_p4", "px4_gazebo_p5", "px4_gazebo_p7"))
for _path in (SIM / "validation", SIM / "scripts", P7 / "scripts", P7 / "analysis", P7E / "validation", P7E / "scripts"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p7_analysis  # noqa: E402  (frozen P7 analysis code, used unchanged; interval functions only)
import p7e_assess  # noqa: E402
import p7e_manifest  # noqa: E402
import run_p7e_block  # noqa: E402
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

CONTRACT = "p7e-engineering-report-v1"
PLAN = P7E / "configs/p7e_report_plan.yaml"
CLASSIFY_WRAPPER = P7 / "scripts/verify_p7_evidence.sh"
EVALUATIONS = {"p4": "p4-evaluation.json", "p5": "p5-evaluation.json"}
CONTROLLERS = ("p4", "p5")
P2_COMMANDS = [176, 400, 21]
STALL_RTF = 0.5
PASSED, NOT_PASSED = "engineering validation passed", "engineering validation not passed"
PASS_STATEMENT = ("the accepted P4 and P5 stacks completed the declared 60-flight engineering validation through the "
                  "P7 infrastructure, with every hard criterion H1-H9 met")
# the frozen classifier's safety-event items, each under the hard criterion that covers it
H_ITEMS = {"H1": ("safety:collision_contact",), "H2": ("safety:truth_clearance",),
           "H3": ("safety:setpoint_inside_inflated_obstacle", "competing_flight_publisher", "unsafe:hold_tick_motion",
                  "unsafe:unrevalidated_resume"),
           "H5": ("p2:command_sequence", "p2:final_landed_disarmed")}
H_PREFIXES = {"H4": ("native_commands:",), "H5": ("p2_ulog:",)}


# ---- the plan and its accepted limits -----------------------------------------------------------------------------
def plan():
    value = strict_json(PLAN)
    if canonical_sha256(value) != p7e_manifest.APPROVED_SPECS["report_plan"]:
        raise SystemExit("The report plan differs from the approved 15e7f7b4…")
    return value


def limits(doc):
    """The accepted limits, from the plan, each checked against its source file."""
    v4 = strict_json(P4 / "configs/p4_validation.yaml")
    v5 = strict_json(P5 / "configs/p5_validation.yaml")
    h = {c["id"]: c for c in doc["hard_criteria"]}
    if h["H1"]["limit"] != v4["safety"]["collisions_allowed"] or h["H2"]["limit"] != v4["safety"]["min_truth_clearance_m"]:
        raise SystemExit("The report plan's H1/H2 limits differ from p4_validation.yaml")
    return {"collisions": h["H1"]["limit"], "min_truth_clearance_m": h["H2"]["limit"], "performance": v4["performance"],
            "safety": v4["safety"], "policy": v5["policy"],
            "p2_max_sample_gap_sim_s": strict_json(SIM / "configs/p2_control.yaml")["max_sample_gap_sim_s"]}


# ---- evidence -----------------------------------------------------------------------------------------------------
def checked(path, sha):
    """The JSON at `path` if it exists and its SHA-256 is `sha`; else None."""
    path = Path(path)
    if sha is None or not path.exists() or digest(path) != sha:
        return None
    return strict_json(path)


def load_flight(block, entry):
    """One ledger flight with its evidence, each file accepted only if its hash matches the ledger."""
    run = Path(entry["run"]) if entry.get("run") else None
    flight = {k: entry.get(k) for k in ("slot", "pair_in_block", "layout", "controller", "attempt", "plan_index")}
    flight.update(block=block, run=str(run) if run else None, summary=None, evaluation=None, record=None, assessment=None)
    if run is not None:
        flight["summary"] = checked(run / "summary.json", entry.get("summary_sha256"))
        flight["evaluation"] = checked(run / EVALUATIONS.get(entry.get("controller"), "-"), entry.get("evaluation_sha256"))
        flight["record"] = checked(run / p7e_assess.RECORD, entry.get("flight_record_sha256"))
        flight["assessment"] = checked(run / p7e_assess.OUTPUT, entry.get("assessment_sha256"))
    flight["evidence"] = {"run_directory": run is not None and run.is_dir(), "summary": flight["summary"] is not None,
                          "evaluation": flight["evaluation"] is not None, "flight_record": flight["record"] is not None,
                          "assessment": flight["assessment"] is not None,
                          "ledger_record_hash": entry.get("flight_record_sha256") is not None}
    flight["evidence_complete"] = all(flight["evidence"].values())
    return flight


def load(root, sha):
    """The block results present for this manifest, and every flight their ledgers list."""
    results, flights = {}, []
    for block in range(1, p7e_manifest.BLOCKS + 1):
        path = p7e_manifest.result_path(root, block, sha)
        if path.exists():
            results[block] = strict_json(path)
            flights += [load_flight(block, e) for e in results[block].get("flights") or []]
    return results, flights


def validity(flight):
    return (flight["assessment"] or {}).get("validity")


def finals(flights):
    """The final attempt per (slot, controller): the valid one if any, else the last."""
    chosen = {}
    for f in flights:
        key = (f["slot"], f["controller"])
        if key not in chosen or validity(chosen[key]) != "valid":
            chosen[key] = f
    return chosen


def ref(f):
    return {"block": f["block"], "slot": f["slot"], "controller": f["controller"], "attempt": f["attempt"],
            "run": Path(f["run"]).name if f["run"] else None}


def classes(f):
    return (f["record"] or {}).get("classes") or {}


def failures(f):
    return list((f["evaluation"] or {}).get("failures") or [])


def dist(values):
    v = [x for x in values if x is not None]
    return {"n": len(v), "median": statistics.median(v) if v else None, "min": min(v, default=None),
            "max": max(v, default=None)}


# ---- hard criteria ------------------------------------------------------------------------------------------------
def criterion(cid, met, value, **detail):
    return dict({"id": cid, "met": bool(met), "value": value}, **detail)


def h1(flights, lim):
    hits = [ref(f) for f in flights
            if "safety:collision_contact" in classes(f).get("safety_events", []) + failures(f)
            or (((f["evaluation"] or {}).get("safety") or {}).get("collisions") or 0) > 0]
    return criterion("H1", len(hits) <= lim["collisions"], len(hits), limit=lim["collisions"], flights=hits)


def h2(flights, lim):
    valid = [f for f in flights if validity(f) == "valid"]
    values = [(f, ((f["evaluation"] or {}).get("safety") or {}).get("min_truth_clearance_m")) for f in valid]
    below = [ref(f) for f, v in values if (v is not None and v < lim["min_truth_clearance_m"])
             or "safety:truth_clearance" in classes(f).get("safety_events", [])]
    unmeasured = [ref(f) for f, v in values if v is None]
    measured = [v for _, v in values if v is not None]
    return criterion("H2", not below and not unmeasured, min(measured, default=None), limit=lim["min_truth_clearance_m"],
                     comparison=">=", valid_flights=len(valid), below=below, unmeasured=unmeasured)


def h3(flights):
    hits = [dict(ref(f), item=i) for f in flights for i in classes(f).get("safety_events", []) if i in H_ITEMS["H3"]]
    return criterion("H3", not hits, len(hits), limit=0, events=hits)


def h4(flights):
    hits = [dict(ref(f), item=i) for f in flights for i in set(classes(f).get("safety_events", []) + failures(f))
            if i.startswith(H_PREFIXES["H4"])]
    unchecked = [ref(f) for f in flights if "native_commands" not in (f["evaluation"] or {})]
    return criterion("H4", not hits and not unchecked, len(hits), limit=0, breaches=hits, flights_checked=len(flights) - len(unchecked),
                     unchecked=unchecked)


def landing_facts(f):
    """Landed, disarmed and the own command sequence: the accepted P2 integrity check, or under D15 the ULog facts."""
    check = (f["record"] or {}).get("p2_ulog_check") or {}
    if check.get("failures") is not None and check.get("facts"):
        facts = check["facts"]
        return {"source": "ulog (D15)", "landed": facts["final_landed"], "disarmed": facts["final_disarmed"],
                "commands": facts["own_commands"], "failsafe_observed": facts["failsafe_observed"]}
    integrity = (f["evaluation"] or {}).get("p2_integrity") or {}
    if "final_landed" in integrity and "final_disarmed" in integrity and "command_ids" in integrity:
        return {"source": "p2_integrity", "landed": integrity["final_landed"], "disarmed": integrity["final_disarmed"],
                "commands": integrity["command_ids"], "failsafe_observed": integrity.get("failsafe_observed")}
    return None


def h5(flights):
    valid = [f for f in flights if validity(f) == "valid"]
    bad = []
    for f in valid:
        facts = landing_facts(f)
        items = [i for i in classes(f).get("safety_events", []) if i in H_ITEMS["H5"] or i.startswith(H_PREFIXES["H5"])]
        if facts is None or not (facts["landed"] is True and facts["disarmed"] is True and facts["commands"] == P2_COMMANDS) or items:
            bad.append(dict(ref(f), facts=facts, items=items))
    return criterion("H5", not bad, len(valid) - len(bad), limit="every valid flight landed and disarmed, own commands %s" % P2_COMMANDS,
                     valid_flights=len(valid), failed=bad)


def h6(flights):
    bad = []
    for f in flights:
        a = f["assessment"] or {}
        reasons = list(classes(f).get("safety_unverified", []))
        reasons += [s for s in a.get("stop_reasons") or [] if s.startswith("safety_unverified:")]
        if f["record"] is None or f["assessment"] is None:
            reasons.append("evidence_unreadable")
        if a.get("validity") == "unknown":
            reasons.append("fc3_arm_state_unknown")
        if reasons:
            bad.append(dict(ref(f), reasons=sorted(set(reasons))))
    return criterion("H6", not bad, len(bad), limit=0, flights=bad)


LEDGER_KEYS = ("plan_index", "attempt", "slot", "controller", "run", "summary_sha256", "evaluation_sha256",
               "flight_record_sha256", "assessment_sha256")


def h7(results, flights):
    """Every flown flight's evidence, and each block's ledger: present, hash-matched, and line for line the result's
    flights (on the identifying fields; the frozen loop adds `residual_processes` after the ledger line)."""
    incomplete = [dict(ref(f), evidence=f["evidence"]) for f in flights if not f["evidence_complete"]]
    ledgers = {}
    for block, res in results.items():
        path = Path((res.get("ledger") or {}).get("path") or "-")
        lines = run_p7e_block.ledger_records(path) if path.exists() else None
        same = lines is not None and [[r.get(k) for k in LEDGER_KEYS] for r in lines] == \
            [[r.get(k) for k in LEDGER_KEYS] for r in res.get("flights") or []]
        ledgers[block] = {"present": path.exists(), "hash_matches": path.exists() and digest(path) == res["ledger"].get("sha256"),
                          "lines_match_result": same}
    ok_ledgers = all(all(v.values()) for v in ledgers.values())
    return criterion("H7", not incomplete and ok_ledgers, "%d/%d" % (len(flights) - len(incomplete), len(flights)),
                     limit="100 %", incomplete=incomplete, ledgers=ledgers)


def h8(results, sched):
    blocks, earlier_stop = {}, None
    cap = math.floor(run_p7e_block.frozen_block.REPLACEMENT_FRACTION * run_p7e_block.BLOCK_FLIGHTS)
    for block in range(1, p7e_manifest.BLOCKS + 1):
        res = results.get(block)
        planned = run_p7e_block.plan_flights(sched, block)
        if res is None:
            blocks[block] = {"result": False, "explained": earlier_stop is not None, "by": earlier_stop}
            earlier_stop = earlier_stop or "block %d not flown" % block
            continue
        acc = run_p7e_block.accounting(planned, res.get("flights") or [], res.get("not_flown") or [],
                                       res.get("missing_pairs") or [], res.get("interrupted"), res.get("stopped") is not False)
        attempts = Counter(f.get("plan_index") for f in res.get("flights") or [])
        row = {"result": True, "plan_matches": res.get("planned") == planned, "accounting_complete": acc["complete"],
               "all_flown_or_missing": acc["all_flown_or_missing"], "status_counts": acc["status_counts"],
               "replacements": len(res.get("replacements") or []), "replacement_cap": cap,
               "replacements_within_cap": len(res.get("replacements") or []) <= cap and res.get("replacement_cap") == cap,
               "at_most_one_replacement_per_flight": max(attempts.values(), default=0) <= 2,
               "stopped": res.get("stopped") is not False, "stop_reason": res.get("stop_reason"),
               "unflown_explained": not (res.get("not_flown") or res.get("interrupted")) or bool(res.get("stop_reason")),
               "missing_pairs": res.get("missing_pairs") or []}
        row["explained"] = all(row[k] for k in ("plan_matches", "accounting_complete", "replacements_within_cap",
                                                "at_most_one_replacement_per_flight", "unflown_explained"))
        if row["stopped"] and earlier_stop is None:
            earlier_stop = "block %d stopped: %s" % (block, res.get("stop_reason"))
        blocks[block] = row
    completed = all(row["result"] and not row["stopped"] and row["explained"] and row["all_flown_or_missing"]
                    for row in blocks.values())
    met = all(row["explained"] for row in blocks.values())
    return criterion("H8", met, "completed" if completed else "not completed", limit=(
        "all 60 flown, or every unflown flight explained by a stop rule; at most %d replacements per 20-flight block and one per flight" % cap),
        blocks=blocks, batch_completed=completed)


# ---- H9: reproducibility ----------------------------------------------------------------------------------------
def files_state(run):
    """(size, mtime_ns) of every file of the run: re-classification must change none."""
    return {p.relative_to(run).as_posix(): (p.stat().st_size, p.stat().st_mtime_ns)
            for p in sorted(Path(run).rglob("*")) if p.is_file() and not p.is_symlink()}


def make_view(run, workdir):
    """A view directory of the same name: symlinks to the run's entries, without the P7 record and P7-E outputs."""
    run, view = Path(run), Path(workdir) / Path(run).name
    view.mkdir(parents=True)
    for entry in sorted(run.iterdir()):
        if entry.name not in ("p7", "p7e"):
            (view / entry.name).symlink_to(entry)
    (view / "p7").mkdir()
    if (run / "p7/layout.json").exists():
        (view / "p7/layout.json").symlink_to(run / "p7/layout.json")
    return view


def reclassify(run, workdir, runner=subprocess.run):
    """The frozen classifier, through its own wrapper, on a view of the run; returns (record, error)."""
    view = make_view(run, workdir)
    done = runner(["bash", str(CLASSIFY_WRAPPER), str(view)], text=True, capture_output=True)
    out = view / p7e_assess.RECORD
    if done.returncode != 0 or not out.exists():
        return None, "classifier_failed:%s:%s" % (done.returncode, (done.stderr or "").strip()[-300:])
    return strict_json(out), None


def h9(root, manifest, results, flights, sched, sha, reclassify_fn=reclassify, verify_fn=None):
    detail = {"manifest": None, "specs": None, "worlds": [], "reclassification": [], "assessment": [], "unchanged": []}
    try:
        _, current = (verify_fn or p7e_manifest.verify_current)(root, manifest)  # P7 0799c37e and P3-P6 inside it
        detail["manifest"] = {"verified": current == sha, "sha256": current}
    except Exception as exc:
        detail["manifest"] = {"verified": False, "error": type(exc).__name__ + ":" + str(exc)[:300]}
    try:
        detail["specs"] = {"verified": p7e_manifest.specs_identity() == strict_json(manifest)["specs"]}
    except Exception as exc:
        detail["specs"] = {"verified": False, "error": type(exc).__name__ + ":" + str(exc)[:300]}
    detail["results_bind_manifest"] = all(r.get("manifest_sha256") == sha for r in results.values())
    slots = {s["slot"]: s for s in sched["slots"]}
    with tempfile.TemporaryDirectory(prefix="p7e-h9-") as work:
        for n, f in enumerate(flights):
            if not f["run"]:
                detail["reclassification"].append(dict(ref(f), reproduced=False, error="no run"))
                continue
            run = Path(f["run"])
            world = ((f["summary"] or {}).get("p7") or {}).get("world") or {}
            want = (slots.get(f["slot"]) or {}).get("world_sha256")
            detail["worlds"].append(dict(ref(f), verified=world.get("identical") is True and world.get("generated_sha256") == want))
            before = files_state(run) if run.is_dir() else None
            record, error = reclassify_fn(run, Path(work) / str(n)) if run.is_dir() else (None, "run directory missing")
            detail["reclassification"].append(dict(ref(f), reproduced=record is not None and record == f["record"], error=error))
            again = p7e_assess.assess(run, f["record"], f["summary"]) if f["record"] is not None and f["summary"] is not None else None
            detail["assessment"].append(dict(ref(f), reproduced=again is not None and again == f["assessment"]))
            detail["unchanged"].append(dict(ref(f), unchanged=before is not None and files_state(run) == before))
    checks = [detail["manifest"]["verified"], detail["specs"]["verified"], detail["results_bind_manifest"]]
    checks += [w["verified"] for w in detail["worlds"]] + [r["reproduced"] for r in detail["reclassification"]]
    checks += [a["reproduced"] for a in detail["assessment"]] + [u["unchanged"] for u in detail["unchanged"]]
    failed = sum(1 for c in checks if not c)
    return criterion("H9", failed == 0 and bool(flights), "identical" if failed == 0 else "%d checks differ" % failed,
                     limit="identical", **detail)


# ---- reported criteria ----------------------------------------------------------------------------------------------
def by_controller(flights, fn):
    return {c: fn([f for f in flights if f["controller"] == c]) for c in CONTROLLERS}


def items_with(flights, prefixes):
    return [dict(ref(f), item=i) for f in flights for i in failures(f) if i.startswith(prefixes)]


def r1(final):
    def rate(fs):
        valid = [f for f in fs if validity(f) == "valid"]
        x = sum(1 for f in valid if (f["record"] or {}).get("outcome") == "goal_success")
        return {"valid_flights": len(valid), "goal_success": x, "rate": x / len(valid) if valid else None,
                "ci95_clopper_pearson": p7_analysis.clopper_pearson(x, len(valid))}
    return {"id": "R1", "per_controller": by_controller(final, rate)}


def pairs(final, sched):
    out = []
    for s in sched["slots"]:
        p4, p5 = final.get((s["slot"], "p4")), final.get((s["slot"], "p5"))
        both = p4 is not None and p5 is not None and validity(p4) == "valid" and validity(p5) == "valid"
        out.append({"slot": s["slot"], "block": s["block"], "layout": s["layout"], "family": s["family"], "boxes": s["boxes"],
                    "first": s["first"], "p4": p4, "p5": p5, "analysable": both})
    return out


def success(f):
    return (f["record"] or {}).get("outcome") == "goal_success"


def r2(pair_rows):
    views = [p for p in pair_rows if p["analysable"]]
    b = sum(1 for p in views if success(p["p5"]) and not success(p["p4"]))
    c = sum(1 for p in views if success(p["p4"]) and not success(p["p5"]))
    both = sum(1 for p in views if success(p["p4"]) and success(p["p5"]))
    n = len(views)
    return {"id": "R2", "label": "descriptive", "pairs": n, "both_success": both, "p5_only": b, "p4_only": c,
            "neither": n - both - b - c, "difference_p5_minus_p4": (b - c) / n if n else None,
            "difference_ci95_tango": p7_analysis.tango_interval(b, c, n),
            "missing_pairs": [p["layout"] for p in pair_rows if not p["analysable"]]}


def p2_failure(f):
    return ((f["summary"] or {}).get("controller_result") or {}).get("failure")


def r3(flights, lim):
    def view(fs):
        return {"flights": len(fs), "p2_timing_status": dict(Counter(str((f["evaluation"] or {}).get("p2_timing_status")) for f in fs)),
                "p2_failures": dict(Counter(str(p2_failure(f)) for f in fs if p2_failure(f))),
                "observation_gap_aborts": sum(1 for f in fs if p2_failure(f) == "observation_gap")}
    return {"id": "R3", "accepted_limit": {"p2_timing": "p2-timing-v1", "max_sample_gap_sim_s": lim["p2_max_sample_gap_sim_s"]},
            "per_controller": by_controller(flights, view), "breaches": items_with(flights, ("p2:timing",))}


def r4(flights, lim):
    keys = ("rate_hz", "latency_max_sim_s", "latency_p95_sim_s", "compute_p99_s", "compute_max_s", "receipt_gap_max_wall_s")

    def view(fs):
        perf = [(f["evaluation"] or {}).get("performance") or {} for f in fs]
        depth = [((f["evaluation"] or {}).get("depth_stream") or {}).get("delivery") or {} for f in fs]
        out = {k: dist([p.get(k) for p in perf]) for k in keys}
        out["depth_delivered_hz_sim"] = dist([d.get("delivered_hz_sim") for d in depth])
        out["depth_receipt_gap_max_wall_s"] = dist([d.get("receipt_gap_max_wall_s") for d in depth])
        return out
    p = lim["performance"]
    return {"id": "R4", "accepted_limit": {k: p[k] for k in ("snapshot_rate_min_hz", "snapshot_latency_max_sim_s",
                                                             "snapshot_latency_p95_sim_s", "perception_compute_p99_s",
                                                             "perception_compute_max_s", "perception_receipt_gap_max_wall_s")},
            "per_controller": by_controller(flights, view), "breaches": items_with(flights, ("depth:", "performance:"))}


def r5(flights, lim):
    p5 = [f for f in flights if f["controller"] == "p5"]
    pol = [(f["evaluation"] or {}).get("policy") or {} for f in p5]
    values = {"inference_p99_s": dist([(p.get("inference_s") or {}).get("p99") for p in pol]),
              "inference_max_s": dist([(p.get("inference_s") or {}).get("max") for p in pol]),
              **{k: dist([p.get(k) for p in pol]) for k in ("failure_ticks", "outside_mask", "fallback_decisions",
                                                             "participation_fraction")}}
    return {"id": "R5", "accepted_limit": {k: lim["policy"][k] for k in ("inference_p99_s", "inference_max_s", "participation_fraction")},
            "p5_flights": len(p5), "values": values, "breaches": items_with(p5, ("policy:",))}


def r6(flights, lim):
    def view(fs):
        return {"tracking_max_m": dist([((f["evaluation"] or {}).get("safety") or {}).get("tracking_max_m") for f in fs]),
                "response_latency_sim_s": dist([(f["evaluation"] or {}).get("response_latency_sim_s") for f in fs])}
    return {"id": "R6", "accepted_limit": {k: lim["safety"][k] for k in ("tracking_horizontal_m", "response_latency_sim_s")},
            "per_controller": by_controller(flights, view),
            "breaches": items_with(flights, ("safety:tracking", "safety:response_latency"))}


def metric(f, key):
    return ((f["record"] or {}).get("metrics") or {}).get(key)


def r7(pair_rows):
    both = [p for p in pair_rows if p["analysable"] and success(p["p4"]) and success(p["p5"])]
    out = {"id": "R7", "label": "descriptive", "both_success_pairs": len(both)}
    for key in ("time_to_goal_sim_s", "path_length_m"):
        rows = [(metric(p["p4"], key), metric(p["p5"], key)) for p in both]
        rows = [(a, b) for a, b in rows if a is not None and b is not None]
        out[key] = {"p4_median": statistics.median([a for a, _ in rows]) if rows else None,
                    "p5_median": statistics.median([b for _, b in rows]) if rows else None,
                    "paired_difference_p5_minus_p4": p7_analysis.hodges_lehmann([b - a for a, b in rows]), "pairs": len(rows)}
    return out


def jsonl(path):
    path = Path(path)
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def cost(f):
    """Per-flight cost from the run's 1 s resource ledger and the probe's native clock."""
    if not f["run"] or not Path(f["run"]).is_dir():
        return None
    run, s = Path(f["run"]), f["summary"] or {}
    rows = [r for r in jsonl(run / "source/clock.jsonl") if r.get("sim_s", 0) > 0]
    steps = []
    for a, b in zip(rows, rows[1:]):
        wall = (b["receipt_monotonic_ns"] - a["receipt_monotonic_ns"]) / 1e9
        if wall > 0:
            steps.append(((b["sim_s"] - a["sim_s"]) / wall, wall, b["sim_s"] - a["sim_s"]))
    resources = jsonl(run / "resources.jsonl")
    first, last = {}, {}
    for r in resources:
        for name, p in (r.get("processes") or {}).items():
            first.setdefault(name, (r["monotonic_s"], p["cpu_s"]))
            last[name] = (r["monotonic_s"], p["cpu_s"])
    cpu = {n: (last[n][1] - first[n][1]) / (last[n][0] - first[n][0]) for n in first if last[n][0] > first[n][0]}
    return {"cpu_fraction": cpu, "mem_available_kb_min": min((r["mem_available_kb"] for r in resources), default=None),
            "rtf_mean": sum(x[2] for x in steps) / sum(x[1] for x in steps) if steps else None,
            "stalls_below_0_5": sum(1 for x in steps if x[0] < STALL_RTF), "wall_duration_s": s.get("wall_duration_s"),
            "run_bytes": s.get("run_bytes")}


def r8(results, flights):
    costs = [(f, cost(f)) for f in flights]

    def view(rows):
        rows = [c for _, c in rows if c]
        names = sorted({n for c in rows for n in c["cpu_fraction"]})
        return {"flights": len(rows), "cpu_fraction": {n: dist([c["cpu_fraction"].get(n) for c in rows]) for n in names},
                **{k: dist([c[k] for c in rows]) for k in ("mem_available_kb_min", "rtf_mean", "stalls_below_0_5",
                                                           "wall_duration_s", "run_bytes")},
                "flights_with_a_stall": sum(1 for c in rows if c["stalls_below_0_5"])}
    host = {b: {"passed": (r.get("host_check") or {}).get("passed"),
                "cpu_mean_percent": ((r.get("host_check") or {}).get("host_cpu") or {}).get("mean_percent"),
                "ac_overlay": ((r.get("host_check") or {}).get("ac_overlay") or {}).get("passed"),
                "ac_power": ((r.get("host_check") or {}).get("ac_power") or {}).get("passed")} for b, r in results.items()}
    return {"id": "R8", "accepted_limit": "p7_host_check.yaml (9f78af75…)", "host_check_per_block": host,
            "per_controller": {c: view([x for x in costs if x[0]["controller"] == c]) for c in CONTROLLERS},
            "breaches": [{"block": b, "item": "host_check_not_passed"} for b, h in host.items() if h["passed"] is not True]}


def r9(flights):
    def view(fs):
        return {"abort_reason": dict(Counter(str(metric(f, "abort_reason")) for f in fs if metric(f, "abort_reason"))),
                "p2_failure": dict(Counter(str(p2_failure(f)) for f in fs if p2_failure(f)))}
    return {"id": "R9", "per_controller": by_controller(flights, view)}


def r10(flights):
    rows = [dict(ref(f), items=classes(f).get("integrity_events", [])) for f in flights if classes(f).get("integrity_events")]
    return {"id": "R10", "per_flight": rows,
            "per_controller": by_controller(flights, lambda fs: dict(Counter(i for f in fs for i in classes(f).get("integrity_events", []))))}


# ---- the report -------------------------------------------------------------------------------------------------------
def reported(results, flights, final, sched, lim):
    rows = pairs(final, sched)
    finals_list = list(final.values())
    out, errors = {}, {}
    for cid, fn in (("R1", lambda: r1(finals_list)), ("R2", lambda: r2(rows)), ("R3", lambda: r3(flights, lim)),
                    ("R4", lambda: r4(flights, lim)), ("R5", lambda: r5(flights, lim)), ("R6", lambda: r6(flights, lim)),
                    ("R7", lambda: r7(rows)), ("R8", lambda: r8(results, flights)), ("R9", lambda: r9(flights)),
                    ("R10", lambda: r10(flights))):
        try:
            out[cid] = fn()
        except Exception as exc:
            errors[cid] = type(exc).__name__ + ":" + str(exc)[:300]
    extra = {}
    try:
        safety_records = [dict(f["record"], validity=validity(f)) for f in finals_list if f["record"] is not None]
        clearance = [{"slot": p["slot"], "layout": p["layout"],
                      "difference_p5_minus_p4_m": metric(p["p5"], "min_truth_clearance_m") - metric(p["p4"], "min_truth_clearance_m")}
                     for p in rows if p["analysable"] and metric(p["p5"], "min_truth_clearance_m") is not None
                     and metric(p["p4"], "min_truth_clearance_m") is not None]
        extra = {"safety_per_controller": p7_analysis.safety(safety_records),
                 "paired_min_truth_clearance": {"label": "descriptive; no non-inferiority test", "pairs": clearance,
                                                "difference": dist([c["difference_p5_minus_p4_m"] for c in clearance])}}
    except Exception as exc:
        errors["safety_and_clearance"] = type(exc).__name__ + ":" + str(exc)[:300]
    return out, errors, extra


def risk_register(doc, rep, flights):
    entries = [dict(e, criterion=cid) for cid, r in rep.items() for e in r.get("breaches") or []]
    stalls = (rep.get("R8") or {}).get("per_controller") or {}
    gaps = {c: v["observation_gap_aborts"] for c, v in ((rep.get("R3") or {}).get("per_controller") or {}).items()}
    return {"open": doc["risk_register"]["open"], "rule": doc["risk_register"]["rule"],
            "new_entries_from_R_breaches": entries,
            "reported_against_open": {"RR1_observation_gap_aborts_per_controller": gaps,
                                      "RR1_RR2_stalls_per_controller": {c: {"flights_with_a_stall": v.get("flights_with_a_stall"),
                                                                            "stalls_below_0_5": v.get("stalls_below_0_5")}
                                                                        for c, v in stalls.items()}}}


def build_report(root, manifest, reclassify_fn=reclassify, verify_fn=None):
    doc, frozen = plan(), strict_json(manifest)
    sha = p7e_manifest.manifest_sha(frozen)
    sched = strict_json(P7E / p7e_manifest.SPEC_FILES["schedule"])
    lim = limits(doc)
    results, flights = load(root, sha)
    final = finals(flights)
    hard = [h1(flights, lim), h2(flights, lim), h3(flights), h4(flights), h5(flights), h6(flights), h7(results, flights),
            h8(results, sched), h9(root, manifest, results, flights, sched, sha, reclassify_fn, verify_fn)]
    mapped = set(H_ITEMS["H1"] + H_ITEMS["H2"] + H_ITEMS["H3"] + H_ITEMS["H5"])
    unmapped = sorted({i for f in flights for i in classes(f).get("safety_events", [])
                       if i not in mapped and not i.startswith(H_PREFIXES["H4"] + H_PREFIXES["H5"])})
    rep, errors, extra = reported(results, flights, final, sched, lim)
    completed = hard[7]["batch_completed"]
    reasons = ["%s not met" % h["id"] for h in hard if not h["met"]]
    reasons += [] if completed else ["the declared 60-flight validation did not complete (contract §7.1)"]
    reasons += ["R1-R10 incomplete: " + ", ".join(sorted(errors))] if errors else []
    reasons += ["safety events outside H1-H5: " + ", ".join(unmapped)] if unmapped else []
    status = PASSED if not reasons else NOT_PASSED
    return {"contract": CONTRACT, "report_plan_canonical_sha256": canonical_sha256(doc), "report_code_sha256": digest(Path(__file__)),
            "manifest": {"path": str(manifest), "sha256": sha, "p7_sha256": frozen["p7"]["sha256"], "baselines": frozen["baselines"]},
            "schedule_canonical_sha256": canonical_sha256(sched), "sign_convention": doc["sign_convention"],
            "status": status, "statement_of_a_pass": PASS_STATEMENT if status == PASSED else None, "not_passed_reasons": reasons,
            "hard_criteria": hard, "reported_criteria": rep, "reported_errors": errors, **extra,
            "risk_register": risk_register(doc, rep, flights),
            "blocks": {b: {"stopped": r.get("stopped"), "stop_reason": r.get("stop_reason"), "flights": len(r.get("flights") or [])}
                       for b, r in results.items()},
            "flights_flown": len(flights), "p_values": "none (decision E7)",
            "required_statement": doc["required_statement"], "not_claimed": doc["not_claimed"],
            "limits_30_pairs": doc["limits_30_pairs"], "generated_unix_ns": time.time_ns()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    root = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
    if args.output.exists():
        raise SystemExit("A report exists; the first report is retained")
    value = build_report(root, args.manifest)
    atomic_json(args.output, value, exclusive=True)
    print(json.dumps({"status": value["status"], "not_passed_reasons": value["not_passed_reasons"]}))
    return 0 if value["status"] == PASSED else 1


if __name__ == "__main__":
    raise SystemExit(main())
