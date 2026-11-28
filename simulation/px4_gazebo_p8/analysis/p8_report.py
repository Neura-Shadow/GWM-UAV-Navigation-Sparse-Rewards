"""P8 engineering report: the approved report plan `p8-report-plan-v1` (canonical a313a034…) as code (contract
`p8-navigation-recovery-v1` §6, §7, §12). Offline only. It runs once, after the last block.

**Inputs, each checked by hash:**
* the 3 P8 block results and their ledgers;
* for every flown flight, its summary, the accepted P4 or P5 evaluation, the frozen P7 flight record, the P8
  assessment and the H10 record (`p8/`);
* the approved P8 specifications.

**Hard criteria:**
* **H1–H7** reuse the accepted P7-E report functions unchanged (`p7e_report.py`, imported by path), on flights
  loaded with P8's assessment.
* **H8** is the P8 block accounting against the P8 plan.
* **H9:**
  * the frozen classifier, re-run through its own wrapper on a symlink view, reproduces each record;
  * FC-3 and H10 reproduce;
  * no run file changes;
  * every world equals its approved hash;
  * the P8 manifest (with P7-E, P7 and P3-P6 inside it) re-verifies.
* **H10:** every flight's H10 record passed.
* **H11:** the offline records V1 and V2 are bound by the manifest and record a pass.

**Reported (descriptive):** R-P8-1 to R-P8-10. The effectiveness target is none (P8-C1), and there are no p-values.
The report carries the contract's §12 statement verbatim. The output is written once.
"""
import argparse
from collections import Counter
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
P7E = REPO / "simulation/px4_gazebo_p7e"
_spec = importlib.util.spec_from_file_location("p7e_report_accepted", P7E / "analysis/p7e_report.py")
rep7 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rep7)  # the accepted P7-E report code, used unchanged
for _path in (P8 / "scripts", P8 / "validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p8_h10_check  # noqa: E402
import p8_manifest  # noqa: E402
import run_p8_block  # noqa: E402
from p3_provenance import atomic_json, canonical_sha256, digest, strict_json  # noqa: E402

p7_analysis, p7e_assess = rep7.p7_analysis, rep7.p7e_assess
CONTRACT = "p8-engineering-report-v1"
PASSED, NOT_PASSED = "engineering validation passed", "engineering validation not passed"
PASS_STATEMENT = ("the P8 recovery variants completed the declared 60-flight engineering validation, with every hard "
                  "criterion H1-H11 met")
VARIANTS = ("p8_p4r", "p8_p5r")


def plan():
    value = strict_json(P8 / p8_manifest.SPEC_FILES["report_plan"])
    if canonical_sha256(value) != p8_manifest.APPROVED_SPECS["report_plan"]:
        raise SystemExit("The report plan differs from the approved a313a034…")
    return value


def specs():
    return {name: strict_json(P8 / path) for name, path in p8_manifest.SPEC_FILES.items()}


def layout_records(doc):
    return {r["id"]: r for stage in doc["stages"].values() for r in stage["layouts"]}


# ---- evidence -----------------------------------------------------------------------------------------------------
def load_flight(block, entry):
    """As the accepted `load_flight`, with the P8 assessment, the H10 record and the recovery ledger."""
    run = Path(entry["run"]) if entry.get("run") else None
    flight = {k: entry.get(k) for k in ("slot", "pair_in_block", "layout", "controller", "variant", "stage", "attempt", "plan_index")}
    flight.update(block=block, run=str(run) if run else None, summary=None, evaluation=None, record=None, assessment=None,
                  h10=None, ledger=None)
    if run is not None:
        flight["summary"] = rep7.checked(run / "summary.json", entry.get("summary_sha256"))
        flight["evaluation"] = rep7.checked(run / rep7.EVALUATIONS.get(entry.get("controller"), "-"), entry.get("evaluation_sha256"))
        flight["record"] = rep7.checked(run / p7e_assess.RECORD, entry.get("flight_record_sha256"))
        flight["assessment"] = rep7.checked(run / run_p8_block.ASSESSMENT, entry.get("assessment_sha256"))
        flight["h10"] = rep7.checked(run / run_p8_block.H10_RECORD, entry.get("h10_record_sha256"))
        ledger = run / "p8/recovery-ledger.jsonl"
        if entry.get("recovery_ledger_sha256") and ledger.exists() and digest(ledger) == entry["recovery_ledger_sha256"]:
            flight["ledger"] = rep7.jsonl(ledger)
    flight["evidence"] = {"run_directory": run is not None and run.is_dir(), "summary": flight["summary"] is not None,
                          "evaluation": flight["evaluation"] is not None, "flight_record": flight["record"] is not None,
                          "assessment": flight["assessment"] is not None, "h10_record": flight["h10"] is not None,
                          "recovery_ledger": flight["ledger"] is not None,
                          "ledger_record_hash": entry.get("flight_record_sha256") is not None}
    flight["evidence_complete"] = all(flight["evidence"].values())
    return flight


def load(root, sha):
    results, flights = {}, []
    for block in range(1, p8_manifest.BLOCKS + 1):
        path = p8_manifest.result_path(root, block, sha)
        if path.exists():
            results[block] = strict_json(path)
            flights += [load_flight(block, e) for e in results[block].get("flights") or []]
    return results, flights


# ---- hard criteria H8-H11 -------------------------------------------------------------------------------------------
def h8(results, sched):
    blocks, earlier_stop = {}, None
    cap = math.floor(rep7.run_p7e_block.frozen_block.REPLACEMENT_FRACTION * 20)
    for block in range(1, p8_manifest.BLOCKS + 1):
        res = results.get(block)
        planned = run_p8_block.plan_flights(sched, block)
        if res is None:
            blocks[block] = {"result": False, "explained": earlier_stop is not None, "by": earlier_stop}
            earlier_stop = earlier_stop or "block %d not flown" % block
            continue
        acc = rep7.run_p7e_block.accounting(planned, res.get("flights") or [], res.get("not_flown") or [],
                                            res.get("missing_pairs") or [], res.get("interrupted"), res.get("stopped") is not False)
        attempts = Counter(f.get("plan_index") for f in res.get("flights") or [])
        row = {"result": True, "stage": p8_manifest.STAGES[block], "plan_matches": res.get("planned") == planned,
               "accounting_complete": acc["complete"], "all_flown_or_missing": acc["all_flown_or_missing"],
               "status_counts": acc["status_counts"], "replacements": len(res.get("replacements") or []), "replacement_cap": cap,
               "replacements_within_cap": len(res.get("replacements") or []) <= cap and res.get("replacement_cap") == cap,
               "at_most_one_replacement_per_flight": max(attempts.values(), default=0) <= 2,
               "stopped": res.get("stopped") is not False, "stop_reason": res.get("stop_reason"),
               "unflown_explained": not (res.get("not_flown") or res.get("interrupted")) or bool(res.get("stop_reason"))}
        row["explained"] = all(row[k] for k in ("plan_matches", "accounting_complete", "replacements_within_cap",
                                                "at_most_one_replacement_per_flight", "unflown_explained"))
        if row["stopped"] and earlier_stop is None:
            earlier_stop = "block %d stopped: %s" % (block, res.get("stop_reason"))
        blocks[block] = row
    completed = all(r["result"] and not r["stopped"] and r["explained"] and r["all_flown_or_missing"] for r in blocks.values())
    return rep7.criterion("H8", all(r["explained"] for r in blocks.values()), "completed" if completed else "not completed",
                          limit="all 60 flown, or every unflown flight explained by a stop rule", blocks=blocks,
                          batch_completed=completed)


def h9(root, manifest, results, flights, doc, sha, params, reclassify_fn=rep7.reclassify, verify_fn=None):
    detail = {"manifest": None, "worlds": [], "reclassification": [], "assessment": [], "h10": [], "unchanged": []}
    try:
        _, current = (verify_fn or p8_manifest.verify_current)(root, manifest)
        detail["manifest"] = {"verified": current == sha, "sha256": current}
    except Exception as exc:
        detail["manifest"] = {"verified": False, "error": type(exc).__name__ + ":" + str(exc)[:300]}
    detail["results_bind_manifest"] = all(r.get("manifest_sha256") == sha for r in results.values())
    worlds = {k: r["world_sha256"] for k, r in layout_records(doc).items()}
    with tempfile.TemporaryDirectory(prefix="p8-h9-") as work:
        for n, f in enumerate(flights):
            ref = rep7.ref(f)
            if not f["run"] or not Path(f["run"]).is_dir():
                detail["reclassification"].append(dict(ref, reproduced=False, error="no run"))
                continue
            run = Path(f["run"])
            world = ((f["summary"] or {}).get("p7") or {}).get("world") or {}
            detail["worlds"].append(dict(ref, verified=world.get("identical") is True and world.get("generated_sha256") == worlds.get(f["layout"])))
            before = rep7.files_state(run)
            record, error = reclassify_fn(run, Path(work) / str(n))
            detail["reclassification"].append(dict(ref, reproduced=record is not None and record == f["record"], error=error))
            again = p7e_assess.assess(run, f["record"], f["summary"]) if f["record"] is not None and f["summary"] is not None else None
            mine = {k: (f["assessment"] or {}).get(k) for k in (again or {}) if k not in ("stop_reasons",)}
            detail["assessment"].append(dict(ref, reproduced=again is not None and {k: again[k] for k in mine} == mine))
            h10 = p8_h10_check.check_run(run, strict_json(run / "p4_avoidance.yaml"), params)
            detail["h10"].append(dict(ref, reproduced=h10 == f["h10"]))
            detail["unchanged"].append(dict(ref, unchanged=rep7.files_state(run) == before))
    checks = [detail["manifest"]["verified"], detail["results_bind_manifest"]]
    for key, field in (("worlds", "verified"), ("reclassification", "reproduced"), ("assessment", "reproduced"),
                       ("h10", "reproduced"), ("unchanged", "unchanged")):
        checks += [r[field] for r in detail[key]]
    failed = sum(1 for c in checks if not c)
    return rep7.criterion("H9", failed == 0 and bool(flights), "identical" if failed == 0 else "%d checks differ" % failed,
                          limit="identical", **detail)


def h10(flights):
    bad = [dict(rep7.ref(f), violations=((f["h10"] or {}).get("violations") or ["no H10 record"])[:5])
           for f in flights if not (f["h10"] or {}).get("ok")]
    actions = sum((f["h10"] or {}).get("actions", 0) for f in flights)
    return rep7.criterion("H10", not bad and bool(flights), "%d/%d" % (len(flights) - len(bad), len(flights)),
                          limit="100 % reproduce; 0 violations", recovery_actions=actions, flights=bad)


def h11(frozen):
    try:
        current = p8_manifest.offline_records()
        ok = current == frozen.get("offline_records")
        return rep7.criterion("H11", ok, "bound and passed" if ok else "differs", records=current)
    except Exception as exc:
        return rep7.criterion("H11", False, "missing or not passed", error=type(exc).__name__ + ":" + str(exc)[:300])


# ---- reported criteria -------------------------------------------------------------------------------------------------
def outcome(f):
    return (f["record"] or {}).get("outcome") if rep7.validity(f) == "valid" else None


def by_variant(flights, fn):
    return {v: fn([f for f in flights if f.get("variant") == v]) for v in VARIANTS}


def r1(final):
    def view(fs):
        valid = [f for f in fs if rep7.validity(f) == "valid"]
        x = sum(1 for f in valid if outcome(f) == "goal_success")
        return {"valid_flights": len(valid), "goal_success": x,
                "ci95_clopper_pearson": p7_analysis.clopper_pearson(x, len(valid)) if valid else None}
    return {"id": "R-P8-1", "per_variant_and_stage": {s: by_variant([f for f in final if f["stage"] == s], view)
                                                       for s in ("V3", "V4", "V5")}}


def r2(final, doc):
    records = layout_records(doc)
    rows = []
    for f in sorted(final, key=lambda f: (f["slot"], f["variant"])):
        if f["stage"] not in ("V3", "V5"):
            continue
        baseline = (records.get(f["layout"]) or {}).get("p7e_outcomes", {}).get(f["controller"])
        rows.append({"stage": f["stage"], "slot": f["slot"], "layout": f["layout"], "variant": f["variant"],
                     "baseline_p7e": baseline, "p8": outcome(f)})
    changes = Counter((r["stage"], r["baseline_p7e"], r["p8"]) for r in rows)
    return {"id": "R-P8-2", "rows": rows, "transitions": [{"stage": k[0], "baseline": k[1], "p8": k[2], "flights": v}
                                                          for k, v in sorted(changes.items(), key=str)]}


def ledger_view(f):
    led = [r for r in (f["ledger"] or []) if r.get("kind") != "header"]
    starts = [r for r in led if r["kind"] == "episode_start"]
    actions = [r for r in led if r["kind"] == "action"]
    return {"episodes": len(starts), "actions": len(actions), "refusals": sum(1 for r in led if r["kind"] == "refused"),
            "phases": dict(Counter(r.get("phase") for r in actions)), "guard_starts": sum(1 for r in led if r["kind"] == "r4_guard_start"),
            "search_wall_s": [r["search"]["wall_s"] for r in starts],
            "recovery_time_s": sum(max((a["sim_s"] for a in actions if a["episode_hover_since_sim_s"] == s["episode_hover_since_sim_s"]),
                                       default=s["sim_s"]) - s["sim_s"] for s in starts)}


def r3(flights):
    return {"id": "R-P8-3", "per_variant": by_variant(flights, lambda fs: {
        "flights": len(fs), "flights_with_recovery": sum(1 for f in fs if ledger_view(f)["episodes"]),
        "episodes": sum(ledger_view(f)["episodes"] for f in fs), "actions": sum(ledger_view(f)["actions"] for f in fs),
        "refusals_fc4": sum(ledger_view(f)["refusals"] for f in fs), "guard_starts": sum(ledger_view(f)["guard_starts"] for f in fs),
        "phases": dict(sum((Counter(ledger_view(f)["phases"]) for f in fs), Counter())),
        "recovery_time_s": rep7.dist([ledger_view(f)["recovery_time_s"] for f in fs if ledger_view(f)["episodes"]])})}


def failure_mode(f):
    result = (f["summary"] or {}).get("controller_result") or {}
    p4 = result.get("p4") or {}
    if outcome(f) != "failure":
        return None
    if p4.get("abort_reason") != "hover_timeout":
        return "other:" + str(p4.get("abort_reason"))
    hover = [t for t in p4.get("supervisor_transitions", []) if t["to"] == "HOVER"]
    goal = p4.get("goal_enu")
    decisions = [d for d in rep7.jsonl(Path(f["run"]) / "p4/decisions.jsonl") if d.get("event") == "p4_decision"]
    at = next((d["target_enu"] for d in decisions if hover and d["sim_s"] >= hover[-1]["sim_s"]), None)
    if at is None or goal is None:
        return "hover_timeout:unknown"
    return "near_goal" if math.dist(at, goal) < 0.6 else "mid_course"


def r4(flights):
    return {"id": "R-P8-4", "per_variant": by_variant(flights, lambda fs: dict(Counter(failure_mode(f) for f in fs if failure_mode(f))))}


def r5(flights):
    return {"id": "R-P8-5", "recovery_search_wall_s": rep7.dist([w for f in flights for w in ledger_view(f)["search_wall_s"]]),
            "p2_timing_status": dict(Counter(str((f["evaluation"] or {}).get("p2_timing_status")) for f in flights)),
            "breaches": rep7.items_with(flights, ("p2:timing",))}


def r6(flights):
    return {"id": "R-P8-6", "per_variant": by_variant(flights, lambda fs: {
        k: rep7.dist([rep7.metric(f, k) for f in fs]) for k in ("min_truth_clearance_m", "path_length_m", "time_to_goal_sim_s")})}


def r9(final, doc):
    records = layout_records(doc)
    rows = []
    for f in final:
        if f["stage"] == "V5" and (records.get(f["layout"]) or {}).get("p7e_outcomes", {}).get(f["controller"]) == "goal_success" \
                and outcome(f) != "goal_success":
            rows.append(dict(rep7.ref(f), variant=f["variant"], outcome=outcome(f), mode=failure_mode(f),
                             recovery_acted=ledger_view(f)["actions"] > 0))
    return {"id": "R-P8-9", "regressions": rows}


LEDGER_MISMATCH = "policy:ledger:decision_mismatch"


def ledger_reconciliation(f):
    """D-M3-1 (a): reconcile the accepted P5 evaluator's `policy:ledger:decision_mismatch` with the recovery ledger.

    The evaluator compares each policy-ledger record with the executed `p4_decision` on `sim`, `action` and `state`.
    By design (S4) recovery replaces the policy supervisor's HOLD after the P5 ledger records it. The item is
    explained only if all of these hold:
    * the record and decision counts are equal;
    * every mismatch differs in `action` alone;
    * the set of mismatching ticks equals the set of recovery `action` ticks.
    Anything else is an unexplained integrity event."""
    run = Path(f["run"]) if f.get("run") else None
    if run is None or f.get("controller") != "p5":
        return None
    records = rep7.jsonl(run / "p5/policy-ledger.jsonl")
    decisions = [e for e in rep7.jsonl(run / "ros-events.jsonl") if e.get("event") == "p4_decision"]
    actions = {round(r["sim_s"], 6) for r in (f.get("ledger") or []) if r.get("kind") == "action"}
    mism = [(r, e) for r, e in zip(records, decisions)
            if r.get("sim_s") != e.get("sim_s") or r.get("action") != e.get("action") or r.get("state") != e.get("state")]
    ticks = {round(e["sim_s"], 6) for _, e in mism}
    other = sum(1 for r, e in mism if r.get("sim_s") != e.get("sim_s") or r.get("state") != e.get("state"))
    flagged = LEDGER_MISMATCH in rep7.failures(f)
    explained = len(records) == len(decisions) and other == 0 and ticks == actions
    return dict(rep7.ref(f), variant=f.get("variant"), evaluator_item=flagged, records=len(records), decisions=len(decisions),
                mismatches=len(mism), recovery_actions=len(actions), other_field_mismatches=other,
                mismatches_equal_recovery_actions=ticks == actions, explained=explained)


def r10(flights):
    rows = [x for x in (ledger_reconciliation(f) for f in flights) if x is not None]
    flagged = [x for x in rows if x["evaluator_item"]]
    return dict(rep7.r10(flights), id="R-P8-10",
                p5_ledger_reconciliation={"rule": "D-M3-1 (a): every policy:ledger:decision_mismatch must equal the "
                                                  "flight's recovery actions, tick for tick, in the action field only",
                                          "flights": rows, "flagged": len(flagged),
                                          "flagged_and_explained": sum(1 for x in flagged if x["explained"]),
                                          "unexplained": [x for x in rows if not x["explained"]]})


def reported(results, flights, final, doc, lim):
    out, errors = {}, {}
    for cid, fn in (("R-P8-1", lambda: r1(final)), ("R-P8-2", lambda: r2(final, doc)), ("R-P8-3", lambda: r3(flights)),
                    ("R-P8-4", lambda: r4(flights)), ("R-P8-5", lambda: r5(flights)), ("R-P8-6", lambda: r6(flights)),
                    ("R-P8-7", lambda: {"id": "R-P8-7", "perception": rep7.r4(flights, lim), "inference": rep7.r5(flights, lim)}),
                    ("R-P8-8", lambda: dict(rep7.r8(results, flights), id="R-P8-8")), ("R-P8-9", lambda: r9(final, doc)),
                    ("R-P8-10", lambda: r10(flights))):
        try:
            out[cid] = fn()
        except Exception as exc:
            errors[cid] = type(exc).__name__ + ":" + str(exc)[:300]
    return out, errors


# ---- the report ---------------------------------------------------------------------------------------------------------
def build_report(root, manifest, reclassify_fn=rep7.reclassify, verify_fn=None):
    doc_plan, frozen = plan(), strict_json(manifest)
    sha = p8_manifest.manifest_sha(frozen)
    s = specs()
    lim = rep7.limits(doc_plan)
    results, flights = load(root, sha)
    final = [f for f in rep7.finals([dict(f, controller=f["variant"]) for f in flights]).values()]
    final = [dict(f, controller=p8_manifest.VARIANTS[f["variant"]]) for f in final]
    hard = [rep7.h1(flights, lim), rep7.h2(flights, lim), rep7.h3(flights), rep7.h4(flights), rep7.h5(flights),
            rep7.h6(flights), rep7.h7(results, flights), h8(results, s["schedule"]),
            h9(root, manifest, results, flights, s["layouts"], sha, s["recovery"], reclassify_fn, verify_fn), h10(flights),
            h11(frozen)]
    rep, errors = reported(results, flights, final, s["layouts"], lim)
    completed = hard[7]["batch_completed"]
    reasons = ["%s not met" % h["id"] for h in hard if not h["met"]]
    reasons += [] if completed else ["the declared 60-flight validation did not complete"]
    reasons += ["R-P8 incomplete: " + ", ".join(sorted(errors))] if errors else []
    status = PASSED if not reasons else NOT_PASSED
    return {"contract": CONTRACT, "report_plan_canonical_sha256": canonical_sha256(doc_plan), "report_code_sha256": digest(Path(__file__)),
            "manifest": {"path": str(manifest), "sha256": sha, "p7e_sha256": frozen["p7e"]["sha256"], "baselines": frozen["baselines"]},
            "specs": frozen["specs"], "status": status, "statement_of_a_pass": PASS_STATEMENT if status == PASSED else None,
            "not_passed_reasons": reasons, "hard_criteria": hard, "reported_criteria": rep, "reported_errors": errors,
            "effectiveness": doc_plan["effectiveness"],
            "blocks": {b: {"stage": p8_manifest.STAGES[b], "stopped": r.get("stopped"), "stop_reason": r.get("stop_reason"),
                           "flights": len(r.get("flights") or [])} for b, r in results.items()},
            "flights_flown": len(flights), "p_values": "none", "required_statement": doc_plan["required_statement"],
            "not_claimed": doc_plan["not_claimed"], "risk_register": doc_plan["risk_register"], "generated_unix_ns": time.time_ns()}


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
