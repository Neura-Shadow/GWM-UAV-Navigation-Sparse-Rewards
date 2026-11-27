"""M7 (read-only): A/B/C comparison and coverage by class and tier for the accepted real P6 gates of one manifest.

Reads only retained evidence (gate results, run directories, the M3 offline suite of A-4, the frozen fault spec and
the accepted P5 comparison files) and writes one new JSON file (exclusive create). Descriptive only (plan §8).
Usage: m7_arms.py <manifest-sha> <output.json>
"""
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

REPO = Path("/mnt/d/GWM-UAV-Navigation-Sparse-Rewards")
STATE = Path.home() / "uav_autonomy" / "state"
sys.path.insert(0, str(REPO / "simulation" / "px4_gazebo_p6"))
from injection import specs  # noqa: E402  (the frozen, manifest-bound fault spec; read only)

SHA, OUT = sys.argv[1], Path(sys.argv[2])
ARMS = {"B-0": "A", "C-4": "C", "C-5": "C"}  # every other flight row is a single fault (arm B), plan §8
DISPATCH_BUDGET_S = 0.050  # accepted P2 p2_timing.yaml dispatch_budget_wall_s (contract §3)
STABILITY_S = 2.0  # contract §6.5 recovery stability window
LAND = 21  # MAV_CMD_NAV_LAND
P5_REFERENCE = {"S1": "gate_s_s1", "S2": "gate_b_s2", "S3": "gate_c_s3", "S4": "gate_c_s4"}


def jl(path):
    with open(path, encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def jf(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def pct(values, q):
    values = sorted(values)
    return values[min(len(values) - 1, int(q * len(values)))] if values else None


def hover_intervals(transitions, end_sim):
    out, start = [], None
    for t in transitions:
        if t["to"] == "HOVER" and start is None:
            start = t["sim_s"]
        elif t["from"] == "HOVER" and start is not None:
            out.append((start, t["sim_s"], t["to"]))
            start = None
    if start is not None:
        out.append((start, end_sim, None))
    return out


def flight(run_dir, gate_run):
    run = Path(run_dir)
    p6 = jf(run / "p6" / "p6-evaluation.json")
    p5 = jf(run / "p5-evaluation.json")
    ctl = jf(run / "controller-result.json")
    decisions = [d for d in jl(run / "p4" / "decisions.jsonl") if d.get("event") == "p4_decision"]
    ledger = jl(run / "p5" / "policy-ledger.jsonl")
    callbacks = [r for r in jl(run / "timing-trace.jsonl") if r.get("operation") == "control_callback"]
    timing = p6.get("timing") or {}
    trigger = timing.get("trigger_sim_s")
    sup = ctl["p4"]["supervisor_transitions"] if ctl.get("p4") else []
    end_sim = decisions[-1]["sim_s"] if decisions else None

    hovers = hover_intervals(sup, end_sim)
    returns = [h for h in hovers if h[2] in ("AVOIDING", "RESUME", "NORMAL", "OBSTACLE_DETECTED")]
    relapses = sum(1 for h in returns if any(0 < (o[0] - h[1]) <= STABILITY_S for o in hovers))

    failing = {round(r["sim_s"], 6) for r in ledger if r.get("failure") is not None}
    hold = [d for d in decisions if d.get("stale") or d.get("state") in ("HOVER", "ABORT") or round(d["sim_s"], 6) in failing]
    inference = [r["inference_s"] for r in ledger if r.get("invoked") and r.get("failure") is None and r.get("inference_s") is not None]

    lands = sorted(t["sent_sim_s"] for t in ctl.get("transactions", []) if t.get("command") == LAND)
    p2 = {t["to"]: t["sim_s"] for t in ctl.get("transitions", [])}
    resp = p6.get("response") or {}
    landing_detail = (resp.get("LANDING") or {}).get("detail") or {}
    recovery_landing = (landing_detail.get("recovery_landing") or resp.get("RECOVERY_LANDING") or {})
    if "COMPLETE" in p2 and "REQUEST_LAND" in p2:
        land_to_disarmed = {"clock": "sim", "s": p2["COMPLETE"] - p2["REQUEST_LAND"]}
    elif recovery_landing.get("landed_wall_s") and recovery_landing.get("land_wall_s"):
        land_to_disarmed = {"clock": "wall", "s": recovery_landing["landed_wall_s"] - recovery_landing["land_wall_s"]}
    else:
        land_to_disarmed = None
    over = [r["duration_wall_s"] for r in callbacks if r["duration_wall_s"] > DISPATCH_BUDGET_S]
    out = p5["outcome"]
    safety = p5["safety"]
    return {
        "row": gate_run["row"], "arm": ARMS.get(gate_run["row"], "B"), "world": out.get("scenario"), "fault": p6.get("fault"),
        "run": run.name, "evaluation_sha256": hashlib.sha256((run / "p6" / "p6-evaluation.json").read_bytes()).hexdigest(),
        "safety_validation": p6["status"],
        "mission": {"label": out.get("label"), "controller_outcome": out.get("controller_outcome"),
                    "p2_final_state": ctl.get("final_state"), "p2_failure": ctl.get("failure"),
                    "abort_reason": ctl["p4"].get("abort_reason") if ctl.get("p4") else None,
                    "goal_reached": bool(ctl.get("p4") and ctl["p4"].get("goal_reached_sim_s")),
                    "final_goal_distance_m": out.get("final_goal_distance_m"),
                    "time_to_goal_sim_s": (p5.get("reported") or {}).get("time_to_goal_sim_s")},
        "truth": {"collisions": safety.get("collisions"), "min_truth_clearance_m": safety.get("min_truth_clearance_m"),
                  "tracking_max_m": safety.get("tracking_max_m"),
                  "setpoints_inside_inflated_obstacles": len(safety.get("setpoints_inside_inflated_obstacles") or [])},
        "unsafe_commands": len((resp.get("UNSAFE") or {}).get("moves") or []),
        "latency": {"trigger_sim_s": trigger, "detection_sim_s": timing.get("detection_latency_sim_s"),
                    "fault_to_safe_action_sim_s": timing.get("fault_to_safe_action_sim_s"),
                    "degraded_sim_s": timing.get("degraded_sim_s"), "recovered": timing.get("recovered")},
        "hover": {"intervals": len(hovers), "time_sim_s": round(sum(b - a for a, b, _ in hovers), 3),
                  "recoveries": len(returns), "relapses_within_2s": relapses},
        "land": {"land_commands": len(lands), "trigger_to_land_sim_s": (lands[0] - trigger) if lands and trigger is not None else None,
                 "land_to_landed_and_disarmed": land_to_disarmed},
        "availability": {"decision_ticks": len(decisions), "held_ticks": len(hold),
                         "not_held_fraction": (1 - len(hold) / len(decisions)) if decisions else None},
        "deadlines": {"control_callbacks": len(callbacks), "over_dispatch_budget": len(over),
                      "max_callback_wall_s": max((r["duration_wall_s"] for r in callbacks), default=None)},
        "policy": {"invoked": sum(1 for r in ledger if r.get("invoked")), "failure_ticks": len(failing),
                   "failure_reasons": sorted({r["failure"] for r in ledger if r.get("failure") is not None}),
                   "uninjected_inference_p99_s": pct(inference, 0.99), "uninjected_inference_max_s": max(inference, default=None)},
        "rtf": (p5.get("rtf") or {}).get("rtf"), "sim_s": (p5.get("rtf") or {}).get("sim_s"),
    }


def coverage(evaluated_rows, offline):
    faults = specs.faults()
    passing = {}
    for row, fault in evaluated_rows:
        passing.setdefault(fault, []).append(row)
    table, missing = {}, []
    for fid, f in faults.items():
        if f.get("alias_of"):
            continue
        for cls in f.get("classes", []):
            entry = table.setdefault(cls, {"faults": 0, "covered": 0, "items": {}})
            entry["faults"] += 1
            how = None
            if cls == "R-O":
                how = "A-4 offline suite" if (offline.get(fid) or {}).get("passed") else None
            elif cls in ("R-A", "R-B", "R-S", "R-C"):
                prefix = {"R-A": "A-", "R-B": "B-", "R-S": "S-", "R-C": "C-"}[cls]
                rows = [r for r in passing.get(fid, []) if r.startswith(prefix)]
                via = [(r, src) for src in f.get("realized_by", []) for r in passing.get(src, []) if r.startswith(prefix)]
                how = rows or ["%s via %s" % (r, src) for r, src in via] or None
            if cls.startswith("R-"):
                entry["covered"] += 1 if how else 0
                entry["items"][fid] = how
                if not how:
                    missing.append((fid, cls))
            else:
                entry["items"][fid] = "reported" if cls == "C" else cls
    tiers = {}
    for f in faults.values():
        if not f.get("alias_of"):
            for t in f.get("tiers", []):
                tiers[t] = tiers.get(t, 0) + 1
    return {"by_class": table, "required_uncovered": missing, "tiers": tiers,
            "aliases": sorted(k for k, f in faults.items() if f.get("alias_of"))}


def arm_summary(flights):
    out = {}
    for arm in "ABC":
        fs = [f for f in flights if f["arm"] == arm]
        out[arm] = {
            "flights": len(fs), "rows": [f["row"] for f in fs],
            "safety_validations_passed": sum(1 for f in fs if f["safety_validation"] == "passed"),
            "collisions": sum(f["truth"]["collisions"] or 0 for f in fs),
            "min_truth_clearance_m": min((f["truth"]["min_truth_clearance_m"] for f in fs
                                          if f["truth"]["min_truth_clearance_m"] is not None), default=None),
            "flights_without_an_obstacle": [f["row"] for f in fs if f["truth"]["min_truth_clearance_m"] is None],
            "unsafe_commands": sum(f["unsafe_commands"] for f in fs),
            "goal_reached": sum(1 for f in fs if f["mission"]["goal_reached"]),
            "landed_after_supervisor_abort": sum(1 for f in fs if f["mission"]["abort_reason"] and f["mission"]["p2_final_state"] == "COMPLETE"),
            "landed_through_p2_recovery": sum(1 for f in fs if f["mission"]["p2_final_state"] == "RECOVERY"),
            "other_terminal": [f["row"] for f in fs if not f["mission"]["goal_reached"] and not f["mission"]["abort_reason"]
                               and f["mission"]["p2_final_state"] != "RECOVERY"],
            "recoveries": sum(f["hover"]["recoveries"] for f in fs),
            "relapses_within_2s": sum(f["hover"]["relapses_within_2s"] for f in fs),
            "over_dispatch_budget": sum(f["deadlines"]["over_dispatch_budget"] for f in fs),
        }
        detections = [f["latency"]["detection_sim_s"] for f in fs if f["latency"]["detection_sim_s"] is not None]
        out[arm]["detection_sim_s_median"] = statistics.median(detections) if detections else None
    return out


def main():
    gates, flights, evaluated = {}, [], []
    for g in "ABSC":
        path = STATE / f"p6-gate-{g}-{SHA}.result.json"
        result = jf(path)
        assert result["accepted"] is True and result["acceptance_credit"] is True and result["dry_run"] is False
        gates[g] = {"result": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "rows": result["planned"]}
        for run in result["runs"]:
            if run.get("evaluation_sha256") and run.get("run"):
                ev = Path(run["run"]) / "p6" / "p6-evaluation.json"
                assert hashlib.sha256(ev.read_bytes()).hexdigest() == run["evaluation_sha256"], run["row"]
                evaluated.append((run["row"], jf(ev).get("fault")))
            if run.get("scope") == "flight":
                flights.append(flight(run["run"], run))
            if run["row"] == "A-4":
                offline = jf(Path(run["run"]) / "p6-offline-suite.json")
    reference = {}
    for world, name in P5_REFERENCE.items():
        p = REPO / "docs" / "evidence" / f"v3_sim_p5_2becfbbb_{name}_vs_p4.json"
        d = jf(p)["p5"]
        reference[world] = {"file": p.name, "run": d["run"], "outcome": d["outcome"], "safety": d["safety"],
                            "time_to_goal_sim_s": d["reported"].get("time_to_goal_sim_s"), "rtf": d["rtf"].get("rtf")}
    arm_a = [f for f in flights if f["arm"] == "A"]
    report = {
        "contract": "p6-m7-arms-v1", "manifest_sha256": SHA, "gates": gates,
        "note": "descriptive, paired by world, one flight per row: existence demonstrations, no rates or significance (plan §8)",
        "flights": flights, "arms": arm_summary(flights),
        "arm_a_fault_labels": {"flights": len(arm_a), "stale_ticks": sum(f["availability"]["held_ticks"] for f in arm_a),
                               "hover_intervals": sum(f["hover"]["intervals"] for f in arm_a),
                               "sim_hours": sum(f["sim_s"] or 0 for f in arm_a) / 3600},
        "p5_reference_by_world": reference,
        "offline_suite": {"counts": offline["counts"], "passed": offline["passed"], "git_head": offline["git_head"],
                          "characterization_items": sorted(offline.get("characterization", {}))},
        "coverage": coverage(evaluated, offline.get("required_offline", {})),
        "unsafe_outcome_upper_bound_95": 1 - 0.05 ** (1 / len(flights)),
    }
    with OUT.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, sort_keys=True, allow_nan=False, default=str) + "\n")
    print(json.dumps({"flights": len(flights), "arms": {k: v["flights"] for k, v in report["arms"].items()},
                      "required_uncovered": report["coverage"]["required_uncovered"]}))


if __name__ == "__main__":
    main()
