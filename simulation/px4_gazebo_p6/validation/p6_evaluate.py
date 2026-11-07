"""P6 independent evaluator (M4): one run -> `p6/p6-evaluation.json`.

Order (contract §8, C8, §4):
1. the declaration must be the frozen row (strict mode);
2. the accepted P5 evaluator's output (`p5-evaluation.json`, written by the unchanged
   `verify_p5_evidence.sh` before this runs) supplies every P2–P5 check. Each of its failures must be
   allowed by this row's exception table, **and** that exception's replacement check must pass.
   Anything else fails the run. Safety, Policy A, ownership and cleanup are never excepted;
3. the fault-occurrence proof, from evidence the injector did not write;
4. the row's response checks, on the accepted controller's own records;
5. timing metrics (reported).

Characterization rows (`X-*`) are evaluated the same way but end `characterized`, never `passed`.
"""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for path in (HERE, HERE.parent, ROOT / "simulation/px4_gazebo_p4/validation", ROOT / "simulation/px4_gazebo/validation",
             ROOT / "simulation/px4_gazebo_p5/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import p6_fault_evaluate as proof_module  # noqa: E402
import p6_safety_evaluate as response  # noqa: E402
import p6_timing_evaluate as timing  # noqa: E402
from p6_run import Run  # noqa: E402

CONTRACT = "p6-evaluation-v1"
POLICY_EXCEPTIONS = {"policy:failure_ticks": "PROOF", "policy:participation": "PROOF"}
TIMING_EXCEPTIONS = {"policy:inference_max": "LIMITS_UNINJECTED", "policy:inference_p99": "LIMITS_UNINJECTED"}
DEPTH_WINDOW = {"depth:": "FAULT_WINDOW_F1F2", "performance:": "FAULT_WINDOW_F1F2", "fault_injection_outside_s6": "PROOF"}

# Per flight row: response checks and the allowed P5-evaluator failures (prefix -> replacement check).
ROWS = {
    "B-0": {"response": ["GOAL", "UNSAFE"], "exceptions": {}},
    "B-1": {"response": ["PH3", "L45", "UNSAFE", "LIMITS_UNINJECTED"],
            "exceptions": {**POLICY_EXCEPTIONS, **TIMING_EXCEPTIONS, "outcome:": "L45"}},
    "S-1": {"response": ["FAULT_WINDOW", "UNSAFE"],
            "exceptions": {**DEPTH_WINDOW, "outcome:": "FAULT_WINDOW"}},
    "S-2'": {"response": ["H35", "A10:snapshot_stale", "L45", "UNSAFE"],
             "exceptions": {"outcome:": "A10", "performance:": "H35"}},
    "S-3": {"response": ["PH3", "L45", "UNSAFE"], "exceptions": {**POLICY_EXCEPTIONS, "outcome:": "L45"}},
    "S-4": {"response": ["H35", "RECOVERY", "GOAL", "UNSAFE"], "exceptions": {"outcome:detour_sequence": "RECOVERY"}},
    "S-5'": {"response": ["R2:stale_action", "L45R:stale_action", "UNSAFE"],
             "exceptions": {"runtime_status:": "R2", "p2:timing": "PROOF", "outcome:": "L45R"}},
    "C-1": {"response": ["STALE_IFF_AGE", "RECOVERY", "GOAL", "UNSAFE"],
            "exceptions": {"depth:": "PROOF", "performance:": "STALE_IFF_AGE", "outcome:detour_sequence": "RECOVERY"}},
    "C-2": {"response": ["LIMITS_UNINJECTED", "GOAL", "UNSAFE"], "exceptions": {**POLICY_EXCEPTIONS, **TIMING_EXCEPTIONS}},
    "C-3": {"response": ["UNSAFE"], "exceptions": {"policy:ledger:argmax_differs": "PROOF", "outcome:": "UNSAFE"}},
    "C-4": {"response": ["NO_INVOKE_AFTER_STALE", "L45", "UNSAFE"],
            "exceptions": {**DEPTH_WINDOW, **POLICY_EXCEPTIONS, **TIMING_EXCEPTIONS, "outcome:": "L45"}},
    "C-5": {"response": ["H35", "STALE_IFF_AGE", "L45", "UNSAFE"],
            "exceptions": {"performance:": "STALE_IFF_AGE", "outcome:": "L45"}},
}
# Gate A ground fixtures: the accepted stale rule replayed on the recorded ground inputs, or a proven refusal.
GROUND_EXCEPTIONS = {"depth:": "GROUND_REPLAY", "performance:": "GROUND_REPLAY", "fault_injection_outside_s6": "PROOF"}
FAIL_CLOSED = {"runtime_status:": "PROOF", "policy_fault:refusal_reason": "PROOF"}
GROUND = {
    "A-5.F1.01": ["snapshot_stale"], "A-5.F1.02": ["adapter_health_missing"], "A-5.F1.04": ["snapshot_stale"],
    "A-5.F1.06": ["snapshot_stale", "unusable_frames"], "A-5.F1.07": ["snapshot_stale"],
    "A-5.F1.11": ["adapter_health_not_fresh", "calibration_mismatch", "snapshot_stale"],
    "A-5.F2.09": ["snapshot_stale", "unusable_frames"], "A-5.F4.02": ["snapshot_stale"],
    "A-5.F4.06": ["adapter_health_missing"]}
for _row, _reasons in GROUND.items():
    ROWS[_row] = {"response": ["GROUND_REPLAY:" + ",".join(_reasons)], "exceptions": GROUND_EXCEPTIONS}
for _row in ("A-5.F3.01", "A-5.F3.02", "A-5.F3.03", "A-5.F4.07", "A-5.F4.13"):
    ROWS[_row] = {"response": [], "exceptions": FAIL_CLOSED}
ROWS["A-3"] = {"response": [], "exceptions": {}}


def codes():
    sys.path.insert(0, str(HERE.parent))
    from injection import specs
    return specs.codes()


def p5_limits():
    return json.loads((ROOT / "simulation/px4_gazebo_p5/configs/p5_validation.yaml").read_text(encoding="utf-8"))["policy"]


def run_fault_window(run):
    """The unchanged P4 §9.1 window function on the run's `p4/fault_injection.json` (bridge-stop rows)."""
    import p3_evaluate
    import p4_evaluate as p4e
    f = response._Failures()
    validation_p3, sensors = p3_evaluate.configs()
    v4 = json.loads((ROOT / "simulation/px4_gazebo_p4/configs/p4_validation.yaml").read_text(encoding="utf-8"))
    try:
        out = p4e.fault_window(run.path, run.summary, validation_p3, sensors, v4["outcomes"], f) or {}
    except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
        out, f.items = {}, f.items + ["fault_window_error:" + type(exc).__name__]
    out["failures"] = f.items
    return out


def respond(run, row, rules, proof, window):
    c = response.Checks(codes())
    injected = {round(r["sim_s"], 6) for r in run.ledger if r.get("failure") is not None}
    for name in rules["response"]:
        code, _, arg = name.partition(":")
        if code == "H35":
            response.h35(run, c)
        elif code == "A10":
            response.a10(run, c, arg or None)
        elif code == "HT20":
            response.ht20(run, c)
        elif code == "PH3":
            response.ph3(run, c)
        elif code == "L45":
            response.l45(run, c)
        elif code == "L45R":
            response.l45r(run, c, arg)
        elif code == "R2":
            response.r2(run, c, arg)
        elif code == "GOAL":
            response.goal(run, c)
        elif code == "RECOVERY":
            response.recovery(run, c)
        elif code == "STALE_IFF_AGE":
            response.stale_iff_age(run, c)
        elif code == "NO_INVOKE_AFTER_STALE":
            response.no_invoke_after_stale(run, c)
        elif code == "LIMITS_UNINJECTED":
            response.limits_uninjected(run, c, p5_limits(), injected)
        elif code == "UNSAFE":
            response.unsafe_setpoints(run, c)
        elif code == "GROUND_REPLAY":
            config = json.loads(run.file("p4_avoidance.yaml").read_text(encoding="utf-8"))
            p = (run.declaration or {}).get("parameters", {}) if run.declaration else {}
            fault = row.split(".", 1)[-1] if row.startswith("A-5.") else None
            mt = response.delay_window(p["delay_s"], p["window_sim_s"]) if fault == "F4.02" else None
            ht = response.drop_window(p["window_sim_s"]) if fault == "F4.06" else None
            response.ground_replay(run, c, arg.split(","), config, mt, ht)
            if run.exists("p4/fault_injection.json"):  # cross-check against the unchanged P4 ground replay
                import p4_evaluate as p4e
                f = response._Failures()
                accepted = p4e.fault_replay(run.path, run.summary, config, f).get("replayed_hover") or {}
                ours = c.out.get("GROUND_REPLAY", {})
                c.record("GROUND_REPLAY_P4", not f.items and accepted.get("reason") == ours.get("reason"),
                         accepted=accepted, items=f.items)
        elif code == "FAULT_WINDOW":
            c.record("FAULT_WINDOW", window is not None and not window["failures"], items=(window or {}).get("failures"))
        else:
            raise ValueError("unknown response check " + name)
    if window is not None:  # F1/F2 alone (C-4: its F3 may legitimately end in policy_unavailable)
        f12 = [x for x in window["failures"] if x.startswith(("s6:f1_", "s6:f2_", "s6:pre:", "s6:injection", "s6:wrong",
                                                                "s6:pre_window"))]
        c.record("FAULT_WINDOW_F1F2", not f12, items=f12)
    c.out["PROOF"] = {"passed": proof["proved"]}
    return c


def compose(p5_failures, exceptions, checks):
    """Every P5-evaluator failure is either excepted (with its replacement check passed) or a P6 failure."""
    allowed, refused = [], []
    for item in p5_failures:
        rule = next((replacement for prefix, replacement in exceptions.items() if item.startswith(prefix)), None)
        replacement = checks.get(rule.split(":")[0]) if rule else None
        if rule and replacement and replacement["passed"]:
            allowed.append({"item": item, "replacement": rule})
        else:
            refused.append(item)
    return allowed, refused


def evaluate(run_path, declaration=None, strict=True):
    run = Run(run_path)
    declaration = declaration or run.declaration
    out = {"contract": CONTRACT, "run_id": run.path.name, "failures": []}
    if declaration is None:
        out.update(status="failed", failures=["declaration_missing"])
        return out
    if strict:
        sys.path.insert(0, str(HERE.parent))
        from injection import declaration as declaration_module
        try:
            declaration_module.validate(declaration)
        except ValueError as exc:
            out.update(status="failed", failures=["declaration_not_frozen_row:" + str(exc)])
            return out
    row = declaration["row"]
    rules = ROWS.get(row) or (ROWS.get(declaration.get("rules")) if not strict else None)
    out.update(row=row, fault=declaration["fault"], gate=declaration.get("gate"))
    if rules is None:
        out.update(status="failed", failures=["no_evaluation_rules_for_row"])
        return out
    window = run_fault_window(run) if run.exists("p4/fault_injection.json") else None
    proof = proof_module.prove(run, declaration, window)
    checks = respond(run, row, rules, proof, window)
    p5 = run.p5_evaluation
    p5_failures = list((p5 or {}).get("failures", [])) if p5 else ["p5_evaluation_missing"]
    allowed, refused = compose(p5_failures, rules["exceptions"], checks.out)
    out.update(proof=proof, response=checks.out, p5_failures_allowed=allowed, p5_failures_refused=refused,
               timing=timing.metrics(run), fault_window=window)
    out["failures"] = proof["failures"] + checks.failures + ["p5:" + x for x in refused]
    characterization = row.startswith("X-")
    out["status"] = ("characterized" if characterization else "passed") if not out["failures"] else (
        "characterized_with_findings" if characterization else "failed")
    out["acceptance_credit"] = out["status"] == "passed"
    return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args(argv)
    out = evaluate(args.run)
    target = args.run / "p6" / "p6-evaluation.json"
    with target.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(out, indent=2, sort_keys=True, allow_nan=False, default=str) + "\n")
    print(json.dumps({"status": out["status"], "failures": out["failures"]}))
    return 0 if out["status"] in ("passed", "characterized") else 1


if __name__ == "__main__":
    raise SystemExit(main())
