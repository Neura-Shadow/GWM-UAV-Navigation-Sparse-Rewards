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
import p6_integration_evaluate as integration  # noqa: E402
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
    "S-5'": {"response": ["R2:stale_action", "L45R:stale_action", "UNSAFE", "RECOVERY_LANDING"],
             "exceptions": {"runtime_status:": "R2", "p2:timing": "PROOF", "outcome:": "L45R",
                            # M6 decision (operator, 2026-11-14, "Adopt"): the four items every P2 RECOVERY landing
                            # produces in accepted code, exactly these, on this row only, replaced by the
                            # RECOVERY_LANDING proof (plan §16.6): the designed exit 1, the COMPLETE-only
                            # window_end absence snapshot, the replay's post-touchdown reference rejection and its
                            # missing P2 report.
                            **{"=" + item: "RECOVERY_LANDING" for item in response.RECOVERY_LANDING_ITEMS}}},
    "C-1": {"response": ["STALE_IFF_AGE", "RECOVERY", "GOAL", "UNSAFE"],
            "exceptions": {"depth:": "PROOF", "performance:": "STALE_IFF_AGE", "outcome:detour_sequence": "RECOVERY"}},
    "C-2": {"response": ["LIMITS_UNINJECTED", "GOAL", "UNSAFE"], "exceptions": {**POLICY_EXCEPTIONS, **TIMING_EXCEPTIONS}},
    # C-3 (M6 decision, operator 2026-11-14, "Adopt both"): S0 is required and the outcome reported. When the accepted P2
    # controller fails closed during the adversarial flight and lands through RECOVERY, that landing is the reported
    # outcome: its runtime status, P2 timing and the four RECOVERY-landing items are replaced by FAIL_CLOSED_LANDING
    # (L45R for the reported P2 failure plus the RECOVERY_LANDING proof), which is recorded only for such a landing.
    "C-3": {"response": ["UNSAFE", "FAIL_CLOSED_LANDING"],
            "exceptions": {"policy:ledger:argmax_differs": "PROOF", "outcome:": "UNSAFE",
                           "runtime_status:Abort:controller_failed:": "FAIL_CLOSED_LANDING", "=p2:timing": "FAIL_CLOSED_LANDING",
                           **{"=" + item: "FAIL_CLOSED_LANDING" for item in response.RECOVERY_LANDING_ITEMS}}},
    "C-4": {"response": ["NO_INVOKE_AFTER_STALE", "L45", "UNSAFE", "LIMITS_UNINJECTED"],  # M6: its timing exceptions' check
            "exceptions": {**DEPTH_WINDOW, **POLICY_EXCEPTIONS, **TIMING_EXCEPTIONS, "outcome:": "L45"}},
    # C-5 (M6 decision, operator 2026-11-14, "Adopt as recommended", plan §16.10-16.11): the load starts with the map
    # loss. After H35 the row lands through COMPLETE (L45) or, on the contract §5.3 dispatch response, through RECOVERY
    # (FAIL_CLOSED_LANDING); LANDING passes on the path taken. LOAD implements §5.3 and alone allows the P5 inference
    # statistics. LOAD_RECORDER replaces exactly the accepted P3 raw-recorder overflow's three items, when every lost
    # frame is inside the load window and perception kept every period; and (extension, operator 2026-11-14, "Adopt
    # the extension", plan §16.12-16.13) the two P3 delivery items an overflow inside the sensor window produces, when
    # each frame they count as undelivered is matched to an overflowed receipt.
    "C-5": {"response": ["H35", "STALE_IFF_AGE", "LOAD", "LOAD_RECORDER", "FAIL_CLOSED_LANDING", "LANDING", "UNSAFE"],
            "exceptions": {"performance:": "STALE_IFF_AGE", "outcome:": "LANDING",
                           "=policy:inference_p99": "LOAD", "=policy:inference_max": "LOAD",
                           "=runtime_status:Abort:controller_failed:stale_action": "FAIL_CLOSED_LANDING",
                           "=p2:timing": "FAIL_CLOSED_LANDING",
                           **{"=" + item: "FAIL_CLOSED_LANDING" for item in response.RECOVERY_LANDING_ITEMS},
                           **{"=" + item: "LOAD_RECORDER" for item in response.LOAD_RECORDER_ITEMS}}},
}
# Gate A ground fixtures: the accepted stale rule replayed on the recorded ground inputs, or a proven refusal.
GROUND_EXCEPTIONS = {"depth:": "GROUND_REPLAY", "performance:": "GROUND_REPLAY", "fault_injection_outside_s6": "PROOF"}
FAIL_CLOSED = {"runtime_status:": "PROOF", "policy_fault:refusal_reason": "PROOF"}
GROUND = {
    "A-5.F1.01": ["snapshot_stale"], "A-5.F1.02": ["adapter_health_missing"], "A-5.F1.04": ["snapshot_stale"],
    "A-5.F1.06": ["snapshot_stale", "unusable_frames"], "A-5.F1.07": ["snapshot_stale"],
    "A-5.F1.11": ["adapter_health_not_fresh", "calibration_mismatch", "snapshot_stale"],
    "A-5.F2.09": ["snapshot_stale", "unusable_frames"]}  # A-5.F4.02/F4.06 removed (M4 decision): not provable on the ground
for _row, _reasons in GROUND.items():
    ROWS[_row] = {"response": ["GROUND_REPLAY:" + ",".join(_reasons)], "exceptions": GROUND_EXCEPTIONS}
for _row in ("A-5.F3.01", "A-5.F3.02", "A-5.F3.03", "A-5.F4.07", "A-5.F4.13"):
    ROWS[_row] = {"response": [], "exceptions": FAIL_CLOSED}
# The P5 policy_fault checks describe P5's own fixture (a mismatched checkpoint copy); for the absent checkpoint
# and the changed schema they cannot hold, and the P6 refusal proof (declared reason, no node started) replaces them.
# F4.07/F4.13 (operator decision, 2026-11-07, "Allow that one item"): the accepted controller fails closed before its
# connectivity check passes, so it exits 1 by design, which the unchanged P3 classifier reports as this one item (the
# accepted P5 evaluator treats the same item as its own refusal fixture's designed exit). Exactly this item, for these
# two rows only, is replaced by the fail-closed proof; every other cleanup item stays never excepted. At the later
# `ground_prestream` trigger connectivity has passed, so the designed exit is 0 and this item does not arise.
# Operator decision, 2026-11-07, "Adopt both": the accepted P2 timing check reports failed whenever the controller's
# result carries a failure (`timing_evidence.trace_metrics`), so exactly `p2:timing`, for these two rows only, is
# replaced by the fail-closed proof, which then also requires the timing reconciliation to have completed.
FAIL_CLOSED_EXIT = "cleanup:ros-controller:nonzero_exit_1"
FAIL_CLOSED_TIMING = "p2:timing"
FAIL_CLOSED_EXIT_ROWS = ("A-5.F4.07", "A-5.F4.13")
for _row in FAIL_CLOSED_EXIT_ROWS:
    ROWS[_row] = {"response": [], "exceptions": {**FAIL_CLOSED, "=" + FAIL_CLOSED_EXIT: "PROOF",
                                                 "=" + FAIL_CLOSED_TIMING: "PROOF"}}
for _row in ("A-5.F3.01", "A-5.F3.03"):
    ROWS[_row] = {"response": [], "exceptions": {**FAIL_CLOSED, "policy_fault:not_a_checkpoint_mismatch": "PROOF",
                                                 "policy_fault:mismatched_checkpoint_record": "PROOF"}}
ROWS["A-1"] = {"response": [], "exceptions": {}}  # P3 readiness unchanged: the P3 evaluation, no exception
ROWS["A-3"] = {"response": ["GROUND_POLICY", "RELAY"], "exceptions": {}}  # every contract 3 limit, nothing excepted


def codes():
    sys.path.insert(0, str(HERE.parent))
    from injection import specs
    return specs.codes()


def p5_limits():
    return json.loads((ROOT / "simulation/px4_gazebo_p5/configs/p5_validation.yaml").read_text(encoding="utf-8"))["policy"]


def dispatch_budget():
    """The accepted P2 dispatch budget (the run-local copy equals it, rule C11)."""
    return json.loads((ROOT / "simulation/px4_gazebo/configs/p2_timing.yaml").read_text(encoding="utf-8"))["dispatch_budget_wall_s"]


def explained(run, rules):
    """Exact items another passing proof of the same row explains, so the RECOVERY_LANDING proof does not count them as
    unexplained (C-5: the recorder overflow inside the load window, LOAD_RECORDER)."""
    if "LOAD_RECORDER" in rules["response"]:
        proof = response.load_recorder(run)
        if proof is not None and proof["passed"]:
            return response.LOAD_RECORDER_ITEMS
    return ()


def run_fault_window(run):
    """The unchanged P4 §9.1 window function on the run's `p4/fault_injection.json` (bridge-stop rows).
    Any error is kept as a failure item, so a proof that reads the window cannot pass on missing evidence."""
    import p3_evaluate
    import p4_evaluate as p4e
    f = response._Failures()
    validation_p3, sensors = p3_evaluate.configs()
    v4 = json.loads((ROOT / "simulation/px4_gazebo_p4/configs/p4_validation.yaml").read_text(encoding="utf-8"))
    try:
        out = p4e.fault_window(run.path, run.summary, validation_p3, sensors, v4["outcomes"], f) or {}
    except Exception as exc:  # noqa: BLE001  (recorded; never a silent pass)
        out, f.items = {}, f.items + ["fault_window_error:" + type(exc).__name__ + ":" + str(exc)[:200]]
    out["failures"] = f.items
    return out


def scenarios():
    return json.loads((HERE.parent / "configs/p6_scenarios.yaml").read_text(encoding="utf-8"))


def p4_window_applies(run):
    """The accepted P4 §9.1 window is the bridge-stop window. An adapter stop (F1.02) writes the same P4 record but
    has its own proof, and must not be judged by the bridge's F1/F2 items (M5 finding)."""
    injected = run.optional("p4/fault_injection.json") or {}
    return injected.get("process") == "sensor-bridge"


def respond(run, declaration, rules, proof, window):
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
        elif code == "RECOVERY_LANDING":
            response.recovery_landing(run, c)
        elif code == "FAIL_CLOSED_LANDING":
            response.fail_closed_landing(run, c, explained(run, rules))
        elif code == "LANDING":
            response.landing(run, c, explained(run, rules))
        elif code == "LOAD":
            response.load(run, c, p5_limits(), dispatch_budget())
        elif code == "LOAD_RECORDER":
            response.load_recorder_check(run, c)
        elif code == "GROUND_REPLAY":
            config = json.loads(run.file("p4_avoidance.yaml").read_text(encoding="utf-8"))
            response.ground_replay(run, c, arg.split(","), config)
            if run.exists("p4/fault_injection.json"):  # cross-check against the unchanged P4 ground replay
                import p4_evaluate as p4e
                f = response._Failures()
                accepted = p4e.fault_replay(run.path, run.summary, config, f).get("replayed_hover") or {}
                ours = c.out.get("GROUND_REPLAY", {})
                c.record("GROUND_REPLAY_P4", not f.items and accepted.get("reason") == ours.get("reason"),
                         accepted=accepted, items=f.items)
        elif code == "GROUND_POLICY":
            integration.ground_policy(run, c)
        elif code == "RELAY":
            integration.relay(run, c)
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


def _matches(item, key):
    """An exception key is a prefix, or with a leading '=' one exact item."""
    return item == key[1:] if key.startswith("=") else item.startswith(key)


def compose(p5_failures, exceptions, checks):
    """Every P5-evaluator failure is either excepted (with its replacement check passed) or a P6 failure."""
    allowed, refused = [], []
    for item in p5_failures:
        rule = next((replacement for key, replacement in exceptions.items() if _matches(item, key)), None)
        replacement = checks.get(rule.split(":")[0]) if rule else None
        if rule and replacement and replacement["passed"]:
            allowed.append({"item": item, "replacement": rule})
        else:
            refused.append(item)
    return allowed, refused


def evaluate(run_path, declaration=None, strict=True, integration_checks=True):
    """`strict=False` and `integration_checks=False` exist only for tests that evaluate accepted P5 runs as
    stand-ins for P6 rows; a real P6 evaluation is always strict, with the integration checks."""
    if strict and not integration_checks:
        raise ValueError("a strict P6 evaluation always applies the integration checks")
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
    window = run_fault_window(run) if p4_window_applies(run) else None
    proof = proof_module.prove(run, declaration, window)
    checks = respond(run, declaration, rules, proof, window)
    if integration_checks:
        integration.injectors(run, declaration, checks)  # Gate A row 6, on every run
        integration.c11(run, declaration, checks, scenarios()["run_local_config_rule"])  # rule C11, on every run
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
