"""P6 M4: the independent evaluators, offline. No simulator.

* Retained accepted P5 runs (skip where absent): the real Gate S S6 bridge stop evaluated as an S-1-type
  fault, and the clean Gate C S2 flight as the B-0 pass-through baseline.
* Negative cases: a fault declared but not observed fails; a declaration that is not the frozen row fails;
  a tampered ledger fails; a P5 failure outside the row's exceptions fails; target motion while holding fails.
* Rig flights written in the P6 run layout exercise the policy and map rows end to end.
"""
import copy
import json
import os
from pathlib import Path
import shutil
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
import test_px4_gazebo_p5_mission as m5  # noqa: E402
import test_px4_gazebo_p6_injection as m2  # noqa: E402

REPO = p5.syn.REPO
for path in (REPO / "simulation/px4_gazebo_p6", REPO / "simulation/px4_gazebo_p6/validation",
             REPO / "simulation/px4_gazebo_p4/validation", REPO / "simulation/px4_gazebo/validation",
             REPO / "simulation/px4_gazebo_p5/validation"):
    sys.path.insert(0, str(path))
import p6_coverage_report  # noqa: E402
import p6_evaluate  # noqa: E402
from p6_run import Run  # noqa: E402
from p6_safety_evaluate import Checks  # noqa: E402
from injection import declaration, fault_ledger  # noqa: E402

RUNS = Path(os.environ.get("GWM_RUNS", Path.home() / "uav_autonomy/runs"))
S6 = RUNS / "20261106T131246Z-p5-flight-S6-d907451a"
S2 = RUNS / "20261106T132442Z-p5-flight-S2-184a5a5e"
needs_runs = pytest.mark.skipif(not (S6 / "p5-evaluation.json").exists() or not (S2 / "p5-evaluation.json").exists(),
                                reason="retained accepted P5 runs not available")


def bridge_stop_declaration():
    """The S-1 fault as the accepted S6 run injected it (P4 trigger: NORMAL at 2.0 m), for non-strict evaluation."""
    d = declaration.build("S-1")
    return dict(d, trigger="normal_2m")


# ---- retained accepted runs ------------------------------------------------------------------------------
@needs_runs
def test_the_accepted_s6_bridge_stop_is_proved_and_its_response_passes():
    out = p6_evaluate.evaluate(S6, bridge_stop_declaration(), strict=False, integration_checks=False)  # a P5 stand-in
    assert out["status"] == "passed", out["failures"]
    assert out["proof"]["proved"] and out["response"]["FAULT_WINDOW"]["passed"] and out["response"]["UNSAFE"]["passed"]
    assert out["fault_window"]["native_headers_after_injection"] > 0 and not out["p5_failures_refused"]
    t = out["timing"]
    assert t["trigger_sim_s"] == out["fault_window"]["t_inj_sim_s"] and 0 < t["detection_latency_sim_s"] <= 0.5


@needs_runs
def test_a_declaration_that_is_not_the_frozen_row_is_refused():
    out = p6_evaluate.evaluate(S6, bridge_stop_declaration(), strict=True)  # trigger differs from the frozen S-1 row
    assert out["status"] == "failed" and out["failures"][0].startswith("declaration_not_frozen_row")


@needs_runs
def test_the_clean_accepted_flight_passes_as_the_pass_through_baseline():
    # A P5 flight stands in for B-0: it has no P6 relays, so the M5 integration checks are not applied to it.
    out = p6_evaluate.evaluate(S2, declaration.build("B-0"), strict=False, integration_checks=False)
    assert out["status"] == "passed", out["failures"]
    assert out["proof"]["family"] == "pass_through" and out["response"]["GOAL"]["passed"]


@needs_runs
@pytest.mark.parametrize("row", ["S-1", "B-1", "S-4"])
def test_a_fault_declared_but_not_observed_fails(row):
    d = bridge_stop_declaration() if row == "S-1" else declaration.build(row)
    out = p6_evaluate.evaluate(S2, d, strict=False, integration_checks=False)
    assert out["status"] == "failed" and not out["proof"]["proved"]


# ---- composition rules ---------------------------------------------------------------------------------
def test_an_excepted_p5_failure_needs_its_replacement_check_to_pass():
    exceptions = {"policy:failure_ticks": "PH3", "outcome:": "L45"}
    checks = {"PH3": {"passed": True}, "L45": {"passed": False}}
    allowed, refused = p6_evaluate.compose(["policy:failure_ticks", "outcome:not_completed", "safety:collisions"],
                                           exceptions, checks)
    assert [a["item"] for a in allowed] == ["policy:failure_ticks"]
    assert refused == ["outcome:not_completed", "safety:collisions"]


def test_no_row_ever_excepts_safety_policy_a_ownership_or_cleanup():
    """The exceptions: the designed fail-closed exit of F4.07/F4.13 (M5 decision), the four RECOVERY-landing items of
    S-5', C-3 and C-5 (M6 decisions) and C-5's recorder-overflow items (M6 decision, plan §16.11), each an exact item
    replaced by its proof; nothing else touches these families."""
    never = ("safety:", "native", "competing_flight_publisher", "residual_namespace_processes", "cleanup:", "p2:final_landed")
    designed = "=" + p6_evaluate.FAIL_CLOSED_EXIT
    landing = {"S-5'": "RECOVERY_LANDING", "C-3": "FAIL_CLOSED_LANDING", "C-5": "FAIL_CLOSED_LANDING"}
    for row, rules in p6_evaluate.ROWS.items():
        for prefix in rules["exceptions"]:
            if prefix == designed:
                assert row in ("A-5.F4.07", "A-5.F4.13", "S-5'", "C-3", "C-5"), row
                assert rules["exceptions"][prefix] in ("PROOF", "RECOVERY_LANDING", "FAIL_CLOSED_LANDING")
                continue
            if row in landing and prefix.startswith("=") and prefix[1:] in p6_evaluate.response.RECOVERY_LANDING_ITEMS:
                assert rules["exceptions"][prefix] == landing[row]
                continue
            if row == "C-5" and prefix.startswith("=") and prefix[1:] in p6_evaluate.response.LOAD_RECORDER_ITEMS:
                assert rules["exceptions"][prefix] == "LOAD_RECORDER"
                continue
            assert not any(prefix.startswith(n) or n.startswith(prefix) for n in never if prefix), (row, prefix)
    allowed, refused = p6_evaluate.compose(
        ["cleanup:ros-controller:nonzero_exit_1", "cleanup:ros-controller:nonzero_exit_10", "cleanup:p6-witness:forced_kill_after_grace"],
        p6_evaluate.ROWS["A-5.F4.07"]["exceptions"], {"PROOF": {"passed": True}})
    assert [a["item"] for a in allowed] == ["cleanup:ros-controller:nonzero_exit_1"]
    assert refused == ["cleanup:ros-controller:nonzero_exit_10", "cleanup:p6-witness:forced_kill_after_grace"]
    assert p6_evaluate.compose(["cleanup:ros-controller:nonzero_exit_1"], p6_evaluate.ROWS["A-5.F1.01"]["exceptions"],
                               {"PROOF": {"passed": True}})[1] == ["cleanup:ros-controller:nonzero_exit_1"]


def test_every_gated_flight_row_has_rules():
    from injection import specs
    rows = [r["id"] for gate in ("B", "S", "C") for r in specs.strict_json(specs.SCENARIOS)["gates"][gate]]
    assert set(rows) <= set(p6_evaluate.ROWS)


# ---- rig flights in the P6 run layout ---------------------------------------------------------------------
def run_local_configs(run, decl):
    """The run-local configurations as the P6 runner writes them (rule C11): frozen copies, the accepted
    runners' own keys, and for the depth relay the two declared bridge topic names."""
    import p6_integration_evaluate as integration
    for name, folder in integration.CONFIGS.items():
        shutil.copyfile(folder / "configs" / name, run / name)
    for name, changes in (("p4_avoidance.yaml", {"enabled": True}), ("p5_policy.yaml", {"enabled": True})):
        value = json.loads((run / name).read_text(encoding="utf-8"))
        value.update(changes)
        (run / name).write_text(json.dumps(value, indent=2) + "\n")
    if "DR" in decl["mechanisms"]:
        text = (run / "p3_sensor_bridge.yaml").read_text(encoding="utf-8")
        for accepted, raw in integration._depth_topics():
            text = text.replace("- ros_topic_name: %s\n" % accepted, "- ros_topic_name: %s\n" % raw)
        (run / "p3_sensor_bridge.yaml").write_text(text)


def write_run(tmp, m, decl, ledger_path, p5_failures, snapshots=(), complete=True):
    """A P6 run directory from a rig flight. Only the controller-side records exist (no PX4, truth or ULog):
    the landing is synthesized as accepted and COMPLETE so the landing check can be exercised."""
    run = tmp / "run"
    (run / "p5").mkdir(parents=True)
    (run / "p4").mkdir()
    (run / "p6").mkdir()
    events = [e for e in m.events if e["event"] in ("p4_decision", "p4_supervisor_abort", "transition")]
    abort = next((e for e in m.events if e["event"] == "p4_supervisor_abort"), None)
    if abort is not None:
        events.append({"event": "command_sent", "ros_sim_s": abort["sim_s"], "fields": {"command": 21}})
    (run / "ros-events.jsonl").write_text("".join(json.dumps(e, default=str) + "\n" for e in events))
    (run / "p5/policy-ledger.jsonl").write_text("".join(json.dumps(r, default=str) + "\n" for r in m.ledger))
    (run / "p4/local_map.jsonl").write_text("".join(json.dumps(s) + "\n" for s in snapshots))
    summary = m.summary()
    result = {"p4": summary, "p4_outcome": "supervisor_abort" if m.p4_abort else "completed",
              "final_state": "COMPLETE" if complete else m.state, "failure": None,
              "transactions": [{"command": 21, "status": "accepted", "sent_monotonic_s": 100.0}] if m.p4_abort else [],
              "transitions": [{"to": "COMPLETE", "monotonic_s": 110.0}] if complete else []}
    (run / "summary.json").write_text(json.dumps({"controller_result": result, "config": {"recovery_wall_s": 45.0}}))
    (run / "p5-evaluation.json").write_text(json.dumps({"failures": p5_failures}))
    (run / "p6/fault-declaration.json").write_text(json.dumps(decl))
    shutil.copy(ledger_path, run / "p6/fault-ledger-controller_shim.jsonl")
    run_local_configs(run, decl)
    return run


@pytest.fixture(scope="module")
def b1_flight(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("b1")
    ledger = fault_ledger.FaultLedger(tmp / "shim.jsonl", "B-1", "F3.08", "controller_shim")
    m, _, _, _ = m2.fly(m2.S2_BOXES, "B-1", ledger=ledger)
    ledger.close()
    return m, tmp / "shim.jsonl"


@m5.needs_artifact
def test_policy_timeout_row_passes_with_its_declared_exceptions(b1_flight, tmp_path):
    m, ledger = b1_flight
    failures = ["policy:failure_ticks", "policy:participation", "policy:inference_max", "outcome:not_completed"]
    run = write_run(tmp_path, m, declaration.build("B-1"), ledger, failures)
    out = p6_evaluate.evaluate(run, strict=True)
    assert out["status"] == "passed", (out["failures"], out["response"])
    assert {a["item"] for a in out["p5_failures_allowed"]} == set(failures)
    assert out["response"]["PH3"]["passed"] and out["timing"]["fault_to_safe_action_sim_s"] is not None


@m5.needs_artifact
def test_policy_timeout_row_fails_on_tampering_or_an_unexcepted_failure(b1_flight, tmp_path):
    m, ledger = b1_flight
    tampered = copy.deepcopy(m)
    first = next(i for i, r in enumerate(tampered.ledger) if r["failure"] is not None)
    tampered.ledger[first] = dict(tampered.ledger[first], failure=None)  # one failure "never happened"
    out = p6_evaluate.evaluate(write_run(tmp_path / "a", tampered, declaration.build("B-1"), ledger, []), strict=True)
    assert out["status"] == "failed" and not out["proof"]["proved"]
    out = p6_evaluate.evaluate(write_run(tmp_path / "b", m, declaration.build("B-1"), ledger, ["safety:collisions"]), strict=True)
    assert out["status"] == "failed" and out["p5_failures_refused"] == ["safety:collisions"]
    moved = copy.deepcopy(m)
    failing = [r["sim_s"] for r in moved.ledger if r["failure"] is not None]
    for e in moved.events:
        if e["event"] == "p4_decision" and e["sim_s"] == failing[1]:
            e["target_enu"] = [e["target_enu"][0] + 0.2, e["target_enu"][1]]  # target moved on a held tick
    out = p6_evaluate.evaluate(write_run(tmp_path / "c", moved, declaration.build("B-1"), ledger, []), strict=True)
    assert out["status"] == "failed" and not out["response"]["UNSAFE"]["passed"]


@m5.needs_artifact
def test_map_loss_row_is_proved_from_perceptions_own_record_and_recovers(tmp_path):
    published = []

    def flight(row, mode):
        ledger_path = tmp_path / (row + mode + ".jsonl")
        ledger = fault_ledger.FaultLedger(ledger_path, row, "F4.01", "controller_shim")
        from injection import controller_shim
        clock = m2.Clock()
        policy = m5.trained_policy()
        from gwm_policy_control.mission import P5Mission
        boxes = [{"center": [4.0, 0.5], "size": [0.4, 0.4]}]
        shim = controller_shim.Shim(declaration.build(row, mode), ledger, sleep=clock.sleep, boxes=boxes).install()
        try:
            rig = m5.Rig(lambda cache: P5Mission(m5.P2, m5.P4C, m5.P5C, policy, p5.SCHEMA, cache, 0.0, True, clock=clock), boxes)
            original = rig.m.on_snapshot

            def record(snapshot):  # perception's output, before the controller's input wrapper
                published.append({"usable": True, "acquisition_sim_s": snapshot["acquisition_sim_s"],
                                  "processing_sim_s": snapshot["acquisition_sim_s"]})
                original(snapshot)
            rig.m.on_snapshot = record
            while rig.sim < 260 and rig.m.state in ("MAP_SCAN", "TRANSIT"):
                rig.step()
        finally:
            shim.uninstall()
            ledger.close()
        return rig.m, ledger_path

    m, ledger = flight("S-4", "active")
    run = write_run(tmp_path / "s4", m, declaration.build("S-4"), ledger, ["outcome:detour_sequence"], snapshots=published)
    out = p6_evaluate.evaluate(run, strict=True)
    assert out["status"] == "passed", (out["failures"], out["proof"], out["response"])
    assert out["proof"]["evidence"]["published_in_window"] > 0 and out["timing"]["recovered"]
    published.clear()
    m, ledger = flight("S-4", "pass-through")  # nothing dropped: the declared fault did not happen
    run = write_run(tmp_path / "pt", m, declaration.build("S-4"), ledger, [], snapshots=published)
    out = p6_evaluate.evaluate(run, strict=False)
    assert out["status"] == "failed" and not out["proof"]["proved"]


@m5.needs_artifact
def test_coverage_report_aggregates_evaluations(b1_flight, tmp_path):
    m, ledger = b1_flight
    run = write_run(tmp_path, m, declaration.build("B-1"), ledger, ["policy:failure_ticks", "outcome:not_completed"])
    out = p6_evaluate.evaluate(run, strict=True)
    (run / "p6/p6-evaluation.json").write_text(json.dumps(out, default=str))
    report = p6_coverage_report.report(p6_coverage_report.load([run]))
    assert report["passed_rows"] == ["B-1"] and "F3.08" not in report["required_faults_without_a_passing_run"]
    assert report["detection_latency_sim_s"]["n"] == 1


# ---- witness and accepted-record proofs on synthetic run directories ------------------------------------
def synthetic_run(tmp, decl, injector=None, act_sim=10.0, files=None):
    run = tmp / "run"
    (run / "p6/witness").mkdir(parents=True)
    (run / "p6/fault-declaration.json").write_text(json.dumps(decl))
    if injector:
        (run / ("p6/fault-ledger-%s.jsonl" % injector)).write_text(
            json.dumps({"kind": "declared", "seq": 1}) + "\n" + json.dumps({"kind": "activated", "seq": 2, "sim_s": act_sim}) + "\n")
    for name, rows in (files or {}).items():
        target = run / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if name.endswith(".jsonl"):
            target.write_text("".join(json.dumps(r) + "\n" for r in rows))
        else:
            target.write_text(json.dumps(rows))
    return p6_evaluate.Run(run)


def frames(t0, t1, dt=0.033):
    out, t = [], t0
    while t < t1:
        out.append({"stamp_ns": int(round(t * 1e9)), "receipt_sim_s": t + 0.016, "crc32": 1})
        t += dt
    return out


def test_depth_burst_proof_needs_drops_inside_and_only_inside_the_windows(tmp_path):
    from p6_fault_evaluate import prove
    from injection.schedule import Windows
    d = declaration.build("C-1")
    raw = frames(5.0, 25.0)
    w = Windows(d["parameters"]["burst_start_offsets_sim_s"], d["parameters"]["burst_durations_sim_s"])
    relayed = [f for f in raw if not w.active(f["receipt_sim_s"] - 10.0)]
    window = {"sensor_window_sim_s": [5.0, 25.0]}  # M6: the proof judges the accepted sensor window (plan §16.9)
    run = synthetic_run(tmp_path / "ok", d, "depth_relay", files={"p6/witness/raw_image.jsonl": raw,
                                                                    "p6/witness/relayed_image.jsonl": relayed, "summary.json": window})
    assert prove(run, d)["proved"]
    run = synthetic_run(tmp_path / "none", d, "depth_relay", files={"p6/witness/raw_image.jsonl": raw,
                                                                      "p6/witness/relayed_image.jsonl": raw, "summary.json": window})
    assert not prove(run, d)["proved"]  # the ledger says activated, the witness shows nothing dropped


def test_depth_delay_and_adapter_counter_proofs(tmp_path):
    from p6_fault_evaluate import prove
    d = declaration.build("A-5.F1.04")
    raw = frames(5.0, 15.0)
    relayed = [dict(f, receipt_sim_s=f["receipt_sim_s"] + (0.30 if f["receipt_sim_s"] > 10.05 else 0.0)) for f in raw]
    run = synthetic_run(tmp_path / "delay", d, "depth_relay", files={"p6/witness/raw_image.jsonl": raw,
                                                                       "p6/witness/relayed_image.jsonl": relayed})
    assert prove(run, d)["proved"]
    d = declaration.build("A-5.F1.06")
    counts = {"duplicate_acquisition_stamp": 5, "acquisition_stamp_backwards": 5, "missing_acquisition_stamp": 5}
    run = synthetic_run(tmp_path / "c1", d, files={"sensors/adapter-result.json": {"accounting": {"rejected": counts}}})
    assert prove(run, d)["proved"]
    run = synthetic_run(tmp_path / "c2", d, files={"sensors/adapter-result.json": {"accounting": {"rejected": dict(counts, missing_acquisition_stamp=4)}}})
    assert not prove(run, d)["proved"]


def test_pose_to_perception_proof_needs_true_state_to_continue_and_relayed_to_stop(tmp_path):
    from p6_fault_evaluate import prove
    d = declaration.build("S-2'")
    true = [{"receipt_sim_s": 9.0 + 0.01 * k} for k in range(300)]
    relayed = [r for r in true if r["receipt_sim_s"] <= 10.0]
    files = {"p6/witness/true_position.jsonl": true, "p6/witness/relayed_position.jsonl": relayed,
             "p6/witness/true_attitude.jsonl": true, "p6/witness/relayed_attitude.jsonl": relayed,
             "p4/local_map.jsonl": [{"processing_sim_s": 10.5, "usable": False, "reason": "association:position:pending_newer_state"}]}
    assert prove(synthetic_run(tmp_path / "ok", d, "state_relay", files=files), d)["proved"]
    files["p6/witness/true_position.jsonl"] = relayed  # PX4 itself went quiet: not the declared fault
    assert not prove(synthetic_run(tmp_path / "bad", d, "state_relay", files=files), d)["proved"]


def test_controller_input_faults_cannot_be_proved_on_the_ground(tmp_path):
    """Why A-5.F4.02/F4.06 left Gate A (M4 decision): with no decisions logged, the proof refuses, never trusts the shim."""
    from p6_fault_evaluate import prove
    for fault, params in (("F4.02", {"delay_s": 0.30, "window_sim_s": 5.0}), ("F4.06", {"window_sim_s": 5.0})):
        d = {"row": "ground-" + fault, "fault": fault, "mechanisms": ["MR"], "mode": "active", "parameters": params}
        out = prove(synthetic_run(tmp_path / fault, d, "controller_shim"), d)
        assert not out["proved"] and "proof:no_independent_ground_evidence_for_a_controller_input_fault" in out["failures"]
    from injection import specs
    rows = [r["id"] for r in specs.strict_json(specs.SCENARIOS)["gates"]["A"]]
    assert "A-5.F4.02" not in rows and "A-5.F4.06" not in rows


# ---- M6 decision: the S-5' RECOVERY-landing items (operator, 2026-11-14, "Adopt") ----------------------------
def recovery_landing_run(tmp, p5_failures=None, exit_code=1, outcome="failed", rejected_count=0, in_air_false_at=140.6,
                         landed_at=140.4, coverage=(100.0, 141.0), gap=0.5, absence_failures=("snapshot_missing:window_end",)):
    tmp.mkdir(parents=True, exist_ok=True)
    (tmp / "p6").mkdir(exist_ok=True)
    land_wall = 135.0
    items = list(p6_evaluate.response.RECOVERY_LANDING_ITEMS) if p5_failures is None else list(p5_failures)
    result = {"final_state": "RECOVERY", "failure": "stale_action", "p4_outcome": outcome, "connectivity": "passed",
              "transactions": [{"command": 21, "sent_monotonic_s": land_wall}],
              "reference": {"rejected_count": rejected_count, "rejection": None}}
    (tmp / "summary.json").write_text(json.dumps({"controller_exit_code": exit_code, "controller_result": result}))
    events = [{"event": "received", "topic_key": "vehicle_land_detected", "monotonic_s": landed_at, "fields": {"landed": True}},
              {"event": "received", "topic_key": "estimator_status_flags", "monotonic_s": in_air_false_at, "fields": {"cs_in_air": False}}]
    (tmp / "ros-events.jsonl").write_text("".join(json.dumps(e) + chr(10) for e in events))
    (tmp / "p5-evaluation.json").write_text(json.dumps({"failures": items, "rgb_absence": {
        "failures": list(absence_failures), "window_monotonic_s": [101.0, 140.9], "rgb_metadata_rows": 0,
        "monitor": {"coverage_monotonic_s": list(coverage), "max_sample_gap_wall_s": gap}}}))
    return Run(tmp)


def recovery_landing(run):
    c = Checks({})
    p6_evaluate.response.recovery_landing(run, c)
    return c.out["RECOVERY_LANDING"]


def test_the_recovery_landing_proof_accepts_the_s5_shape_and_refuses_each_missing_piece(tmp_path):
    ok = recovery_landing(recovery_landing_run(tmp_path / "ok"))
    assert ok["passed"], ok
    bad = {
        "exit0": dict(exit_code=0), "outcome": dict(outcome="supervisor_abort"), "rejected": dict(rejected_count=1),
        "in_air_before_landing": dict(in_air_false_at=140.3), "coverage_short": dict(coverage=(100.0, 140.5)),
        "gap": dict(gap=1.5), "other_absence": dict(absence_failures=("snapshot_missing:window_end", "unexpected_subscriber")),
        "other_item": dict(p5_failures=list(p6_evaluate.response.RECOVERY_LANDING_ITEMS) + ["cleanup:p6-witness:forced_kill_after_grace"]),
        "no_replay_item": dict(p5_failures=[x for x in p6_evaluate.response.RECOVERY_LANDING_ITEMS if "final_alignment" not in x]),
    }
    for name, kw in bad.items():
        assert not recovery_landing(recovery_landing_run(tmp_path / name, **kw))["passed"], name


def test_s5_allows_exactly_the_four_recovery_landing_items_and_only_with_the_proof():
    rules = p6_evaluate.ROWS["S-5'"]["exceptions"]
    items = list(p6_evaluate.response.RECOVERY_LANDING_ITEMS)
    extra = ["cleanup:ros-controller:nonzero_exit_10", "rgb_absence:unexpected_subscriber", "p2_integrity_error:ValueError:other"]
    allowed, refused = p6_evaluate.compose(items + extra, rules, {"RECOVERY_LANDING": {"passed": True}})
    assert [a["item"] for a in allowed] == items and refused == extra
    assert p6_evaluate.compose(items, rules, {"RECOVERY_LANDING": {"passed": False}})[1] == items
    for row in ("S-1", "B-1", "C-4"):
        assert p6_evaluate.compose(items, p6_evaluate.ROWS[row]["exceptions"],
                                   {"RECOVERY_LANDING": {"passed": True}, "PROOF": {"passed": True}})[1] == items, row
    # A-5.F4.13 allows only its own designed exit item (M5 decision), never the three RECOVERY-landing items
    refused = p6_evaluate.compose(items, p6_evaluate.ROWS["A-5.F4.13"]["exceptions"],
                                  {"RECOVERY_LANDING": {"passed": True}, "PROOF": {"passed": True}})[1]
    assert refused == [x for x in items if x != p6_evaluate.FAIL_CLOSED_EXIT]


def test_the_retained_s5_dry_run_passes_under_the_adopted_rule():
    """The 2026-11-14 S-5' dry run on development candidate 33f33c5a (retained WSL evidence): with the stored P5
    evaluation unchanged, the P6 evaluation now passes with exactly the four items allowed by RECOVERY_LANDING."""
    import glob
    runs = glob.glob(str(Path.home() / "uav_autonomy/runs/20261114T034747Z-p6dry-S-5p-*"))
    if not runs or not (Path(runs[0]) / "p5-evaluation.json").exists():
        pytest.skip("retained S-5' evidence not present")
    out = p6_evaluate.evaluate(Path(runs[0]))
    assert out["status"] == "passed", out["failures"]
    assert sorted(a["item"] for a in out["p5_failures_allowed"] if a["replacement"] == "RECOVERY_LANDING") == sorted(p6_evaluate.response.RECOVERY_LANDING_ITEMS)
    assert out["p5_failures_refused"] == []


# ---- M6 Gate C fixes (plan §16.8-16.9) ------------------------------------------------------------------------
ALWAYS_RECORDED = {"PROOF", "C11", "INJECTORS"}


def test_every_exception_names_a_check_its_row_records():
    """23d513ee C-4: its timing exceptions named LIMITS_UNINJECTED, which its response list never ran, so they could
    never apply. Every replacement check must be recorded for its row: a response code, an always-recorded check, or
    FAULT_WINDOW_F1F2 on a bridge-stop row (recorded whenever the accepted P4 window applies)."""
    from injection import declaration
    for row, rules in p6_evaluate.ROWS.items():
        recorded = {code.split(":")[0] for code in rules["response"]} | ALWAYS_RECORDED
        if "GROUND_REPLAY" in recorded:
            recorded.add("GROUND_REPLAY_P4")
        try:
            d = declaration.build(row)
        except Exception:  # noqa: BLE001  (a rule key that is not a frozen row)
            d = None
        if d is not None and d["parameters"].get("process") == "sensor-bridge":
            recorded.add("FAULT_WINDOW_F1F2")
        needed = {value.split(":")[0] for value in rules["exceptions"].values()}
        assert needed <= recorded, (row, sorted(needed - recorded))


def test_c3_reports_a_p2_fail_closed_landing_and_allows_its_items_only_with_the_check(tmp_path):
    run = recovery_landing_run(tmp_path / "landing")
    c = Checks({"L45R": {"ulog_landed_and_disarmed_within_wall_s_of_land": 45.0}})
    p6_evaluate.response.fail_closed_landing(run, c)
    assert "FAIL_CLOSED_LANDING" in c.out and c.out["FAIL_CLOSED_LANDING"]["p2_failure"] == "stale_action"
    completed = tmp_path / "completed"
    completed.mkdir()
    (completed / "summary.json").write_text(json.dumps({"controller_exit_code": 0,
                                                        "controller_result": {"final_state": "COMPLETE", "failure": None}}))
    c2 = Checks({})
    p6_evaluate.response.fail_closed_landing(Run(completed), c2)
    assert "FAIL_CLOSED_LANDING" not in c2.out and not c2.failures  # nothing recorded, so nothing can be excepted
    rules = p6_evaluate.ROWS["C-3"]["exceptions"]
    items = ["runtime_status:Abort:controller_failed:stale_action", "p2:timing"] + list(p6_evaluate.response.RECOVERY_LANDING_ITEMS)
    other = ["runtime_status:Abort:deadline:controller", "cleanup:gazebo:forced_kill_after_grace"]
    allowed, refused = p6_evaluate.compose(items + other, rules, {"FAIL_CLOSED_LANDING": {"passed": True}})
    assert [a["item"] for a in allowed] == items and refused == other
    assert p6_evaluate.compose(items, rules, {})[1] == items


def _retained(prefix):
    import glob
    runs = glob.glob(str(Path.home() / "uav_autonomy/runs" / prefix))
    if not runs or not (Path(runs[0]) / "p5-evaluation.json").exists():
        pytest.skip("retained evidence not present: " + prefix)
    return Path(runs[0])


def test_the_retained_gate_c_runs_under_the_fixes():
    """The 2026-11-14 Gate C dry run on 23d513ee, stored evaluations unchanged: C-1 and C-3 now pass; C-4 fails on the
    Gazebo shutdown hang alone (a cleanup item is never excepted); C-5's load harness never ran, so it still fails
    (and since the C-5 load decision its declaration is no longer the frozen row, so strict evaluation refuses it)."""
    c1 = p6_evaluate.evaluate(_retained("20261114T083341Z-p6dry-C-1-*"))
    assert c1["status"] == "passed", c1["failures"]
    c3 = p6_evaluate.evaluate(_retained("20261114T084020Z-p6dry-C-3-*"))
    assert c3["status"] == "passed", c3["failures"]
    assert c3["response"]["FAIL_CLOSED_LANDING"]["p2_failure"] == "stale_action"
    c4 = p6_evaluate.evaluate(_retained("20261114T084352Z-p6dry-C-4-*"))
    assert c4["failures"] == ["p5:cleanup:gazebo:forced_kill_after_grace"], c4["failures"]
    c5_path = _retained("20261114T084636Z-p6dry-C-5-*")
    assert p6_evaluate.evaluate(c5_path)["failures"][0].startswith("declaration_not_frozen_row:")
    c5 = p6_evaluate.evaluate(c5_path, strict=False)
    assert "response:INJECTORS" in c5["failures"]


# ---- M6 C-5 load decision (operator 2026-11-14, "Adopt as recommended", plan §16.10-16.11) ------------------------
C5_CODES = {"H35": {"hover_after_last_usable_sim_s": 0.35, "stale_target_motion_m": 0.01, "stale_target_yaw_deg": 0.5},
            "L45R": {"ulog_landed_and_disarmed_within_wall_s_of_land": 45.0}}
LIMITS = {"inference_p99_s": 0.005, "inference_max_s": 0.010, "failure_wall_s": 0.020}


def load_run(tmp, failure="stale_action", abort_wall=1003.02, abort_callback_s=0.0538, cleared=True, ticks=None,
             decisions=None, workers=24):
    """A C-5-shaped run: the load starts at wall 1000.0 (sim 80.0) for 60 s; control callbacks at 20 Hz on one clock."""
    (tmp / "p6").mkdir(parents=True, exist_ok=True)
    d = declaration.build("C-5")
    (tmp / "p6/fault-declaration.json").write_text(json.dumps(d))
    harness = [{"kind": "activated", "sim_s": 80.0, "wall_s": 1000.0, "workers": workers, "seconds": 60.0, "start_wall_s": 1000.0}]
    if cleared:
        harness.append({"kind": "cleared", "sim_s": 80.0, "end_wall_s": 1014.0, "stopped_early": "SIGINT", "workers": workers,
                        "seconds": 60.0, "start_wall_s": 1000.0})
    (tmp / "p6/fault-ledger-load_harness.jsonl").write_text("".join(json.dumps(r) + chr(10) for r in harness))
    trace = []
    for k in range(400):  # wall 995.0 .. 1015.0, sim = wall - 920
        start = 995.0 + 0.05 * k
        duration = 0.004
        if failure is not None and start <= abort_wall <= start + 0.05:
            duration = abort_callback_s
            start = abort_wall - duration + 0.0001
        trace.append({"operation": "control_callback", "start_wall_s": start, "end_wall_s": start + duration,
                      "duration_wall_s": duration, "sim_s": round(start - 920.0, 3)})
    (tmp / "timing-trace.jsonl").write_text("".join(json.dumps(r) + chr(10) for r in trace))
    events = [] if failure is None else [{"event": "abort", "reason": failure, "sim_s": round(abort_wall - 920.0, 3),
                                          "monotonic_s": abort_wall}]
    events += decisions or []
    (tmp / "ros-events.jsonl").write_text("".join(json.dumps(e) + chr(10) for e in events))
    ledger = ticks if ticks is not None else [{"tick": k, "sim_s": 76.0 + 0.05 * k, "invoked": True, "inference_s": 0.0009,
                                               "failure": None, "action": "TRANSLATE"} for k in range(150)]
    (tmp / "p5").mkdir(exist_ok=True)
    (tmp / "p5/policy-ledger.jsonl").write_text("".join(json.dumps(r) + chr(10) for r in ledger))
    result = {"final_state": "RECOVERY" if failure else "COMPLETE", "failure": failure}
    (tmp / "summary.json").write_text(json.dumps({"controller_result": result}))
    return Run(tmp)


def load_check(run):
    c = Checks(C5_CODES)
    p6_evaluate.response.load(run, c, LIMITS, 0.05)
    return c.out["LOAD"]


def test_the_load_check_matches_each_breach_to_its_declared_response(tmp_path):
    """Contract §5.3: dispatch above the budget -> P2 abort; inference above the failure limit -> HOLD; stale snapshot
    -> H35; the P5 inference statistics hold outside the window."""
    ok = load_check(load_run(tmp_path / "ok"))
    assert ok["passed"], ok
    assert ok["window"]["wall_s"] == [1000.0, 1014.0] and ok["window"]["sim_s"][0] == pytest.approx(80.0)
    assert ok["dispatch"]["abort_callback_s"] == pytest.approx(0.0538)
    assert load_check(load_run(tmp_path / "held", failure=None))["passed"]  # limits held: nothing to match
    assert load_check(load_run(tmp_path / "uncleared", cleared=False))["window"]["wall_s"] == [1000.0, 1060.0]
    bad = {
        "not_dispatch": (dict(failure="stale_state"), "p2_failure_not_a_dispatch_response:stale_state"),
        "before_load": (dict(abort_wall=999.0), "p2_abort_outside_load_window"),
        "no_overrun": (dict(abort_callback_s=0.049), "p2_abort_without_measured_overrun"),
        "workers": (dict(workers=8), "load_not_as_declared"),
    }
    for name, (kw, problem) in bad.items():
        out = load_check(load_run(tmp_path / name, **kw))
        assert not out["passed"] and problem in out["problems"], (name, out)
    slow = [{"tick": 1, "sim_s": 81.0, "invoked": True, "inference_s": 0.021, "failure": None, "action": "TRANSLATE"}]
    assert load_check(load_run(tmp_path / "slow", ticks=slow))["inference"]["without_hold"] == [1]
    held = [dict(slow[0], failure="timeout", action="HOLD")]
    assert load_check(load_run(tmp_path / "slow_held", ticks=held))["passed"]
    outside = [{"tick": 1, "sim_s": 70.0, "invoked": True, "inference_s": 0.006, "failure": None, "action": "TRANSLATE"}]
    assert not load_check(load_run(tmp_path / "outside", ticks=outside))["passed"]  # the p99 limit holds outside
    assert load_check(load_run(tmp_path / "inside", ticks=[dict(outside[0], sim_s=81.0)]))["passed"]  # §5.3 inside

    def decision(sim, stale=None, acq=None, state="AVOIDING", action="TRANSLATE"):
        return {"event": "p4_decision", "sim_s": sim, "stale": stale, "snapshot_acquisition_sim_s": acq, "state": state,
                "action": action, "target_enu": [0.0, 0.0, 1.2]}
    timely = [decision(81.0, acq=80.95), decision(81.3, "snapshot_stale", 80.95, "HOVER", "HOLD")]
    assert load_check(load_run(tmp_path / "h35", decisions=timely))["stale"] == {"onsets": 1, "without_h35": []}
    late = [decision(81.0, acq=80.95), decision(81.4, "snapshot_stale", 80.95, "HOVER", "HOLD")]
    assert load_check(load_run(tmp_path / "late", decisions=late))["stale"]["without_h35"] == [81.4]
    moving = [decision(81.0, acq=80.95), decision(81.3, "snapshot_stale", 80.95, "AVOIDING", "TRANSLATE")]
    assert not load_check(load_run(tmp_path / "moving", decisions=moving))["passed"]
    no_load = tmp_path / "no_load"
    load_run(no_load)
    (no_load / "p6/fault-ledger-load_harness.jsonl").write_text("")
    assert load_check(Run(no_load)) == {"passed": False, "error": "no_load_activation"}


def _stamp(k):
    return 1_000_000_000 + 33_000_000 * k


def recorder_run(tmp, missing=(5, 6, 9), lost_wall=None, missed_periods=0, depth_failures=None, error=None, exit_code=3,
                 native_extra=(), foreign=()):
    """A recorder overflow: 12 receipts at wall 1001.0 + 0.1 k, the load window [1000.0, 1014.0]; the native source
    ledger has one frame per receipt (plus `native_extra` stamps), the sensor window is [0.5, 2.0] s."""
    load_run(tmp)
    (tmp / "sensors").mkdir(exist_ok=True)
    walls = {k: 1001.0 + 0.1 * k for k in range(1, 13)}
    walls.update(lost_wall or {})
    index = [{"receipt_id": k, "receipt_wall_s": walls[k], "acquisition_stamp_ns": _stamp(k) + (7 if k in foreign else 0),
              "accepted": True} for k in range(1, 13) if k not in missing]
    native = sorted([_stamp(k) for k in range(1, 13)] + list(native_extra))
    (tmp / "source").mkdir(exist_ok=True)
    (tmp / "source/depth-headers.jsonl").write_text("".join(
        json.dumps({"id": n, "seq": n, "stamp_ns": s}) + chr(10) for n, s in enumerate(native, 1)))
    (tmp / "sensors/depth-index.jsonl").write_text("".join(json.dumps(r) + chr(10) for r in index))
    recorder = {"accepted": len(index), "written": len(index), "overflow": len(missing), "drained": True, "fsynced": True,
                "error": error, "complete": False}
    (tmp / "sensors/depth-recorder-closed.json").write_text(json.dumps(recorder))
    (tmp / "sensors/adapter-result.json").write_text(json.dumps({
        "status": "failed", "error": "raw_recorder_overflow", "recorder": recorder,
        "accounting": {"received": 12, "missed_periods": missed_periods, "rejected": {}}}))
    summary = json.loads((tmp / "summary.json").read_text())
    summary["processes"] = [{"name": "sensor-adapter", "exit_code": exit_code, "stop_method": "SIGINT", "reaped": True,
                             "forced": False, "graceful_codes": [0], "stop_requested_monotonic_s": 1013.0,
                             "exit_observed_monotonic_s": 1013.2, "grace_s": 15.0, "remaining_group_members": []}]
    summary["sensor_window_sim_s"] = [0.5, 2.0]
    (tmp / "summary.json").write_text(json.dumps(summary))
    (tmp / "p5-evaluation.json").write_text(json.dumps({"depth_stream": {"failures": depth_failures or [
        "adapter_result_not_passed", "raw_recorder_incomplete"]}}))
    return Run(tmp)


def test_the_load_recorder_proof_accepts_an_overflow_inside_the_window_only(tmp_path):
    ok = p6_evaluate.response.load_recorder(recorder_run(tmp_path / "ok"))
    assert ok["passed"] and ok["missing_receipts"] == 3, ok
    bad = {"outside": dict(lost_wall={4: 999.0, 5: 999.1, 6: 999.2}, missing=(5,)),
           "missed_period": dict(missed_periods=1),
           "other_depth": dict(depth_failures=["adapter_result_not_passed", "raw_recorder_incomplete", "observation_age"]),
           "recorder_error": dict(error="write_failed"), "exit": dict(exit_code=1)}
    for name, kw in bad.items():
        assert not p6_evaluate.response.load_recorder(recorder_run(tmp_path / name, **kw))["passed"], name
    clean = tmp_path / "clean"
    recorder_run(clean)
    (clean / "sensors/adapter-result.json").write_text(json.dumps({"status": "passed", "error": None}))
    c = Checks({})
    p6_evaluate.response.load_recorder_check(Run(clean), c)
    assert "LOAD_RECORDER" not in c.out  # no overflow: nothing recorded, so its exceptions apply to nothing


DELIVERY = ["adapter_result_not_passed", "raw_recorder_incomplete", "source_to_ros_delivery_loss",
            "ros_receipts_not_in_native_ledger"]


def test_the_extended_recorder_proof_matches_each_undelivered_frame_to_an_overflowed_receipt(tmp_path):
    """0db5c60a C-5 (plan §16.12): with the overflow inside the sensor window, the accepted P3 evaluator (which judges
    delivery by the recorder's index) also reports its two delivery items. They are explained only when each frame it
    counts as undelivered lies in a gap between written receipts holding an overflowed receipt of its own."""
    ok = p6_evaluate.response.load_recorder(recorder_run(tmp_path / "ok", depth_failures=DELIVERY))
    assert ok["passed"] and ok["delivery"]["undelivered_by_index"] == 3 and ok["delivery"]["gaps"] == 2, ok
    real_loss = recorder_run(tmp_path / "real_loss", depth_failures=DELIVERY, native_extra=(_stamp(2) + 10_000_000,))
    out = p6_evaluate.response.load_recorder(real_loss)  # a native frame between receipts 2 and 3: nothing overflowed there
    assert not out["passed"] and out["delivery"]["gaps_short_of_overflowed_receipts"] == [[2, 3]], out
    two_in_one = recorder_run(tmp_path / "two", depth_failures=DELIVERY, native_extra=(_stamp(9) + 10_000_000,))
    assert not p6_evaluate.response.load_recorder(two_in_one)["passed"]  # 2 undelivered, 1 overflowed receipt in the gap
    foreign = recorder_run(tmp_path / "foreign", depth_failures=DELIVERY, foreign=(3,))
    assert p6_evaluate.response.load_recorder(foreign)["delivery"]["receipts_not_native_ns"], "a receipt without a native frame"
    edge = recorder_run(tmp_path / "edge", depth_failures=DELIVERY, native_extra=(_stamp(12) + 10_000_000,))
    assert not p6_evaluate.response.load_recorder(edge)["passed"]  # after the last written receipt: not in any gap
    other = recorder_run(tmp_path / "other", depth_failures=DELIVERY + ["observation_age"])
    assert not p6_evaluate.response.load_recorder(other)["passed"] and p6_evaluate.response.load_recorder(other)["delivery"] is None
    rules = p6_evaluate.ROWS["C-5"]["exceptions"]
    items = ["depth:source_to_ros_delivery_loss", "depth:ros_receipts_not_in_native_ledger"]
    assert p6_evaluate.compose(items, rules, {"LOAD_RECORDER": {"passed": True}})[1] == []
    assert p6_evaluate.compose(items, rules, {})[1] == items
    for row in ("C-1", "C-4", "S-1"):  # nowhere else
        assert p6_evaluate.compose(items, p6_evaluate.ROWS[row]["exceptions"], {"LOAD_RECORDER": {"passed": True}})[1] == items, row


def test_the_retained_0db5c60a_c5_run_passes_under_the_extension():
    """The 2026-11-14 C-5 rerun on 0db5c60a (its declaration is the frozen row): the map loss under load, H35, recovery,
    the 52.2 ms dispatch abort matched by LOAD, a clean harness stop; the 43-frame overflow began inside the sensor
    window, and all 32 frames the P3 evaluator counted as undelivered match overflowed receipts. Stored evaluation
    unchanged; it failed under the rule as first adopted and passes under the extension."""
    run = _retained("20261114T123723Z-p6dry-C-5-*")
    assert json.loads((run / "p6/p6-evaluation.json").read_text())["status"] == "failed"
    out = p6_evaluate.evaluate(run)
    assert out["status"] == "passed", out["failures"]
    proof = out["response"]["LOAD_RECORDER"]
    assert proof["missing_receipts"] == 43 and proof["delivery"]["undelivered_by_index"] == 32, proof
    assert out["response"]["LANDING"]["path"] == "FAIL_CLOSED_LANDING" and out["response"]["H35"]["passed"]
    allowed = {a["item"]: a["replacement"] for a in out["p5_failures_allowed"]}
    assert all(allowed[item] == "LOAD_RECORDER" for item in p6_evaluate.response.LOAD_RECORDER_ITEMS), allowed


def test_c5_allows_its_load_items_only_with_their_checks():
    rules = p6_evaluate.ROWS["C-5"]
    assert rules["response"].index("H35") == 0 and "LANDING" in rules["response"] and "LOAD" in rules["response"]
    landing = (["runtime_status:Abort:controller_failed:stale_action", "p2:timing"]
               + list(p6_evaluate.response.RECOVERY_LANDING_ITEMS))
    recorder = list(p6_evaluate.response.LOAD_RECORDER_ITEMS)
    stats = ["policy:inference_p99", "policy:inference_max"]
    other = ["runtime_status:Abort:controller_failed:stale_state", "cleanup:p6-load-harness:terminated_by_SIGINT",
             "depth:observation_age", "policy:failure_ticks"]
    passed = {code: {"passed": True} for code in ("FAIL_CLOSED_LANDING", "LOAD_RECORDER", "LOAD", "LANDING", "STALE_IFF_AGE")}
    allowed, refused = p6_evaluate.compose(landing + recorder + stats + other, rules["exceptions"], passed)
    assert [a["item"] for a in allowed] == landing + recorder + stats and refused == other
    for code in ("FAIL_CLOSED_LANDING", "LOAD_RECORDER", "LOAD"):
        without = {k: v for k, v in passed.items() if k != code}
        assert p6_evaluate.compose(landing + recorder + stats, rules["exceptions"], without)[1], code


def landed_and_disarmed(tmp, **kw):
    recovery_landing_run(tmp, **kw)
    with (tmp / "ros-events.jsonl").open("a") as stream:
        stream.write(json.dumps({"event": "received", "topic_key": "vehicle_status", "monotonic_s": 140.5,
                                 "fields": {"arming_state": 1}}) + chr(10))
    return Run(tmp)


def test_the_landing_check_takes_the_path_the_run_took(tmp_path):
    c = Checks(C5_CODES)
    p6_evaluate.response.landing(landed_and_disarmed(tmp_path / "fail_closed"), c)
    assert c.out["LANDING"]["path"] == "FAIL_CLOSED_LANDING" and c.out["LANDING"]["passed"], c.out
    items = list(p6_evaluate.response.RECOVERY_LANDING_ITEMS) + list(p6_evaluate.response.LOAD_RECORDER_ITEMS)
    c2 = Checks(C5_CODES)
    p6_evaluate.response.landing(landed_and_disarmed(tmp_path / "extra", p5_failures=items), c2)
    assert not c2.out["LANDING"]["passed"]  # the recorder items are unexplained unless LOAD_RECORDER passed
    c3 = Checks(C5_CODES)
    p6_evaluate.response.landing(landed_and_disarmed(tmp_path / "explained", p5_failures=items), c3,
                                 p6_evaluate.response.LOAD_RECORDER_ITEMS)
    assert c3.out["LANDING"]["passed"], c3.out
    completed = tmp_path / "completed"
    completed.mkdir()
    (completed / "summary.json").write_text(json.dumps({"controller_result": {"final_state": "COMPLETE", "failure": None,
                                                                              "p4_outcome": "completed"}}))
    c4 = Checks(C5_CODES)
    p6_evaluate.response.landing(Run(completed), c4)
    assert c4.out["LANDING"]["path"] == "L45" and c4.out["LANDING"]["passed"]


def test_the_retained_75289deb_c5_run_under_the_load_decision():
    """The 2026-11-14 C-5 rerun on 75289deb (load from transit_start, before the decision; evaluated non-strictly since
    its declaration is no longer the frozen row): LOAD matches the 53.8 ms overrun to the P2 abort; LOAD_RECORDER proves
    all 31 lost frames inside the window; the fail-closed landing's L45R passes, and its RECOVERY_LANDING proof finds
    exactly one unexplained item, the old harness's death on cleanup's SIGINT (fixed since). It still fails, as it
    should: the map loss never fired (no H35), and that harness item is never excepted."""
    out = p6_evaluate.evaluate(_retained("20261114T110631Z-p6dry-C-5-*"), strict=False)
    response = out["response"]
    assert response["LOAD"]["passed"], response["LOAD"]
    assert response["LOAD"]["dispatch"]["abort_callback_s"] > 0.05 and response["LOAD"]["dispatch"]["p2_failure"] == "stale_action"
    assert response["LOAD_RECORDER"]["passed"] and response["LOAD_RECORDER"]["missing_receipts"] == 31, response["LOAD_RECORDER"]
    landing = response["FAIL_CLOSED_LANDING"]
    assert landing["l45r"]["passed"], landing
    assert landing["recovery_landing"]["other_items"] == ["cleanup:p6-load-harness:terminated_by_SIGINT"], landing
    allowed = {a["item"]: a["replacement"] for a in out["p5_failures_allowed"]}
    recorder_items = [x for x in p6_evaluate.response.LOAD_RECORDER_ITEMS if not x.startswith(("depth:source_to_ros",
                                                                                              "depth:ros_receipts"))]
    assert len(recorder_items) == 3 and all(allowed.get(item) == "LOAD_RECORDER" for item in recorder_items), allowed
    # its overflow came after the sensor window ended (76.568 s), so the P3 delivery items never arose
    assert response["LOAD_RECORDER"]["delivery"] is None and "depth:source_to_ros_delivery_loss" not in allowed
    assert allowed.get("policy:inference_p99") == "LOAD"
    assert {"proof:no_activation", "response:H35", "response:INJECTORS",
            "p5:cleanup:p6-load-harness:terminated_by_SIGINT"} <= set(out["failures"]), out["failures"]


# ---- M6 map-drop proof fix (operator 2026-11-15, "Adopt the proof fix", plan §16.14-16.15) -------------------------
def map_drop_run(tmp, uses, t0=10.0):
    """Perception publishes every 0.1 s (acquired at k/10, processed 0.03 s later); the shim's drop window is
    [t0, t0 + 1.0); `uses` maps each controller decision time to the snapshot acquisition it used."""
    (tmp / "p6").mkdir(parents=True, exist_ok=True)
    (tmp / "p4").mkdir(exist_ok=True)
    (tmp / "p6/fault-ledger-controller_shim.jsonl").write_text(json.dumps({"kind": "activated", "sim_s": t0}) + chr(10))
    snapshots = [{"usable": True, "acquisition_sim_s": round(k / 10, 3), "processing_sim_s": round(k / 10 + 0.03, 3)}
                 for k in range(90, 125)]
    (tmp / "p4/local_map.jsonl").write_text("".join(json.dumps(s) + chr(10) for s in snapshots))
    events = [{"event": "p4_decision", "sim_s": t, "snapshot_acquisition_sim_s": a} for t, a in sorted(uses.items())]
    (tmp / "ros-events.jsonl").write_text("".join(json.dumps(e) + chr(10) for e in events))
    return Run(tmp)


def test_the_map_drop_proof_judges_the_controllers_decisions_inside_the_drop_window(tmp_path):
    """6b1d952d C-5: under load the last snapshot processed inside the window (10.93 s here) reached the controller
    after the window and was first used after it, as the declared drop allows. Inside the window every decision must
    hold the one snapshot current at the trigger; any snapshot let through, from before or inside the window, fails."""
    from p6_fault_evaluate import prove
    d = declaration.build("C-5")
    window = [round(10.0 + 0.05 * k, 3) for k in range(20)]  # decisions in [10.0, 11.0)
    held = {t: 9.9 for t in window}
    late = {**held, 11.02: 10.9, 11.07: 11.0}
    ok = prove(map_drop_run(tmp_path / "late", late), d)
    assert ok["proved"], ok
    assert ok["evidence"]["snapshot_held_in_window"] == 9.9 and ok["evidence"]["first_used_after_window_sim_s"] == [11.02]
    leak = {**late, 10.5: 10.4}  # a snapshot processed inside the window, used inside it
    out = prove(map_drop_run(tmp_path / "leak", leak), d)
    assert {"proof:controller_used_a_dropped_snapshot", "proof:controller_map_input_changed_in_window"} <= set(out["failures"])
    early = {**late, 10.3: 10.0}  # processed at 10.03, after the trigger, before any decision could hold it
    assert "proof:controller_map_input_changed_in_window" in prove(map_drop_run(tmp_path / "early", early), d)["failures"]
    pre = {t: (9.9 if t < 10.3 else 9.95) for t in window}  # switched to another pre-trigger snapshot inside the window
    pre_out = prove(map_drop_run(tmp_path / "pre", {**pre, 9.95: 9.9}), d)
    assert pre_out["failures"] == ["proof:controller_map_input_changed_in_window"], pre_out
    none_inside = prove(map_drop_run(tmp_path / "none", {9.95: 9.9, 11.02: 10.9}), d)
    assert "proof:no_controller_decision_in_window" in none_inside["failures"]


def test_the_retained_6b1d952d_c5_run_passes_under_the_proof_fix():
    """The 2026-11-15 C-5 rerun on 6b1d952d: every response check passed; its stored evaluation failed only the old
    map-drop proof (the 80.092 s snapshot, processed at 80.152 s, reached the controller after the window closed at
    80.196 s and was used at 80.232 s). All 22 decisions inside the window held the 79.136 s snapshot."""
    run = _retained("20261115T032134Z-p6dry-C-5-*")
    stored = json.loads((run / "p6/p6-evaluation.json").read_text())
    assert stored["failures"] == ["proof:controller_used_a_dropped_snapshot"]
    out = p6_evaluate.evaluate(run)
    assert out["status"] == "passed", out["failures"]
    evidence = out["proof"]["evidence"]
    assert evidence["decisions_in_window"] == 22 and evidence["snapshot_held_in_window"] == 79.136
    assert evidence["first_used_after_window_sim_s"] == [80.232]
