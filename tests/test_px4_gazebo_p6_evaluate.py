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
    never = ("safety:", "native", "competing_flight_publisher", "residual_namespace_processes", "cleanup:", "p2:final_landed")
    for row, rules in p6_evaluate.ROWS.items():
        for prefix in rules["exceptions"]:
            assert not any(prefix.startswith(n) or n.startswith(prefix) for n in never if prefix), (row, prefix)


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
    run = synthetic_run(tmp_path / "ok", d, "depth_relay", files={"p6/witness/raw_image.jsonl": raw,
                                                                    "p6/witness/relayed_image.jsonl": relayed})
    assert prove(run, d)["proved"]
    run = synthetic_run(tmp_path / "none", d, "depth_relay", files={"p6/witness/raw_image.jsonl": raw,
                                                                      "p6/witness/relayed_image.jsonl": raw})
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
             "p4/perception-result.json": {"unusable_reasons": {"association:position:pending_newer_state": 40}}}
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
