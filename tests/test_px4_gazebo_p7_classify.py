"""P7 M3: the flight classifier, offline (contract §5 with D13/D14, §7.1).

* **The four required reference flights (R1-R4):** valid, goal success, no events. The metrics equal the accepted
  evaluations: final goal distance, clearance, and P5 time to goal and path. These are read-only; nothing is
  written into retained evidence.
* **The full chain on a copy of R3:** `classify()` through `verify_p7_evidence.sh`, in the accepted evaluators'
  environment, reading the real ULog (where `venv-px4` exists).
* **Negative cases on copies:** each D14 class, goal conditions, validity, and the two restated P6 §6.3 rules.
* **The restated rules agree with the accepted P6 functions** on every retained accepted P6 flight.
"""
import json
import os
from pathlib import Path
import shutil
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
for path in (P7 / "validation", REPO / "simulation/px4_gazebo_p6", REPO / "simulation/px4_gazebo_p6/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7_classify as c  # noqa: E402

ROOT = Path(os.environ.get("GWM_SIM_ROOT", Path.home() / "uav_autonomy"))
RUNS = ROOT / "runs"
R = {"R1": "20261030T144136Z-p4-flight-S2-abeea2d5", "R2": "20261030T141506Z-p4-flight-S2-9b8ef2b0",
     "R3": "20261106T132442Z-p5-flight-S2-184a5a5e", "R4": "20261106T130044Z-p5-flight-S2-679dd050"}
needs_refs = pytest.mark.skipif(not all((RUNS / n / "summary.json").exists() for n in R.values()),
                                reason="the required reference runs are not available")
SMALL = ("summary.json", "p4-evaluation.json", "p5-evaluation.json", "p4_avoidance.yaml", "ros-events.jsonl",
         "resources.jsonl", "p4/scenario.json", "p4/truth_poses.jsonl", "p4/contacts.jsonl", "p5/policy-ledger.jsonl")
ARMED = lambda summary: True  # noqa: E731  (the ULog itself is read in the full-chain test)


def copy(name, tmp_path):
    src, dst = RUNS / name, tmp_path / name
    for rel in SMALL:
        if (src / rel).exists():
            (dst / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src / rel, dst / rel)
    return dst


def edit_json(path, change):
    value = json.loads(path.read_text())
    change(value)
    path.write_text(json.dumps(value))


def with_failures(run, *items):
    name = "p5-evaluation.json" if (run / "p5-evaluation.json").exists() else "p4-evaluation.json"
    edit_json(run / name, lambda v: v.__setitem__("failures", list(v["failures"]) + list(items)))
    return run


# ---- the reference flights ----------------------------------------------------------------------------------
@needs_refs
@pytest.mark.parametrize("ref", sorted(R))
def test_the_reference_flights_are_valid_goal_successes_with_the_accepted_numbers(ref):
    run = RUNS / R[ref]
    record = c.build_record(run, ulog_reader=ARMED)
    evaluation = json.loads((run / ("p5-evaluation.json" if "-p5-" in run.name else "p4-evaluation.json")).read_text())
    assert record["validity"] == "valid" and record["outcome"] == "goal_success" and record["stop_reasons"] == []
    assert all(record["goal_conditions"].values()) and not any(record["classes"].values())
    assert record["unsafe_rules"] == {"hold_tick_motion": [], "unrevalidated_resumes": []}
    m = record["metrics"]
    assert m["final_goal_distance_m"] == pytest.approx(evaluation["outcome"]["final_goal_distance_m"], abs=1e-12)
    assert m["min_truth_clearance_m"] == evaluation["safety"]["min_truth_clearance_m"]
    if "reported" in evaluation:
        assert (m["time_to_goal_sim_s"], m["path_length_m"]) == (evaluation["reported"]["time_to_goal_sim_s"],
                                                                 evaluation["reported"]["path_length_m"])
    assert not (run / "p7").exists()  # retained evidence is only read


@needs_refs
@pytest.mark.skipif(not (ROOT / "venv-px4/bin/python").exists(), reason="the accepted evaluators' venv is not available")
def test_the_full_chain_reads_the_real_ulog_and_writes_the_record_once(tmp_path):
    run = copy(R["R3"], tmp_path)
    record = c.classify(run)
    assert record["armed_evidence"] == {"ulog_armed": True, "takeoff_transition": True}
    assert record["outcome"] == "goal_success" and (run / "p7/flight-record.json").exists()
    with pytest.raises(RuntimeError):
        c.classify(run)  # the first classification is retained; a second one refuses


# ---- negative cases -------------------------------------------------------------------------------------------
@needs_refs
@pytest.mark.parametrize("item, klass, stops", [
    ("safety:collision_contact", "safety_events", True), ("safety:truth_clearance", "safety_events", True),
    ("safety:setpoint_inside_inflated_obstacle", "safety_events", True), ("native_commands:unexpected", "safety_events", True),
    ("p2:command_sequence", "safety_events", True), ("competing_flight_publisher", "safety_events", True),
    ("p2:final_landed_disarmed", "safety_events", True),
    ("safety:truth_poses_missing", "safety_unverified", True), ("safety:truth_pose_interval", "safety_unverified", True),
    ("safety:truth_observer_error", "safety_unverified", True), ("p2_integrity_error:KeyError:x", "safety_unverified", True),
    ("safety:tracking", "mission_failures", False), ("safety:response_latency", "mission_failures", False),
    ("performance:compute_p99", "integrity_events", False), ("p2:timing", "integrity_events", False),
    ("cleanup:dds-agent:signal_11", "integrity_events", False), ("policy:fallback_decisions", "integrity_events", False)])
def test_each_evaluator_item_falls_into_its_d14_class(tmp_path, item, klass, stops):
    record = c.build_record(with_failures(copy(R["R3"], tmp_path), item), ulog_reader=ARMED)
    assert item in record["classes"][klass]
    assert bool(record["stop_reasons"]) == stops
    assert record["outcome"] == ("goal_success" if klass == "integrity_events" else "failure")


@needs_refs
def test_missing_safety_evidence_is_safety_unverified(tmp_path):
    run = copy(R["R1"], tmp_path)
    (run / "p4/contacts.jsonl").unlink()
    assert "contact_ledger_missing" in c.build_record(run, ulog_reader=ARMED)["classes"]["safety_unverified"]
    run = copy(R["R2"], tmp_path / "b")
    (run / "p4-evaluation.json").unlink()
    record = c.build_record(run, ulog_reader=ARMED)
    assert "evaluation_missing" in record["classes"]["safety_unverified"] and record["outcome"] == "failure"
    run = copy(R["R2"], tmp_path / "c")
    edit_json(run / "p4-evaluation.json", lambda v: v.pop("safety"))
    assert "no_safety_evaluation" in c.build_record(run, ulog_reader=ARMED)["stop_reasons"][0]


@needs_refs
def test_goal_conditions(tmp_path):
    run = copy(R["R3"], tmp_path)
    edit_json(run / "summary.json", lambda s: (s["controller_result"].__setitem__("p4_outcome", "supervisor_abort"),
                                               s["controller_result"]["p4"].__setitem__("goal_reached_sim_s", None)))
    record = c.build_record(run, ulog_reader=ARMED)
    assert record["outcome"] == "failure" and record["stop_reasons"] == []
    assert not record["goal_conditions"]["goal_reached_recorded"] and not record["goal_conditions"]["p4_outcome_completed"]
    run = copy(R["R4"], tmp_path / "far")
    lines = (run / "p4/truth_poses.jsonl").read_text().splitlines()
    last = json.loads(lines[-1])
    last["poses"]["x500_depth_71"]["position"][1] += 0.5  # the vehicle ends 0.5 m beside the goal
    (run / "p4/truth_poses.jsonl").write_text("\n".join(lines[:-1] + [json.dumps(last)]) + "\n")
    record = c.build_record(run, ulog_reader=ARMED)
    assert record["metrics"]["final_goal_distance_m"] > 0.30 and record["outcome"] == "failure"
    assert not record["goal_conditions"]["final_goal_distance_within_tolerance"]
    run = copy(R["R3"], tmp_path / "landed")
    edit_json(run / "p5-evaluation.json", lambda v: v["p2_integrity"].__setitem__("final_disarmed", False))
    assert c.build_record(run, ulog_reader=ARMED)["outcome"] == "failure"


@needs_refs
def test_validity_follows_the_armed_evidence_only(tmp_path):
    run = copy(R["R1"], tmp_path)
    edit_json(run / "summary.json", lambda s: s["controller_result"].__setitem__(
        "transitions", [t for t in s["controller_result"]["transitions"] if t["to"] in ("WAIT_FOR_VALID_ESTIMATION", "PRESTREAM_SAFE_SETPOINTS")]))
    with_failures(run, "safety:truth_poses_missing", "runtime_status:Abort:host_timing")
    record = c.build_record(run, ulog_reader=lambda s: None)
    assert record["validity"] == "infrastructure_invalid" and record["outcome"] is None and record["stop_reasons"] == []
    with_failures(run, "native_commands:unexpected_command")
    assert c.build_record(run, ulog_reader=lambda s: None)["stop_reasons"] == ["safety_event:native_commands:unexpected_command"]
    assert c.build_record(run, ulog_reader=lambda s: True)["validity"] == "valid"  # the ULog alone is armed evidence
    run = copy(R["R2"], tmp_path / "dis")
    record = c.build_record(run, ulog_reader=lambda s: False)
    assert record["validity"] == "valid" and "armed_evidence_disagreement" in record["classes"]["integrity_events"]


def decision(sim, target, stale=None, acquired=None):
    return {"event": "p4_decision", "sim_s": sim, "target_enu": target, "stale": stale,
            "snapshot_acquisition_sim_s": sim - 0.1 if acquired is None else acquired}


def test_the_hold_tick_rule():
    still = [decision(1.0, [0, 0], stale="snapshot_stale"), decision(1.05, [0.005, 0]), decision(1.1, [0.5, 0])]
    assert c.hold_tick_motion(still, []) == []
    moved = [decision(1.0, [0, 0], stale="snapshot_stale"), decision(1.05, [0.02, 0])]
    assert c.hold_tick_motion(moved, []) == [1.0]
    policy = [decision(2.0, [0, 0]), decision(2.05, [0.03, 0])]
    assert c.hold_tick_motion(policy, [{"sim_s": 2.0, "failure": "timeout"}]) == [2.0]
    assert c.hold_tick_motion(policy, [{"sim_s": 2.0, "failure": None}]) == []
    assert c.HOLD_MOTION_M == 0.01


def test_the_revalidated_resume_rule():
    back = [{"from": "HOVER", "to": "AVOIDING", "sim_s": 5.0}]
    assert c.unrevalidated_resumes(back, [decision(5.0, [0, 0])], 0.25) == []
    assert c.unrevalidated_resumes(back, [decision(5.0, [0, 0], stale="snapshot_stale")], 0.25) == [5.0]
    assert c.unrevalidated_resumes(back, [decision(5.0, [0, 0], acquired=4.6)], 0.25) == [5.0]
    assert c.unrevalidated_resumes(back, [], 0.25) == [5.0]
    assert c.unrevalidated_resumes([{"from": "NORMAL", "to": "OBSTACLE_DETECTED", "sim_s": 1.0}], [], 0.25) == []


@needs_refs
def test_a_restated_rule_breach_stops_the_batch(tmp_path):
    run = copy(R["R3"], tmp_path)
    rows = [json.loads(x) for x in (run / "ros-events.jsonl").read_text().splitlines()]
    idx = [i for i, e in enumerate(rows) if e.get("event") == "p4_decision"]
    rows[idx[100]]["stale"] = "snapshot_stale"
    rows[idx[101]]["target_enu"] = [rows[idx[100]]["target_enu"][0] + 0.05, rows[idx[100]]["target_enu"][1]]
    (run / "ros-events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in rows))
    record = c.build_record(run, ulog_reader=ARMED)
    assert "safety_event:unsafe:hold_tick_motion" in record["stop_reasons"] and record["outcome"] == "failure"


# ---- agreement with the accepted P6 functions --------------------------------------------------------------------
def p6_flights():
    sha = "222927752a507fdb2fb0ef13f44d6ce8163d50b7d22f405ebbf27df6c21bfa41"
    runs = []
    for path in sorted((ROOT / "state").glob("p6-gate-*-%s.result.json" % sha)):
        runs += [Path(r["run"]) for r in json.loads(path.read_text())["runs"] if r.get("scope") == "flight" and r.get("run")]
    return [r for r in runs if (r / "summary.json").exists()]


@pytest.mark.skipif(not p6_flights(), reason="the retained P6 flights are not available")
def test_the_restated_rules_agree_with_the_accepted_p6_functions_on_every_retained_p6_flight():
    import p6_safety_evaluate as p6
    from p6_run import Run
    codes = json.loads((REPO / "simulation/px4_gazebo_p6/configs/p6_faults.yaml").read_text())["codes"]
    flights = p6_flights()
    assert len(flights) == 12
    returns = 0
    for path in flights:
        run = Run(path)
        checks = p6.Checks(codes)
        p6.unsafe_setpoints(run, checks)
        p6.recovery(run, checks)
        assert c.hold_tick_motion(run.decisions, run.ledger) == checks.out["UNSAFE"]["moves"], path.name
        ours = c.unrevalidated_resumes(run.transitions, run.decisions, 0.25)
        assert ours == checks.out["RECOVERY"]["unrevalidated"], path.name
        returns += checks.out["RECOVERY"]["returns"]
    assert returns > 0  # at least one retained flight exercises a HOVER -> AVOIDING return
