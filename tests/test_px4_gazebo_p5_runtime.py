"""P5 M5 offline: evaluator rules on synthetic evidence, the policy replays, runner and gate logic.

No PX4, Gazebo, ROS node or QGC is started. The flight ledger comes from the trained policy flying a
held-out layout in the P5Mission rig (never an acceptance scenario); ground-fixture snapshots come
from production perception on rendered depth. Each pass and fail rule of the P5 evaluator is
exercised on copies of that evidence; the composition test shows the reused P4 checks still apply.
"""
import ast
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
import test_px4_gazebo_p5_mission as rig_module  # noqa: E402

for path in (p5.P5 / "validation", p5.syn.P4 / "validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p4_evaluate  # noqa: E402
import p5_evaluate  # noqa: E402
import p5_manifest  # noqa: E402
import p5_policy_eval  # noqa: E402
import run_p5_gates  # noqa: E402
from gwm_obstacle_avoidance import local_map, perception  # noqa: E402
from gwm_px4_control.contracts import strict_json  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402
from p3_stream_eval import Failures  # noqa: E402

syn = p5.syn
V5 = json.loads((p5.P5 / "configs/p5_validation.yaml").read_text(encoding="utf-8"))
LIMITS = V5["policy"]
CONFIG = syn.p4_config()
P5C = json.loads((p5.P5 / "configs/p5_policy.yaml").read_text(encoding="utf-8"))
SCHEMA_PATH = p5.P5 / "configs/p5_model_schema.json"
ARTIFACT = rig_module.ARTIFACT
SHA = ARTIFACT.stem[len("p5-ranker-"):] if ARTIFACT else None
needs_artifact = pytest.mark.skipif(ARTIFACT is None, reason="no exported P5 model artifact")


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(strict_json(r) + "\n" for r in rows))


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def configs(run):
    (run / "p4_avoidance.yaml").write_text(json.dumps(CONFIG))
    (run / "p5_policy.yaml").write_text(json.dumps(P5C))
    return run


# ---- flight evidence: the trained policy on a held-out layout ----------------------------------------------
@pytest.fixture(scope="module")
def flight_evidence(tmp_path_factory):
    if ARTIFACT is None:
        pytest.skip("no exported P5 model artifact")
    policy, layout = rig_module.trained_policy(), rig_module.held_out_layout()
    rig = rig_module.Rig(rig_module.p5_mission(policy), layout["boxes"])
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    rig.run_until(lambda a: rig.m.state == "GOAL_HOVER", 100.0)
    run = configs(tmp_path_factory.mktemp("flight"))
    ledger = [json.loads(strict_json(r)) for r in rig.m.ledger]
    for record in ledger:  # host-independent timing; the timing rules are exercised by explicit changes below
        if record["invoked"]:
            record["inference_s"] = 0.0005
    write_jsonl(run / "p5/policy-ledger.jsonl", ledger)
    write_jsonl(run / "ros-events.jsonl", rig.m.events)
    return run


def variant(flight_evidence, tmp_path, change):
    run = tmp_path / "run"
    shutil.copytree(flight_evidence, run)
    records = read_jsonl(run / "p5/policy-ledger.jsonl")
    change(records)
    write_jsonl(run / "p5/policy-ledger.jsonl", records)
    return run


def ledger_of(run):
    return p5_policy_eval.evaluate(run, "flight", ARTIFACT, SHA, SCHEMA_PATH)


def limits_failures(replayed):
    f = Failures()
    p5_evaluate.policy_ledger(replayed, {}, LIMITS, f)
    return f.items


def first(records, predicate):
    return next(r for r in records if predicate(r))


def translating(r):
    return r["invoked"] and r["failure"] is None and r["action"] == "TRANSLATE" and r["intervention"] is None


@needs_artifact
def test_recorded_flight_ledger_passes_every_policy_rule(flight_evidence):
    replayed = ledger_of(flight_evidence)
    ledger = replayed["ledger"]
    assert ledger["failures"] == [] and limits_failures(replayed) == []
    assert ledger["records"] == ledger["decisions"] > 500 and ledger["invoked"] > 0.9 * ledger["records"]
    assert ledger["score_mismatches"] == ledger["argmax_mismatches"] == ledger["executed_mismatches"] == 0
    assert ledger["participation_fraction"] == 1.0 and ledger["fallback_decisions"] == ledger["failure_ticks"] == 0
    assert replayed["numpy"] == np.__version__ and replayed["model_sha256"] == SHA


def tamper_score(records):
    record = first(records, lambda r: r["invoked"] and r["failure"] is None and sum(r["allowed"]) >= 2)
    other = next(i for i, a in enumerate(record["allowed"]) if a and i != record["choice_slot"])
    record["scores"][other] -= 1.0  # still not the maximum: only the reproduction changes


def tamper_choice(records):
    record = first(records, lambda r: r["invoked"] and r["failure"] is None and sum(r["allowed"]) >= 2)
    record["choice_slot"] = next(i for i, a in enumerate(record["allowed"]) if a and i != record["choice_slot"])


def tamper_executed(records):
    first(records, translating)["end"][0] += 0.1


def tamper_sha(records):
    first(records, lambda r: r["invoked"])["policy_sha256"] = "0" * 64


def drop_record(records):
    del records[len(records) // 2]


def shift_time(records):
    records[10]["sim_s"] += 0.05


@needs_artifact
@pytest.mark.parametrize("change,failure", [
    (tamper_score, "ledger:score_not_reproduced"),
    (tamper_choice, "ledger:argmax_differs"),
    (tamper_executed, "ledger:executed_differs_from_choice"),
    (tamper_sha, "ledger:policy_sha256_differs"),
    (drop_record, "ledger:record_count_differs_from_decisions"),
    (drop_record, "ledger:tick_sequence"),
    (shift_time, "ledger:decision_mismatch")])
def test_ledger_structural_rules(flight_evidence, tmp_path, change, failure):
    replayed = ledger_of(variant(flight_evidence, tmp_path, change))
    assert failure in replayed["ledger"]["failures"], replayed["ledger"]["failures"]
    assert "policy:" + failure in limits_failures(replayed)
    if change is tamper_score:
        assert replayed["ledger"]["failures"] == ["ledger:score_not_reproduced"]  # the argmax still holds


def make_failure(records):
    record = first(records, translating)
    record.update(failure="timeout", scores=None, features=None, allowed=None, choice_slot=None)


def make_fallback(records):
    record = first(records, translating)
    for key in ("policy_sha256", "planning_state", "choice_slot", "p4_slot", "failure", "inference_s", "scores",
                "allowed", "features"):
        record.pop(key)
    record["invoked"] = False


def mask_out(records):
    record = first(records, lambda r: r["invoked"] and r["failure"] is None)
    record["allowed"][record["choice_slot"]] = False


def slow_p99(records):
    for record in records:
        if record["invoked"]:
            record["inference_s"] = 0.006


def slow_max(records):
    first(records, lambda r: r["invoked"])["inference_s"] = 0.011


@needs_artifact
@pytest.mark.parametrize("change,failure", [
    (make_failure, "policy:failure_ticks"),
    (make_fallback, "policy:fallback_decisions"),
    (mask_out, "policy:outside_mask"),
    (slow_p99, "policy:inference_p99"),
    (slow_max, "policy:inference_max"),
    (tamper_executed, "policy:participation")])
def test_p5_limits_on_the_recorded_decisions(flight_evidence, tmp_path, change, failure):
    assert failure in limits_failures(ledger_of(variant(flight_evidence, tmp_path, change)))


@needs_artifact
def test_a_missing_ledger_or_a_ledger_without_decisions_fails(flight_evidence, tmp_path):
    run = variant(flight_evidence, tmp_path, lambda records: None)
    (run / "p5/policy-ledger.jsonl").unlink()
    replayed = ledger_of(run)
    assert replayed["ledger"]["failures"] == ["ledger:missing"]
    assert {"policy:ledger:missing", "policy:no_policy_decisions"} <= set(limits_failures(replayed))


def test_the_replay_must_run_on_the_controllers_numpy():
    summary = {"controller_runtime": {"numpy": "1.26.4"}, "p5_model": {"sha256": "a", "schema_sha256": "s"},
               "controller_result": {"p5": {"numpy": "1.26.4"}}}
    f = Failures()
    p5_evaluate.replay_runtime({"numpy": "1.26.4", "model_sha256": "a", "schema_sha256": "s"}, summary, f)
    assert not f.items
    p5_evaluate.replay_runtime({"numpy": "2.5.3", "model_sha256": "a", "schema_sha256": "s"}, summary, f)
    p5_evaluate.replay_runtime({"numpy": "1.26.4", "model_sha256": "b", "schema_sha256": "s"}, summary, f)
    assert f.items == ["policy:replay_numpy_differs", "policy:replay_model_differs"]


# ---- checkpoint identity at start ---------------------------------------------------------------------------
def identity_run(tmp_path, events=1, **p5_changes):
    model = {"path": "models/p5-ranker-%s.npz" % SHA, "sha256": SHA, "architecture": "p5-ranker-mlp-v1",
             "schema_sha256": p5.SCHEMA_SHA}
    p5_block = {"model_sha256": SHA, "trained": True, "architecture": "p5-ranker-mlp-v1", "schema_sha256": p5.SCHEMA_SHA,
                "numpy": "1.26.4", **p5_changes}
    write_jsonl(tmp_path / "ros-events.jsonl", [{"event": "p5_policy_verified", "model_sha256": p5_block["model_sha256"]}] * events)
    return tmp_path, {"p5_model": model, "schema_sha256": p5.SCHEMA_SHA, "controller_runtime": {"numpy": "1.26.4"},
                      "controller_result": {"p5": p5_block}}


@needs_artifact
@pytest.mark.parametrize("kwargs,failure", [
    ({}, None),
    ({"events": 0}, "policy:checkpoint_verification_event"),
    ({"model_sha256": "f" * 64}, "policy:checkpoint_sha256_differs_from_manifest"),
    ({"trained": False}, "policy:checkpoint_identity"),
    ({"schema_sha256": "0" * 64}, "policy:checkpoint_identity"),
    ({"numpy": "2.5.3"}, "policy:controller_numpy_differs_from_manifest")])
def test_checkpoint_identity_at_start(tmp_path, kwargs, failure):
    run, summary = identity_run(tmp_path, **kwargs)
    f = Failures()
    p5_evaluate.checkpoint_identity(run, summary, f)
    assert (failure in f.items) if failure else not f.items, f.items


@needs_artifact
def test_a_changed_bound_artifact_fails_identity(tmp_path):
    run, summary = identity_run(tmp_path)
    summary["p5_model"]["path"] = "models/README.txt"  # exists, but does not hash to the bound SHA-256
    f = Failures()
    p5_evaluate.checkpoint_identity(run, summary, f)
    assert f.items == ["policy:bound_artifact_changed"]


def test_ground_scopes_must_not_make_policy_decisions(tmp_path):
    write_jsonl(tmp_path / "p5/policy-ledger.jsonl", [])
    write_jsonl(tmp_path / "ros-events.jsonl", [{"event": "startup"}])
    f = Failures()
    p5_evaluate.ground_ledger(tmp_path, f)
    assert not f.items
    write_jsonl(tmp_path / "p5/policy-ledger.jsonl", [{"tick": 1}])
    p5_evaluate.ground_ledger(tmp_path, f)
    assert f.items == ["policy:ground_scope_made_decisions"]


# ---- Gate A item 4: ground policy evaluation on recorded snapshots (revision 2) -------------------------------
def fixture_rows(name):
    import p4_worlds
    _, declaration = p4_worlds.generate('<sdf version="1.9"><world name="default"></world></sdf>', name, syn.scenarios())
    sensors = syn.sensors()
    extrinsic = Extrinsic(sensors["optical_origin_base_flu_m"])
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], 2.0, 0.1)
    vehicle = (0.0, 0.0, 0.24)
    position, q = syn.px4_pose(vehicle, 0.0)
    scene = syn.boxes_3d([{"center": b["center_m"][:2], "size": b["size_m"][:2]} for b in declaration["boxes"]])
    image = syn.render(vehicle, 0.0, scene, columns=np.arange(2, 640, 4))
    camera_xy, rays = perception.extract_scan(image, syn.CAL, extrinsic, position, q, CONFIG["perception"],
                                              sensors["near_m"], sensors["far_m"], 0.0)
    perception.integrate(grid, camera_xy, rays, 8.0, sensors["near_m"])
    return [{"usable": True, "processing_sim_s": 11.0 + k, "acquisition_sim_s": 10.95 + k, "grid": grid.encode(),
             "position_ned": list(position), "q_wxyz": list(q), "calibration_identity": "cal",
             "scan": [[r["azimuth"], r["kind"], r["range_m"]] for r in rays]} for k in range(5)]


@needs_artifact
@pytest.mark.parametrize("name", ["G2", "G4", "G6"])
def test_ground_evaluation_checks_inference_and_masking_without_ai_directed_motion(tmp_path, name):
    """Revision 2 item 4: on the ground nothing is allowed, so the policy is never invoked and P4's SCAN stands."""
    run = configs(tmp_path)
    write_jsonl(run / "p4/local_map.jsonl", fixture_rows(name))
    replayed = p5_policy_eval.evaluate(run, "fixture", ARTIFACT, SHA, SCHEMA_PATH, (10.0, 20.0))
    ground = replayed["ground"]
    assert ground["snapshots"] == 5 and ground["policy_invocations"] == 0 and ground["actions"] == {"SCAN": 5}
    assert ground["non_finite"] == ground["not_repeatable"] == ground["mask_mismatches"] == ground["decision_mismatches"] == 0
    ground["inference_s"] = {"p99": 0.001, "max": 0.002}  # host-independent pass path; timing failures below
    f = Failures()
    p5_evaluate.policy_ground(replayed, LIMITS, f)
    assert not f.items, f.items


def ground_replayed(**changes):
    ground = {"snapshots": 5, "non_finite": 0, "not_repeatable": 0, "mask_mismatches": 0, "decision_mismatches": 0,
              "policy_invocations": 0, "policy_failures": 0, "unsafe_policy_moves": 0, "actions": {"SCAN": 5},
              "inference_s": {"p99": 0.001, "max": 0.002}}
    ground.update(changes)
    return {"ground": ground}


@pytest.mark.parametrize("changes,failure", [
    ({"snapshots": 0}, "policy_ground:no_snapshots"),
    ({"non_finite": 1}, "policy_ground:non_finite_scores"),
    ({"not_repeatable": 1}, "policy_ground:not_bit_exact"),
    ({"mask_mismatches": 1}, "policy_ground:mask_differs_from_p4"),
    ({"decision_mismatches": 1}, "policy_ground:decision_differs_from_p4"),
    ({"policy_invocations": 1, "policy_failures": 1}, "policy_ground:policy_failure"),
    ({"policy_invocations": 1, "unsafe_policy_moves": 1}, "policy_ground:unsafe_policy_move"),
    ({"actions": {"SCAN": 4, "TRANSLATE": 1}}, "policy_ground:move_without_progress_candidate"),
    ({"inference_s": {"p99": 0.006, "max": 0.006}}, "policy_ground:inference_p99"),
    ({"inference_s": {"p99": 0.001, "max": 0.011}}, "policy_ground:inference_max")])
def test_ground_policy_rules(changes, failure):
    f = Failures()
    p5_evaluate.policy_ground(ground_replayed(**changes), LIMITS, f)
    assert failure in f.items, f.items


@needs_artifact
def test_ground_evaluation_rejects_a_planted_policy_translation(tmp_path, monkeypatch):
    """A decision step that translated without any allowed candidate must be caught (mask or decision rule)."""
    from gwm_policy_control import decision
    run = configs(tmp_path)
    write_jsonl(run / "p4/local_map.jsonl", fixture_rows("G4"))
    real = decision.plan_with_ranker

    def planted(*args, **kwargs):
        out = real(*args, **kwargs)
        return dict(out, action="TRANSLATE", heading=0.0, length=0.75, end=(0.75, 0.0))
    monkeypatch.setattr(p5_policy_eval.decision, "plan_with_ranker", planted)
    replayed = p5_policy_eval.evaluate(run, "fixture", ARTIFACT, SHA, SCHEMA_PATH, (10.0, 20.0))
    f = Failures()
    p5_evaluate.policy_ground(replayed, LIMITS, f)
    assert {"policy_ground:decision_differs_from_p4", "policy_ground:move_without_progress_candidate"} <= set(f.items)


# ---- Gate A item 5: flight-snapshot fault replay on accepted P4 S2-style evidence ------------------------------
def avoidance_layout():
    """A held-out layout P4 completed with a box near the straight path (never an acceptance scenario)."""
    report = json.loads(ARTIFACT.with_name(ARTIFACT.stem + ".readiness_report.json").read_text(encoding="utf-8"))
    layouts = {l["id"]: l for l in report["layouts"]}
    for e in report["episodes"]:
        boxes = layouts[e["id"]]["boxes"]
        if e["p4"]["outcome"] == "goal_reached" and len(boxes) == 1 and abs(boxes[0]["center"][1]) < 0.4 \
                and 2.5 < boxes[0]["center"][0] < 6.0 and max(boxes[0]["size"]) < 0.6:
            return layouts[e["id"]]
    raise AssertionError("no held-out avoidance layout")


class RecordingRig(rig_module.Rig):
    """The P4 mission rig recording what the node would: samples, decision events and local-map rows."""

    def __init__(self, boxes):
        self.events, self.maps = [], []
        super().__init__(rig_module.p4_mission_factory, boxes)
        self.events.append({"event": "transition", "to": "PRESTREAM_SAFE_SETPOINTS", "ros_sim_s": self.sim})

    def perceive(self):
        super().perceive()
        self.maps.append({"usable": True, "acquisition_sim_s": self.sim, "processing_sim_s": self.sim,
                          "grid": self.grid.encode(), "calibration_identity": "cal"})

    def step(self):
        before = len(self.m.events)
        actions = super().step()
        if actions.get("sample") is not None:
            self.events.append({"event": "sample", "ros_sim_s": self.sim, "sample": actions["sample"]})
        self.events += [dict(e, ros_sim_s=self.sim) for e in self.m.events[before:]]
        return actions


@pytest.fixture(scope="module")
def p4_s2_evidence(tmp_path_factory):
    """A synthetic accepted-P4-style flight run plus its accepted gate result (the real gate uses the retained
    c110bd30 S2 runs; the same code path)."""
    if ARTIFACT is None:
        pytest.skip("no exported P5 model artifact")
    rig = RecordingRig(avoidance_layout()["boxes"])
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    rig.run_until(lambda a: rig.m.state == "GOAL_HOVER", 100.0)
    root = tmp_path_factory.mktemp("p4root")
    run = root / "runs/20261030T000000Z-p4-flight-S2-synthetic"
    (run / "p4").mkdir(parents=True)
    (run / "p4_avoidance.yaml").write_text(json.dumps(CONFIG))
    (run / "controller-result.json").write_text(json.dumps({"p4": {"origin_enu": [0.0, 0.0]}}))
    write_jsonl(run / "ros-events.jsonl", rig.events)
    write_jsonl(run / "p4/local_map.jsonl", rig.maps)
    (run / "p4-evaluation.json").write_text('{"status": "passed"}')
    (root / "state").mkdir()
    result = root / ("state/p4-gate-B-%s.result.json" % V5["policy_replay"]["p4_manifest_sha256"])
    result.write_text(json.dumps({"accepted": True, "manifest_sha256": V5["policy_replay"]["p4_manifest_sha256"],
                                  "runs": [{"role": "smoke", "run": str(run), "evaluation_status": "passed",
                                            "evaluation_sha256": hashlib.sha256(b'{"status": "passed"}').hexdigest()}]}))
    return root, run, result


def s2_sources(root, run, result):
    return [{"gate": "B", "role": "smoke", "run": run.name, "run_dir": str(run), "gate_result": str(result),
             "p4_manifest_sha256": V5["policy_replay"]["p4_manifest_sha256"]}]


def replay_sources(sources):
    return p5_policy_eval.flight_snapshot_replay(sources, rig_module.trained_policy(), p5.SCHEMA, ARTIFACT, p5.SCHEMA_SHA,
                                                 P5C["inference"], V5["policy_replay"]["avoidance_states"])


@needs_artifact
def test_flight_snapshot_replay_injects_every_fault_on_an_exactly_reproduced_avoidance_tick(p4_s2_evidence):
    out = replay_sources(s2_sources(*p4_s2_evidence))
    selected = out["selected"]
    assert out["sources"][0]["accepted"] and selected is not None
    assert selected["state"] in V5["policy_replay"]["avoidance_states"] and selected["progress_candidates"] > 0
    assert selected["mask_equals_p4_progress"] and selected["true_policy"]["failure"] is None
    assert selected["true_policy"]["inside_mask"] and selected["true_policy"]["finite"]
    cases = out["cases"]
    assert set(cases) == set(V5["policy_replay"]["failure_cases"]) and all(c["passed"] for c in cases.values()), cases
    for case in cases.values():
        assert case["states"] == [selected["state"]] * 2 + ["ABORT"] and case["actions"] == ["HOLD"] * 3
        assert case["reasons"][-1] == "policy_unavailable"
    assert cases["hash_mismatch"]["load_refusal"] == "checkpoint_sha256_mismatch"
    summary = {"replay_sources": [dict(s, files_sha256=out["sources"][0]["files_sha256"]) for s in s2_sources(*p4_s2_evidence)]}
    f = Failures()
    p5_evaluate.flight_snapshot({"flight_snapshot": out}, summary, V5["policy_replay"]["failure_cases"], f)
    assert not f.items, f.items


def edited_copy(p4_s2_evidence, tmp_path, edit):
    root, run, result = p4_s2_evidence
    copy = tmp_path / "runs" / run.name
    shutil.copytree(run, copy)
    edit(copy)
    gate = json.loads(result.read_text())
    gate["runs"][0]["run"] = str(copy)
    (tmp_path / "result.json").write_text(json.dumps(gate))
    return s2_sources(tmp_path, copy, tmp_path / "result.json")


@needs_artifact
def test_selection_requires_exact_reproduction_of_the_recorded_p4_plan(p4_s2_evidence, tmp_path):
    first = replay_sources(s2_sources(*p4_s2_evidence))["selected"]

    def falsify(run):  # the recorded plan of the first eligible tick no longer matches what P4 reproduces
        events = read_jsonl(run / "ros-events.jsonl")
        tick = next(e for e in events if e.get("event") == "p4_decision" and e["sim_s"] == first["sim_s"])
        tick["plan"]["candidates_progress"] += 1
        write_jsonl(run / "ros-events.jsonl", events)
    selected = replay_sources(edited_copy(p4_s2_evidence, tmp_path, falsify))["selected"]
    assert selected is not None and selected["sim_s"] > first["sim_s"]


@needs_artifact
def test_evidence_without_a_reproducible_valid_candidate_tick_is_a_reported_limitation(p4_s2_evidence, tmp_path):
    def no_maps(run):
        write_jsonl(run / "p4/local_map.jsonl", [])
    sources = edited_copy(p4_s2_evidence, tmp_path, no_maps)
    out = replay_sources(sources)
    assert out["selected"] is None and out["cases"] is None
    f = Failures()
    summary = {"replay_sources": [dict(s, files_sha256=out["sources"][0]["files_sha256"]) for s in sources]}
    p5_evaluate.flight_snapshot({"flight_snapshot": out}, summary, V5["policy_replay"]["failure_cases"], f)
    assert "policy_replay:no_reproducible_valid_candidate_snapshot" in f.items
    assert all("policy_replay:" + c in f.items for c in V5["policy_replay"]["failure_cases"])


@needs_artifact
def test_a_changed_or_unaccepted_source_is_rejected(p4_s2_evidence, tmp_path):
    def tamper(run):
        (run / "p4-evaluation.json").write_text('{"status": "passed", "edited": true}')
    out = replay_sources(edited_copy(p4_s2_evidence, tmp_path, tamper))
    assert out["sources"][0]["accepted"] is False and out["selected"] is None
    f = Failures()
    summary = {"replay_sources": [{"run": out["sources"][0]["run"], "files_sha256": {"p4-evaluation.json": "0" * 64}}]}
    p5_evaluate.flight_snapshot({"flight_snapshot": out}, summary, V5["policy_replay"]["failure_cases"], f)
    run_name = out["sources"][0]["run"]
    assert {"policy_replay:source_not_accepted:" + run_name, "policy_replay:source_changed:" + run_name} <= set(f.items)


@pytest.mark.parametrize("change,failure", [
    ("mask", "policy_replay:mask_differs_from_p4"),
    ("true_policy", "policy_replay:true_policy"),
    ("case", "policy_replay:selection_outside_mask")])
def test_flight_snapshot_rules(change, failure):
    cases = {c: {"passed": True} for c in V5["policy_replay"]["failure_cases"]}
    selected = {"progress_candidates": 25, "mask_equals_p4_progress": change != "mask",
                "true_policy": {"failure": "timeout" if change == "true_policy" else None, "finite": True, "inside_mask": True}}
    if change == "case":
        cases["selection_outside_mask"]["passed"] = False
    fs = {"sources": [{"run": "r", "accepted": True, "files_sha256": {"a": "1"}}], "selected": selected, "cases": cases}
    f = Failures()
    p5_evaluate.flight_snapshot({"flight_snapshot": fs}, {"replay_sources": [{"run": "r", "files_sha256": {"a": "1"}}]},
                                V5["policy_replay"]["failure_cases"], f)
    assert f.items == [failure]


def test_policy_replay_module_has_no_publication_path():
    text = (p5.P5 / "validation/p5_policy_eval.py").read_text(encoding="utf-8")
    assert "rclpy" not in text and "px4_msgs" not in text and "create_publisher" not in text and "gwm_px4_control.contracts" not in text


def test_offline_replay_row_declares_read_only_sources_and_starts_nothing(tmp_path):
    root = tmp_path
    for source in V5["policy_replay"]["sources"]:
        run = root / "runs" / source["run"]
        (run / "p4").mkdir(parents=True)
        for name in ("p4-evaluation.json", "controller-result.json", "ros-events.jsonl", "p4/local_map.jsonl", "p4_avoidance.yaml"):
            (run / name).write_text(name)
    (root / "auth.json").write_text("{}")
    frozen = {"model": {"sha256": "m", "path": "models/x.npz"}, "schema_sha256": "s",
              "controller_runtime": {"python": "/usr/bin/python3", "numpy": "1.26.4"}}
    run = run_p5_gates.prepare_offline_replay(root, frozen, "f" * 64, "A", "policy-failure-replay", "S2", root / "auth.json")
    summary = json.loads((run / "summary.json").read_text())
    assert summary["p5_scope"] == "flight_snapshot_replay" and summary["simulator_started"] is False
    assert summary["px4_commands_possible"] is False and summary["controller_runtime"]["openblas_num_threads"] == "1"
    assert [s["run"] for s in summary["replay_sources"]] == [s["run"] for s in V5["policy_replay"]["sources"]]
    first = summary["replay_sources"][0]
    assert first["files_sha256"]["ros-events.jsonl"] == hashlib.sha256(b"ros-events.jsonl").hexdigest()
    assert first["gate_result"].endswith("p4-gate-B-%s.result.json" % V5["policy_replay"]["p4_manifest_sha256"])
    assert sorted(p.name for p in run.iterdir()) == ["p5_model_schema.json", "p5_policy.yaml", "p5_validation.yaml", "summary.json"]


# ---- Gate A policy fault: the live start with a mismatched checkpoint ------------------------------------------
def fault_run(tmp_path, *, exit_code=1, log="PolicyError: checkpoint_sha256_mismatch", started=(), kind="checkpoint_mismatch"):
    bad = tmp_path / "p5/mismatched-checkpoint.npz"
    bad.parent.mkdir(parents=True)
    bad.write_bytes(b"model bytes plus one")
    (tmp_path / "ros-controller.log").write_text("Traceback (most recent call last):\n" + log + "\n")
    for name in started:
        (tmp_path / name).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / name).write_text("{}")
    summary = {"controller_exit_code": exit_code, "p5_model": {"sha256": "a" * 64},
               "policy_fault": {"kind": kind, "model_path": str(bad), "expected_sha256": "a" * 64,
                                "actual_sha256": hashlib.sha256(bad.read_bytes()).hexdigest()}}
    return tmp_path, summary


@pytest.mark.parametrize("kwargs,failure", [
    ({}, None),
    ({"exit_code": 0}, "policy_fault:controller_did_not_refuse"),
    ({"log": "ValueError: something else"}, "policy_fault:refusal_reason"),
    ({"started": ("ros-events.jsonl",)}, "policy_fault:controller_started"),
    ({"started": ("controller-result.json",)}, "policy_fault:controller_started"),
    ({"kind": "other"}, "policy_fault:not_a_checkpoint_mismatch")])
def test_mismatched_checkpoint_must_be_refused_before_any_node(tmp_path, kwargs, failure):
    run, summary = fault_run(tmp_path, **kwargs)
    f = Failures()
    out = p5_evaluate.policy_fault(run, summary, V5["policy_fault"]["checkpoint_mismatch"]["refusal"], f)
    assert (failure in f.items) if failure else not f.items, f.items
    assert out["refusal_found"] == ("checkpoint_sha256_mismatch" in kwargs.get("log", "checkpoint_sha256_mismatch"))


# ---- composition: the reused P4 checks still apply --------------------------------------------------------------
@pytest.fixture
def composed(monkeypatch, tmp_path):
    """Every heavy P3/P4 evaluator function replaced by a recorder; the P5 composition is real."""
    import p3_evaluate
    calls = []

    def recorder(name, result=None, failure=None):
        def call(*args, **kwargs):
            calls.append(name)
            f = next((a for a in args if isinstance(a, Failures)), None)
            if failure and f is not None:
                f.items.append(failure)
            return result() if callable(result) else result
        return call
    monkeypatch.setattr(p3_evaluate, "configs", lambda: ({"rgb_absence_monitor": {}}, {"rgb_sensor": {}}))
    cleanup = []  # cleanup failures the P3 common check reports in a test
    monkeypatch.setattr(p3_evaluate, "common", lambda run, summary, f, out: calls.append("p3.common") or f.items.extend(cleanup))
    monkeypatch.setattr(p5_evaluate, "policy_fault", recorder("p5.policy_fault", {}))
    monkeypatch.setattr(p5_evaluate, "policy_fault_native_commands", recorder("p5.policy_fault_native_commands"))
    monkeypatch.setattr(p3_evaluate, "mandatory_artifacts", recorder("p3.mandatory_artifacts"))
    monkeypatch.setattr(p5_evaluate, "evaluate_stream", recorder("p3.evaluate_stream", {"status": "passed", "failures": []}))
    monkeypatch.setattr(p5_evaluate, "evaluate_rgb", recorder("p3.evaluate_rgb", {"failures": []}))
    monkeypatch.setattr(p5_evaluate, "evaluate_rgb_absence", recorder("p3.evaluate_rgb_absence", {"failures": []}))
    monkeypatch.setattr(p5_evaluate, "native_rtf", recorder("p3.native_rtf", 1.0))
    monkeypatch.setattr(p4_evaluate, "performance", recorder("p4.performance", {}))
    monkeypatch.setattr(p4_evaluate, "p2_integrity", recorder("p4.p2_integrity", ({"timing": {"status": "passed"}}, None, None, None)))
    monkeypatch.setattr(p4_evaluate, "native_commands", recorder("p4.native_commands"))
    monkeypatch.setattr(p4_evaluate, "mc", recorder("p4.mc"))
    monkeypatch.setattr(p4_evaluate, "safety", recorder("p4.safety", ({}, (0.0, 0.0), []), failure="safety:truth_clearance"))
    monkeypatch.setattr(p4_evaluate, "response_latency", recorder("p4.response_latency", 0.05))
    monkeypatch.setattr(p4_evaluate, "outcome", recorder("p4.outcome", {"label": "goal_reached_with_detour"}))
    monkeypatch.setattr(p4_evaluate, "truth_track", lambda run: [])
    monkeypatch.setattr(p4_evaluate, "fixture", recorder("p4.fixture", {}))
    monkeypatch.setattr(p5_evaluate, "checkpoint_identity", recorder("p5.checkpoint_identity", {}))
    ledger = {"failures": [], "records": 10, "invoked": 10, "failure_ticks": 0, "outside_mask": 0, "fallback_decisions": 0,
              "participation_fraction": 1.0, "inference_s": {"p99": 0.001, "max": 0.002}}
    monkeypatch.setattr(p5_evaluate, "replay", lambda run, summary, mode, window=None: calls.append("p5.replay:" + mode) or {
        "numpy": "1.26.4", "model_sha256": "m", "schema_sha256": "s", "ledger": ledger,
        **ground_replayed()})
    configs(tmp_path)
    (tmp_path / "p4").mkdir(exist_ok=True)
    write_jsonl(tmp_path / "p5/policy-ledger.jsonl", [])
    write_jsonl(tmp_path / "ros-events.jsonl", [])

    def evaluate(scope, **summary_changes):
        summary = {"p5_scope": scope, "status": "passed", "sensor_window_sim_s": [10.0, 30.0],
                   "p3_baseline": {"sha256": V5["p3_baseline_manifest_sha256"]},
                   "p4_baseline": {"sha256": V5["p4_baseline_manifest_sha256"]},
                   "p5_model": {"sha256": "m", "schema_sha256": "s"}, "controller_runtime": {"numpy": "1.26.4"},
                   "renderer": {"identified": True, "software": False,
                                "strings": [json.loads((syn.SIM / "configs/p3_d3d12_profile.json").read_text())["renderer_identity"]]},
                   "controller_publishers_only_from_controller": True, "processes": [],
                   "controller_result": {"controller": "p5_control", "transitions": [
                       {"to": "PRESTREAM_SAFE_SETPOINTS", "monotonic_s": 1.0}, {"to": "COMPLETE", "monotonic_s": 2.0}],
                       "p5": {"numpy": "1.26.4"}}, **summary_changes}
        (tmp_path / "summary.json").write_text(json.dumps(summary))
        if scope == "flight":
            (tmp_path / "p4/scenario.json").write_text(json.dumps({"scenario": "S2", "boxes": [], "outcome": "goal_reached_with_detour"}))
        calls.clear()
        return p5_evaluate.evaluate(tmp_path), list(calls)
    evaluate.cleanup = cleanup
    return evaluate


def test_flight_evaluation_applies_every_reused_p4_check_and_the_p5_checks(composed):
    out, calls = composed("flight")
    for name in ("p3.common", "p3.mandatory_artifacts", "p3.evaluate_stream", "p3.evaluate_rgb_absence", "p3.native_rtf",
                 "p4.performance", "p4.p2_integrity", "p4.native_commands", "p4.mc", "p4.safety", "p4.response_latency",
                 "p4.outcome", "p5.checkpoint_identity", "p5.replay:flight"):
        assert name in calls, name
    assert out["failures"] == ["safety:truth_clearance"] and out["status"] == "failed"  # a P4 failure still fails P5


def test_controller_identity_must_be_p5_control(composed):
    out, _ = composed("flight", controller_result={"controller": "p4_control", "transitions": [
        {"to": "PRESTREAM_SAFE_SETPOINTS", "monotonic_s": 1.0}, {"to": "COMPLETE", "monotonic_s": 2.0}]})
    assert "controller_not_p5" in out["failures"]


def test_fixture_evaluation_applies_p4_fixture_and_the_policy_replays(composed):
    out, calls = composed("fixture")
    assert {"p4.fixture", "p3.evaluate_rgb", "p5.replay:fixture", "p4.p2_integrity"} <= set(calls)
    assert "p4.safety" not in calls and out["failures"] == [] and out["status"] == "passed"


def test_the_refusal_exit_is_accepted_only_in_the_declared_policy_fault_scope(composed):
    composed.cleanup[:] = [p5_evaluate.REFUSED_CONTROLLER_CLEANUP]
    out, calls = composed("policy_fault")
    assert out["failures"] == [] and {"p5.policy_fault", "p5.policy_fault_native_commands"} <= set(calls)
    for scope in ("p5_readiness", "fixture", "flight"):  # the same exit anywhere else still fails
        out, _ = composed(scope)
        assert p5_evaluate.REFUSED_CONTROLLER_CLEANUP in out["failures"], scope
    composed.cleanup[:] = [p5_evaluate.REFUSED_CONTROLLER_CLEANUP, "cleanup:sensor-bridge:forced_kill_after_grace"]
    out, _ = composed("policy_fault")
    assert out["failures"] == ["cleanup:sensor-bridge:forced_kill_after_grace"]  # other violations still fail
    composed.cleanup[:] = []
    out, _ = composed("policy_fault")
    assert out["failures"] == ["policy_fault:refusal_exit_not_recorded_by_cleanup"]


def test_baselines_must_be_the_accepted_p3_and_p4(composed):
    out, _ = composed("p5_readiness", p4_baseline={"sha256": "0" * 64})
    assert out["failures"] == ["p4_baseline_mismatch"]


# ---- runner, gates and manifest bookkeeping -------------------------------------------------------------------
def source_of(path, qualname):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    names = qualname.split(".")
    nodes = tree.body
    for name in names:
        node = next(n for n in nodes if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name == name)
        nodes = getattr(node, "body", [])
    return [line.strip() for line in ast.get_source_segment(Path(path).read_text(encoding="utf-8"), node).splitlines()[1:]
            if line.strip() and not line.strip().startswith(("#", '"""'))]


def test_runner_preconditions_and_qgc_receipt_differ_from_p4_only_where_declared():
    p4_runner, p5_runner = syn.P4 / "scripts/p4_runner.py", p5.P5 / "scripts/p5_runner.py"
    p4_lines, p5_lines = source_of(p4_runner, "P4Runner.preconditions"), source_of(p5_runner, "P5Runner.preconditions")
    assert [l for l in p4_lines if l not in p5_lines] == [
        'flight = self.p4_scope == "flight"',
        'raise ValueError("Exclusive P1/P2/P3/P4 workspace lock is not held by this owner")',
        'frozen, sha = p4_manifest.verify_current(self.root, self.args.manifest)  # includes the P3 baseline check',
        'authorization = p4_manifest.require_authorization(self.args.authorization, self.args.gate, sha)',
        'marker = p4_manifest.marker_path(self.root, self.args.gate, sha)',
        'raise PermissionError("Gate authorization was not consumed by the P4 gate sequencer")',
        'if flight != (self.args.gate in p4_manifest.FLIGHT_GATES):',
        'raise PermissionError("Flights belong to P4 Gates B, S and C only")',
        'raise PermissionError("P4 gates run only on the WSL D3D12 host")']
    assert len(p4_lines) == len(p5_lines)
    assert [l.replace("signal.SIGTERM", "SIGTERM") for l in source_of(p4_runner, "P4Runner.stop_qgc")] == \
        [l.replace("signal.SIGTERM", "SIGTERM") for l in source_of(p5_runner, "P5Runner.stop_qgc")]


def test_gate_sequencer_differs_from_p4_only_in_names_and_the_bound_identity():
    p4_lines = source_of(syn.P4 / "scripts/run_p4_gates.py", "main")
    p5_lines = source_of(p5.P5 / "scripts/run_p5_gates.py", "main")
    renamed = [l.replace("p4_manifest", "p5_manifest").replace("p4-gate-result-v1", "p5-gate-result-v1")
               .replace("P4 Gate", "P5 Gate").replace("This P4 gate", "This P5 gate")
               .replace("p4-evaluation.json", "p5-evaluation.json") for l in p4_lines]
    assert [l for l in renamed if l not in p5_lines] == [
        '"manifest_sha256": sha, "p3_baseline": frozen["p3_baseline"],']
    assert [l for l in p5_lines if l not in renamed] == [
        '"manifest_sha256": sha, "baselines": frozen["baselines"], "model_sha256": frozen["model"]["sha256"],',
        'if scope == OFFLINE_SCOPE:',  # revision 2: the offline flight-snapshot replay row starts no runtime
        'run = prepare_offline_replay(root, frozen, sha, args.gate, role, item, args.authorization)',
        'completed = subprocess.CompletedProcess([], 0, "P5 evidence: %s\\n" % run, "")']


def test_gate_plans_follow_the_contract():
    assert [r[0] for r in run_p5_gates.plan("A")] == ["p3_readiness", "p5_readiness", "fixture", "fixture", "fixture",
                                                       "flight_snapshot_replay", "policy_fault"]
    assert run_p5_gates.plan("B") == [("flight", "S2", "smoke")]
    assert [r[1] for r in run_p5_gates.plan("S")] == ["S1", "S5", "S6"]
    assert [r[1] for r in run_p5_gates.plan("C")] == ["S2", "S3", "S4"]
    assert p5_manifest.FLIGHT_GATES == ("B", "S", "C") and V5["predecessor"] == {"B": "A", "S": "B", "C": "S"}
    command = run_p5_gates.runtime_command("policy_fault", "checkpoint_mismatch", "r", "A", Path("/m"), Path("/a"))
    assert command[-2:] == ["--fixture", "checkpoint_mismatch"] and command[1].endswith("run_p5.sh")
    assert run_p5_gates.runtime_command("flight", "S6", "s", "S", Path("/m"), Path("/a"))[-3:] == \
        ["--scenario", "S6", "--allow-simulated-flight"]


def test_p5_authorization_binds_gate_scope_and_manifest_and_p4_authorizations_never_count(tmp_path):
    sha = "f" * 64
    path = tmp_path / "auth.json"
    path.write_text(json.dumps(p5_manifest.authorization_template("S", sha, "op")))
    assert p5_manifest.require_authorization(path, "S", sha)["authorized_by"] == "op"
    for gate, manifest in (("C", sha), ("S", "0" * 64)):
        with pytest.raises(PermissionError):
            p5_manifest.require_authorization(path, gate, manifest)
    import p4_manifest
    path.write_text(json.dumps(p4_manifest.authorization_template("S", sha, "op")))
    with pytest.raises(PermissionError):
        p5_manifest.require_authorization(path, "S", sha)


def test_predecessor_and_consumption_are_permanent(tmp_path):
    (tmp_path / "state").mkdir()
    auth = tmp_path / "auth.json"
    auth.write_text("{}")
    marker = p5_manifest.consume(tmp_path, "A", "a" * 64, auth)
    with pytest.raises(FileExistsError):
        p5_manifest.consume(tmp_path, "A", "a" * 64, auth)
    assert marker.name == "p5-gate-A-" + "a" * 64 + ".consumed.json"
    with pytest.raises(PermissionError, match="requires an accepted Gate A"):
        run_p5_gates.require_predecessor(tmp_path, "B", "a" * 64)
    run = tmp_path / "run"
    run.mkdir()
    (run / "p5-evaluation.json").write_text("{}")
    rows = [{"run": str(run), "evaluation_sha256": hashlib.sha256(b"{}").hexdigest()}]
    run_p5_gates.result_path(tmp_path, "A", "a" * 64).write_text(json.dumps({"accepted": True, "manifest_sha256": "a" * 64, "runs": rows}))
    assert run_p5_gates.require_predecessor(tmp_path, "B", "a" * 64)["sha256"]
    (run / "p5-evaluation.json").write_text('{"tampered": 1}')
    with pytest.raises(PermissionError, match="changed after acceptance"):
        run_p5_gates.require_predecessor(tmp_path, "B", "a" * 64)


@needs_artifact
def test_manifest_model_identity_binds_the_checkpoint_and_its_reports(tmp_path, monkeypatch):
    identity = p5_manifest.model_identity(SHA)
    assert identity["sha256"] == SHA and identity["trained"] is True and identity["schema_sha256"] == p5.SCHEMA_SHA
    assert identity["path"] == "models/" + ARTIFACT.name
    copy = tmp_path / "p5"
    shutil.copytree(p5.P5 / "configs", copy / "configs")
    shutil.copytree(p5.P5 / "models", copy / "models")
    report = copy / "models" / (ARTIFACT.stem + ".readiness_report.json")
    report.write_bytes(report.read_bytes() + b" ")
    monkeypatch.setattr(p5_manifest, "P5", copy)
    with pytest.raises(ValueError, match="readiness_report"):
        p5_manifest.model_identity(SHA)


def test_controller_runtime_reports_the_interpreters_numpy():
    assert p5_manifest.controller_runtime(sys.executable) == {"python": sys.executable, "numpy": np.__version__}


def test_namespace_owner_is_byte_identical_and_the_shim_dispatches_to_p5():
    owner = (p5.P5 / "scripts/p5_namespace_owner.py").read_bytes()
    assert hashlib.sha256(owner).digest() == hashlib.sha256((syn.SIM / "scripts/p3_namespace_owner.py").read_bytes()).digest()
    assert "from p5_runner import main" in (p5.P5 / "scripts/p3_runner.py").read_text(encoding="utf-8")


@pytest.fixture
def runner_module():
    if sys.platform == "win32":
        pytest.skip("the accepted P3 runner imports POSIX-only modules")
    import p5_runner
    return p5_runner


def runner_with(runner_module, tmp_path, scope, fixture=None, scenario=None):
    (tmp_path / "state").mkdir(parents=True, exist_ok=True)
    (tmp_path / "state/p5-built.json").write_text(json.dumps({"install": str(tmp_path / "install")}))
    runner = object.__new__(runner_module.P5Runner)
    runner.root, runner.p5_scope, runner.marker, runner.policy_fault = tmp_path, scope, tmp_path / "marker", None
    runner.args = SimpleNamespace(fixture=fixture, scenario=scenario)
    runner.frozen = {"model": {"path": "models/" + ARTIFACT.name, "sha256": SHA}, "schema_sha256": p5.SCHEMA_SHA}
    return runner


@needs_artifact
@pytest.mark.parametrize("scope,fixture,mode", [("p5_readiness", None, "--ground-diagnostic"), ("fixture", "G4", "--ground-diagnostic"),
                                                ("policy_fault", "checkpoint_mismatch", "--ground-diagnostic"),
                                                ("flight", None, "--flight")])
def test_runner_prepares_p5_configs_and_the_p5_controller_command(runner_module, tmp_path, scope, fixture, mode):
    from gwm_px4_control.contracts import gates
    runner = runner_with(runner_module, tmp_path, scope, fixture=fixture, scenario="S2" if scope == "flight" else None)
    runner.prepare_run(runner.frozen)
    local = json.loads((runner.run / "p5_policy.yaml").read_text())
    assert local["enabled"] is True and local["model"]["sha256"] == SHA and Path(local["model"]["path"]) == ARTIFACT
    assert json.loads((runner.run / "p4_avoidance.yaml").read_text())["enabled"] is True
    assert (runner.run / "p5_model_schema.json").read_bytes() == SCHEMA_PATH.read_bytes()
    base_env = {"GWM_ALLOW_OPTIONAL_RUNTIME": "1", "GWM_RUN_GAZEBO_PX4_TESTS": "1", "GWM_ALLOW_PX4_LAUNCH": "1"}
    command, env = runner.p4_controller_command(dict(base_env))
    gates(env, flight=True)
    assert env["OPENBLAS_NUM_THREADS"] == "1" and env["GWM_P4_OWNED_NAMESPACE"] == "1"
    assert command[0].endswith("gwm_policy_control/lib/gwm_policy_control/p5_control") and command[-1] == mode
    model = command[command.index("--model") + 1]
    assert command[command.index("--model-sha256") + 1] == SHA
    if scope == "policy_fault":
        fault = runner.policy_fault
        assert model == fault["model_path"] and Path(model).read_bytes() == ARTIFACT.read_bytes() + b"\0"
        assert fault["actual_sha256"] != SHA and fault["expected_refusal"] == "checkpoint_sha256_mismatch"
    else:
        assert Path(model) == ARTIFACT


@needs_artifact
def test_policy_fault_scope_passes_only_on_a_refusal(runner_module, tmp_path):
    for refused in (True, False):
        runner = runner_with(runner_module, tmp_path / str(refused), "policy_fault", fixture="checkpoint_mismatch")
        runner.prepare_run(runner.frozen)
        (runner.run / "ros-controller.log").write_text("PolicyError: checkpoint_sha256_mismatch\n" if refused else "started\n")
        runner.controller = SimpleNamespace(poll=lambda: 1, record={"exit_code": 1})
        clock = iter(range(100, 10_000))
        runner.summary, runner.start, runner.pump, runner.save = {}, 0.0, lambda: None, lambda: None
        runner.sim_now = lambda: float(next(clock))
        runner.p4_validation = json.loads((syn.P4 / "configs/p4_validation.yaml").read_text())
        if refused:
            runner.run_controller({"wall_deadline_s": 1e9})
            assert runner.summary["status"] == "passed" and runner.summary["policy_fault"]["refusal_observed"] is True
        else:
            with pytest.raises(runner_module.base.Abort, match="not_refused"):
                runner.run_controller({"wall_deadline_s": 1e9})


# ---- M7: package build receipt and freeze / verify / authorize tooling -------------------------------------
import p5_build  # noqa: E402


def test_package_manifest_covers_the_whole_package_and_its_hash_tracks_content(tmp_path, monkeypatch):
    manifest = p5_build.package_manifest()
    assert {"setup.py", "package.xml", "test/test_installed.py", "gwm_policy_control/control_node.py",
            "gwm_policy_control/policy_ledger.py"} <= set(manifest)
    assert not any("__pycache__" in name for name in manifest)
    copy = tmp_path / "gwm_policy_control"
    shutil.copytree(p5_build.PACKAGE, copy, ignore=shutil.ignore_patterns("__pycache__"))
    monkeypatch.setattr(p5_build, "PACKAGE", copy)
    assert p5_build.package_hash(p5_build.package_manifest()) == p5_build.package_hash(manifest)
    (copy / "gwm_policy_control/mission.py").write_text("# changed\n")
    assert p5_build.package_hash(p5_build.package_manifest()) != p5_build.package_hash(manifest)


def test_manifest_binds_the_build_receipt_of_the_current_source(tmp_path):
    (tmp_path / "state").mkdir()
    receipt = {"contract": p5_build.CONTRACT, "package_hash": p5_build.package_hash(p5_build.package_manifest())}
    (tmp_path / "state/p5-built.json").write_text(json.dumps(receipt))
    assert p5_manifest.p5_build(tmp_path)["package_hash"] == receipt["package_hash"]
    (tmp_path / "state/p5-built.json").write_text(json.dumps(dict(receipt, package_hash="0" * 64)))
    with pytest.raises(ValueError, match="build receipt"):
        p5_manifest.p5_build(tmp_path)


def test_freeze_is_exclusive_and_verify_names_every_differing_input(tmp_path, monkeypatch):
    value = {"contract": "p5-candidate-manifest-v1", "model": {"sha256": "m"}, "p5_build": {"package_hash": "b"},
             "runtime": {"native_command_policy": {"name": "A", "approval_path": "/a"}, "mc_benchmark": {"run": "/r"}}}
    monkeypatch.setattr(p5_manifest, "candidate", lambda root, model, **kw: json.loads(json.dumps(value)))
    output = tmp_path / "p5-candidate.json"
    sha = p5_manifest.freeze(tmp_path, output, "m", policy_name="A")
    with pytest.raises(FileExistsError):
        p5_manifest.freeze(tmp_path, output, "m", policy_name="A")
    assert p5_manifest.verify_current(tmp_path, output)[1] == sha
    value["p5_build"] = {"package_hash": "changed"}
    with pytest.raises(ValueError, match="p5_build"):
        p5_manifest.verify_current(tmp_path, output)


def test_authorize_requires_the_exact_confirmation_phrase(tmp_path, monkeypatch):
    sha = "e" * 64
    monkeypatch.setattr(p5_manifest, "verify_current", lambda root, manifest: ({}, sha))
    common = ["authorize", str(tmp_path / "m.json"), "--gate", "A", "--operator", "op", "--output", str(tmp_path / "auth.json")]
    with pytest.raises(SystemExit, match="Confirmation phrase"):
        p5_manifest.main(common + ["--confirm", "authorize P4 Gate A " + sha])
    assert not (tmp_path / "auth.json").exists()
    p5_manifest.main(common + ["--confirm", "authorize P5 Gate A " + sha])
    assert p5_manifest.require_authorization(tmp_path / "auth.json", "A", sha)["authorized_by"] == "op"
