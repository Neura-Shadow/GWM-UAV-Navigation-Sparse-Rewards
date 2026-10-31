"""P5 M6: P4 vs P5 comparison (plan §9) — the offline paired report and the SITL flight comparison, offline."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402

sys.path.insert(0, str(p5.P5 / "validation"))
import p5_comparison  # noqa: E402

REPORT = next(iter(sorted((p5.P5 / "models").glob("p5-ranker-*.readiness_report.json"))), None)
EVIDENCE = p5.syn.REPO / "docs/evidence/v3_sim_p5_offline_comparison.json"
needs_report = pytest.mark.skipif(REPORT is None, reason="no exported P5 readiness report")


@needs_report
def test_offline_comparison_is_paired_deterministic_and_matches_the_recorded_evidence():
    first, again = p5_comparison.offline(REPORT), p5_comparison.offline(REPORT)
    assert first == again and first["acceptance_evidence"] is False
    paired = first["paired"]
    assert (paired["both_reached"], paired["p5_only"], paired["p4_only"], paired["neither"]) == (210, 26, 5, 59)
    success = paired["goal_success_difference_points"]
    assert success["pairs"] == 300 and success["mean"] == pytest.approx(7.0)
    assert success["ci95"][0] < 7.0 < success["ci95"][1]
    assert paired["time_to_goal_difference_s"]["pairs"] == paired["path_difference_m"]["pairs"] == 210
    assert first["p4"]["safety"]["collisions"] == first["p5"]["safety"]["collisions"] == 0
    assert first["per_decision"]["failures"] == first["per_decision"]["outside_mask"] == 0
    assert sum(f["layouts"] for f in first["by_family"].values()) == 300
    assert json.loads(EVIDENCE.read_text(encoding="utf-8")) == json.loads(json.dumps(first))


def test_sign_test_and_bootstrap_interval():
    assert p5_comparison.sign_test(5, 5) == 1.0 and p5_comparison.sign_test(0, 0) is None
    assert p5_comparison.sign_test(10, 0) == pytest.approx(2 / 1024)
    rng = np.random.default_rng(0)
    assert p5_comparison.interval([0.3] * 20, rng) == [pytest.approx(0.3), pytest.approx(0.3)]
    assert p5_comparison.interval([], rng) is None


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


def flight(tmp_path, name, controller, evaluation_name, lateral, goal_at, scenario="S2", clearance=0.6):
    run = tmp_path / name
    xs = np.linspace(0.0, 8.0, 200)
    write_jsonl(run / "p4/truth_poses.jsonl", [{"stamp_ns": 20_000_000_000 + k * 40_000_000,
                                                "poses": {"x500_depth_71": {"position": [x, lateral * math.exp(-((x - 4.0) / 1.2) ** 2), 2.0]}}}
                                               for k, x in enumerate(xs)])
    (run / "p4/scenario.json").write_text(json.dumps({"scenario": scenario}))
    write_jsonl(run / "ros-events.jsonl", [{"event": "p4_decision", "sim_s": 41.0, "snapshot_acquisition_sim_s": 40.95}])
    lineage = [{"to": t} for t in ("OBSTACLE_DETECTED", "AVOIDING", "RESUME", "NORMAL")]
    (run / "summary.json").write_text(json.dumps({"controller_result": {
        "controller": controller, "transitions": [{"to": "TRANSIT", "sim_s": 40.0}],
        "p4": {"goal_reached_sim_s": goal_at, "supervisor_transitions": lineage, "abort_reason": None}}}))
    evaluation = {"status": "passed", "outcome": {"label": "goal_reached_with_detour"}, "rtf": 0.9,
                  "safety": {"min_truth_clearance_m": clearance, "collisions": 0, "tracking_max_m": 0.24}}
    if controller == "p5_control":
        evaluation["policy"] = {"invoked": 900, "disagreements": 450, "disagreement_rate": 0.5, "interventions": {"none": 900},
                                "participation_fraction": 1.0, "inference_s": {"p99": 0.002, "max": 0.004}}
    (run / evaluation_name).write_text(json.dumps(evaluation))
    return run


def test_sitl_comparison_pairs_a_p5_flight_with_the_accepted_p4_flight(tmp_path):
    p5_run = flight(tmp_path, "p5", "p5_control", "p5-evaluation.json", 1.1, 82.0, clearance=0.55)
    p4_run = flight(tmp_path, "p4", "p4_control", "p4-evaluation.json", 1.3, 83.0)
    out = p5_comparison.sitl(p5_run, p4_run)
    assert out["acceptance_evidence"] is False and out["scenario"] == "S2" and out["safety"]["same_outcome"]
    assert out["safety"]["min_truth_clearance_difference_m"] == pytest.approx(-0.05)
    assert out["decision_quality"]["time_to_goal_difference_s"] == pytest.approx(-1.0)
    assert out["decision_quality"]["path_difference_m"] < 0 and out["decision_quality"]["same_lineage"]
    assert out["per_decision"]["disagreement_rate"] == 0.5 and out["p5"]["controller"] == "p5_control"


def test_sitl_comparison_refuses_different_scenarios(tmp_path):
    p5_run = flight(tmp_path, "p5", "p5_control", "p5-evaluation.json", 1.1, 82.0)
    p4_run = flight(tmp_path, "p4", "p4_control", "p4-evaluation.json", 1.3, 83.0, scenario="S3")
    with pytest.raises(ValueError, match="different_scenarios"):
        p5_comparison.sitl(p5_run, p4_run)


def test_accepted_p4_runs_lists_only_passed_flights_of_accepted_gates(tmp_path):
    (tmp_path / "state").mkdir()
    sha = "c" * 64
    (tmp_path / f"state/p4-gate-C-{sha}.result.json").write_text(json.dumps({"accepted": True, "runs": [
        {"scope": "flight", "item": "S2", "run": "/r/c1", "evaluation_status": "passed"},
        {"scope": "flight", "item": "S3", "run": "/r/c2", "evaluation_status": "failed"}]}))
    (tmp_path / f"state/p4-gate-B-{sha}.result.json").write_text(json.dumps({"accepted": False, "runs": [
        {"scope": "flight", "item": "S2", "run": "/r/b", "evaluation_status": "passed"}]}))
    assert p5_comparison.accepted_p4_runs(tmp_path, sha) == {"S2": ["/r/c1"]}
