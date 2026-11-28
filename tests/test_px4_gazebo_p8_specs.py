"""P8-M1 specification tests (contract `p8-navigation-recovery-v1` §4, §6, §7). Offline; never starts a simulator.

Run in WSL from /tmp: PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider THIS_FILE
The tests that read the accepted P7-E runs skip when the evidence root is absent. The full regeneration of V4 (the
1,650 P5 layouts and the held-out screen) is `p8_specs.py verify`, which is not repeated here.
"""
import copy
import json
import os
from pathlib import Path
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
P8 = REPO / "simulation/px4_gazebo_p8"
CONFIGS = P8 / "configs"
for _path in (P8 / "scripts", REPO / "simulation/px4_gazebo/validation", REPO / "simulation/px4_gazebo_p5/training"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import p5_layouts  # noqa: E402
import p8_geometry  # noqa: E402
import p8_specs  # noqa: E402
from p3_provenance import canonical_sha256, strict_json  # noqa: E402

ROOT = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
HAVE_RUNS = (ROOT / "state" / ("p7e-engineering-30-block-01-%s.result.json" % p8_specs.P7E_SHA)).exists()
needs_runs = pytest.mark.skipif(not HAVE_RUNS, reason="the accepted P7-E evidence root is not present")
V5_SLOTS = [1, 2, 4, 5, 8, 10, 11, 13, 17, 20]


@pytest.fixture(scope="module")
def specs():
    return {name: strict_json(CONFIGS / name) for name in ("p8_layouts.json", "p8_schedule.json", "p8_recovery.json",
                                                           "p8_report_plan.yaml")}


def stripped(boxes):
    return [{"center": list(b["center"]), "size": list(b["size"])} for b in boxes]


def test_v3_and_v5_are_the_ruled_p7e_layouts_with_their_frozen_records(specs):
    layouts = specs["p8_layouts.json"]
    p7 = strict_json(REPO / "simulation/px4_gazebo_p7/configs/p7_layouts.json")
    assert canonical_sha256(p7) == p8_specs.P7_LAYOUTS_CANONICAL
    v3, v5 = layouts["stages"]["V3"]["layouts"], layouts["stages"]["V5"]["layouts"]
    assert [r["p7e_slot"] for r in v3] == p8_specs.CONTRACT_V3_SLOTS
    assert [r["p7e_slot"] for r in v5] == V5_SLOTS
    assert not {r["id"] for r in v3} & {r["id"] for r in v5}
    for r in v3 + v5:
        frozen = p8_specs.p7_record(p7, r["id"])
        assert (r["boxes"], r["world_sha256"], r["seed"], r["family"]) == (frozen["boxes"], frozen["world_sha256"], frozen["seed"], frozen["family"])
    assert all(any(o != "goal_success" for o in r["p7e_outcomes"].values()) for r in v3)
    assert all(set(r["p7e_outcomes"].values()) == {"goal_success"} for r in v5)
    assert layouts["v5_rule_inputs"]["v5_quota"] == {"mixed": 3, "partial_wall": 1, "pillars": 3, "wall_with_gap": 3}


def test_every_v3_layout_falls_in_one_of_the_two_selection_types(specs):
    types = [p8_specs.layout_type(r) for r in specs["p8_layouts.json"]["stages"]["V3"]["layouts"]]
    assert types.count("F2") == 6 and types.count("F1") == 4  # the rule describes the observed failures


def test_v4_records_regenerate_from_their_seeds_and_satisfy_the_rule(specs):
    v4 = specs["p8_layouts.json"]["stages"]["V4"]
    p7 = strict_json(REPO / "simulation/px4_gazebo_p7/configs/p7_layouts.json")
    p7_boxes = [stripped(r["boxes"]) for name in ("pilot", "confirmatory") for r in p7["sets"][name]["layouts"]]
    seeds = [r["seed"] for r in v4["layouts"]]
    assert v4["seed_base"] == 6_000_000 and seeds == sorted(seeds) and len(set(seeds)) == 10
    assert all(6_000_000 <= s < 7_000_000 for s in seeds)
    assert [r["type"] for r in v4["layouts"]].count("F1") == 5 and [r["type"] for r in v4["layouts"]].count("F2") == 5
    for r in v4["layouts"]:
        generated, reason = p5_layouts._screen(r["seed"])
        assert reason is None and stripped(generated["boxes"]) == stripped(r["boxes"]) and generated["family"] == r["family"]
        assert [b["name"] for b in r["boxes"]] == ["p7_box_%d" % k for k in range(len(r["boxes"]))]
        assert r["id"] == "p8h-%07d" % r["seed"]
        assert r["clearance_capacity_m"] == p8_geometry.clearance_capacity(r["boxes"])
        assert r["goal_clearance_m"] == round(p8_geometry.goal_clearance(r["boxes"]), 6)
        assert p8_specs.layout_type(r) == r["type"]
        assert not any(p5_layouts.matches(stripped(r["boxes"]), other) for other in p7_boxes)
    assert v4["seeds_tried"] == sum(v4["rejected"].values()) + v4["admitted_not_selected"] + 10


def test_the_type_rule_bounds():
    assert p8_specs.layout_type({"clearance_capacity_m": 1.10, "goal_clearance_m": 9}) == "F2"
    assert p8_specs.layout_type({"clearance_capacity_m": 1.35, "goal_clearance_m": 1.0}) == "F2"
    assert p8_specs.layout_type({"clearance_capacity_m": 1.36, "goal_clearance_m": 2.3}) == "F1"
    assert p8_specs.layout_type({"clearance_capacity_m": 1.36, "goal_clearance_m": 2.31}) is None
    assert p8_specs.layout_type({"clearance_capacity_m": 1.09, "goal_clearance_m": 1.0}) is None
    assert p8_specs.layout_type({"clearance_capacity_m": None, "goal_clearance_m": 1.0}) is None


def test_the_schedule_regenerates_from_the_layouts_and_is_balanced(specs):
    schedule = specs["p8_schedule.json"]
    assert schedule == p8_specs.schedule_document(specs["p8_layouts.json"])
    assert schedule["layouts_canonical_sha256"] == canonical_sha256(specs["p8_layouts.json"])
    assert [s["slot"] for s in schedule["slots"]] == list(range(1, 31))
    for b, stage in ((1, "V3"), (2, "V4"), (3, "V5")):
        rows = [s for s in schedule["slots"] if s["block"] == b]
        assert {s["layout"] for s in rows} == {r["id"] for r in specs["p8_layouts.json"]["stages"][stage]["layouts"]}
        assert sorted(s["first"] for s in rows) == ["p8_p4r"] * 5 + ["p8_p5r"] * 5
        assert all(sorted(s["order"]) == ["p8_p4r", "p8_p5r"] and s["order"][0] == s["first"] for s in rows)


def test_a_changed_layout_changes_the_whole_schedule(specs):
    layouts = copy.deepcopy(specs["p8_layouts.json"])
    layouts["stages"]["V4"]["layouts"][0]["seed"] += 1
    assert p8_specs.schedule_document(layouts)["layouts_canonical_sha256"] != specs["p8_schedule.json"]["layouts_canonical_sha256"]


def test_the_recovery_parameters_come_from_the_accepted_configs(specs):
    rec = specs["p8_recovery.json"]
    avoid = strict_json(REPO / p8_specs.CONFIGS["p4_avoidance"])
    p2 = strict_json(REPO / p8_specs.CONFIGS["p2_control"])
    assert rec["episode"]["window_s"] == avoid["supervisor"]["hover_timeout_sim_s"] == 20.0
    assert rec["episode"]["yaw_rate_deg_s"] == p2["yaw_ramp_deg_s"] == 10.0
    assert rec["episode"]["max_rotation_deg"] == 200.0 and rec["episode"]["max_translation_m"] == pytest.approx(4.0)
    assert rec["r1"]["goal_radius_m"] == pytest.approx(avoid["mission"]["goal_tolerance_m"] + 0.5)
    assert rec["r3"]["segment_max_m"] == max(avoid["planner"]["lengths_m"])
    assert rec["dead_end_memory"]["radius_m"] == avoid["envelope"]["r_body_m"]
    assert rec["r4"]["net_progress_below_m"] == avoid["planner"]["min_progress_m"]
    successes = [r for r in rec["r4"]["evidence"]["p5_flights"] if r["outcome"] == "goal_success"]
    assert rec["r4"]["yaw_to_l4_at_least"] == max(r["max_any_window"] for r in successes) + 1
    assert not any(r["max_no_progress_window"] >= rec["r4"]["yaw_to_l4_at_least"] for r in successes)
    assert rec["r4"]["evidence"]["would_trigger_on_slots"] == [12]


def test_the_report_plan_inherits_h1_to_h9_unchanged(specs):
    plan = specs["p8_report_plan.yaml"]
    p7e = strict_json(REPO / "simulation/px4_gazebo_p7e/configs/p7e_report_plan.yaml")
    assert canonical_sha256(p7e) == "15e7f7b460277e650823342bbf33a63426c226730192f3bbc2c536ab70c31d94"
    assert [h["id"] for h in plan["hard_criteria"]] == ["H%d" % i for i in range(1, 12)]
    for mine, theirs in zip(plan["hard_criteria"][:9], p7e["hard_criteria"]):
        assert (mine["limit"], mine["limit_source"], mine["scope"]) == (theirs["limit"], theirs["limit_source"], theirs["scope"])
    assert [r["id"] for r in plan["reported_criteria"]] == ["R-P8-%d" % i for i in range(1, 11)]
    assert plan["effectiveness"]["target"] is None and plan["reporting"]["p_values"] == "none"
    assert plan["required_statement"].startswith("P8 is an engineering validation of a recovery layer.")


@needs_runs
def test_the_geometry_reproduces_the_p8_evidence_on_all_60_flights():
    evidence = strict_json(REPO / "docs/evidence/v3_sim_p8_hover_timeout_analysis.json")
    p7 = strict_json(REPO / "simulation/px4_gazebo_p7/configs/p7_layouts.json")
    for flight in evidence["flights"]:
        origin = tuple(strict_json(ROOT / "runs" / flight["run"] / "controller-result.json")["p4"]["origin_enu"])
        boxes = p8_specs.p7_record(p7, flight["layout"])["boxes"]
        assert p8_geometry.clearance_capacity(boxes, origin) == flight["clearance_capacity_m"]
        assert p8_geometry.goal_clearance(boxes, origin) == pytest.approx(flight["goal_clearance_truth_m"], abs=1e-9)


@needs_runs
def test_the_recovery_document_and_v3_v5_regenerate_from_the_accepted_batch(specs):
    assert p8_specs.recovery_document() == specs["p8_recovery.json"]
    v3, v5, rule = p8_specs.v3_v5(*p8_specs.frozen_inputs())
    assert v3 == specs["p8_layouts.json"]["stages"]["V3"]["layouts"] and v5 == specs["p8_layouts.json"]["stages"]["V5"]["layouts"]


@needs_runs
def test_a_changed_block_result_is_refused(monkeypatch):
    monkeypatch.setitem(p8_specs.P7E_RESULTS, 2, "00" * 32)
    with pytest.raises(SystemExit, match="block 2"):
        p8_specs.frozen_inputs()
