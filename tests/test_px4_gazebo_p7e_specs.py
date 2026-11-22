"""P7-E E1: the 30-pair schedule and the engineering report plan (offline; nothing is launched).

* The schedule is the deterministic function of the frozen P7 layouts: pool indices 200-229, 3 blocks of 10 pairs,
  5/5 first-controller balance. It has no overlap with the P7-A 200-pair schedule; the 250-alternative overlap is
  recorded.
* The frozen P7 files are untouched. The P7 candidate's source binding does not pick up any P7-E file.
* The report plan cites only existing accepted limits, each checked against its source file. It carries the
  contract's required statement verbatim, no p-values, and the open risk-register entries.
"""
import json
from pathlib import Path
import re
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
P7E, P7 = REPO / "simulation/px4_gazebo_p7e", REPO / "simulation/px4_gazebo_p7"
for path in (P7E / "scripts", P7 / "scripts", REPO / "simulation/px4_gazebo/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7e_specs  # noqa: E402
from p3_provenance import canonical_sha256, strict_json  # noqa: E402


@pytest.fixture(scope="module")
def schedule():
    return strict_json(P7E / "configs/p7e_schedule.json")


@pytest.fixture(scope="module")
def plan():
    return strict_json(P7E / "configs/p7e_report_plan.yaml")


# ---- schedule ----------------------------------------------------------------------------------------------------
def test_the_schedule_is_the_deterministic_function_of_the_frozen_layouts(schedule):
    layouts, p7a = p7e_specs.frozen_inputs()
    assert schedule == p7e_specs.schedule_document(layouts, p7a)
    assert schedule["layouts_canonical_sha256"] == canonical_sha256(layouts) == p7e_specs.LAYOUTS_CANONICAL


def test_thirty_pool_layouts_200_to_229_in_three_balanced_blocks(schedule):
    layouts = strict_json(P7 / "configs/p7_layouts.json")
    pool = sorted(layouts["sets"]["confirmatory"]["layouts"], key=lambda r: r["index"])
    slots = schedule["slots"]
    assert (schedule["pairs"], schedule["flights"], schedule["blocks"], schedule["block_pairs"]) == (30, 60, 3, 10)
    assert sorted(s["layout"] for s in slots) == sorted(r["id"] for r in pool[200:230])
    assert sorted(s["pool_index"] for s in slots) == list(range(200, 230))
    assert [s["slot"] for s in slots] == list(range(1, 31))
    assert [s["pool_index"] for s in slots] != list(range(200, 230))  # shuffled, not pool order
    by_id = {r["id"]: r for r in pool}
    for s in slots:
        r = by_id[s["layout"]]
        assert (s["seed"], s["family"], s["boxes"], s["world_sha256"]) == (r["seed"], r["family"], len(r["boxes"]), r["world_sha256"])
        assert s["order"] == [s["first"], "p5" if s["first"] == "p4" else "p4"]
    for b in (1, 2, 3):
        block = [s for s in slots if s["block"] == b]
        assert len(block) == 10 and sum(s["first"] == "p4" for s in block) == 5
        assert [s["pair_in_block"] for s in block] == list(range(1, 11))


def test_no_contamination_of_the_p7a_200_pair_schedule_and_the_250_overlap_is_recorded(schedule):
    p7a = strict_json(P7 / "configs/p7_schedule.json")
    ids = {s["layout"] for s in schedule["slots"]}
    assert not ids & {s["layout"] for s in p7a["confirmatory"]["200"]["slots"]}
    assert len(ids & {s["layout"] for s in p7a["confirmatory"]["250"]["slots"]}) == 30
    nc = schedule["non_contamination"]
    assert (nc["overlap_with_p7a_200_pair_schedule"], nc["overlap_with_unselected_250_pair_alternative"]) == (0, 30)
    assert nc["p7a_schedule_canonical_sha256"] == p7e_specs.P7A_SCHEDULE_CANONICAL == canonical_sha256(p7a)
    pilot = {r["id"] for r in strict_json(P7 / "configs/p7_layouts.json")["sets"]["pilot"]["layouts"]}
    assert not ids & pilot


def test_the_frozen_p7_files_and_candidate_binding_are_untouched():
    import p7_manifest
    assert canonical_sha256(strict_json(P7 / "configs/p7_layouts.json")) == p7_manifest.APPROVED_SPECS["layouts"]
    assert canonical_sha256(strict_json(P7 / "configs/p7_schedule.json")) == p7_manifest.APPROVED_SPECS["schedule"]
    assert not any("p7e" in path for path in p7_manifest.p7_source_files())  # the P7 candidate does not bind P7-E files


def test_a_changed_frozen_layout_document_is_refused(monkeypatch):
    monkeypatch.setattr(p7e_specs, "LAYOUTS_CANONICAL", "0" * 64)
    with pytest.raises(SystemExit, match="layouts"):
        p7e_specs.frozen_inputs()


# ---- report plan -------------------------------------------------------------------------------------------------
def test_the_hard_criteria_cite_the_accepted_limits(plan):
    h = {c["id"]: c for c in plan["hard_criteria"]}
    assert list(h) == ["H%d" % i for i in range(1, 10)]
    v4 = strict_json(REPO / "simulation/px4_gazebo_p4/configs/p4_validation.yaml")
    assert h["H1"]["limit"] == v4["safety"]["collisions_allowed"] == 0
    assert h["H2"]["limit"] == v4["safety"]["min_truth_clearance_m"] == 0.20 and h["H2"]["comparison"] == ">="
    assert "bf1ef9b1080a52eab1673ac2a1ac6e20d30cfa7fd85b703cae8433dc55f7b3b0" in h["H4"]["limit_source"]
    assert "176, 400, 21" in h["H5"]["limit"]
    for i in range(1, 7):
        assert "the batch stops" in h["H%d" % i]["on_failure"]
    codes = strict_json(REPO / "simulation/px4_gazebo_p6/configs/p6_faults.yaml")["codes"]
    assert codes["H35"]["stale_target_motion_m"] == 0.01 and "0.01 m" in h["H3"]["limit_source"]


def test_the_reported_criteria_cite_the_accepted_limits(plan):
    r = {c["id"]: c for c in plan["reported_criteria"]}
    assert list(r) == ["R%d" % i for i in range(1, 11)]
    v4 = strict_json(REPO / "simulation/px4_gazebo_p4/configs/p4_validation.yaml")
    v5 = strict_json(REPO / "simulation/px4_gazebo_p5/configs/p5_validation.yaml")
    p2 = strict_json(REPO / "simulation/px4_gazebo/configs/p2_control.yaml")
    host = strict_json(P7 / "configs/p7_host_check.yaml")
    perf, pol = v4["performance"], v5["policy"]

    def numbers(text):
        return [float(x) for x in re.findall(r"\d+\.\d+|\d+", text)]
    assert {perf["snapshot_rate_min_hz"], perf["snapshot_latency_max_sim_s"], perf["snapshot_latency_p95_sim_s"],
            perf["perception_compute_p99_s"], perf["perception_compute_max_s"],
            perf["perception_receipt_gap_max_wall_s"]} <= set(numbers(r["R4"]["accepted_limit"]))
    assert {pol["inference_p99_s"], pol["inference_max_s"], pol["participation_fraction"]} <= set(numbers(r["R5"]["accepted_limit"]))
    assert {v4["safety"]["tracking_horizontal_m"], v4["safety"]["response_latency_sim_s"]} <= set(numbers(r["R6"]["accepted_limit"]))
    assert p2["max_sample_gap_sim_s"] in numbers(r["R3"]["accepted_limit"])
    assert host["windows"]["cpu_load_max_percent"] in numbers(r["R8"]["accepted_limit"])
    assert host["windows"]["ac_overlay_guid"].startswith("ded574b5") and "ded574b5" in r["R8"]["accepted_limit"]


def test_the_plan_carries_the_contract_statement_no_p_values_and_the_open_risks(plan):
    contract = (REPO / "docs/v3_sim_p7e_contract_draft.md").read_text(encoding="utf-8")
    statement = " ".join(contract.split("> \"", 1)[1].split("\"\n", 1)[0].replace("\n   > ", " ").split())
    assert plan["required_statement"] == statement
    assert plan["reporting"]["p_values"].startswith("none")
    assert plan["risk_register"]["open"] == ["RR1 P2 observation-gap RECOVERY", "RR2 P5 repeat-behaviour inconsistency"]
    assert "bd036a2b" in plan["reporting"]["functions"] and plan["sign_convention"] == "every paired difference is P5 minus P4"
