"""P7 M1: the frozen layout sets, schedule and analysis plan (offline; nothing is launched).

* The layout sets follow contract §4.1 and decision D8: the bound P5 generator, its admission filter, the fresh
  seed ranges, seed order and box names. The generator files they were made from are unchanged.
* The schedule is the deterministic function of the layout document, a permutation with balanced blocks.
* The analysis plan carries the approved numbers (D2, D3, D7, D9, the pilot rule).
* D13's premise: the accepted `p4_evaluate.outcome()` adds no failure for the P7 declaration label, and crashes on a
  null outcome (M0 audit §3).
* The full regeneration (P5 splits, screening, world hashes) runs where the runtime tree is present.
"""
import json
import math
import os
from pathlib import Path
import re
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
for path in (P7 / "scripts", REPO / "simulation/px4_gazebo_p4/validation", REPO / "simulation/px4_gazebo/validation",
             REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control", REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_sensor_adapter",
             REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance"):
    if str(path) not in sys.path:
        sys.path.append(str(path))
import p7_specs  # noqa: E402
import p5_layouts  # noqa: E402
import p4_worlds  # noqa: E402
from p3_provenance import canonical_sha256, digest, strict_json  # noqa: E402

ROOT = Path(os.environ.get("GWM_SIM_ROOT", Path.home() / "uav_autonomy"))
needs_runtime_tree = pytest.mark.skipif(not (ROOT / "state/px4-mc-trace-built.json").exists(),
                                        reason="the pinned PX4 default world (runtime tree) is not available")


@pytest.fixture(scope="module")
def layouts():
    return strict_json(P7 / "configs/p7_layouts.json")


@pytest.fixture(scope="module")
def schedule():
    return strict_json(P7 / "configs/p7_schedule.json")


@pytest.fixture(scope="module")
def plan():
    return strict_json(P7 / "configs/p7_analysis_plan.yaml")


# ---- layouts ----------------------------------------------------------------------------------------------------
def test_layout_document_identity(layouts):
    assert layouts["contract"] == "p7-layouts-v1"
    assert layouts["outcome_label"] == p7_specs.OUTCOME_LABEL == "p7_not_predeclared"
    assert layouts["runtime"]["numpy"] == "1.26.4"
    assert layouts["exclusion"]["p5_layouts"] == 1200 + 150 + 300
    assert layouts["base_world"]["sha256"] == strict_json(ROOT / p7_specs.P3_MANIFEST)["runtime"]["assets"][p7_specs.WORLD_ASSET] \
        if (ROOT / p7_specs.P3_MANIFEST).exists() else True


def test_generator_files_are_the_ones_the_layouts_were_made_from(layouts):
    assert layouts["generator_sha256"] == {path: digest(REPO / path) for path in p7_specs.GENERATOR_FILES}


@pytest.mark.parametrize("name, prefix, base, count", [("pilot", "p7p", 5_000_000, 10), ("confirmatory", "p7c", 4_000_000, 250)])
def test_sets_are_seed_ordered_counted_and_named(layouts, name, prefix, base, count):
    s = layouts["sets"][name]
    rows = s["layouts"]
    assert s["seed_base"] == base and s["count"] == count == len(rows)
    assert [r["index"] for r in rows] == list(range(count))
    seeds = [r["seed"] for r in rows]
    assert seeds == sorted(seeds) and len(set(seeds)) == count and base <= seeds[0] and seeds[-1] < base + s["seeds_tried"]
    assert sum(s["rejected"].values()) + count == s["seeds_tried"]
    assert all(r["id"] == "%s-%07d" % (prefix, r["seed"]) for r in rows)
    for r in rows:
        assert [b["name"] for b in r["boxes"]] == ["p7_box_%d" % k for k in range(len(r["boxes"]))]
        assert 1 <= len(r["boxes"]) <= 4 and re.fullmatch("[0-9a-f]{64}", r["world_sha256"])
    assert len({r["world_sha256"] for r in rows}) == count
    families = {}
    for r in rows:
        families[r["family"]] = families.get(r["family"], 0) + 1
    assert families == s["family_counts"]
    if name == "confirmatory":
        assert s["primary_count"] == 200


def test_each_layout_is_what_the_bound_generator_makes_from_its_seed_and_is_admitted(layouts):
    _, config, _ = p5_layouts.p5_episode.configs()
    for s in layouts["sets"].values():
        for r in s["layouts"]:
            made = p5_layouts.generate(r["seed"])
            assert made["family"] == r["family"]
            assert [{"center": b["center"], "size": b["size"]} for b in r["boxes"]] == made["boxes"]
            assert p5_layouts.rejection(made, config) is None, r["id"]


def test_the_two_seed_ranges_are_fresh(layouts):
    used = [(base, base + p5_layouts.SEED_RANGE) for base, _ in p5_layouts.SPLITS.values()]
    for s in layouts["sets"].values():
        assert all(not (lo <= r["seed"] < hi) for r in s["layouts"] for lo, hi in used)


# ---- the scenario mapping and D13 -----------------------------------------------------------------------------
SYNTHETIC_BASE = '<sdf version="1.9"><world name="default"><gravity>0 0 -9.8</gravity></world></sdf>'


def test_scenario_mapping_feeds_the_accepted_world_generator(layouts):
    rows = layouts["sets"]["pilot"]["layouts"]
    mapping = p7_specs.scenario_mapping(rows)
    for r in rows:
        text, declaration = p4_worlds.generate(SYNTHETIC_BASE, r["id"], mapping)
        assert declaration["outcome"] == "p7_not_predeclared" and declaration["fault"] is None
        assert [b["name"] for b in declaration["boxes"]] == [b["name"] for b in r["boxes"]]
        assert all(text.count('name="%s"' % b["name"]) == 1 for b in r["boxes"])


def p4_outcome(label):
    import p4_evaluate
    from p3_stream_eval import Failures
    f = Failures()
    summary = {"controller_result": {"p4_outcome": "supervisor_abort", "final_state": "COMPLETE",
                                     "p4": {"origin_enu": [0.0, 0.0], "supervisor_transitions": [], "abort_reason": "hover_timeout"}}}
    declaration = {"scenario": "p7c-4000002", "outcome": label}
    track = [(0, [0.0, 0.0, 2.0]), (1, [3.0, 0.5, 2.0])]
    out = p4_evaluate.outcome(None, summary, declaration, [], (0.0, 0.0), track,
                              strict_json(REPO / "simulation/px4_gazebo_p4/configs/p4_validation.yaml")["outcomes"], f)
    return out, f.items


def test_d13_the_p7_label_runs_no_accepted_outcome_check():
    out, failures = p4_outcome(p7_specs.OUTCOME_LABEL)
    assert failures == [] and "label" not in out and out["expected"] == "p7_not_predeclared"
    assert out["abort_reason"] == "hover_timeout"  # the accepted record is still reported for the P7 classifier


def test_d13_a_null_outcome_crashes_the_accepted_function():
    with pytest.raises(AttributeError):
        p4_outcome(None)


# ---- schedule ---------------------------------------------------------------------------------------------------
def test_schedule_is_the_deterministic_function_of_the_layouts(layouts, schedule):
    assert schedule["layouts_canonical_sha256"] == canonical_sha256(layouts)
    assert schedule == p7_specs.schedule_document(layouts)


def test_pilot_schedule(layouts, schedule):
    pilot = schedule["pilot"]
    ids = [r["id"] for r in layouts["sets"]["pilot"]["layouts"]]
    assert pilot["pairs"] == 20 and pilot["flights"] == 40 and len(pilot["slots"]) == 20
    assert sorted((s["layout"], s["repeat"]) for s in pilot["slots"]) == sorted((i, r) for i in ids for r in (1, 2))
    assert sum(s["first"] == "p4" for s in pilot["slots"]) == 10
    assert all(s["order"] == [s["first"], "p5" if s["first"] == "p4" else "p4"] for s in pilot["slots"])
    for i in ids:  # one repeat each way round, numbered in flight order
        mine = [s for s in pilot["slots"] if s["layout"] == i]
        assert [s["repeat"] for s in mine] == [1, 2] and {s["first"] for s in mine} == {"p4", "p5"}


@pytest.mark.parametrize("n", [200, 250])
def test_confirmatory_schedules(layouts, schedule, n):
    c = schedule["confirmatory"][str(n)]
    rows = c["slots"]
    first_n = [r["id"] for r in layouts["sets"]["confirmatory"]["layouts"][:n]]
    assert c["pairs"] == n == len(rows) and c["flights"] == 2 * n and c["blocks"] == math.ceil(n / 20)
    assert sorted(r["layout"] for r in rows) == sorted(first_n)
    assert [r["slot"] for r in rows] == list(range(1, n + 1))
    assert [r["layout"] for r in rows] != first_n  # shuffled, not seed order
    for b in range(1, c["blocks"] + 1):
        block = [r for r in rows if r["block"] == b]
        assert len(block) == min(20, n - 20 * (b - 1))
        assert sum(r["first"] == "p4" for r in block) * 2 == len(block)
        assert [r["pair_in_block"] for r in block] == list(range(1, len(block) + 1))


def test_shuffle_is_a_permutation_and_depends_on_the_label():
    items = list(range(50))
    a, b = p7_specs.shuffle(items, "x", "one"), p7_specs.shuffle(items, "x", "two")
    assert sorted(a) == items == sorted(b) and a != b and a == p7_specs.shuffle(items, "x", "one")


# ---- analysis plan ----------------------------------------------------------------------------------------------
def test_analysis_plan_carries_the_approved_numbers(plan):
    assert plan["contract"] == "p7-analysis-plan-v1" and plan["sign_convention"] == "every difference is P5 minus P4"
    assert plan["primary"]["alpha"] == 0.05 and "exact two-sided McNemar" in plan["primary"]["test"]
    size = plan["sample_size"]
    assert (size["planned_pairs"], size["ceiling_pairs"]) == p7_specs.CONFIRMATORY_SIZES == (200, 250)
    assert "27/243" in size["pilot_rule"]["planning_value"] and "d < 27/243" in size["pilot_rule"]["rule"]
    assert size["power_at_planning_values"] == {"200": 0.83, "250": 0.91}
    sec = plan["secondaries"]
    assert sec["S1"]["margin_m"] == 0.05 and "20261121" in sec["S1"]["interval"] and "10,000" in sec["S1"]["interval"]
    assert "time to goal" in sec["S2"]["name"] and "path length" in sec["S3"]["name"]
    assert "S1 is tested only if the primary rejects" in sec["procedure"]


def test_power_values_follow_from_the_planning_values(plan):
    def sign_p(k, n):
        return 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, i) for i in range(min(k, n - k) + 1)) / 2 ** n)

    def power(n, d, s):
        total = 0.0
        for m in range(n + 1):
            w = math.comb(n, m) * d ** m * (1 - d) ** (n - m)
            if w > 1e-14:
                total += w * sum(math.comb(m, k) * s ** k * (1 - s) ** (m - k) for k in range(m + 1) if sign_p(k, m) < 0.05)
        return round(total, 2)
    assert {str(n): power(n, 27 / 243, 22 / 27) for n in (200, 250)} == plan["sample_size"]["power_at_planning_values"]


# ---- full regeneration ------------------------------------------------------------------------------------------
@needs_runtime_tree
def test_full_regeneration_reproduces_the_frozen_layouts(layouts):
    assert p7_specs.layouts_document() == layouts
