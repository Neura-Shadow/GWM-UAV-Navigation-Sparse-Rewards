"""P6 M1: the frozen fault and scenario specifications agree with each other, the approved contract and
the accepted P2/P4/P5 limits (offline; nothing is launched)."""
import hashlib
import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
P6 = REPO / "simulation/px4_gazebo_p6"
P5_VALIDATION = REPO / "simulation/px4_gazebo_p5/configs/p5_validation.yaml"
P2_TIMING = REPO / "simulation/px4_gazebo/configs/p2_timing.yaml"
P4_SCENARIOS = REPO / "simulation/px4_gazebo_p4/configs/p4_scenarios.json"
CATEGORY_COUNTS = {"F1": 12, "F2": 10, "F3": 14, "F4": 13, "F5": 12, "F6": 10, "F7": 9, "F8": 6}
CLASSES = {"R-O", "R-A", "R-B", "R-S", "R-C", "C", "D", "cov", "alias"}
GATE_CLASS = {"A": "R-A", "B": "R-B", "S": "R-S", "C": "R-C"}


def strict(path):
    def reject(value):
        raise ValueError("non-finite constant " + value)
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


@pytest.fixture(scope="module")
def faults():
    return strict(P6 / "configs/p6_faults.yaml")


@pytest.fixture(scope="module")
def scenarios():
    return strict(P6 / "configs/p6_scenarios.yaml")


def by_id(faults):
    return {f["id"]: f for f in faults["faults"]}


def gate_rows(scenarios):
    return [(gate, row) for gate, rows in scenarios["gates"].items() for row in rows]


def test_fault_ids_are_unique_complete_and_numbered_per_category(faults):
    ids = [f["id"] for f in faults["faults"]]
    assert len(ids) == len(set(ids)) == sum(CATEGORY_COUNTS.values()) == 86
    for category, count in CATEGORY_COUNTS.items():
        assert [i for i in ids if i.startswith(category + ".")] == ["%s.%02d" % (category, n) for n in range(1, count + 1)]
    assert all(f["category"] == f["id"].split(".")[0] for f in faults["faults"])


def test_fault_fields_classes_tiers_mechanisms_and_aliases_are_valid(faults):
    table = by_id(faults)
    for fault in faults["faults"]:
        assert set(fault["classes"]) <= CLASSES, fault["id"]
        if "alias_of" in fault:
            assert all(target in table for target in fault["alias_of"].split(",")), fault["id"]
            continue
        assert set(fault["tiers"]) <= {"O", "G", "S"}, fault["id"]
        mechanism = fault["mechanism"]
        if mechanism is not None and not mechanism.startswith("via "):
            assert set(mechanism.split("+")) <= set(faults["mechanisms"]), fault["id"]
        if {"R-A"} & set(fault["classes"]):
            assert "G" in fault["tiers"], fault["id"]
        if {"R-B", "R-S", "R-C"} & set(fault["classes"]) and not str(mechanism).startswith("via "):
            assert "S" in fault["tiers"], fault["id"]
        if "R-O" in fault["classes"]:
            assert "O" in fault["tiers"], fault["id"]


def test_every_gate_row_references_a_fault_of_its_gate_class_and_back(faults, scenarios):
    table = by_id(faults)
    used = {"R-A": set(), "R-B": set(), "R-S": set(), "R-C": set()}
    for gate, row in gate_rows(scenarios):
        fault = row.get("fault")
        if fault is None:
            continue
        assert GATE_CLASS[gate] in table[fault]["classes"], row["id"]
        used[GATE_CLASS[gate]].add(fault)
    for fault in faults["faults"]:
        realizers = {fault["id"], *fault.get("realized_by", [])}
        for cls, rows in used.items():
            if cls in fault["classes"]:
                assert realizers & rows, (fault["id"], cls)  # a required fault is flown, directly or inside a compound row


def test_worlds_triggers_and_baselines_are_bound(scenarios):
    p4 = json.loads(P4_SCENARIOS.read_text(encoding="utf-8"))
    assert hashlib.sha256(P4_SCENARIOS.read_bytes()).hexdigest() == scenarios["worlds"]["source_sha256"]
    for gate, row in gate_rows(scenarios):
        if row["scope"] == "flight":
            assert row["world"] in p4["scenarios"], row["id"]
        if "trigger" in row:
            assert row["trigger"] in scenarios["triggers"], row["id"]
    for row in scenarios["characterization"]["flights"]:
        assert row["trigger"] in scenarios["triggers"], row["id"]
    validation = strict(P5_VALIDATION)
    assert scenarios["baselines"]["p3_manifest_sha256"] == validation["p3_baseline_manifest_sha256"]
    assert scenarios["baselines"]["p4_manifest_sha256"] == validation["p4_baseline_manifest_sha256"]
    assert scenarios["baselines"]["p5_manifest_sha256"].startswith("2becfbbb")


def test_counts_match_the_approved_gate_structure(faults, scenarios):
    gates, counts = scenarios["gates"], scenarios["counts"]
    fixtures = [row for row in gates["A"] if row["id"].startswith("A-5.")]
    live_ground = [row for row in gates["A"] if row["scope"] in ("p3_readiness", "ground_readiness")] + fixtures
    assert len(fixtures) == counts["gate_a_fault_fixtures"] == 14  # M1 correction 1 added F2.09
    assert len(live_ground) == counts["gate_a_live_ground_runs"] == 16
    assert [row["id"] for row in gates["B"]] == ["B-0", "B-1"]
    assert [row["id"] for row in gates["S"]] == ["S-1", "S-2'", "S-3", "S-4", "S-5'"]
    assert [row["id"] for row in gates["C"]] == ["C-1", "C-2", "C-3", "C-4", "C-5"]
    assert len(gates["B"]) + len(gates["S"]) + len(gates["C"]) == counts["flights"] == 12
    required_offline = [f for f in faults["faults"] if "R-O" in f["classes"]]
    assert len(required_offline) == counts["offline_required_items"] == 31
    assert scenarios["predecessor"] == {"B": "A", "S": "B", "C": "S"}


def test_injected_timings_sit_where_the_accepted_limits_put_them(faults, scenarios):
    policy = strict(P5_VALIDATION)["policy"]
    budget = strict(P2_TIMING)["dispatch_budget_wall_s"]
    rows = {row["id"]: row for _, row in gate_rows(scenarios)}
    for row_id in ("B-1", "C-2", "C-4"):  # policy timeout, but inside the dispatch budget: the HOLD path, not P2
        assert policy["failure_wall_s"] < rows[row_id]["parameters"]["sleep_s"] < budget
    assert rows["B-1"]["parameters"]["consecutive_invocations"] == policy["consecutive_failures_to_abort"]
    assert rows["S-3"]["parameters"]["consecutive_invocations"] == policy["consecutive_failures_to_abort"]
    assert rows["S-5'"]["parameters"]["mission_tick_delay_s"] > budget  # must trip the dispatch guard
    gaps = rows["C-2"]["parameters"]["invocation_offsets_sim_s"]
    assert all(b - a >= 5.0 for a, b in zip(gaps, gaps[1:]))  # isolated: never consecutive
    codes = faults["codes"]
    assert codes["PH3"]["consecutive_failures_to_abort"] == policy["consecutive_failures_to_abort"]
    assert codes["LOAD"] == {"max_workers": 24, "max_window_s": 60.0, "max_memory_fraction_of_free": 0.25,
                             "breach_requires_matching_safe_response": True}
    assert rows["C-5"]["parameters"]["load_workers"] <= codes["LOAD"]["max_workers"]
    assert rows["C-5"]["parameters"]["load_window_s"] <= codes["LOAD"]["max_window_s"]
    assert codes["PX"]["gated"] is False and codes["S0"]["truth_clearance_min_m"] == 0.20


def test_px4_managed_and_recovery_landings_are_never_gated_as_complete(faults, scenarios):
    table = by_id(faults)
    for gate, row in gate_rows(scenarios):
        fault = table.get(row.get("fault"))
        if fault is not None:
            assert "PX" not in fault["expected"], row["id"]  # D4a: PX4-managed runs are characterization only
    assert "L45R" in rows_expected(scenarios, "S-5'") and "L45" not in rows_expected(scenarios, "S-5'")
    assert {row["fault"] for row in scenarios["characterization"]["flights"]} >= {"F2.01", "F5.01"}


def rows_expected(scenarios, row_id):
    return next(row for _, row in gate_rows(scenarios) if row["id"] == row_id)["expected"]
