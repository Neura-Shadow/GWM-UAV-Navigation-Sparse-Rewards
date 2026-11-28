"""Offline tests of p8_h9_addendum.py (kept outside tests/, so the P8 candidate's test binding is unchanged).
Run: cd /tmp && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider THIS_FILE
"""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p8_h9_addendum as add  # noqa: E402
from p3_provenance import digest  # noqa: E402

SHA = "ab" * 32
H10_OK = {"contract": "p8-h10-record-v1", "ok": True, "actions": 12, "refusals": 1, "violations": [], "records": 15}


def fresh(run="r1"):
    """A fresh P7-E assessment, as `p7e_assess.assess` returns it."""
    return {"contract": add.P7E_LABEL, "run": run, "assessor_sha256": "e0" * 32, "flight_record_sha256": "f1" * 32,
            "summary_sha256": "5a" * 32, "frozen": {"validity": "valid", "stop_reasons": [], "outcome": "failure"},
            "fc3": {"applied": False, "basis": None, "evidence": None}, "validity": "valid", "outcome": "failure",
            "stop_reasons": []}


def p8_assessment(again, h10=H10_OK, h10_sha="77" * 32):
    """What `run_p8_block.assess` writes from the same assessment."""
    return dict(again, contract=add.P8_LABEL, stop_reasons=add.expected_stop_reasons(again, h10),
                h10={"ok": h10["ok"], "actions": h10["actions"], "refusals": h10["refusals"],
                     "violations": h10["violations"][:20], "record_sha256": h10_sha})


# ---- rule O and rule C --------------------------------------------------------------------------------------------
def test_the_sequencers_relabel_alone_fails_rule_o_and_passes_rule_c():
    again = fresh()
    mine = p8_assessment(again)
    assert add.rule_o(again, mine) is False
    assert add.differing_keys(again, mine) == ["contract"]
    assert add.rule_c(again, mine, H10_OK, "77" * 32) == (True, [])


def test_an_identical_p7e_assessment_passes_rule_o():
    again = fresh()
    assert add.rule_o(again, dict(again)) is True


@pytest.mark.parametrize("key,value", [("validity", "infrastructure_invalid"), ("outcome", "goal_success"),
                                       ("fc3", {"applied": True}), ("flight_record_sha256", "00" * 32)])
def test_a_substantive_difference_fails_rule_c(key, value):
    again = fresh()
    mine = dict(p8_assessment(again), **{key: value})
    holds, problems = add.rule_c(again, mine, H10_OK, "77" * 32)
    assert holds is False and problems == ["differs:" + key]


def test_a_wrong_label_fails_rule_c():
    again = fresh()
    mine = dict(p8_assessment(again), contract="p8-flight-assessment-v2")
    assert add.rule_c(again, mine, H10_OK, "77" * 32) == (False, ["p8_label:p8-flight-assessment-v2"])
    other = dict(again, contract="something-else")
    assert "p7e_label:something-else" in add.rule_c(other, p8_assessment(other), H10_OK, "77" * 32)[1]


def test_an_extra_stop_reason_fails_rule_c():
    again = fresh()
    mine = dict(p8_assessment(again), stop_reasons=["safety_event:fc6_recovery_invariant"])
    assert add.rule_c(again, mine, H10_OK, "77" * 32) == (False, ["stop_reasons"])


@pytest.mark.parametrize("violation,reason", [("action on a stale tick at 3.2", "FC6"), ("unreproduced yaw", "FC5")])
def test_a_failed_h10_adds_exactly_the_sequencers_stop(violation, reason):
    again = fresh()
    h10 = dict(H10_OK, ok=False, violations=[violation])
    assert add.expected_stop_reasons(again, h10) == [getattr(add.run_p8_block, reason)]
    assert add.rule_c(again, p8_assessment(again, h10), h10, "77" * 32) == (True, [])


def test_an_h10_summary_that_differs_from_the_record_fails_rule_c():
    again = fresh()
    mine = p8_assessment(again)
    mine["h10"]["actions"] = 13
    assert add.rule_c(again, mine, H10_OK, "77" * 32) == (False, ["h10_summary"])
    assert add.rule_c(again, p8_assessment(again), H10_OK, "88" * 32) == (False, ["h10_summary"])


def test_an_unexpected_key_fails_rule_c():
    again = fresh()
    mine = dict(p8_assessment(again), note="x")
    assert add.rule_c(again, mine, H10_OK, "77" * 32) == (False, ["extra_keys:note"])


# ---- the summary --------------------------------------------------------------------------------------------------
def row(**kw):
    base = {"rule_o_reproduced": False, "rule_o_matches_report": True, "rule_o_differing_keys": ["contract"],
            "rule_c_holds": True, "run_files_unchanged": True}
    return dict(base, **kw)


def test_summary_explained_only_when_every_flight_is():
    assert add.summarize([row(), row()], 2)["status"] == "explained"
    for bad in (dict(rule_c_holds=False), dict(rule_o_matches_report=False), dict(run_files_unchanged=False),
                dict(rule_o_differing_keys=["contract", "validity"])):
        assert add.summarize([row(), row(**bad)], 2)["status"] == "not explained", bad
    assert add.summarize([row()], 2)["status"] == "not explained"  # a flight is missing
    assert add.summarize([], 0)["status"] == "not explained"


# ---- the report checks --------------------------------------------------------------------------------------------
def report(n=2, **change):
    hard = [{"id": "H%d" % i, "met": i != 9} for i in range(1, 12)]
    rows = [{"block": 1, "slot": i + 1, "controller": "p4", "attempt": 1} for i in range(n)]
    hard[8].update(manifest={"verified": True}, results_bind_manifest=True,
                   worlds=[dict(r, verified=True) for r in rows], reclassification=[dict(r, reproduced=True) for r in rows],
                   h10=[dict(r, reproduced=True) for r in rows], unchanged=[dict(r, unchanged=True) for r in rows],
                   assessment=[dict(r, reproduced=False) for r in rows])
    value = {"manifest": {"sha256": SHA}, "status": add.NOT_PASSED, "not_passed_reasons": [add.H9_REASON],
             "hard_criteria": hard}
    value.update(change)
    return value


def test_the_not_passed_report_is_accepted():
    assert len(add.check_report(report(), SHA, 2)) == 2


@pytest.mark.parametrize("mutate", [
    lambda r: r.update(manifest={"sha256": "cd" * 32}),
    lambda r: r.update(status="engineering validation passed"),
    lambda r: r.update(not_passed_reasons=["H9 not met", "H2 not met"]),
    lambda r: r["hard_criteria"][1].update(met=False),
    lambda r: r["hard_criteria"][8].update(met=True),
    lambda r: r["hard_criteria"][8]["reclassification"][0].update(reproduced=False),
    lambda r: r["hard_criteria"][8]["unchanged"][1].update(unchanged=False),
    lambda r: r["hard_criteria"][8]["assessment"][0].update(reproduced=True),
    lambda r: r["hard_criteria"][8].update(manifest={"verified": False}),
])
def test_any_other_report_is_refused(mutate):
    value = copy.deepcopy(report())
    mutate(value)
    with pytest.raises(add.Refused):
        add.check_report(value, SHA, 2)


def test_a_flight_count_other_than_the_results_is_refused():
    with pytest.raises(add.Refused):
        add.check_report(report(n=2), SHA, 3)


# ---- preconditions ------------------------------------------------------------------------------------------------
def test_preconditions_refuse_a_wrong_report_hash_and_an_existing_output(tmp_path, monkeypatch):
    monkeypatch.setattr(add.p8_manifest, "verify_current", lambda root, manifest: ({"p8_source_files": {}}, SHA))
    monkeypatch.setattr(add.p8_manifest, "source_drift", lambda frozen, root: [])
    path = tmp_path / "report.json"
    path.write_text(json.dumps(report()))
    out = tmp_path / "addendum.json"
    assert add.preconditions(tmp_path, tmp_path / "m.json", path, digest(path), out)[1] == SHA
    with pytest.raises(add.Refused):
        add.preconditions(tmp_path, tmp_path / "m.json", path, "00" * 32, out)
    out.write_text("{}")
    with pytest.raises(add.Refused):
        add.preconditions(tmp_path, tmp_path / "m.json", path, digest(path), out)


def test_preconditions_refuse_drift_and_a_bound_tool(tmp_path, monkeypatch):
    path = tmp_path / "report.json"
    path.write_text("{}")
    monkeypatch.setattr(add.p8_manifest, "verify_current", lambda root, manifest: ({"p8_source_files": {}}, SHA))
    monkeypatch.setattr(add.p8_manifest, "source_drift", lambda frozen, root: ["changed:x"])
    with pytest.raises(add.Refused):
        add.preconditions(tmp_path, tmp_path / "m.json", path, digest(path), tmp_path / "o.json")
    me = Path(add.__file__).resolve().relative_to(add.REPO).as_posix()
    monkeypatch.setattr(add.p8_manifest, "verify_current", lambda root, manifest: ({"p8_source_files": {me: "x"}}, SHA))
    monkeypatch.setattr(add.p8_manifest, "source_drift", lambda frozen, root: [])
    with pytest.raises(add.Refused):
        add.preconditions(tmp_path, tmp_path / "m.json", path, digest(path), tmp_path / "o.json")


def test_the_tool_is_not_a_bound_p8_file():
    me = Path(add.__file__).resolve().relative_to(add.REPO).as_posix()
    assert me not in add.p8_manifest.p8_source_files()
    assert Path(__file__).resolve().relative_to(add.REPO).as_posix() not in add.p8_manifest.p8_source_files()


# ---- one real flight, if present (read-only) ----------------------------------------------------------------------
def test_a_real_block_1_flight_is_explained_by_the_label_alone():
    root = Path.home() / "uav_autonomy"
    result = root / "state" / ("p8-p8-validation-60-block-01-%s.result.json"
                               % "fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a")
    if not result.exists():
        pytest.skip("no real P8 block 1 result on this host")
    entry = json.loads(result.read_text())["flights"][0]
    f = add.rep.load_flight(1, entry)
    run = Path(f["run"])
    before = add.rep7.files_state(run)
    again = add.p7e_assess.assess(run, f["record"], f["summary"])
    r = add.compare_flight(f, again, {"reproduced": False}, before, add.rep7.files_state(run))
    assert r["rule_o_reproduced"] is False and r["rule_o_matches_report"] is True
    assert r["rule_o_differing_keys"] == ["contract"]
    assert r["rule_c_holds"] is True and r["rule_c_problems"] == [] and r["run_files_unchanged"] is True
