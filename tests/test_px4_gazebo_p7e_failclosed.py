"""P7-E E2: the fail-closed rules FC-1..FC-3 (contract `p7-engineering-validation-v1` §8), offline. Nothing is launched.

* **FC-1:**
  * an exception during a block is a stop; the flown flights come back from the ledger, the flight in progress is
    named, and every other flight is listed as not flown;
  * the next block needs the previous result unstopped and accounting for every planned flight;
  * the sequencer refuses before consuming anything if the ledger exists or a live lock is held;
  * end to end through `main` with stub flights.
* **FC-2:** a failed `pgrep` fails the pre-block check and stops the batch after a flight. The frozen check would
  read it as "no process", which is the gap this rule closes.
* **FC-3:** an invalid flight with an unknown arm state is safety-unverified and stops the batch. It is re-flown
  only on positive evidence that it never armed; the frozen classifier's own record is used as input.
"""
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
P7E, P7 = REPO / "simulation/px4_gazebo_p7e", REPO / "simulation/px4_gazebo_p7"
for path in (P7E / "scripts", P7E / "validation", P7 / "scripts", REPO / "simulation/px4_gazebo/scripts",
             REPO / "simulation/px4_gazebo/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7_host  # noqa: E402
import p7e_assess as fc3  # noqa: E402
import p7e_host  # noqa: E402
import p7e_manifest as m  # noqa: E402
import run_p7e_block as seq  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

SHA = "cd" * 32
GOOD_WINDOWS = {"overlay_ac": "DED574B5-45A0-4F42-8737-46345C09C238", "power_line": "Online", "cpu_percent": [3, 4, 5]}


def schedule():
    return strict_json(P7E / "configs/p7e_schedule.json")


def done(code, out="", err=""):
    return SimpleNamespace(returncode=code, stdout=out, stderr=err)


def valid(flight, run="r"):
    return dict(flight, run=run, classification={"validity": "valid", "stop_reasons": []}, flight_record_sha256="ff" * 32)


# ---- FC-2 ---------------------------------------------------------------------------------------------------------
def test_fc2_a_failed_pgrep_is_a_failed_check_where_the_frozen_check_read_no_process():
    pattern = p7e_host.pattern()
    assert p7_host.simulator_processes(pattern, run=lambda *a, **k: done(2, "", "pgrep: bad")) == []  # the frozen gap
    with pytest.raises(p7e_host.ProcessCheckFailed, match="pgrep_failed:2"):
        p7e_host.simulator_processes(pattern, run=lambda *a, **k: done(2, "", "pgrep: bad"))

    def missing(*a, **k):
        raise FileNotFoundError("pgrep")
    with pytest.raises(p7e_host.ProcessCheckFailed, match="pgrep_unavailable"):
        p7e_host.simulator_processes(pattern, run=missing)
    assert p7e_host.simulator_processes(pattern, run=lambda *a, **k: done(1)) == []
    me = str(os.getpid())
    found = p7e_host.simulator_processes(pattern, run=lambda *a, **k: done(0, "12 /bin/px4 -d\n%s python3 x\n13 pgrep -af x\n" % me))
    assert found == ["12 /bin/px4 -d"]
    with pytest.raises(p7e_host.ProcessCheckFailed):
        p7e_host.residual(run=lambda *a, **k: done(3))


def test_fc2_the_pre_block_check_fails_closed_on_a_failed_pgrep(tmp_path):
    big = 10 ** 13
    ok = p7e_host.check(tmp_path, 20, windows=GOOD_WINDOWS, free_bytes=big, run=lambda *a, **k: done(1))
    assert ok["passed"] is True and ok["contract"] == "p7e-host-check-record-v1" and ok["frozen_contract"] == "p7-host-check-record-v1"
    bad = p7e_host.check(tmp_path, 20, windows=GOOD_WINDOWS, free_bytes=big, run=lambda *a, **k: done(2, "", "x"))
    assert bad["passed"] is False and bad["no_simulator_process"]["error"].startswith("pgrep_failed:2")
    busy = p7e_host.check(tmp_path, 20, windows=GOOD_WINDOWS, free_bytes=big, run=lambda *a, **k: done(0, "9 gz sim -s\n"))
    assert busy["passed"] is False and busy["no_simulator_process"]["found"] == ["9 gz sim -s"]
    hot = p7e_host.check(tmp_path, 20, windows=dict(GOOD_WINDOWS, cpu_percent=[40]), free_bytes=big, run=lambda *a, **k: done(1))
    assert hot["passed"] is False  # the frozen Windows checks, unchanged


def test_fc2_a_failed_pgrep_after_a_flight_stops_the_batch_with_the_flight_in_the_ledger(tmp_path):
    flights = seq.plan_flights(schedule(), 1)
    ledger = seq.Ledger(tmp_path / "ledger.jsonl")
    rules, result = seq.BlockRules(len(flights)), {}

    def residual():
        raise p7e_host.ProcessCheckFailed("pgrep_failed:2:x")
    seq.sequence(result, flights, rules, valid, lambda: [], residual, ledger)
    assert rules.stop == "process_check_failed:pgrep_failed:2:x"
    assert len(result["flights"]) == 1 and result["interrupted"] is None and len(result["not_flown"]) == 19
    acc = seq.accounting(flights, result["flights"], result["not_flown"], rules.missing, None, True)
    assert acc["complete"] and acc["status_counts"] == {"flown_valid": 1, "not_flown": 19}


# ---- FC-1 ---------------------------------------------------------------------------------------------------------
def test_fc1_an_exception_in_flight_is_a_stop_and_the_state_comes_back_from_the_ledger(tmp_path):
    flights = seq.plan_flights(schedule(), 1)
    ledger = seq.Ledger(tmp_path / "ledger.jsonl")
    rules, result, flown = seq.BlockRules(len(flights)), {}, []

    def fly(flight):
        if len(flown) == 2:
            raise RuntimeError("evaluator crashed")
        flown.append(flight)
        return valid(flight)
    seq.sequence(result, flights, rules, fly, lambda: [], lambda: [], ledger)
    assert rules.stop == "sequencer_exception:RuntimeError" and result["recovered_from_ledger"] is True
    assert [f["plan_index"] for f in result["flights"]] == [0, 1] and result["interrupted"]["plan_index"] == 2
    assert [f["plan_index"] for f in result["not_flown"]] == list(range(3, 20))
    acc = seq.accounting(flights, result["flights"], result["not_flown"], [], result["interrupted"], True)
    assert acc["complete"] and acc["status_counts"] == {"flown_valid": 2, "interrupted": 1, "not_flown": 17}


def test_fc1_an_interrupt_or_a_failing_drift_check_is_a_stop_too(tmp_path):
    flights = seq.plan_flights(schedule(), 2)
    for n, (fly, drift, expected) in enumerate(((lambda f: (_ for _ in ()).throw(KeyboardInterrupt()), lambda: [], "KeyboardInterrupt"),
                                                 (valid, lambda: (_ for _ in ()).throw(ValueError("bad json")), "ValueError"))):
        rules, result = seq.BlockRules(len(flights)), {}
        seq.sequence(result, flights, rules, fly, drift, lambda: [], seq.Ledger(tmp_path / ("l%d.jsonl" % n)))
        assert rules.stop == "sequencer_exception:" + expected
        assert len(result["flights"]) + len(result["not_flown"]) + (result["interrupted"] is not None) == 20


def test_fc1_replacements_queued_before_an_exception_are_listed_as_not_flown(tmp_path):
    flights = seq.plan_flights(schedule(), 1)
    rules, result, calls = seq.BlockRules(len(flights)), {}, []

    def fly(flight):
        calls.append(flight)
        if len(calls) == 1:
            return dict(flight, run="r", classification={"validity": "infrastructure_invalid", "stop_reasons": []})
        if len(calls) == 5:
            raise OSError("disk")
        return valid(flight)
    seq.sequence(result, flights, rules, fly, lambda: [], lambda: [], seq.Ledger(tmp_path / "l.jsonl"))
    replacement = [f for f in result["not_flown"] if f["attempt"] == 2]
    assert [f["plan_index"] for f in replacement] == [0] and result["interrupted"]["plan_index"] == 4
    acc = seq.accounting(flights, result["flights"], result["not_flown"], [], result["interrupted"], True)
    assert acc["complete"] and acc["flights"][0]["status"] == "not_flown"


def test_fc1_accounting_finds_an_unaccounted_or_unresolved_flight():
    flights = seq.plan_flights(schedule(), 1)
    records = [valid(f) for f in flights[:19]]
    assert not seq.accounting(flights, records, [], [], None, False)["complete"]  # one flight vanished
    invalid = dict(flights[19], run="r", classification={"validity": "infrastructure_invalid", "stop_reasons": []})
    assert not seq.accounting(flights, records + [invalid], [], [], None, False)["complete"]  # neither replaced nor missing
    lost = [{"slot": flights[19]["slot"], "layout": flights[19]["layout"], "controller": flights[19]["controller"]}]
    full = seq.accounting(flights, records + [invalid], [], lost, None, False)
    assert full["complete"] and full["all_flown_or_missing"] and full["status_counts"]["missing"] == 1


def block_result(root, block, flights=None, **changes):
    """A result written as the sequencer writes it, with its ledger."""
    planned = seq.plan_flights(schedule(), block)
    flights = [valid(f) for f in planned] if flights is None else flights
    ledger = Path(root) / "state" / ("ledger-%d.jsonl" % block)
    ledger.write_text("".join(json.dumps(f, sort_keys=True) + "\n" for f in flights))
    value = {"manifest_sha256": SHA, "block": block, "planned": planned, "flights": flights, "not_flown": [],
             "interrupted": None, "missing_pairs": [], "stopped": False, "stop_reason": None,
             "ledger": {"path": str(ledger), "sha256": digest(ledger)}}
    value.update(changes)
    path = m.result_path(root, block, SHA)
    path.write_text(json.dumps(value))
    return path


def test_fc1_the_next_block_needs_an_unstopped_complete_result_of_the_previous_one(tmp_path):
    (tmp_path / "state").mkdir()
    assert seq.previous_block_ok(tmp_path, 1, SHA) == (True, None)
    assert seq.previous_block_ok(tmp_path, 2, SHA)[0] is False  # no result
    planned = seq.plan_flights(schedule(), 1)
    cases = {"stopped": {"stopped": True, "stop_reason": "x"},
             "frozen FC-1 gap: unstopped with no flights": {"flights": []},
             "unflown": {"not_flown": planned[19:]},
             "other plan": {"planned": planned[:19]},
             "other manifest": {"manifest_sha256": "ef" * 32},
             "stopped missing": {"stopped": None}}
    for name, change in cases.items():
        path = block_result(tmp_path, 1, **change)
        assert seq.previous_block_ok(tmp_path, 2, SHA)[0] is False, name
        path.unlink()
    path = block_result(tmp_path, 1)
    assert seq.previous_block_ok(tmp_path, 2, SHA) == (True, None)
    Path(strict_json(path)["ledger"]["path"]).write_text("")  # a ledger that no longer matches its result
    assert seq.previous_block_ok(tmp_path, 2, SHA)[0] is False


def test_fly_one_never_raises_after_the_runtime_returned(tmp_path):
    flight = seq.plan_flights(schedule(), 1)[0]
    none = seq.fly_one(flight, "engineering-30", "m", "a", run=lambda *a, **k: done(1, "no evidence line"))
    assert none["run"] is None and "classification" not in none
    rules = seq.BlockRules(20)
    rules.after(flight, none)
    assert rules.stop == "runtime_did_not_create_evidence"
    run = tmp_path / "run"
    (run / "p7").mkdir(parents=True)
    (run / "summary.json").write_text("{}")
    (run / "p4-evaluation.json").write_text("{not json")
    calls = []

    def runner(command, **kwargs):
        calls.append(command)
        return done(0, "P7-E evidence: %s\n" % run) if len(calls) == 1 else done(0)

    def classify(path):
        raise RuntimeError("classifier_failed:1")
    record = seq.fly_one(flight, "engineering-30", "m", "a", classify=classify, run=runner)
    assert calls[0][:2] == ["bash", str(P7E / "scripts/run_p7e.sh")] and "--block" in calls[0]
    assert calls[1] == ["bash", str(seq.EVIDENCE["p4"]), str(run)]
    assert record["evaluator_error"] and record["classification"]["validity"] is None
    assert record["summary_sha256"] == digest(run / "summary.json") and record["flight_record_sha256"] is None
    rules = seq.BlockRules(20)
    rules.after(flight, record)
    assert rules.stop == "unclassified_flight"


def staged_main(tmp_path, monkeypatch, development=False):
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"):
        monkeypatch.setenv(key, "1")
    monkeypatch.setenv("GWM_SIM_ROOT", str(tmp_path))
    (tmp_path / "state").mkdir(exist_ok=True)
    frozen = {"development": development, "runtime": {"mc_trace_build": "B"}, "p7": {"sha256": m.P7_SHA256}, "baselines": {}}
    monkeypatch.setattr(seq.p7e_manifest, "verify_current", lambda root, path: (frozen, SHA))
    monkeypatch.setattr(seq.p7e_manifest, "source_drift", lambda frozen, root=None: [])
    monkeypatch.setitem(sys.modules, "mc_trace_runtime", SimpleNamespace(selected_build=lambda root, sim: "B", CONTRACT="c"))
    monkeypatch.setattr(seq.p7e_host, "check", lambda root, n: {"passed": True, "flights": n})
    monkeypatch.setattr(seq.p7e_host, "residual", lambda: [])
    auth = tmp_path / "auth-1.json"
    if not auth.exists():
        atomic_json(auth, m.authorization_template(1, SHA, "operator"), exclusive=True)
    return ["--scope", "engineering-30", "--block", "1", "--manifest", str(tmp_path / "m.json"), "--authorization", str(auth)]


def test_main_flies_a_block_end_to_end_with_stub_flights_and_block_2_may_then_start(tmp_path, monkeypatch):
    argv = staged_main(tmp_path, monkeypatch)
    monkeypatch.setattr(seq, "fly_one", lambda f, scope, manifest, authorization: valid(f))
    assert seq.main(argv) == 0
    result = strict_json(m.result_path(tmp_path, 1, SHA))
    assert result["stopped"] is False and len(result["flights"]) == 20 and result["accounting"]["complete"]
    assert result["contract"] == "p7e-block-result-v1" and result["fail_closed_rules"] == ["FC-1", "FC-2", "FC-3"]
    assert m.marker_path(tmp_path, 1, SHA).exists() and not m.live_locks(tmp_path)
    assert seq.previous_block_ok(tmp_path, 2, SHA) == (True, None)
    with pytest.raises(SystemExit, match="already has a result"):
        seq.main(argv)


def test_main_writes_a_stopped_result_on_an_exception_and_releases_the_lock(tmp_path, monkeypatch):
    argv = staged_main(tmp_path, monkeypatch)
    count = []

    def fly(f, scope, manifest, authorization):
        count.append(f)
        if len(count) == 4:
            raise RuntimeError("boom")
        return valid(f)
    monkeypatch.setattr(seq, "fly_one", fly)
    assert seq.main(argv) == 1
    result = strict_json(m.result_path(tmp_path, 1, SHA))
    assert result["stopped"] is True and result["stop_reason"] == "sequencer_exception:RuntimeError"
    assert len(result["flights"]) == 3 and result["interrupted"]["plan_index"] == 3 and len(result["not_flown"]) == 16
    assert result["accounting"]["complete"] and not m.live_locks(tmp_path)
    assert seq.previous_block_ok(tmp_path, 2, SHA)[0] is False


def test_main_refuses_before_consuming_anything(tmp_path, monkeypatch):
    argv = staged_main(tmp_path, monkeypatch)
    monkeypatch.setattr(seq.p7e_host, "check", lambda root, n: {"passed": False})
    assert seq.main(argv) == 1  # not flown: a record, and the authorization is not consumed
    assert list((tmp_path / "state").glob("*.not-flown-*.json")) and not m.marker_path(tmp_path, 1, SHA).exists()
    argv = staged_main(tmp_path, monkeypatch)
    import p7_manifest
    p7_manifest.acquire_live_lock(tmp_path, SHA)
    with pytest.raises(SystemExit, match="live P7 or P7-E"):
        seq.main(argv)
    p7_manifest.release_live_lock(tmp_path, SHA)
    ledger = m.result_path(tmp_path, 1, SHA).with_name(m.result_path(tmp_path, 1, SHA).name.replace(".result.json", ".ledger.jsonl"))
    ledger.write_text("")
    with pytest.raises(SystemExit, match="Ledger exists"):
        seq.main(argv)
    assert not m.marker_path(tmp_path, 1, SHA).exists()
    ledger.unlink()
    argv = staged_main(tmp_path, monkeypatch, development=True)
    with pytest.raises(SystemExit, match="development candidate"):
        seq.main(argv)
    argv = staged_main(tmp_path, monkeypatch)
    argv[3] = "2"
    with pytest.raises(PermissionError, match="Authorization does not bind block"):
        seq.main(argv)
    assert not m.marker_path(tmp_path, 2, SHA).exists()


# ---- FC-3 ---------------------------------------------------------------------------------------------------------
def staged_run(tmp_path, record=None, finalized=True, processes=("px4", "ros-controller"), states=("PRESTREAM_SAFE_SETPOINTS",),
               result=True, listed=True):
    run = tmp_path / "run"
    (run / "p7").mkdir(parents=True)
    artifacts = []
    if result:
        (run / "controller-result.json").write_text(json.dumps({"transitions": [{"to": s, "sim_s": 1.0} for s in states]}))
        artifacts.append({"path": "controller-result.json", "sha256": digest(run / "controller-result.json") if listed else "00" * 32})
    summary = {"finalized": finalized, "processes": [{"name": n} for n in processes], "artifacts": artifacts}
    (run / "summary.json").write_text(json.dumps(summary))
    record = record or {"validity": "infrastructure_invalid", "stop_reasons": [], "outcome": None,
                        "armed_evidence": {"ulog_armed": None, "takeoff_transition": False}}
    (run / "p7/flight-record.json").write_text(json.dumps(record))
    return run, record


@pytest.mark.parametrize("case, kwargs, ulog, validity, basis", [
    ("ULog read, never armed", {}, False, "infrastructure_invalid", "ulog_never_armed"),
    ("controller never started", {"processes": ("px4",)}, None, "infrastructure_invalid", "controller_never_started"),
    ("no REQUEST_ARM in the controller's own result", {}, None, "infrastructure_invalid", "no_request_arm"),
    ("REQUEST_ARM reached", {"states": ("REQUEST_ARM", "VERIFY_ARMED")}, None, "unknown", None),
    ("controller result not hashed by the summary", {"listed": False}, None, "unknown", None),
    ("no controller result", {"result": False}, None, "unknown", None),
    ("stale summary (not finalized)", {"finalized": False, "processes": ("px4",)}, None, "unknown", None),
])
def test_fc3_an_invalid_flight_is_re_flown_only_on_positive_evidence_that_it_never_armed(tmp_path, case, kwargs, ulog, validity, basis):
    record = {"validity": "infrastructure_invalid", "stop_reasons": [], "outcome": None,
              "armed_evidence": {"ulog_armed": ulog, "takeoff_transition": False}}
    run, record = staged_run(tmp_path, record, **kwargs)
    value = fc3.assess(run, record)
    assert (value["validity"], value["fc3"]["basis"]) == (validity, basis), case
    assert (fc3.FC3_STOP in value["stop_reasons"]) == (validity == "unknown")
    assert value["frozen"]["validity"] == "infrastructure_invalid" and value["flight_record_sha256"] == digest(run / fc3.RECORD)
    rules = seq.BlockRules(20)
    flight = seq.plan_flights(schedule(), 1)[0]
    rules.after(flight, dict(flight, run=str(run), classification=value))
    if validity == "unknown":
        assert rules.stop == "stop_rule:" + fc3.FC3_STOP  # safety-unverified: the batch stops
    else:
        assert rules.stop is None and rules.replacements[0]["attempt"] == 2  # re-flown once at the end of the block


def test_fc3_valid_flights_and_frozen_stop_reasons_pass_through(tmp_path):
    record = {"validity": "valid", "outcome": "goal_success", "stop_reasons": ["safety_event:safety:collision_contact"],
              "armed_evidence": {"ulog_armed": True, "takeoff_transition": True}}
    run, record = staged_run(tmp_path, record)
    value = fc3.assess(run, record)
    assert value["validity"] == "valid" and value["outcome"] == "goal_success" and value["fc3"]["applied"] is False
    assert value["stop_reasons"] == ["safety_event:safety:collision_contact"]
    odd = fc3.assess(run, dict(record, validity=None))
    assert odd["validity"] == "unknown" and fc3.FC3_STOP in odd["stop_reasons"]


def test_fc3_on_the_frozen_classifiers_own_record_of_a_flight_without_a_readable_ulog(tmp_path):
    run, _ = staged_run(tmp_path, states=("PRESTREAM_SAFE_SETPOINTS", "REQUEST_OFFBOARD", "VERIFY_OFFBOARD", "REQUEST_ARM"))
    (run / "p7/flight-record.json").unlink()
    summary = strict_json(run / "summary.json")
    summary["controller_result"] = strict_json(run / "controller-result.json")
    (run / "summary.json").write_text(json.dumps(summary))
    (run / "p4_avoidance.yaml").write_text((REPO / "simulation/px4_gazebo_p4/configs/p4_avoidance.yaml").read_text())
    frozen = fc3.frozen_classifier()
    record = frozen.build_record(run, ulog_reader=lambda s: None)
    assert record["validity"] == "infrastructure_invalid" and record["stop_reasons"] == []  # the frozen rule would re-fly it
    value = fc3.assess(run, record)
    assert value["validity"] == "unknown" and value["stop_reasons"] == [fc3.FC3_STOP]  # FC-3 stops instead


def test_fc3_classify_runs_the_frozen_classifier_first_and_writes_the_assessment_once(tmp_path, monkeypatch):
    run, record = staged_run(tmp_path, processes=("px4",))
    seen, original = [], fc3.frozen_classifier
    assert original().WRAPPER == P7 / "scripts/verify_p7_evidence.sh"  # the frozen wrapper and environment
    monkeypatch.setattr(fc3, "frozen_classifier", lambda: SimpleNamespace(classify=lambda r: seen.append(r) or record))
    value = fc3.classify(run)
    assert seen == [run] and value == strict_json(run / fc3.OUTPUT) and value["fc3"]["basis"] == "controller_never_started"
    with pytest.raises(RuntimeError, match="first assessment is retained"):
        fc3.classify(run)
