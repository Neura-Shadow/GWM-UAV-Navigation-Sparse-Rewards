"""P7-E E2: the engineering report code (plan `p7e-report-plan-v1`), offline, on a synthetic 60-flight batch.

The batch is written exactly as the sequencer would leave it: 3 block results, their ledgers, and per flight a
summary, an accepted-shape evaluation, a frozen-shape flight record, and the FC-3 assessment, all hash-matched.
* A clean batch passes. The report carries the contract's statement verbatim and holds no p-value.
* Each hard criterion H1-H9 fails on its own negative case. A stopped or incomplete batch is never a pass.
* An R breach is a risk-register entry, not an engineering failure.
* The statistics are the frozen interval functions, with every difference P5 minus P4.
* The H9 view and re-classification use the frozen wrapper and change no file of the run.
"""
import json
from pathlib import Path
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
P7E, P7 = REPO / "simulation/px4_gazebo_p7e", REPO / "simulation/px4_gazebo_p7"
for path in (P7E / "analysis", P7E / "scripts", P7E / "validation", P7 / "scripts", P7 / "analysis",
             REPO / "simulation/px4_gazebo/scripts", REPO / "simulation/px4_gazebo/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7_analysis  # noqa: E402
import p7e_assess as fc3  # noqa: E402
import p7e_manifest as m  # noqa: E402
import p7e_report as rep  # noqa: E402
import run_p7e_block as seq  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

SCHEDULE = strict_json(P7E / "configs/p7e_schedule.json")
SLOTS = {s["slot"]: s for s in SCHEDULE["slots"]}


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True))


def success_of(controller, slot):
    return not (slot % 4 == 0 if controller == "p4" else slot % 7 == 0)


def documents(flight):
    c, slot = flight["controller"], flight["slot"]
    ok = success_of(c, slot)
    summary = {"finalized": True, "processes": [{"name": "px4"}, {"name": "ros-controller"}], "artifacts": [],
               "p7": {"item": flight["layout"], "world": {"identical": True, "generated_sha256": SLOTS[slot]["world_sha256"]}},
               "controller_result": {"failure": None}, "wall_duration_s": 160.0 + slot, "run_bytes": 1_200_000_000}
    evaluation = {"status": "passed", "failures": [], "native_commands": {"status": "passed"}, "p2_timing_status": "passed",
                  "safety": {"min_truth_clearance_m": 0.40 + 0.01 * slot + (0.02 if c == "p5" else 0.0), "collisions": 0,
                             "tracking_max_m": 0.12},
                  "p2_integrity": {"final_landed": True, "final_disarmed": True, "command_ids": [176, 400, 21],
                                   "failsafe_observed": False},
                  "performance": {"rate_hz": 10.0, "latency_max_sim_s": 0.06, "latency_p95_sim_s": 0.05, "compute_p99_s": 0.016,
                                  "compute_max_s": 0.04, "receipt_gap_max_wall_s": 0.07},
                  "depth_stream": {"delivery": {"delivered_hz_sim": 30.3, "receipt_gap_max_wall_s": 0.06}},
                  "response_latency_sim_s": None, "rtf": {"rtf": 0.99}}
    if c == "p5":
        evaluation["policy"] = {"inference_s": {"p99": 0.0008, "max": 0.002}, "failure_ticks": 0, "outside_mask": 0,
                                "fallback_decisions": 0, "participation_fraction": 1.0}
    record = {"contract": "p7-flight-record-v1", "controller": c, "item": flight["layout"], "validity": "valid",
              "outcome": "goal_success" if ok else "failure", "stop_reasons": [], "p2_ulog_check": None,
              "armed_evidence": {"ulog_armed": True, "takeoff_transition": True},
              "classes": {"safety_events": [], "safety_unverified": [], "mission_failures": [], "integrity_events": []},
              "metrics": {"min_truth_clearance_m": evaluation["safety"]["min_truth_clearance_m"],
                          "time_to_goal_sim_s": (60.0 if c == "p4" else 57.5) + 0.3 * slot if ok else None,
                          "path_length_m": (40.0 if c == "p4" else 39.0) + 0.1 * slot if ok else None,
                          "abort_reason": None if ok else "no_progress", "p2_failure": None}}
    return summary, evaluation, record


def write_flight(root, flight, tweak=None):
    run = Path(root) / "runs" / ("b%d-s%03d-%s-a%d" % (flight["block"], flight["slot"], flight["controller"], flight["attempt"]))
    summary, evaluation, record = documents(flight)
    if tweak:
        tweak(flight, summary, evaluation, record, run)
    write(run / "summary.json", summary)
    write(run / rep.EVALUATIONS[flight["controller"]], evaluation)
    write(run / fc3.RECORD, record)
    write(run / "p7/layout.json", {"item": flight["layout"]})
    assessment = fc3.assess(run, record)
    write(run / fc3.OUTPUT, assessment)
    return dict(flight, run=str(run), summary_sha256=digest(run / "summary.json"),
                evaluation_sha256=digest(run / rep.EVALUATIONS[flight["controller"]]),
                flight_record_sha256=digest(run / fc3.RECORD), assessment_sha256=digest(run / fc3.OUTPUT), classification=assessment)


def batch(root, tweak=None, blocks=(1, 2, 3), stop=None):
    """The 3 blocks as the sequencer writes them; `stop` = (block, flights flown, reason) cuts one block short."""
    root = Path(root)
    (root / "state").mkdir(parents=True, exist_ok=True)
    frozen = {"p7": {"sha256": m.P7_SHA256}, "baselines": {"p7_sha256": m.P7_SHA256}, "specs": m.specs_identity(), "frozen_unix_ns": 1}
    write(root / "manifest.json", frozen)
    sha = m.manifest_sha(frozen)
    for block in blocks:
        planned = seq.plan_flights(SCHEDULE, block)
        flown = planned if not stop or stop[0] != block else planned[:stop[1]]
        records = [write_flight(root, f, tweak) for f in flown]
        not_flown = [] if flown is planned else planned[len(flown):]
        ledger = m.result_path(root, block, sha).with_name("ledger-%d.jsonl" % block)
        ledger.write_text("".join(json.dumps(r, sort_keys=True) + "\n" for r in records))
        stopped = bool(not_flown)
        result = {"contract": "p7e-block-result-v1", "scope": m.SCOPE, "block": block, "manifest_sha256": sha,
                  "planned": planned, "flights": records, "not_flown": not_flown, "interrupted": None, "replacements": [],
                  "missing_pairs": [], "stopped": stopped, "stop_reason": stop[2] if stopped else None, "replacement_cap": 2,
                  "host_check": {"passed": True, "host_cpu": {"mean_percent": 4.0}, "ac_overlay": {"passed": True},
                                 "ac_power": {"passed": True}},
                  "ledger": {"path": str(ledger), "sha256": digest(ledger)}}
        write(m.result_path(root, block, sha), result)
    return root / "manifest.json", sha


def report(root, manifest, sha, reclassify_fn=None, verify_fn=None):
    same = reclassify_fn or (lambda run, work: (strict_json(Path(run) / fc3.RECORD), None))
    return rep.build_report(root, manifest, reclassify_fn=same, verify_fn=verify_fn or (lambda root, path: (None, sha)))


def hard(value):
    return {h["id"]: h for h in value["hard_criteria"]}


def keys(value):
    if isinstance(value, dict):
        return {str(k) for k in value} | {k for v in value.values() for k in keys(v)}
    if isinstance(value, list):
        return {k for v in value for k in keys(v)}
    return set()


# ---- a clean batch ----------------------------------------------------------------------------------------------
def test_a_clean_batch_passes_with_the_required_statement_and_no_p_value(tmp_path):
    manifest, sha = batch(tmp_path)
    value = report(tmp_path, manifest, sha)
    assert value["status"] == rep.PASSED == "engineering validation passed" and value["not_passed_reasons"] == []
    assert all(h["met"] for h in value["hard_criteria"]) and [h["id"] for h in value["hard_criteria"]] == ["H%d" % i for i in range(1, 10)]
    assert value["flights_flown"] == 60 and hard(value)["H8"]["batch_completed"] is True
    assert value["required_statement"] == rep.plan()["required_statement"] and value["statement_of_a_pass"] == rep.PASS_STATEMENT
    assert not {k for k in keys(value) if "p_value" in k} - {"p_values"} and value["p_values"] == "none (decision E7)"
    assert value["report_plan_canonical_sha256"] == m.APPROVED_SPECS["report_plan"] and value["manifest"]["sha256"] == sha
    assert value["risk_register"]["open"] == ["RR1 P2 observation-gap RECOVERY", "RR2 P5 repeat-behaviour inconsistency"]
    assert value["risk_register"]["new_entries_from_R_breaches"] == [] and not value["reported_errors"]
    json.dumps(value, allow_nan=False)  # serializable as written


def test_the_reported_statistics_are_the_frozen_interval_functions_with_p5_minus_p4(tmp_path):
    manifest, sha = batch(tmp_path)
    value = report(tmp_path, manifest, sha)
    r = value["reported_criteria"]
    slots = range(1, 31)
    for c in ("p4", "p5"):
        x = sum(success_of(c, s) for s in slots)
        assert r["R1"]["per_controller"][c]["goal_success"] == x and r["R1"]["per_controller"][c]["valid_flights"] == 30
        assert r["R1"]["per_controller"][c]["ci95_clopper_pearson"] == p7_analysis.clopper_pearson(x, 30)
    b = sum(success_of("p5", s) and not success_of("p4", s) for s in slots)
    c_ = sum(success_of("p4", s) and not success_of("p5", s) for s in slots)
    assert (r["R2"]["p5_only"], r["R2"]["p4_only"], r["R2"]["pairs"]) == (b, c_, 30) and b > c_
    assert r["R2"]["difference_p5_minus_p4"] == (b - c_) / 30 and r["R2"]["difference_ci95_tango"] == p7_analysis.tango_interval(b, c_, 30)
    both = [s for s in slots if success_of("p4", s) and success_of("p5", s)]
    diffs = [((57.5 + 0.3 * s) - (60.0 + 0.3 * s)) for s in both]
    assert r["R7"]["time_to_goal_sim_s"]["paired_difference_p5_minus_p4"] == p7_analysis.hodges_lehmann(diffs)
    assert r["R7"]["time_to_goal_sim_s"]["paired_difference_p5_minus_p4"]["estimate"] == pytest.approx(-2.5)
    assert value["paired_min_truth_clearance"]["difference"]["median"] == pytest.approx(0.02)
    assert value["safety_per_controller"]["p4"]["safety_event_bound"] == pytest.approx(1 - 0.05 ** (1 / 30))
    assert r["R5"]["p5_flights"] == 30 and r["R8"]["host_check_per_block"][1]["passed"] is True
    assert r["R3"]["per_controller"]["p4"]["observation_gap_aborts"] == 0 and r["R9"]["per_controller"]["p4"]["abort_reason"] == {"no_progress": 7}


# ---- each hard criterion fails on its own negative case ---------------------------------------------------------
def at(slot, controller, change):
    def tweak(flight, summary, evaluation, record, run):
        if (flight["slot"], flight["controller"]) == (slot, controller):
            change(summary, evaluation, record)
    return tweak


def collision(s, e, r):
    e["failures"].append("safety:collision_contact")
    r["classes"]["safety_events"].append("safety:collision_contact")


def low_clearance(s, e, r):
    e["safety"]["min_truth_clearance_m"] = 0.15


def no_clearance(s, e, r):
    del e["safety"]["min_truth_clearance_m"]


def hold_motion(s, e, r):
    r["classes"]["safety_events"].append("unsafe:hold_tick_motion")


def policy_breach(s, e, r):
    e["failures"].append("native_commands:unexpected_command")
    r["classes"]["safety_events"].append("native_commands:unexpected_command")


def policy_unchecked(s, e, r):
    del e["native_commands"]


def not_disarmed(s, e, r):
    e["p2_integrity"]["final_disarmed"] = False


def d15_short_sequence(s, e, r):
    r["p2_ulog_check"] = {"facts": {"own_commands": [176, 400], "final_landed": True, "final_disarmed": True,
                                    "failsafe_observed": False}, "policy_a_checked": True, "failures": ["p2_ulog:command_sequence"]}
    r["classes"]["safety_events"].append("p2_ulog:command_sequence")


def unverified(s, e, r):
    r["classes"]["safety_unverified"].append("safety:truth_poses_missing")


def arm_state_unknown(s, e, r):  # FC-3: invalid, no readable ULog, the controller started, no own result
    r.update(validity="infrastructure_invalid", outcome=None, armed_evidence={"ulog_armed": None, "takeoff_transition": False})


@pytest.mark.parametrize("change, criterion", [
    (collision, "H1"), (low_clearance, "H2"), (no_clearance, "H2"), (hold_motion, "H3"), (policy_breach, "H4"),
    (policy_unchecked, "H4"), (not_disarmed, "H5"), (d15_short_sequence, "H5"), (unverified, "H6"), (arm_state_unknown, "H6"),
])
def test_each_hard_criterion_fails_on_its_own_event(tmp_path, change, criterion):
    manifest, sha = batch(tmp_path, at(17, "p5", change))
    value = report(tmp_path, manifest, sha)
    failed = [h["id"] for h in value["hard_criteria"] if not h["met"]]
    assert value["status"] == rep.NOT_PASSED and "%s not met" % criterion in value["not_passed_reasons"]
    # each event fails its own criterion only; a flight with an unknown arm state inside an unstopped block is also
    # an accounting inconsistency (the sequencer would have stopped), so H8 fails with it
    assert failed == sorted({criterion} | ({"H8"} if change is arm_state_unknown else set()))


def test_d15_landing_facts_from_the_ulog_satisfy_h5(tmp_path):
    def d15(s, e, r):
        e["p2_integrity"] = {}
        e["failures"].append("p2_integrity_error:KeyError")
        r["classes"]["integrity_events"].append("p2_integrity_error:KeyError")
        r["p2_ulog_check"] = {"facts": {"own_commands": [176, 400, 21], "final_landed": True, "final_disarmed": True,
                                        "failsafe_observed": False}, "policy_a_checked": True, "failures": []}
    manifest, sha = batch(tmp_path, at(3, "p4", d15))
    value = report(tmp_path, manifest, sha)
    assert hard(value)["H5"]["met"] and value["status"] == rep.PASSED
    assert value["reported_criteria"]["R10"]["per_controller"]["p4"] == {"p2_integrity_error:KeyError": 1}


def test_evidence_changed_after_its_block_fails_h7_and_counts_as_unverified(tmp_path):
    manifest, sha = batch(tmp_path)
    run = Path(strict_json(m.result_path(tmp_path, 2, sha))["flights"][5]["run"])
    record = strict_json(run / fc3.RECORD)
    record["metrics"]["min_truth_clearance_m"] = 9.9
    write(run / fc3.RECORD, record)
    value = report(tmp_path, manifest, sha)
    assert not hard(value)["H7"]["met"] and not hard(value)["H6"]["met"] and value["status"] == rep.NOT_PASSED
    assert hard(value)["H7"]["value"] == "59/60"


def test_a_changed_ledger_fails_h7(tmp_path):
    manifest, sha = batch(tmp_path)
    ledger = Path(strict_json(m.result_path(tmp_path, 1, sha))["ledger"]["path"])
    ledger.write_text("\n".join(ledger.read_text().splitlines()[:-1]) + "\n")
    value = report(tmp_path, manifest, sha)
    assert not hard(value)["H7"]["met"] and hard(value)["H7"]["ledgers"][1] == \
        {"present": True, "hash_matches": False, "lines_match_result": False}


def test_a_stopped_block_explains_its_unflown_flights_but_the_batch_is_never_a_pass(tmp_path):
    manifest, sha = batch(tmp_path, blocks=(1, 2), stop=(2, 5, "simulator_process_outlived_flight"))
    value = report(tmp_path, manifest, sha)
    h8 = hard(value)["H8"]
    assert h8["met"] and h8["batch_completed"] is False and h8["blocks"][3] == {"result": False, "explained": True,
                                                                               "by": "block 2 stopped: simulator_process_outlived_flight"}
    assert value["status"] == rep.NOT_PASSED and "the declared 60-flight validation did not complete (contract §7.1)" in value["not_passed_reasons"]


def test_a_missing_block_without_an_earlier_stop_fails_h8(tmp_path):
    manifest, sha = batch(tmp_path, blocks=(1, 3))
    h8 = hard(report(tmp_path, manifest, sha))["H8"]
    assert not h8["met"] and h8["blocks"][2]["explained"] is False


def test_h8_rejects_a_result_whose_plan_or_replacement_cap_differs(tmp_path):
    manifest, sha = batch(tmp_path)
    path = m.result_path(tmp_path, 3, sha)
    result = strict_json(path)
    result["replacement_cap"] = 5
    write(path, result)
    h8 = hard(report(tmp_path, manifest, sha))["H8"]
    assert not h8["met"] and h8["blocks"][3]["replacements_within_cap"] is False


@pytest.mark.parametrize("case", ["record differs", "classifier failed", "manifest does not verify", "world differs"])
def test_h9_fails_on_any_irreproducible_record_or_hash(tmp_path, case):
    world = (lambda f, s, e, r, run: s["p7"]["world"].update(generated_sha256="00" * 32) if f["slot"] == 9 else None)
    manifest, sha = batch(tmp_path, world if case == "world differs" else None)

    def differs(run, work):
        record = strict_json(Path(run) / fc3.RECORD)
        return (dict(record, outcome="failure") if record["item"] == SLOTS[12]["layout"] else record), None

    def broken(root, path):
        raise ValueError("Current inputs differ from the frozen P7-E candidate: p7e_source_files")
    kwargs = {"record differs": {"reclassify_fn": differs},
              "classifier failed": {"reclassify_fn": lambda run, work: (None, "classifier_failed:1:x")},
              "manifest does not verify": {"verify_fn": broken}, "world differs": {}}[case]
    value = report(tmp_path, manifest, sha, **kwargs)
    assert not hard(value)["H9"]["met"] and value["status"] == rep.NOT_PASSED


def test_an_r_breach_is_a_risk_register_entry_and_not_an_engineering_failure(tmp_path):
    def slow(s, e, r):
        e["failures"].append("performance:snapshot_rate")
        r["classes"]["integrity_events"].append("performance:snapshot_rate")
    manifest, sha = batch(tmp_path, at(20, "p4", slow))
    value = report(tmp_path, manifest, sha)
    assert value["status"] == rep.PASSED
    entries = value["risk_register"]["new_entries_from_R_breaches"]
    assert [(e["criterion"], e["item"], e["slot"], e["controller"]) for e in entries] == [("R4", "performance:snapshot_rate", 20, "p4")]


def test_the_report_refuses_a_changed_plan_and_is_written_once(tmp_path, monkeypatch):
    monkeypatch.setitem(m.APPROVED_SPECS, "report_plan", "00" * 32)
    with pytest.raises(SystemExit, match="report plan differs"):
        rep.plan()
    monkeypatch.undo()
    monkeypatch.setenv("GWM_SIM_ROOT", str(tmp_path))
    monkeypatch.setattr(rep, "build_report", lambda root, manifest: {"status": rep.NOT_PASSED, "not_passed_reasons": ["x"]})
    out = tmp_path / "report.json"
    assert rep.main(["--manifest", "m.json", "--output", str(out)]) == 1 and strict_json(out)["status"] == rep.NOT_PASSED
    with pytest.raises(SystemExit, match="first report is retained"):
        rep.main(["--manifest", "m.json", "--output", str(out)])


# ---- H9 view and the cost readers ---------------------------------------------------------------------------------
def test_the_h9_view_runs_the_frozen_wrapper_without_the_record_and_changes_no_file(tmp_path):
    manifest, sha = batch(tmp_path, blocks=(1,))
    run = Path(strict_json(m.result_path(tmp_path, 1, sha))["flights"][0]["run"])
    before = rep.files_state(run)
    seen = []

    def runner(command, **kwargs):
        seen.append(command)
        view = Path(command[2])
        assert sorted(p.name for p in view.iterdir()) == sorted(p.name for p in run.iterdir() if p.name != "p7e")
        assert not (view / fc3.RECORD).exists() and not (view / "p7e").exists() and (view / "p7/layout.json").is_symlink()
        assert (view / "summary.json").resolve() == (run / "summary.json").resolve() and view.name == run.name
        (view / fc3.RECORD).write_text((run / fc3.RECORD).read_text())  # what the frozen classifier writes
        return type("Done", (), {"returncode": 0, "stderr": ""})()
    record, error = rep.reclassify(run, tmp_path / "work", runner=runner)
    assert error is None and record == strict_json(run / fc3.RECORD)
    assert seen[0][:2] == ["bash", str(P7 / "scripts/verify_p7_evidence.sh")] and rep.files_state(run) == before


def test_cost_counts_stalls_and_process_cpu_from_the_runs_own_ledgers(tmp_path):
    run = tmp_path / "run"
    (run / "source").mkdir(parents=True)
    ticks = [(0.004 * i, 1_000_000_000 + 4_000_000 * i) for i in range(1, 6)] + [(0.024, 1_000_000_000 + 4_000_000 * 5 + 40_000_000)]
    (run / "source/clock.jsonl").write_text("".join(json.dumps({"sim_s": s, "receipt_monotonic_ns": t}) + "\n" for s, t in ticks))
    (run / "resources.jsonl").write_text("".join(json.dumps(r) + "\n" for r in (
        {"monotonic_s": 10.0, "mem_available_kb": 7_000_000, "processes": {"gazebo": {"cpu_s": 1.0}}},
        {"monotonic_s": 12.0, "mem_available_kb": 6_500_000, "processes": {"gazebo": {"cpu_s": 2.5}}})))
    c = rep.cost({"run": str(run), "summary": {"wall_duration_s": 100.0, "run_bytes": 5}})
    assert c["stalls_below_0_5"] == 1 and c["cpu_fraction"] == {"gazebo": pytest.approx(0.75)}
    assert c["mem_available_kb_min"] == 6_500_000 and c["rtf_mean"] == pytest.approx(0.020 / 0.056)
