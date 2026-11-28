"""P8-M2 integration tests: nodes, manifest, runner, sequencer and report helpers (contract `p8-navigation-recovery-v1`
§2, §4, §8, §13). Offline: no ROS, no simulator, no build. The runtime paths are exercised in P8-M3's dry run."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_px4_gazebo_p8_recovery as v0  # noqa: E402  (paths and the V0 rig)
import test_px4_gazebo_p8_h10 as h10t  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
P8 = REPO / "simulation/px4_gazebo_p8"
for _path in (P8 / "scripts", P8 / "validation", P8 / "analysis"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
import p8_manifest  # noqa: E402
import p8_nodes  # noqa: E402
from gwm_navigation_recovery import control_node, recovery_ledger  # noqa: E402
from p3_provenance import canonical_sha256, digest, strict_json  # noqa: E402


# ---- nodes ----------------------------------------------------------------------------------------------------------------
def test_the_nodes_are_exactly_the_declared_edits_of_the_accepted_nodes():
    assert p8_nodes.main(["check"]) == 0
    for name, (source, sha) in p8_nodes.SOURCES.items():
        assert digest(REPO / source) == sha
        generated = (p8_nodes.PACKAGE / name).read_text(encoding="utf-8")
        original = (REPO / source).read_text(encoding="utf-8")
        removed = [line for line in original.splitlines() if line not in generated.splitlines()]
        assert all(("from ." in line) or ("Mission(" in line) for line in removed), removed  # only imports and the mission


def test_the_nodes_import_without_ros_and_keep_the_accepted_identity():
    from gwm_navigation_recovery import p8_p4r_node, p8_p5r_node
    assert p8_p5r_node.CONTROLLER == "p5_control"
    assert '"controller": "p4_control"' in (p8_nodes.PACKAGE / "p8_p4r_node.py").read_text()
    assert callable(control_node.main_p4r) and callable(control_node.main_p5r) and callable(p8_p4r_node.main)


def test_the_recovery_parameters_must_match_their_approved_hash(tmp_path):
    path = P8 / "configs/p8_recovery.json"
    assert recovery_ledger.canonical_sha256(json.loads(path.read_text())) == p8_manifest.APPROVED_SPECS["recovery"]
    assert recovery_ledger.load_params(path, p8_manifest.APPROVED_SPECS["recovery"])["r4"]["yaw_to_l4_at_least"] == 225
    with pytest.raises(ValueError, match="differ"):
        recovery_ledger.load_params(path, "00" * 32)


def test_the_recovery_ledger_writes_a_header_every_record_and_fsyncs_on_close(tmp_path):
    led = recovery_ledger.RecoveryLedger(tmp_path / "ledger.jsonl", "ab" * 32)
    pending = [{"ledger": "p8-recovery-ledger-v1", "kind": "action", "k": 1}, {"ledger": "p8-recovery-ledger-v1", "kind": "refused"}]
    led.write(pending)
    assert pending == []
    summary = led.close()
    lines = [json.loads(x) for x in (tmp_path / "ledger.jsonl").read_text().splitlines()]
    assert lines[0]["kind"] == "header" and lines[0]["params_sha256"] == "ab" * 32 and len(lines) == 3
    assert summary["fsynced"] and summary["records"] == 3 and led.close() is summary
    with pytest.raises(FileExistsError):
        recovery_ledger.RecoveryLedger(tmp_path / "ledger.jsonl", "ab" * 32)


# ---- manifest --------------------------------------------------------------------------------------------------------------
def test_the_manifest_binds_the_approved_specs_and_the_passed_offline_records():
    assert p8_manifest.specs_identity() == p8_manifest.APPROVED_SPECS
    records = p8_manifest.offline_records()
    assert set(records) == {"V1", "V2"} and all(digest(REPO / r["path"]) == r["sha256"] for r in records.values())
    files = p8_manifest.p8_source_files()
    for name in ("simulation/px4_gazebo_p8/ros2_ws/src/gwm_navigation_recovery/gwm_navigation_recovery/recovery.py",
                 "simulation/px4_gazebo_p8/scripts/p8_runner.py", "simulation/px4_gazebo_p8/configs/p8_layouts.json",
                 "tests/test_px4_gazebo_p8_recovery.py"):
        assert name in files
    assert not any("__pycache__" in k for k in files)
    assert p8_manifest.package_hash() == p8_manifest.package_hash(p8_manifest.package_manifest())
    assert p8_manifest.owner_identity()["sha256"] == digest(REPO / "simulation/px4_gazebo/scripts/p3_namespace_owner.py")


def test_policy_approvals_and_builds_are_checked(tmp_path):
    for phase, dev, ok in (("P8-M3", False, True), ("P8-M3 development", True, True), ("P7-E", False, False), ("P8", True, False)):
        path = tmp_path / ("a-%s-%s.json" % (phase.replace(" ", "_"), dev))
        path.write_text(json.dumps({"candidate_phase": phase, "development_only": dev and phase.endswith("development")}))
        if ok:
            p8_manifest.check_policy_approval(path, dev)
        else:
            with pytest.raises(PermissionError):
                p8_manifest.check_policy_approval(path, dev)
    with pytest.raises(ValueError, match="not built"):
        p8_manifest.build_identity(tmp_path)
    (tmp_path / "state").mkdir()
    (tmp_path / p8_manifest.BUILD_RECEIPT).write_text(json.dumps({"package_hash": "00" * 32, "install": "/x"}))
    with pytest.raises(ValueError, match="differs"):
        p8_manifest.build_identity(tmp_path)


def test_block_authorizations_bind_block_stage_manifest_and_schedule(tmp_path):
    sha = "cd" * 32
    path = tmp_path / "auth.json"
    path.write_text(json.dumps(p8_manifest.authorization_template(2, sha, "operator")))
    assert p8_manifest.require_authorization(path, 2, sha)["stage"] == "V4"
    for block, s in ((1, sha), (2, "ef" * 32), (3, sha)):
        with pytest.raises(PermissionError):
            p8_manifest.require_authorization(path, block, s)
    with pytest.raises(ValueError):
        p8_manifest.authorization_template(4, sha, "operator")
    dry = tmp_path / "dry.json"
    dry.write_text(json.dumps(p8_manifest.dry_run_template(sha, "operator", "s", [["p7c-4000355", "p8_p4r"]], 2)))
    assert p8_manifest.require_dry_run_approval(dry, sha, {"development": True})["items"] == [["p7c-4000355", "p8_p4r"]]
    with pytest.raises(PermissionError):
        p8_manifest.require_dry_run_approval(dry, sha, {"development": False})
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps(p8_manifest.dry_run_template(sha, "operator", "s", [["p7c-4000355", "p4"]], 2)))
    with pytest.raises(PermissionError, match="variant"):
        p8_manifest.require_dry_run_approval(bad, sha, {"development": True})


# ---- runner -----------------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def runner():
    import p8_runner
    return p8_runner


def test_the_runner_maps_every_p8_layout_and_keeps_the_frozen_scenarios(runner):
    layouts = runner.layouts()
    scenarios = runner.p8_scenarios()["scenarios"]
    for stage in ("V3", "V4", "V5"):
        for r in layouts["stages"][stage]["layouts"]:
            assert r["id"] in scenarios and runner.expected_world(r["id"]) == r["world_sha256"]
    v4 = layouts["stages"]["V4"]["layouts"][0]
    assert scenarios[v4["id"]]["boxes"] == v4["boxes"] and scenarios[v4["id"]]["outcome"] == "p7_not_predeclared"
    assert all(s in scenarios for s in ("S1", "S2", "S6"))
    slot = runner.schedule()["slots"][12]
    assert runner.schedule_slot(slot["block"], slot["slot"])["layout"] == slot["layout"]
    with pytest.raises(PermissionError):
        runner.schedule_slot(1, 15)


def test_the_runner_arguments_keep_the_accepted_variant_field_empty(runner):
    args = runner.parse(["--run", "--variant", "p8_p5r", "--item", "p8h-6000014", "--scope", "p8-validation-60", "--block", "2",
                         "--slot", "11", "--manifest", "/m", "--authorization", "/a", "--allow-simulated-flight"])
    assert args.variant is None and args.p8_variant == "p8_p5r" and args.controller == "p5" and args.gate == "P8"
    with pytest.raises(SystemExit):
        runner.parse(["--run", "--variant", "p8_p4r", "--item", "x", "--scope", "DRY", "--block", "1", "--manifest", "/m",
                      "--authorization", "/a"])
    mro = runner.P8P5RRunner.__mro__
    order = [mro.index(c) for c in (runner.P8Flight, runner.p7er.P7EFlight, runner.p7r.P7Flight, runner.p5r.P5Runner)]
    assert order == sorted(order)


def test_the_controller_command_replaces_only_the_executable_and_appends_the_parameters(runner, tmp_path):
    class Accepted:
        def p4_controller_command(self, env):
            return ["/install/p5/lib/gwm_policy_control/p5_control", "--config", "c", "--flight"], dict(env, X="1")

    class Fake(runner.P8Flight, Accepted):
        variant = "p8_p5r"
    (tmp_path / "state").mkdir()
    (tmp_path / p8_manifest.BUILD_RECEIPT).write_text(json.dumps({"install": "/p8"}))
    f = Fake()
    f.root, f.run = tmp_path, tmp_path / "run"
    command, env = f.p4_controller_command({"A": "b"})
    assert command[0] == "/p8/gwm_navigation_recovery/lib/gwm_navigation_recovery/p8_p5r_control"
    assert command[1:4] == ["--config", "c", "--flight"] and env == {"A": "b", "X": "1"}
    assert command[-4:] == ["--recovery-params", str(tmp_path / "run/p8_recovery.json"), "--recovery-params-sha256",
                            p8_manifest.APPROVED_SPECS["recovery"]]


# ---- sequencer ----------------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def seq():
    import run_p8_block
    return run_p8_block


def test_the_plan_flies_each_pair_back_to_back_with_both_variants(seq):
    sched = seq.schedule()
    for block, stage in ((1, "V3"), (2, "V4"), (3, "V5")):
        flights = seq.plan_flights(sched, block)
        assert len(flights) == 20 and {f["stage"] for f in flights} == {stage}
        assert [f["plan_index"] for f in flights] == list(range(20))
        for a, b in zip(flights[::2], flights[1::2]):
            assert a["slot"] == b["slot"] and {a["variant"], b["variant"]} == {"p8_p4r", "p8_p5r"}
            assert {a["controller"], b["controller"]} == {"p4", "p5"}
    command = seq.runtime_command(seq.plan_flights(sched, 2)[0], "p8-validation-60", "/m", "/a")
    assert command[:4] == ["bash", str(seq.RUN_P8), "--run", "--variant"] and "--allow-simulated-flight" in command
    assert seq.dry_flights({"items": [["p7c-4000355", "p8_p4r"]]})[0]["controller"] == "p4"


def test_fly_one_records_everything_and_never_raises_after_the_runtime(seq, tmp_path):
    run_dir = tmp_path / "runs/r1"
    (run_dir / "p8").mkdir(parents=True)
    calls = []

    def fake_run(command, **kw):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout="P8 evidence: %s\n" % run_dir, stderr="")
    flight = seq.plan_flights(seq.schedule(), 1)[0]
    record = seq.fly_one(flight, "p8-validation-60", "/m", "/a", classify=lambda p: {"validity": "valid", "stop_reasons": []},
                         run=fake_run)
    assert record["run"] == str(run_dir) and record["classification"]["validity"] == "valid" and len(calls) == 2
    broken = seq.fly_one(flight, "p8-validation-60", "/m", "/a", classify=lambda p: 1 / 0, run=fake_run)
    assert broken["classification"]["validity"] is None and "ZeroDivisionError" in broken["classification"]["error"]


class FakeClassifier:
    def classify(self, run):
        rec = {"validity": "valid", "stop_reasons": [], "outcome": "goal_success"}
        (Path(run) / "p7").mkdir(exist_ok=True)
        (Path(run) / "p7/flight-record.json").write_text(json.dumps(rec))
        return rec


@pytest.mark.parametrize("case,expected", [("clean", None), ("no ledger", "safety_unverified:fc5_recovery_ledger"),
                                           ("stale", "safety_event:fc6_recovery_invariant")])
def test_the_p8_assessment_adds_h10_as_fc5_or_fc6(seq, tmp_path, case, expected):
    run, _ = h10t.write_run(tmp_path)
    (run / "summary.json").write_text(json.dumps({"finalized": True}))
    (run / "p4_avoidance.yaml").write_text(json.dumps(v0.CONFIG))
    if case == "no ledger":
        (run / "p8/recovery-ledger.jsonl").unlink()
    elif case == "stale":
        lines = [json.loads(x) for x in (run / "p4/decisions.jsonl").read_text().splitlines()]
        next(d for d in lines if str(d["reason"]).startswith("p8_recovery:"))["stale"] = "snapshot_stale"
        (run / "p4/decisions.jsonl").write_text("".join(json.dumps(d) + "\n" for d in lines))
    out = seq.assess(run, v0.PARAMS, FakeClassifier())
    assert (expected in out["stop_reasons"]) if expected else (out["stop_reasons"] == [] and out["h10"]["ok"])
    assert strict_json(run / seq.ASSESSMENT) == out and (run / seq.H10_RECORD).exists()


def test_fc1_requires_the_previous_block_unstopped_and_complete(seq, tmp_path):
    sha = "ab" * 32
    assert seq.previous_block_ok(tmp_path, 1, sha) == (True, None)
    assert seq.previous_block_ok(tmp_path, 2, sha)[0] is False
    (tmp_path / "state").mkdir()
    planned = seq.plan_flights(seq.schedule(), 1)
    ledger = tmp_path / "state/l.jsonl"
    ledger.write_text("".join(json.dumps(dict(f, classification={"validity": "valid", "stop_reasons": []})) + "\n" for f in planned))
    result = {"manifest_sha256": sha, "block": 1, "planned": planned, "stopped": False, "not_flown": [], "interrupted": None,
              "missing_pairs": [], "flights": [dict(f, classification={"validity": "valid", "stop_reasons": []}) for f in planned],
              "ledger": {"path": str(ledger), "sha256": digest(ledger)}}
    p8_manifest.result_path(tmp_path, 1, sha).write_text(json.dumps(result))
    assert seq.previous_block_ok(tmp_path, 2, sha) == (True, None)
    p8_manifest.result_path(tmp_path, 1, sha).write_text(json.dumps(dict(result, stopped=True)))
    assert seq.previous_block_ok(tmp_path, 2, sha)[0] is False


# ---- report helpers ----------------------------------------------------------------------------------------------------------
def ledger_run(tmp_path, recovery_ticks=(3, 4), extra=None, state_change=None, drop=False):
    run = tmp_path / "run"
    (run / "p5").mkdir(parents=True)
    records, events, ledger = [], [], []
    for k in range(8):
        sim = 100.0 + 0.05 * k
        records.append({"tick": k + 1, "sim_s": sim, "action": "HOLD", "state": "HOVER"})
        executed = "YAW_TO" if k in recovery_ticks or k == extra else "HOLD"
        events.append({"event": "p4_decision", "sim_s": sim, "action": executed,
                       "state": "AVOIDING" if k == state_change else "HOVER"})
        if k in recovery_ticks:
            ledger.append({"kind": "action", "sim_s": sim})
    if drop:
        records.pop()
    (run / "p5/policy-ledger.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
    (run / "ros-events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
    return {"run": str(run), "controller": "p5", "variant": "p8_p5r", "ledger": ledger, "block": 1, "slot": 1, "attempt": 1,
            "record": None, "evaluation": {"failures": ["policy:ledger:decision_mismatch"]}}


@pytest.mark.parametrize("case,explained", [("exact", True), ("extra mismatch", False), ("state differs", False),
                                            ("count differs", False)])
def test_the_p5_ledger_mismatch_is_explained_only_by_the_recovery_actions(tmp_path, case, explained):
    import p8_report
    kwargs = {"extra mismatch": {"extra": 6}, "state differs": {"state_change": 3}, "count differs": {"drop": True}}.get(case, {})
    out = p8_report.ledger_reconciliation(ledger_run(tmp_path, **kwargs))
    assert out["explained"] is explained and out["evaluator_item"] is True
    r10 = p8_report.r10([ledger_run(tmp_path / "b", **kwargs)])["p5_ledger_reconciliation"]
    assert r10["flagged"] == 1 and r10["flagged_and_explained"] == (1 if explained else 0)
    assert p8_report.ledger_reconciliation(dict(ledger_run(tmp_path / "c"), controller="p4")) is None


DRY_RUN_P5 = Path.home() / "uav_autonomy/runs/20261128T025633Z-p8dry-p8_p5r-p7c-4000375-3b109641"


@pytest.mark.skipif(not DRY_RUN_P5.exists(), reason="the P8-M3 dry-run evidence is not present")
def test_the_dry_run_p5_flight_reconciles_tick_for_tick():
    import p8_report
    ledger = [json.loads(x) for x in (DRY_RUN_P5 / "p8/recovery-ledger.jsonl").read_text().splitlines()]
    evaluation = strict_json(DRY_RUN_P5 / "p5-evaluation.json")
    out = p8_report.ledger_reconciliation({"run": str(DRY_RUN_P5), "controller": "p5", "variant": "p8_p5r", "ledger": ledger,
                                           "block": 0, "slot": 0, "attempt": 1, "record": None, "evaluation": evaluation})
    assert out["explained"] and out["mismatches"] == out["recovery_actions"] == 286 and out["other_field_mismatches"] == 0


def test_the_report_plan_and_h10_criterion():
    import p8_report
    assert canonical_sha256(p8_report.plan()) == p8_manifest.APPROVED_SPECS["report_plan"]
    ok = {"h10": {"ok": True, "actions": 3}, "block": 1, "slot": 1, "controller": "p4", "attempt": 1, "run": "/r/a"}
    bad = dict(ok, h10={"ok": False, "violations": ["x"]})
    assert p8_report.h10([ok])["met"] and not p8_report.h10([ok, bad])["met"] and not p8_report.h10([])["met"]
