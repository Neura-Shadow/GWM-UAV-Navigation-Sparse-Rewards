"""P7-E E2: the manifest, the authorizations and the runner, offline. Nothing is launched.

* **Manifest:**
  * what it binds: the P7-E sources and tests, the approved E1 specs, the reused frozen P7 specs, and the owner copy;
  * the frozen P7 candidate 0799c37e (and through it P3-P6), where the accepted state is present;
  * Policy A non-transfer; block and dry-run authorizations; the live locks, which exclude P7 and P7-E sequences.
* **Runner:**
  * the frozen P7 classes are reused, and only the declared methods are overridden;
  * the approved P7-E slots and the parse;
  * the flight gating, and the full preconditions with the host stubbed: block, marker, slot and dry-run cap;
  * the run-local inputs and `p7/layout.json`; the summary keys the frozen classifier reads;
  * C6 for the 30 layouts, where the pinned runtime tree is present.
* **The frozen P7 tree is untouched,** and the P7 candidate binds no P7-E file.
"""
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
P7E, P7 = REPO / "simulation/px4_gazebo_p7e", REPO / "simulation/px4_gazebo_p7"
for path in (P7E / "scripts", P7 / "scripts", REPO / "simulation/px4_gazebo/scripts", REPO / "simulation/px4_gazebo/validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7_manifest as m7  # noqa: E402
import p7e_manifest as m  # noqa: E402
import p7e_runner as r  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json  # noqa: E402

ROOT = Path(os.environ.get("GWM_SIM_ROOT", Path.home() / "uav_autonomy"))
needs_state = pytest.mark.skipif(not (ROOT / m.P7_MANIFEST).exists(), reason="the frozen P7 candidate state is not available")
SHA = "cd" * 32


def fake_frozen(development=False):
    p5 = {"model": {"path": "models/m.npz", "sha256": "11" * 32}, "schema_sha256": "22" * 32,
          "controller_runtime": {"python": "/usr/bin/python3", "numpy": "1.26.4"},
          "baselines": {"p3_sha256": m7.BASELINES["p3_sha256"], "p4_sha256": m7.BASELINES["p4_sha256"]}}
    return {"p4": {"p3_baseline": {"manifest": "x", "sha256": m7.BASELINES["p3_sha256"]}}, "p5": p5,
            "runtime": {"native_command_policy": {"name": "A"}}, "specs": dict(m.APPROVED_SPECS), "development": development,
            "baselines": dict(m7.BASELINES, p7_sha256=m.P7_SHA256), "p7": {"sha256": m.P7_SHA256}}


def schedule():
    return strict_json(P7E / "configs/p7e_schedule.json")


# ---- manifest ---------------------------------------------------------------------------------------------------
def test_the_manifest_binds_every_p7e_source_and_test_and_the_owner_copy():
    files = m.p7e_source_files()
    tree = {p.relative_to(REPO).as_posix() for f in m.SOURCE_FOLDERS for p in (P7E / f).rglob("*")
            if p.is_file() and "__pycache__" not in p.parts}
    tests = {p.relative_to(REPO).as_posix() for p in (REPO / "tests").glob("test_px4_gazebo_p7e_*.py")}
    assert set(files) == tree | tests and len(tests) >= 4
    assert all(files[k] == digest(REPO / k) for k in files)
    assert not any("px4_gazebo_p7/" in k for k in files)  # the frozen P7 tree is bound through the P7 candidate
    assert {"simulation/px4_gazebo_p7e/analysis/p7e_report.py", "simulation/px4_gazebo_p7e/validation/p7e_assess.py",
            "simulation/px4_gazebo_p7e/scripts/run_p7e_block.py", "simulation/px4_gazebo_p7e/scripts/p7e_host.py"} <= set(files)
    assert m.owner_identity()["sha256"] == digest(REPO / "simulation/px4_gazebo/scripts/p3_namespace_owner.py")


def test_the_specs_are_bound_by_their_approved_canonical_hashes(tmp_path, monkeypatch):
    specs = m.specs_identity()
    assert specs == dict(m.APPROVED_SPECS, p7_layouts=m.REUSED_P7["layouts"][1], p7_host_check=m.REUSED_P7["host_check"][1])
    changed = strict_json(P7E / "configs/p7e_schedule.json")
    changed["slots"][0]["first"] = "p5" if changed["slots"][0]["first"] == "p4" else "p4"
    (tmp_path / "s.json").write_text(json.dumps(changed))
    monkeypatch.setitem(m.SPEC_FILES, "schedule", str(tmp_path / "s.json"))
    with pytest.raises(ValueError, match="schedule"):
        m.specs_identity()


def test_a_policy_approval_of_another_phase_never_transfers(tmp_path):
    path = tmp_path / "a.json"
    for phase, development, ok in (("P7 p7-evaluation-v1", False, False), ("P6", False, False),
                                   ("P7-E p7-engineering-validation-v1", True, False), ("P7-E p7-engineering-validation-v1", False, True)):
        path.write_text(json.dumps({"candidate_phase": phase, "development_only": development}))
        if ok:
            assert m.check_policy_approval(path, False)["candidate_phase"].startswith("P7-E")
        else:
            with pytest.raises(PermissionError):
                m.check_policy_approval(path, False)


def test_block_authorizations_bind_block_manifest_and_schedule_and_are_consumed_once(tmp_path):
    path = tmp_path / "auth.json"
    atomic_json(path, m.authorization_template(2, SHA, "operator"), exclusive=True)
    assert m.require_authorization(path, 2, SHA)["max_attempts"] == 1
    for block, sha in ((1, SHA), (2, "ef" * 32)):
        with pytest.raises(PermissionError):
            m.require_authorization(path, block, sha)
    with pytest.raises(ValueError):
        m.authorization_template(4, SHA, "operator")
    for key, value in (("schedule_canonical_sha256", "00" * 32), ("authorized", False), ("scope", "pilot"), ("max_attempts", 2)):
        bad = dict(m.authorization_template(2, SHA, "operator"), **{key: value})
        (tmp_path / "bad.json").write_text(json.dumps(bad))
        with pytest.raises(PermissionError):
            m.require_authorization(tmp_path / "bad.json", 2, SHA)
    (tmp_path / "state").mkdir()
    marker = m.consume(tmp_path, 2, SHA, path)
    assert marker.name == "p7e-engineering-30-block-02-%s.consumed.json" % SHA
    assert strict_json(marker)["authorization_sha256"] == digest(path)
    with pytest.raises(FileExistsError):
        m.consume(tmp_path, 2, SHA, path)
    assert m.result_path(tmp_path, 2, SHA).name == "p7e-engineering-30-block-02-%s.result.json" % SHA
    assert m.marker_path(tmp_path, 2, SHA) != m7.marker_path(tmp_path, "pilot", 2, SHA)  # never a P7 marker


def test_dry_runs_need_a_development_candidate_and_name_each_flight(tmp_path):
    path = tmp_path / "dry.json"
    path.write_text(json.dumps(m.dry_run_template(SHA, "operator", "statement", [["S2", "p4"], ["S2", "p5"]], 2)))
    assert m.require_dry_run_approval(path, SHA, {"development": True})["acceptance_credit"] is False
    with pytest.raises(PermissionError):
        m.require_dry_run_approval(path, SHA, {"development": False})
    with pytest.raises(PermissionError):
        m.require_dry_run_approval(path, "ef" * 32, {"development": True})
    for key, value in (("items", [["S2", "p6"]]), ("acceptance_credit", True), ("flight", False),
                       ("contract", m7.DRY_RUN_CONTRACT)):
        path.write_text(json.dumps(dict(m.dry_run_template(SHA, "o", "s", [["S2", "p4"]], 1), **{key: value})))
        with pytest.raises(PermissionError):
            m.require_dry_run_approval(path, SHA, {"development": True})


def test_the_live_locks_exclude_p7_and_p7e_sequences_and_a_freeze(tmp_path):
    (tmp_path / "state").mkdir()
    m.acquire_live_lock(tmp_path, SHA)
    with pytest.raises(SystemExit, match="P7-E"):
        m.acquire_live_lock(tmp_path, SHA)
    with pytest.raises(SystemExit, match="no candidate may be frozen"):
        m.freeze(tmp_path, tmp_path / "out.json", tmp_path / "approval.json")
    m.release_live_lock(tmp_path, SHA)
    assert not m.live_locks(tmp_path)
    m7.acquire_live_lock(tmp_path, SHA)  # a live P7 sequence
    with pytest.raises(SystemExit, match="live P7 sequence"):
        m.acquire_live_lock(tmp_path, SHA)
    with pytest.raises(SystemExit, match="no candidate may be frozen"):
        m.freeze(tmp_path, tmp_path / "out.json", tmp_path / "approval.json")
    assert not (tmp_path / "out.json").exists()


def test_source_drift_covers_p7e_files_specs_owner_and_the_frozen_p7_code(tmp_path, monkeypatch):
    (tmp_path / "state").mkdir()
    (tmp_path / m.P7_MANIFEST).write_text("{}")
    monkeypatch.setattr(m.p7_manifest, "source_drift", lambda frozen: [])
    frozen = {"p7e_source_files": m.p7e_source_files(), "specs": m.specs_identity(), "namespace_owner": m.owner_identity()}
    assert m.source_drift(frozen, tmp_path) == []
    key = "simulation/px4_gazebo_p7e/scripts/run_p7e_block.py"
    changed = dict(frozen, p7e_source_files=dict(frozen["p7e_source_files"], **{key: "00" * 32}))
    assert m.source_drift(changed, tmp_path) == [key]
    monkeypatch.setattr(m.p7_manifest, "source_drift", lambda frozen: ["simulation/px4_gazebo_p7/scripts/p7_runner.py"])
    assert m.source_drift(frozen, tmp_path) == ["p7:simulation/px4_gazebo_p7/scripts/p7_runner.py"]


def test_runner_views_are_the_frozen_p7_views():
    frozen = fake_frozen()
    for controller in ("p4", "p5"):
        assert m.runner_view(frozen, controller) == m7.runner_view(frozen, controller)


@needs_state
def test_the_p7_binding_verifies_the_frozen_p7_candidate_and_its_tree_is_untouched():
    frozen, sha = m.p7_binding(ROOT)  # recomputes P7, and P3-P6 inside it
    assert sha == m.P7_SHA256 == "0799c37e800c06c278acbc45fdec90b84159ee9d41eb7aaf868cb65a0b950aa5"
    assert frozen["p7_source_files"] == m7.p7_source_files()
    assert not any("p7e" in k for k in frozen["p7_source_files"])


# ---- runner -----------------------------------------------------------------------------------------------------
def test_the_owner_copy_shim_and_launcher():
    assert digest(m.OWNER) == digest(REPO / "simulation/px4_gazebo/scripts/p3_namespace_owner.py")
    assert "from p7e_runner import main" in (P7E / "scripts/p3_runner.py").read_text()

    def code(path, renames=()):
        text = "\n".join(x for x in path.read_text().splitlines() if not x.startswith("#") and "echo 'No runtime started" not in x)
        for a, b in renames:
            text = text.replace(a, b)
        return text
    assert code(P7E / "scripts/run_p7e.sh", (("P7E_SCRIPTS", "P7_SCRIPTS"), ("p7e_namespace_owner", "p7_namespace_owner"))) == \
        code(P7 / "scripts/run_p7.sh")  # the frozen launcher, renamed only
    assert r.p7r.OWNER == r.p4.OWNER == r.p5r.OWNER == m.OWNER  # the QGC receipt names the owner copy running as PID 1


def test_only_the_declared_methods_are_overridden():
    p7r = r.p7r
    for cls, frozen, accepted in ((r.P7EP4Runner, p7r.P7P4Runner, r.p4.P4Runner), (r.P7EP5Runner, p7r.P7P5Runner, r.p5r.P5Runner)):
        for name in ("preconditions", "prepare_run", "execute"):
            assert getattr(cls, name) is getattr(r.P7EFlight, name), (cls, name)
        for name in ("prepare_world", "run_controller", "p7_init"):  # C6 and D15, frozen
            assert getattr(cls, name) is getattr(p7r.P7Flight, name), (cls, name)
        for name in ("__init__", "controller_summary"):
            assert getattr(cls, name) is getattr(frozen, name), (cls, name)
        for name in ("launch", "stop", "stop_qgc", "p4_controller_command", "point_px4_at_world", "watch_injection",
                     "launch_p4_sensing", "shutdown", "finalize", "px4_stack"):
            assert getattr(cls, name) is getattr(accepted, name), (cls, name)
    assert set(vars(r.P7EFlight)) - {"__module__", "__doc__", "__dict__", "__weakref__", "__qualname__", "__firstlineno__",
                                     "__static_attributes__"} == {"preconditions", "prepare_run", "execute"}


def test_the_slots_are_the_approved_ones_and_the_parse():
    sched = schedule()
    assert r.schedule_slot(1, 1) == sched["slots"][0] and r.schedule_slot(3, 30) == sched["slots"][29]
    for block, slot in ((1, 11), (4, 1), (2, 0)):
        with pytest.raises(PermissionError):
            r.schedule_slot(block, slot)
    layout = sched["slots"][0]["layout"]
    common = ["--manifest", "m.json", "--authorization", "a.json", "--allow-simulated-flight"]
    a = r.parse(["--run", "--controller", "p4", "--item", layout, "--scope", "engineering-30", "--block", "1", "--slot", "1"] + common)
    assert (a.scope, a.scenario, a.gate, a.p7_scope, a.role) == ("flight", layout, "P7-E", "engineering-30", "p7e-engineering-30-b01-s001-p4")
    assert r.parse(["--run", "--controller", "p5", "--item", "S2", "--scope", "DRY"] + common).p7_scope == "DRY"
    for argv in (["--item", layout, "--scope", "engineering-30"], ["--item", "S2", "--scope", "engineering-30", "--block", "1", "--slot", "1"],
                 ["--item", "S2", "--scope", "DRY", "--block", "1"], ["--item", layout, "--scope", "pilot", "--block", "1", "--slot", "1"]):
        with pytest.raises(SystemExit):
            r.parse(["--run", "--controller", "p4"] + argv + common)


def test_a_p7e_flight_needs_the_accepted_flight_gating_before_anything_else(monkeypatch):
    fake = SimpleNamespace(args=SimpleNamespace(allow_simulated_flight=True))
    monkeypatch.setenv("GWM_ALLOW_SITL_COMMANDS", "0")
    with pytest.raises(PermissionError, match="GWM_ALLOW_SITL_COMMANDS"):
        r.P7EFlight.preconditions(fake)
    monkeypatch.setenv("GWM_ALLOW_SITL_COMMANDS", "1")
    for key in r.base.GATES:
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(ValueError, match="Missing explicit gates"):
        r.P7EFlight.preconditions(fake)


def staged(tmp_path, monkeypatch, controller, item, scope, block=0, slot=0, development=False, authorization=None):
    """A runner whose namespace, lock, disk and manifest checks are stubbed, so preconditions run offline."""
    for key in r.base.GATES + ("GWM_ALLOW_SITL_COMMANDS",):
        monkeypatch.setenv(key, "1")
    lock = tmp_path / "lock"
    lock.write_text("")
    monkeypatch.setattr(r.os, "getpid", lambda: 1)
    monkeypatch.setattr(r.subprocess, "check_output", lambda *a, **k: json.dumps([{"ifname": "lo"}]))
    monkeypatch.setattr(r, "Path", lambda p: SimpleNamespace(resolve=lambda: lock))

    def held(fd, flags):
        raise BlockingIOError
    monkeypatch.setattr(r.base, "fcntl", SimpleNamespace(flock=held, LOCK_EX=2, LOCK_NB=4))
    monkeypatch.setattr(r.base.p3_manifest, "wsl_host", lambda: True)
    monkeypatch.setattr(r.p7e_manifest, "verify_current", lambda root, path: (fake_frozen(development), SHA))
    fake = object.__new__(r.RUNNERS[controller])
    fake.root, fake.validation, fake.controller, fake.item = tmp_path, {"disk_reserve_bytes": 0}, controller, item
    fake.p7_scope, fake.block, fake.slot, fake.dry = scope, block, slot, scope == "DRY"
    fake.slot_record, fake.scenarios, fake.p7_frozen = None, r.p7r.p7_scenarios(), None
    fake.args = SimpleNamespace(allow_simulated_flight=True, manifest=tmp_path / "m.json", authorization=authorization)
    return fake


def test_block_preconditions_need_the_consumed_marker_and_the_flights_slot(tmp_path, monkeypatch):
    (tmp_path / "state").mkdir()
    auth = tmp_path / "auth.json"
    atomic_json(auth, m.authorization_template(1, SHA, "operator"), exclusive=True)
    slot = schedule()["slots"][0]
    fake = staged(tmp_path, monkeypatch, "p4", slot["layout"], "engineering-30", 1, 1, authorization=auth)
    with pytest.raises(PermissionError, match="not consumed"):
        fake.preconditions()
    m.consume(tmp_path, 1, SHA, auth)
    view, sha, authorization = fake.preconditions()
    assert sha == SHA and view == m.runner_view(fake_frozen(), "p4") and fake.slot_record == slot
    assert fake.marker == m.marker_path(tmp_path, 1, SHA) and fake.p7e_sha == SHA
    other = staged(tmp_path, monkeypatch, "p4", schedule()["slots"][1]["layout"], "engineering-30", 1, 1, authorization=auth)
    with pytest.raises(PermissionError, match="Slot 1"):
        other.preconditions()
    wrong_block = staged(tmp_path, monkeypatch, "p4", slot["layout"], "engineering-30", 2, 1, authorization=auth)
    with pytest.raises(PermissionError, match="Authorization does not bind block"):
        wrong_block.preconditions()
    dev = staged(tmp_path, monkeypatch, "p4", slot["layout"], "engineering-30", 1, 1, development=True, authorization=auth)
    with pytest.raises(PermissionError, match="development candidate"):
        dev.preconditions()


def test_dry_run_preconditions_name_the_flight_and_count_only_p7e_dry_runs_to_the_cap(tmp_path, monkeypatch):
    auth = tmp_path / "dry.json"
    auth.write_text(json.dumps(m.dry_run_template(SHA, "operator", "s", [["S2", "p4"]], 1)))
    fake = staged(tmp_path, monkeypatch, "p4", "S2", "DRY", development=True, authorization=auth)
    for name, block in (("20261201T000000Z-p7dry-p4-S2-aaaa", "p7"), ("20261201T000001Z-p7e-p4-S2-bbbb", "p7")):
        (tmp_path / "runs" / name).mkdir(parents=True)  # a P7 dry run never counts toward a P7-E cap
        (tmp_path / "runs" / name / "summary.json").write_text(json.dumps({block: {"dry_run_approval_sha256": digest(auth)}}))
    assert fake.preconditions()[1] == SHA and fake.marker is None
    (tmp_path / "runs/20261201T000002Z-p7edry-p4-S2-cccc").mkdir()
    (tmp_path / "runs/20261201T000002Z-p7edry-p4-S2-cccc/summary.json").write_text(
        json.dumps({"p7e": {"dry_run_approval_sha256": digest(auth)}}))
    with pytest.raises(PermissionError, match="cap reached"):
        fake.preconditions()
    unnamed = staged(tmp_path, monkeypatch, "p5", "S2", "DRY", development=True, authorization=auth)
    with pytest.raises(PermissionError, match="does not name"):
        unnamed.preconditions()
    real = staged(tmp_path, monkeypatch, "p4", "S2", "DRY", development=False, authorization=auth)
    with pytest.raises(PermissionError, match="development candidate"):
        real.preconditions()


def prepared(tmp_path, controller, item, dry=False):
    fake = object.__new__(r.RUNNERS[controller])
    (tmp_path / "auth.json").write_text("{}")
    fake.root, fake.controller, fake.item, fake.dry = tmp_path, controller, item, dry
    fake.p7_scope, fake.block, fake.slot = ("DRY", 0, 0) if dry else ("engineering-30", 1, 1)
    fake.slot_record = None if dry else r.schedule_slot(1, 1)
    fake.scenarios, fake.p7_frozen = r.p7r.p7_scenarios(), fake_frozen(dry)
    fake.args = SimpleNamespace(authorization=tmp_path / "auth.json")
    fake.prepare_run(m.runner_view(fake.p7_frozen, controller))
    return fake


@pytest.mark.parametrize("controller", ["p4", "p5"])
def test_run_local_inputs_are_the_accepted_configs_with_only_the_enable_flags_and_the_layout_record(tmp_path, controller):
    item = r.schedule_slot(1, 1)["layout"]
    run = prepared(tmp_path, controller, item).run
    assert run.parent == tmp_path / "runs" and ("-p7e-%s-%s-" % (controller, item)) in run.name
    sim, p4 = REPO / "simulation/px4_gazebo/configs", REPO / "simulation/px4_gazebo_p4/configs"
    for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml", "p3_native_command_policy.json"):
        assert digest(run / name) == digest(sim / name)
    assert digest(run / "p4_scenarios.json") == digest(p4 / "p4_scenarios.json")
    local, accepted = strict_json(run / "p4_avoidance.yaml"), strict_json(p4 / "p4_avoidance.yaml")
    assert local.pop("enabled") is True and accepted.pop("enabled") is False and local == accepted
    assert (run / "p5_policy.yaml").exists() == (controller == "p5") and (run / "p7e").is_dir()
    record = strict_json(run / "p7/layout.json")  # the path the frozen classifier hashes
    assert record["contract"] == "p7e-flight-layout-v1" and record["scope"] == "engineering-30"
    assert record["expected_world_sha256"] == r.schedule_slot(1, 1)["world_sha256"] == r.p7r.expected_world(item)
    assert record["slot_record"]["slot"] == 1 and record["specs"] == dict(m.APPROVED_SPECS)
    assert record["scenario"]["outcome"] == "p7_not_predeclared"
    assert not list(run.rglob("dry-run-marker.json"))


def test_a_dry_run_writes_its_own_run_local_marker(tmp_path):
    fake = prepared(tmp_path, "p4", "S2", dry=True)
    assert "-p7edry-p4-S2-" in fake.run.name and fake.marker == fake.run / "p7e/dry-run-marker.json"
    assert strict_json(fake.marker)["credit"] is False and strict_json(fake.marker)["contract"] == "p7e-dry-run-marker-v1"


def test_the_summary_keeps_the_keys_the_frozen_classifier_reads_and_adds_the_p7e_block(tmp_path, monkeypatch):
    item = r.schedule_slot(1, 1)["layout"]
    fake = object.__new__(r.P7EP4Runner)
    calls = []

    def preconditions():
        fake.p7_frozen = fake_frozen()
        return m.runner_view(fake.p7_frozen, "p4"), SHA, {}

    def flown(config):
        calls.append("run_controller")
        fake.summary["status"] = "passed"
    (tmp_path / "auth.json").write_text("{}")
    fake.root, fake.controller, fake.item, fake.dry = tmp_path, "p4", item, False
    fake.p7_scope, fake.block, fake.slot, fake.slot_record = "engineering-30", 1, 1, r.schedule_slot(1, 1)
    fake.scenarios, fake.p4_scope, fake.owned = r.p7r.p7_scenarios(), "flight", []
    fake.args = SimpleNamespace(authorization=tmp_path / "auth.json", scope="flight", gate="P7-E", role="role")
    fake.preconditions = preconditions
    fake.save = lambda: calls.append("save")
    fake.prepare_world = lambda: calls.append("prepare_world")
    fake.px4_stack = lambda frozen: calls.append("px4_stack") or {}
    fake.run_controller = flown
    fake.shutdown = lambda: calls.append("shutdown") or []
    fake.finalize = lambda: calls.append("finalize")
    monkeypatch.setattr(r.base, "ResourceSampler", lambda *a: SimpleNamespace(start=lambda: None))
    assert fake.execute() == 0
    assert calls == ["save", "prepare_world", "px4_stack", "run_controller", "shutdown", "finalize"]
    s = fake.summary
    assert s["contract"] == "p7e-runtime-summary-v1" and s["frozen_manifest_sha256"] == SHA and s["gate"] == "P7-E"
    assert s["acceptance_credit_possible"] is False
    assert {k: s["p7"][k] for k in ("item", "scope", "block", "slot", "dry_run")} == \
        {"item": item, "scope": "engineering-30", "block": 1, "slot": 1, "dry_run": False}
    assert s["p7"]["p7_manifest_sha256"] == m.P7_SHA256 and "world" in s["p7"]
    assert s["p7e"]["p7e_manifest_sha256"] == SHA and s["p7e"]["order"] == r.schedule_slot(1, 1)["order"]
    assert s["p7e"]["schedule_canonical_sha256"] == m.APPROVED_SPECS["schedule"] and s["p7e"]["engineering_credit_possible"]
    assert s["p3_baseline"]["sha256"] == m7.BASELINES["p3_sha256"]  # the frozen P7P4Runner.controller_summary


@pytest.mark.skipif(not (ROOT / "state/px4-mc-trace-built.json").exists(), reason="the pinned runtime tree is not available")
def test_c6_every_p7e_world_through_the_frozen_path_is_the_scheduled_one():
    import hashlib
    import p4_worlds
    import p7_specs
    base, mapping = p7_specs.base_world()[0], r.p7r.p7_scenarios()
    for slot in schedule()["slots"]:
        text, _ = p4_worlds.generate(base, slot["layout"], mapping)
        assert hashlib.sha256(text.encode()).hexdigest() == slot["world_sha256"] == r.p7r.expected_world(slot["layout"])
