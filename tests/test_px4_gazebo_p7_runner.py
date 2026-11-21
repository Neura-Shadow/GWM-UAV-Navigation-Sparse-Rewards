"""P7 M2: the runner, the manifest, the host check and the block sequencer, offline. Nothing is launched.

* Manifest:
  * the bindings: P7 sources and tests, the approved specification hashes, and the owner copy;
  * Policy A non-transfer;
  * block, pilot and dry-run authorizations, the schedule selection and the live lock.
* Runner:
  * the accepted classes are reused and only the declared methods are overridden;
  * the frozen schedule slots;
  * the run-local inputs: the accepted configs unchanged, P4/P5 enabled only, and `p7/layout.json`;
  * the summary fields the unchanged evaluators read.
* C6: S1-S6 through the P7 mapping give the accepted worlds, and every P7 layout gives its frozen world (where the
  pinned runtime tree is present).
* Host check: Windows and WSL checks from supplied readings, and the disk floor.
* Sequencer: the flight plan, and the replacement and stop rules of contract §7 with stub flights.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
P7 = REPO / "simulation/px4_gazebo_p7"
for path in (P7 / "scripts", REPO / "simulation/px4_gazebo/scripts", REPO / "simulation/px4_gazebo/validation",
             REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control",
             REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
             REPO / "simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import p7_host  # noqa: E402
import p7_manifest as m  # noqa: E402
import p7_runner as r  # noqa: E402
import run_p7_block as seq  # noqa: E402
from p3_provenance import canonical_sha256, digest, strict_json  # noqa: E402

ROOT = Path(os.environ.get("GWM_SIM_ROOT", Path.home() / "uav_autonomy"))
needs_state = pytest.mark.skipif(not (ROOT / "state/p6-candidate-8e29f3f.json").exists(),
                                 reason="the accepted P3-P6 state is not available")
SHA = "ab" * 32


# ---- manifest -------------------------------------------------------------------------------------------------
def test_the_manifest_binds_every_p7_source_and_test_and_the_owner_copy():
    files = m.p7_source_files()
    for path in ("simulation/px4_gazebo_p7/scripts/p7_runner.py", "simulation/px4_gazebo_p7/scripts/run_p7_block.py",
                 "simulation/px4_gazebo_p7/scripts/p7_manifest.py", "simulation/px4_gazebo_p7/scripts/p7_host.py",
                 "simulation/px4_gazebo_p7/scripts/run_p7.sh", "simulation/px4_gazebo_p7/configs/p7_host_check.yaml",
                 "simulation/px4_gazebo_p7/configs/p7_layouts.json", "tests/test_px4_gazebo_p7_runner.py",
                 "tests/test_px4_gazebo_p7_specs.py"):
        assert path in files, path
    assert all("__pycache__" not in path for path in files)
    assert m.owner_identity()["sha256"] == digest(REPO / "simulation/px4_gazebo/scripts/p3_namespace_owner.py")


def test_the_specs_are_bound_by_their_approved_canonical_hashes(tmp_path, monkeypatch):
    identity = m.specs_identity()
    assert {k: identity[k] for k in m.APPROVED_SPECS} == m.APPROVED_SPECS
    assert identity["host_check"] == canonical_sha256(strict_json(P7 / m.HOST_CHECK))
    changed = strict_json(P7 / "configs/p7_analysis_plan.yaml")
    changed["primary"]["alpha"] = 0.10
    (tmp_path / "plan.yaml").write_text(json.dumps(changed))
    monkeypatch.setitem(m.SPEC_FILES, "analysis_plan", str(tmp_path / "plan.yaml"))
    with pytest.raises(ValueError, match="analysis_plan"):
        m.specs_identity()


def test_a_policy_approval_of_another_phase_never_transfers(tmp_path):
    for phase in ("P6 p6-acceptance-v1, the real candidate", "P5 p5-acceptance-v1"):
        (tmp_path / "a.json").write_text(json.dumps({"candidate_phase": phase, "development_only": False}))
        with pytest.raises(PermissionError):
            m.check_policy_approval(tmp_path / "a.json", development=False)
    (tmp_path / "dev.json").write_text(json.dumps({"candidate_phase": "P7 development", "development_only": True}))
    assert m.check_policy_approval(tmp_path / "dev.json", development=True)
    with pytest.raises(PermissionError):
        m.check_policy_approval(tmp_path / "dev.json", development=False)


def test_block_authorizations_bind_scope_block_manifest_and_schedule_and_are_consumed_once(tmp_path):
    (tmp_path / "state").mkdir()
    assert (m.block_count("pilot"), m.block_count("confirmatory-200"), m.block_count("confirmatory-250")) == (1, 10, 13)
    with pytest.raises(ValueError):
        m.authorization_template("pilot", 2, SHA, "op")
    with pytest.raises(ValueError):
        m.authorization_template("confirmatory-200", 11, SHA, "op")
    path = tmp_path / "auth.json"
    path.write_text(json.dumps(m.authorization_template("confirmatory-200", 3, SHA, "op")))
    assert m.require_authorization(path, "confirmatory-200", 3, SHA)["max_attempts"] == 1
    for scope, block, sha in (("confirmatory-250", 3, SHA), ("confirmatory-200", 4, SHA), ("confirmatory-200", 3, "cd" * 32)):
        with pytest.raises(PermissionError):
            m.require_authorization(path, scope, block, sha)
    forged = dict(json.loads(path.read_text()), schedule_canonical_sha256="0" * 64)
    (tmp_path / "forged.json").write_text(json.dumps(forged))
    with pytest.raises(PermissionError):
        m.require_authorization(tmp_path / "forged.json", "confirmatory-200", 3, SHA)
    marker = m.consume(tmp_path, "confirmatory-200", 3, SHA, path)
    assert marker == m.marker_path(tmp_path, "confirmatory-200", 3, SHA)
    with pytest.raises(FileExistsError):
        m.consume(tmp_path, "confirmatory-200", 3, SHA, path)
    assert "simulated flight only" in json.loads(path.read_text())["statement"]


def test_a_confirmatory_block_needs_the_pilot_rules_selection(tmp_path):
    (tmp_path / "state").mkdir()
    with pytest.raises(PermissionError):
        m.require_selection(tmp_path, SHA, "confirmatory-200")
    m.selection_path(tmp_path, SHA).write_text(json.dumps({"contract": m.SELECTION_CONTRACT, "manifest_sha256": SHA, "pairs": 250}))
    assert m.require_selection(tmp_path, SHA, "confirmatory-250")["pairs"] == 250
    with pytest.raises(PermissionError):
        m.require_selection(tmp_path, SHA, "confirmatory-200")
    with pytest.raises(PermissionError):
        m.require_selection(tmp_path, "cd" * 32, "confirmatory-250")


def test_dry_runs_need_a_development_candidate_and_name_each_flight(tmp_path):
    path = tmp_path / "dry.json"
    path.write_text(json.dumps(m.dry_run_template(SHA, "op", "Authorize M4", [["S2", "p4"], ["S2", "p5"]], 4)))
    value = m.require_dry_run_approval(path, SHA, {"development": True})
    assert value["items"] == [["S2", "p4"], ["S2", "p5"]] and value["acceptance_credit"] is False
    with pytest.raises(PermissionError):
        m.require_dry_run_approval(path, SHA, {"development": False})
    with pytest.raises(PermissionError):
        m.require_dry_run_approval(path, "cd" * 32, {"development": True})
    for key, bad in (("simulated_flight_only", False), ("items", [["S2", "p6"]]), ("items", []), ("acceptance_credit", True)):
        (tmp_path / "bad.json").write_text(json.dumps(dict(json.loads(path.read_text()), **{key: bad})))
        with pytest.raises(PermissionError):
            m.require_dry_run_approval(tmp_path / "bad.json", SHA, {"development": True})


def test_the_live_lock_blocks_a_second_sequence_and_a_freeze_and_drift_is_detected(tmp_path):
    (tmp_path / "state").mkdir()
    m.acquire_live_lock(tmp_path, SHA)
    with pytest.raises(SystemExit):
        m.acquire_live_lock(tmp_path, "cd" * 32)
    with pytest.raises(SystemExit):
        m.freeze(tmp_path, tmp_path / "x.json", tmp_path / "none.json", development=True)
    m.release_live_lock(tmp_path, SHA)
    assert not m.live_lock(tmp_path).exists()
    frozen = {"p7_source_files": m.p7_source_files(), "specs": m.specs_identity(), "namespace_owner": m.owner_identity()}
    assert m.source_drift(frozen) == []
    frozen["p7_source_files"] = dict(frozen["p7_source_files"], **{"simulation/px4_gazebo_p7/scripts/p7_runner.py": "0" * 64})
    assert m.source_drift(frozen) == ["simulation/px4_gazebo_p7/scripts/p7_runner.py"]


def fake_frozen():
    p5 = {"model": {"path": "models/m.npz", "sha256": "11" * 32}, "schema_sha256": "22" * 32,
          "controller_runtime": {"python": "/usr/bin/python3", "numpy": "1.26.4"},
          "baselines": {"p3_sha256": m.BASELINES["p3_sha256"], "p4_sha256": m.BASELINES["p4_sha256"]}}
    return {"p4": {"p3_baseline": {"manifest": "x", "sha256": m.BASELINES["p3_sha256"]}}, "p5": p5,
            "runtime": {"native_command_policy": {"name": "A"}}, "specs": dict(m.APPROVED_SPECS), "development": False,
            "baselines": dict(m.BASELINES)}


def test_runner_views_carry_what_the_accepted_runners_and_evaluators_read():
    frozen = fake_frozen()
    assert set(m.runner_view(frozen, "p4")) == {"p3_baseline", "runtime"}
    assert set(m.runner_view(frozen, "p5")) == {"model", "schema_sha256", "controller_runtime", "baselines", "runtime"}
    with pytest.raises(ValueError):
        m.runner_view(frozen, "p6")


@needs_state
def test_the_p7_binding_verifies_p3_to_p6_now_and_takes_the_p4_baseline_from_the_verified_p4_manifest():
    bound, p6_frozen, p4_frozen = m.baselines(ROOT)
    assert bound == m.BASELINES
    v4 = strict_json(REPO / "simulation/px4_gazebo_p4/configs/p4_validation.yaml")
    v5 = strict_json(REPO / "simulation/px4_gazebo_p5/configs/p5_validation.yaml")
    assert p4_frozen["p3_baseline"]["sha256"] == v4["p3_baseline_manifest_sha256"]  # p4_evaluate.py:436
    assert p6_frozen["p5"]["baselines"] == {"p3_sha256": v5["p3_baseline_manifest_sha256"],
                                            "p4_sha256": v5["p4_baseline_manifest_sha256"]}  # p5_evaluate.py:266-267


# ---- runner ---------------------------------------------------------------------------------------------------
def test_the_owner_copy_shim_and_launcher():
    assert digest(P7 / "scripts/p7_namespace_owner.py") == digest(REPO / "simulation/px4_gazebo/scripts/p3_namespace_owner.py")
    assert "from p7_runner import main" in (P7 / "scripts/p3_runner.py").read_text()
    def code(path, renames=()):
        lines = [x for x in path.read_text().splitlines() if not x.startswith("#") and "echo 'No runtime started" not in x]
        text = "\n".join(lines)
        for a, b in renames:
            text = text.replace(a, b)
        return text
    p6 = code(REPO / "simulation/px4_gazebo_p6/scripts/run_p6.sh",
              (("P6_SCRIPTS", "P7_SCRIPTS"), ("p6_namespace_owner", "p7_namespace_owner"), ("P1-P6", "P1-P7")))
    assert code(P7 / "scripts/run_p7.sh") == p6  # the accepted launcher, renamed only
    assert "unshare --user --map-current-user" in p6 and r.OWNER.name == "p7_namespace_owner.py"
    assert r.p4.OWNER == r.p5r.OWNER == r.OWNER  # the QGC receipt names the owner copy running as PID 1


def test_only_the_declared_methods_are_overridden():
    for cls, accepted in ((r.P7P4Runner, r.p4.P4Runner), (r.P7P5Runner, r.p5r.P5Runner)):
        for name in ("preconditions", "prepare_run", "prepare_world", "execute", "run_controller"):
            assert getattr(cls, name) is getattr(r.P7Flight, name), (cls, name)
        for name in ("launch", "stop", "stop_qgc", "p4_controller_command", "point_px4_at_world", "watch_injection",
                     "launch_p4_sensing", "shutdown", "finalize", "px4_stack"):
            assert getattr(cls, name) is getattr(accepted, name), (cls, name)
    assert r.P7P5Runner.p4_controller_command is r.p5r.P5Runner.p4_controller_command


def test_the_p7_mapping_holds_every_frozen_layout_with_the_d13_label_and_the_accepted_s1_s6_unchanged():
    mapping = r.p7_scenarios()
    accepted = strict_json(REPO / "simulation/px4_gazebo_p4/configs/p4_scenarios.json")
    records = r.layout_records()
    assert len(records) == 260 and set(mapping["scenarios"]) == set(records) | set(accepted["scenarios"])
    assert all(mapping["scenarios"][i]["outcome"] == "p7_not_predeclared" for i in records)
    assert all(mapping["scenarios"][s] == accepted["scenarios"][s] for s in accepted["scenarios"])
    assert (mapping["box_height_m"], mapping["box_center_height_m"], mapping["frame"]) == \
        (accepted["box_height_m"], accepted["box_center_height_m"], accepted["frame"])
    assert all(r.expected_world(i) == records[i]["world_sha256"] for i in records)


def pinned_base():
    import p7_specs
    return p7_specs.base_world()[0]


@pytest.mark.skipif(not (ROOT / "state/px4-mc-trace-built.json").exists(), reason="the pinned runtime tree is not available")
def test_c6_every_world_through_the_p7_path_is_the_frozen_one():
    import p4_worlds
    base, mapping = pinned_base(), r.p7_scenarios()
    for name in list(r.ACCEPTED_WORLDS) + list(r.layout_records()):
        text, declaration = p4_worlds.generate(base, name, mapping)
        assert hashlib.sha256(text.encode()).hexdigest() == r.expected_world(name), name


@needs_state
def test_c6_the_accepted_worlds_are_those_every_accepted_p4_and_p5_flight_recorded():
    seen = {}
    for phase, sha in (("p4", m.BASELINES["p4_sha256"]), ("p5", m.BASELINES["p5_sha256"])):
        for path in sorted((ROOT / "state").glob("%s-gate-*-%s.result.json" % (phase, sha))):
            for row in strict_json(path)["runs"]:
                if row.get("scope") == "flight":
                    run = Path(row["run"]) if row["run"].startswith("/") else ROOT / "runs" / row["run"]
                    s = strict_json(run / "summary.json")
                    seen.setdefault(s["scenario"]["scenario"], set()).add(s["generated_world_sha256"])
    assert {k: v for k, v in seen.items()} == {k: {v} for k, v in r.ACCEPTED_WORLDS.items()}


def test_schedule_slots_are_the_frozen_ones():
    first = r.schedule()["pilot"]["slots"][0]
    assert r.schedule_slot("pilot", 1, 1) == first
    row = r.schedule_slot("confirmatory-200", 4, 61)
    assert row["block"] == 4 and row["slot"] == 61 and row["layout"].startswith("p7c-")
    for scope, block, slot in (("pilot", 1, 21), ("confirmatory-200", 4, 60), ("confirmatory-250", 13, 239)):
        with pytest.raises(PermissionError):
            r.schedule_slot(scope, block, slot)


def test_parse_maps_a_p7_flight_onto_the_accepted_flight_scope():
    a = r.parse(["--run", "--controller", "p5", "--item", "p7p-5000001", "--scope", "pilot", "--block", "1", "--slot", "4",
                 "--manifest", "m", "--authorization", "a", "--allow-simulated-flight"])
    assert (a.scope, a.scenario, a.gate, a.fixture, a.variant) == ("flight", "p7p-5000001", "P7", None, None)
    assert a.role == "p7-pilot-b01-s004-p5" and a.p7_scope == "pilot"
    for argv in (["--scope", "DRY", "--block", "1", "--item", "S2"], ["--scope", "pilot", "--item", "p7p-5000001"],
                 ["--scope", "pilot", "--block", "1", "--slot", "1", "--item", "S2"]):
        with pytest.raises(SystemExit):
            r.parse(["--run", "--controller", "p4", "--manifest", "m", "--authorization", "a"] + argv)


def test_a_p7_flight_needs_the_accepted_flight_gating_before_anything_else(monkeypatch):
    monkeypatch.setenv("GWM_ALLOW_SITL_COMMANDS", "0")
    fake = SimpleNamespace(args=SimpleNamespace(allow_simulated_flight=True))
    with pytest.raises(PermissionError, match="GWM_ALLOW_SITL_COMMANDS"):
        r.P7Flight.preconditions(fake)
    monkeypatch.setenv("GWM_ALLOW_SITL_COMMANDS", "1")
    fake.args.allow_simulated_flight = False
    with pytest.raises(PermissionError, match="allow-simulated-flight"):
        r.P7Flight.preconditions(fake)
    fake.args.allow_simulated_flight = True
    for key in r.base.GATES:
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(ValueError, match="Missing explicit gates"):
        r.P7Flight.preconditions(fake)


def prepared(tmp_path, controller, item, dry=False):
    fake = object.__new__(r.RUNNERS[controller])
    (tmp_path / "auth.json").write_text("{}")
    fake.root, fake.controller, fake.item, fake.dry = tmp_path, controller, item, dry
    fake.p7_scope, fake.block, fake.slot = ("DRY", 0, 0) if dry else ("pilot", 1, 1)
    fake.slot_record = None if dry else r.schedule_slot("pilot", 1, 1)
    fake.scenarios, fake.p7_frozen = r.p7_scenarios(), fake_frozen()
    fake.args = SimpleNamespace(authorization=tmp_path / "auth.json")
    fake.prepare_run(m.runner_view(fake_frozen(), controller))
    return fake


@pytest.mark.parametrize("controller", ["p4", "p5"])
def test_run_local_inputs_are_the_accepted_configs_with_only_the_enable_flags_and_the_layout_record(tmp_path, controller):
    item = r.schedule_slot("pilot", 1, 1)["layout"]
    fake = prepared(tmp_path, controller, item)
    run = fake.run
    assert run.parent == tmp_path / "runs" and ("-p7-%s-%s-" % (controller, item)) in run.name
    sim, p4 = REPO / "simulation/px4_gazebo/configs", REPO / "simulation/px4_gazebo_p4/configs"
    for name in ("p3_sensors.yaml", "p3_validation.yaml", "p3_sensor_bridge.yaml", "p3_fastdds.xml", "p3_native_command_policy.json"):
        assert digest(run / name) == digest(sim / name)
    assert digest(run / "p4_scenarios.json") == digest(p4 / "p4_scenarios.json")  # the accepted file, unchanged
    local, accepted = strict_json(run / "p4_avoidance.yaml"), strict_json(p4 / "p4_avoidance.yaml")
    assert local.pop("enabled") is True and accepted.pop("enabled") is False and local == accepted
    assert (run / "p5").exists() == (controller == "p5") and (run / "p5_policy.yaml").exists() == (controller == "p5")
    if controller == "p5":
        policy = strict_json(run / "p5_policy.yaml")
        assert policy["enabled"] is True and policy["model"]["sha256"] == "11" * 32
    record = strict_json(run / "p7/layout.json")
    assert record["expected_world_sha256"] == r.expected_world(item) and record["scenario"]["outcome"] == "p7_not_predeclared"
    assert record["layout"]["id"] == item and record["slot_record"]["slot"] == 1 and record["specs"] == dict(m.APPROVED_SPECS)
    assert not (run / "p7/dry-run-marker.json").exists()  # a block flight uses the sequencer's consumption marker


def test_a_dry_run_writes_its_own_run_local_marker(tmp_path):
    fake = prepared(tmp_path, "p4", "S2", dry=True)
    assert "-p7dry-p4-S2-" in fake.run.name and fake.marker == fake.run / "p7/dry-run-marker.json"
    assert strict_json(fake.marker)["credit"] is False
    assert strict_json(fake.run / "p7/layout.json")["expected_world_sha256"] == r.ACCEPTED_WORLDS["S2"]


def test_the_summary_carries_what_the_unchanged_evaluators_check():
    frozen = fake_frozen()
    p4_fields = r.P7P4Runner.controller_summary(object.__new__(r.P7P4Runner), m.runner_view(frozen, "p4"))
    v4 = strict_json(REPO / "simulation/px4_gazebo_p4/configs/p4_validation.yaml")
    assert p4_fields["p3_baseline"]["sha256"] == v4["p3_baseline_manifest_sha256"]
    fake = object.__new__(r.P7P5Runner)
    fake.p5_scope = "flight"
    fake.p5_validation = strict_json(REPO / "simulation/px4_gazebo_p5/configs/p5_validation.yaml")
    p5_fields = r.P7P5Runner.controller_summary(fake, m.runner_view(frozen, "p5"))
    v5 = fake.p5_validation
    assert p5_fields["p3_baseline"]["sha256"] == v5["p3_baseline_manifest_sha256"]
    assert p5_fields["p4_baseline"]["sha256"] == v5["p4_baseline_manifest_sha256"]
    assert p5_fields["p5_scope"] == "flight" and p5_fields["controller_runtime"]["openblas_num_threads"] == "1"
    assert {"p5_model", "schema_sha256", "policy_fault"} <= set(p5_fields)
    assert '"acceptance_credit_possible": False' in (P7 / "scripts/p7_runner.py").read_text()  # never P4/P5 gate credit


# ---- host check -----------------------------------------------------------------------------------------------
GOOD = {"overlay_ac": "DED574B5-45A0-4F42-8737-46345C09C238", "power_line": "Online", "cpu_percent": [3, 5, 4, 2, 6]}


def test_windows_checks_need_the_overlay_ac_power_and_an_idle_host():
    c = p7_host.config()
    assert all(v["passed"] for v in p7_host.windows_checks(GOOD, c).values())
    assert not p7_host.windows_checks(dict(GOOD, overlay_ac="961cc777-2547-4f9d-8174-7d86181b8a7a"), c)["ac_overlay"]["passed"]
    assert not p7_host.windows_checks(dict(GOOD, power_line="Offline"), c)["ac_power"]["passed"]
    assert not p7_host.windows_checks(dict(GOOD, cpu_percent=[30, 40, 20, 10, 5]), c)["host_cpu"]["passed"]
    assert p7_host.windows_checks(dict(GOOD, cpu_percent=7), c)["host_cpu"]["passed"]
    assert not p7_host.windows_checks(dict(GOOD, cpu_percent=[]), c)["host_cpu"]["passed"]


def test_the_disk_floor_counts_the_blocks_planned_use():
    c = p7_host.config()
    assert p7_host.disk_check(int(202e9), 40, c)["passed"] and not p7_host.disk_check(int(201e9), 40, c)["passed"]
    assert p7_host.disk_check(int(202e9), 40, c)["planned_bytes"] == 40 * 1.3e9


def test_the_full_check_fails_closed_on_any_item(tmp_path):
    (tmp_path / "state").mkdir()
    assert p7_host.check(tmp_path, 40, windows=GOOD, processes=[], free_bytes=int(800e9))["passed"]
    assert not p7_host.check(tmp_path, 40, windows=GOOD, processes=["123 /opt/px4/build/px4_sitl_default/bin/px4 -d"],
                             free_bytes=int(800e9))["passed"]
    assert not p7_host.check(tmp_path, 40, windows=dict(GOOD, power_line="Offline"), processes=[], free_bytes=int(800e9))["passed"]
    assert not p7_host.check(tmp_path, 40, windows={}, processes=[], free_bytes=int(800e9))["passed"]
    lock = tmp_path / "state/p1.lock"
    lock.write_text("")
    with lock.open() as held:
        fcntl.flock(held.fileno(), fcntl.LOCK_EX)
        assert not p7_host.check(tmp_path, 40, windows=GOOD, processes=[], free_bytes=int(800e9))["passed"]
    assert p7_host.check(tmp_path, 40, windows=GOOD, processes=[], free_bytes=int(800e9))["passed"]


def test_the_process_pattern_finds_simulators_and_runners_but_not_the_sequencer():
    import re
    pattern = re.compile(p7_host.config()["wsl"]["simulator_process_pattern"])
    for line in ("/home/u/uav_autonomy/mc_trace_builds/x/build/px4_sitl_default/bin/px4 -d /tmp/rootfs",
                 "gz sim --verbose 1 -r -s default.sdf", "/usr/bin/MicroXRCEAgent udp4 -p 8888",
                 "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge --ros-args",
                 "/usr/bin/python3 /mnt/d/r/simulation/px4_gazebo_p7/scripts/p7_namespace_owner.py --run"):
        assert pattern.search(line), line
    for line in ("/usr/bin/python3 /mnt/d/r/simulation/px4_gazebo_p7/scripts/run_p7_block.py --scope pilot",
                 "bash -c cd /tmp && python3 /mnt/d/r/simulation/px4_gazebo_p7/scripts/p7_host.py"):
        assert not pattern.search(line), line


# ---- sequencer ------------------------------------------------------------------------------------------------
def schedule():
    return strict_json(P7 / "configs/p7_schedule.json")


def test_the_plan_flies_each_pair_back_to_back_in_schedule_order():
    flights = seq.plan_flights(schedule(), "pilot", 1)
    slots = schedule()["pilot"]["slots"]
    assert len(flights) == 40 and [f["controller"] for f in flights[:2]] == slots[0]["order"]
    assert [(f["slot"], f["layout"]) for f in flights[::2]] == [(s["slot"], s["layout"]) for s in slots]
    assert all(f["attempt"] == 1 for f in flights) and {f["repeat"] for f in flights} == {1, 2}
    block3 = seq.plan_flights(schedule(), "confirmatory-200", 3)
    assert len(block3) == 40 and {f["block"] for f in block3} == {3} and [f["slot"] for f in block3[::2]] == list(range(41, 61))
    assert len(seq.plan_flights(schedule(), "confirmatory-250", 13)) == 20
    with pytest.raises(SystemExit):
        seq.plan_flights(schedule(), "confirmatory-200", 11)


def flight(slot, controller="p4", attempt=1):
    return {"slot": slot, "block": 1, "layout": "L%d" % slot, "controller": controller, "attempt": attempt}


def rec(validity="valid", stop=(), run="r"):
    return {"run": run, "classification": {"validity": validity, "stop_reasons": list(stop)}}


def test_the_replacement_and_stop_rules():
    rules = seq.BlockRules(40)
    assert rules.cap == 4
    rules.after(flight(1), rec())
    assert rules.stop is None and not rules.replacements
    rules.after(flight(2), rec("infrastructure_invalid"))
    assert rules.replacements == [dict(flight(2), attempt=2)] and rules.stop is None
    rules.after(dict(flight(2), attempt=2), rec("infrastructure_invalid"))
    assert rules.stop == "replacement_limit_per_flight" and rules.missing == [{"slot": 2, "layout": "L2", "controller": "p4"}]
    capped = seq.BlockRules(40)
    for slot in range(1, 5):
        capped.after(flight(slot), rec("infrastructure_invalid"))
    assert capped.stop is None and len(capped.replacements) == 4
    capped.after(flight(5), rec("infrastructure_invalid"))
    assert capped.stop == "replacement_cap"
    for record, reason in ((rec(stop=["safety:collision_contact"]), "stop_rule:safety:collision_contact"),
                           (rec(stop=["safety_unverified:truth_pose_interval"]), "stop_rule:safety_unverified:truth_pose_interval"),
                           (rec(run=None), "runtime_did_not_create_evidence"), (rec(validity=None), "unclassified_flight")):
        rules = seq.BlockRules(40)
        rules.after(flight(1), record)
        assert rules.stop == reason
    dry = seq.BlockRules(2, replacements_allowed=False)
    dry.after(flight(1), rec("infrastructure_invalid"))
    assert dry.stop is None and not dry.replacements and len(dry.missing) == 1


def test_the_sequence_flies_replacements_last_and_stops_on_drift_or_a_residual_process():
    plan = [flight(1, "p4"), flight(1, "p5"), flight(2, "p5"), flight(2, "p4")]
    flown, ledger = [], []

    def fly(f):
        flown.append((f["slot"], f["controller"], f["attempt"]))
        return rec("infrastructure_invalid" if (f["slot"], f["controller"], f["attempt"]) == (1, "p5", 1) else "valid")
    rules = seq.BlockRules(len(plan))  # cap floor(0.4) = 0: the first invalid flight exceeds it
    seq.run_sequence(plan, rules, fly, lambda: [], lambda: [], ledger.append)
    assert rules.stop == "replacement_cap" and flown == [(1, "p4", 1), (1, "p5", 1)]
    flown.clear()
    rules = seq.BlockRules(40)
    records, not_flown = seq.run_sequence(plan, rules, fly, lambda: [], lambda: [], ledger.append)
    assert rules.stop is None and not_flown == [] and flown[-1] == (1, "p5", 2) and len(records) == 5
    flown.clear()
    calls = iter([[], ["simulation/px4_gazebo_p7/scripts/p7_runner.py"]])
    rules = seq.BlockRules(40)
    records, not_flown = seq.run_sequence(plan, rules, fly, lambda: next(calls), lambda: [], lambda x: None)
    assert rules.stop.startswith("manifest_drift:") and len(records) == 1 and not_flown == plan[1:]
    rules = seq.BlockRules(40)
    records, not_flown = seq.run_sequence(plan, rules, fly, lambda: [], lambda: ["9 gz sim -s"], lambda x: None)
    assert rules.stop == "simulator_process_outlived_flight" and records[0]["residual_processes"] == ["9 gz sim -s"]


def test_runtime_commands_always_carry_the_flight_gating_and_blocks_their_slot():
    command = seq.runtime_command(dict(flight(7), layout="p7p-5000001"), "pilot", Path("m.json"), Path("a.json"))
    assert command[:3] == ["bash", str(seq.RUN_P7), "--run"] and "--allow-simulated-flight" in command
    assert command[command.index("--slot") + 1] == "7" and command[command.index("--item") + 1] == "p7p-5000001"
    dry = seq.runtime_command(dict(flight(0), layout="S2"), "DRY", Path("m.json"), Path("a.json"))
    assert "--slot" not in dry and "--block" not in dry
    assert seq.runtime_env()["GWM_ALLOW_SITL_COMMANDS"] == "1"


def test_the_ledger_is_append_only_and_a_block_never_runs_twice(tmp_path):
    ledger = seq.Ledger(tmp_path / "b.ledger.jsonl")
    ledger({"slot": 1})
    ledger({"slot": 2})
    assert [json.loads(x)["slot"] for x in (tmp_path / "b.ledger.jsonl").read_text().splitlines()] == [1, 2]
    with pytest.raises(SystemExit):
        seq.Ledger(tmp_path / "b.ledger.jsonl")


def test_blocks_run_in_order_and_not_before_the_m3_classifier(tmp_path):
    (tmp_path / "state").mkdir()
    assert seq.previous_block_ok(tmp_path, "confirmatory-200", 1, SHA)
    assert not seq.previous_block_ok(tmp_path, "confirmatory-200", 2, SHA)
    m.result_path(tmp_path, "confirmatory-200", 1, SHA).write_text(json.dumps({"stopped": True}))
    assert not seq.previous_block_ok(tmp_path, "confirmatory-200", 2, SHA)
    m.result_path(tmp_path, "confirmatory-200", 1, SHA).write_text(json.dumps({"stopped": False}))
    assert seq.previous_block_ok(tmp_path, "confirmatory-200", 2, SHA)
    if not seq.CLASSIFIER.exists():
        with pytest.raises(SystemExit, match="M3"):
            seq.classifier()


# ---- D15: the flight window before a fail-closed exit -------------------------------------------------------
P2_PATH = ["WAIT_FOR_VALID_ESTIMATION", "PRESTREAM_SAFE_SETPOINTS", "REQUEST_OFFBOARD", "VERIFY_OFFBOARD", "REQUEST_ARM",
           "VERIFY_ARMED", "TAKEOFF", "TRANSIT"]


def transitions(*ends):
    return [{"to": name, "sim_s": 10.0 + i} for i, name in enumerate(P2_PATH + list(ends))]


def test_the_flight_window_is_the_accepted_prestream_to_complete_or_recovery():
    assert r.flight_window({"transitions": transitions("COMPLETE")}) == [11.0, 18.0]
    assert r.flight_window({"transitions": transitions("RECOVERY")}) == [11.0, 18.0]
    assert r.flight_window({"transitions": transitions()}) is None and r.flight_window(None) is None
    p6_text = (REPO / "simulation/px4_gazebo_p6/scripts/p6_runner.py").read_text()
    assert 'end = transitions.get("COMPLETE") or transitions.get("RECOVERY")' in p6_text  # the same rule as P6


def controlled(run, exit_code, ends):
    run.mkdir()
    fake = object.__new__(r.P7P5Runner)
    fake.run, fake.start, fake.declaration, fake.injection = run, 0.0, {"fault": None}, None
    fake.summary = {"status": "incomplete"}
    fake.controller = SimpleNamespace(poll=lambda: exit_code, record={"exit_code": exit_code})
    fake.pump = lambda: None
    (run / "ros-events.jsonl").write_text("")
    (run / "controller-result.json").write_text(json.dumps(
        {"transitions": transitions(*ends), "failure": "observation_gap" if exit_code else None, "graph_valid": True}))
    return fake


def test_a_fail_closed_exit_keeps_the_flight_window_for_the_unchanged_evaluator(tmp_path):
    fake = controlled(tmp_path / "a", 1, ["RECOVERY"])
    with pytest.raises(r.base.Abort, match="controller_failed:observation_gap"):
        fake.run_controller({"wall_deadline_s": 1e9})
    assert fake.summary["sensor_window_sim_s"] == [11.0, 18.0] and fake.summary["controller_exit_code"] == 1
    assert fake.summary["controller_publishers_only_from_controller"] is True and fake.summary["status"] == "incomplete"
    ok = controlled(tmp_path / "b", 0, ["COMPLETE"])
    ok.run_controller({"wall_deadline_s": 1e9})
    assert ok.summary["status"] == "passed" and ok.summary["sensor_window_sim_s"] == [11.0, 18.0]
    early = controlled(tmp_path / "c", 1, [])
    with pytest.raises(r.base.Abort):
        early.run_controller({"wall_deadline_s": 1e9})
    assert "sensor_window_sim_s" not in early.summary  # no PRESTREAM-to-end window exists: nothing is invented
